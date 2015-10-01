#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <poll.h>
#include "at91sam9g20.h"

enum {
	ERR_OK = 0,
	ERR_ARGC,
	ERR_CMD,
	ERR_PIN,
	ERR_I2COPEN,
	ERR_I2CADDR,
	ERR_ADCCHAN,
	ERR_READADC,
	ERR_OUTDIR
};

enum {
	DEV_DS18B20,
	DEV_LM75,
	DEV_ADC,
	DEV_COUNTER,
	DEV_MAGNET,
	DEV_MAGTIME,
	DEV_SHT1XTEMP,
	DEV_SHT1XHUM,

	DEV_LAST
};

enum {
	CMD_SDATA,
	CMD_FLUSH,


	CMD_LAST
};

union CMD_BUFF {
	unsigned char bt[20];
	
	unsigned cmd;
	
	struct {
		unsigned 		cmd; //enum CMD_SDATA
		time_t			time;
		unsigned 		dev;
		unsigned 		lun;
		float			val;
	} sd;

	struct {
		unsigned		cmd;
		time_t			time;
	} fd;
};


#define PIO_B(_b)					((AT91S_PIO*)(_b + 0x600))
#define PMC(_b)						((AT91S_PMC*)(_b + 0xC00))

#define MAP_SIZE					4096UL

#define LED0_MASK					(1 << 0)
#define LED1_MASK					(1 << 1)
#define MAGNET_MASK					0xFFFF2000
#define ADC_CHANNEL					0

#define DS18B20_READ_ROM			0x33
#define DS18B20_SKIP_ROM			0xCC
#define DS18B20_READ_SCRATCHPAD		0xBE
#define DS18B20_CONVERT_T			0x44
static int g_run = 1;

static volatile void* io_map_base 	 = 0; 
static volatile AT91S_PIO* io_port_b = 0;
//static volatile AT91S_TCB* tcb_base  = 0;

//=============================================================================
static void ctrl_c(int sig) {
	(void)sig;
	g_run = 0;
}

static void* lib_open_base(off_t offset) {
	void* map_base = MAP_FAILED;
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if(fd == -1) {
		perror("/dev/mem");
		return 0;
	}
	map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
	close(fd);
	if(MAP_FAILED == map_base) {
		perror("mmap");
		return 0;
	}
	return map_base;
}

static inline void lib_close_base(volatile void* map_base) {
	munmap((void*)map_base, MAP_SIZE);
}

//board pin to bit shift for PIOB
static int lib_piob_from_pin(int pin) {
	if(pin < 3 || pin == 17 || pin == 18 || pin > 34) return -1;
	return pin - 3;
}

// calibrated with good accuracy for GESBC-9G20u board
static void lib_delay_us(unsigned us) {
	volatile unsigned i = 79 * us / 2;
	while(i --);
}

static void lib_export_gpiob(int gpiob)
{
	int fd = open("/sys/class/gpio/export", O_WRONLY);
	char buff[8];
	int len = snprintf(buff, sizeof(buff), "%d", gpiob + 64);
	write(fd, buff, len);
	close(fd);
}

static void lib_unexport_gpiob(int gpiob)
{
	int fd = open("/sys/class/gpio/unexport", O_WRONLY);
	char buff[8];
	int len = snprintf(buff, sizeof(buff), "%d", gpiob + 64);
	write(fd, buff, len);
	close(fd);
}

static void lib_setdir_gpiob(int gpiob, int dir)
{
	char fname[64];
	snprintf(fname, sizeof(fname), "/sys/class/gpio/gpio%d/direction", gpiob + 64);
	int fd = open(fname, O_WRONLY);
	char* sdir = dir ? "out" : "in";
	write(fd, sdir, strlen(sdir));
	close(fd);
}

static void lib_setedge_gpiob(int gpiob)
{
	char fname[64];
	snprintf(fname, sizeof(fname), "/sys/class/gpio/gpio%d/edge", gpiob + 64);
	int fd = open(fname, O_WRONLY);
	char* edge = "both";
	write(fd, edge, strlen(edge));
	close(fd);
}

static int lib_open_gpiob_read(int gpiob)
{
	char fname[64];
	snprintf(fname, sizeof(fname), "/sys/class/gpio/gpio%d/value", gpiob + 64);
	return open(fname, O_RDONLY | O_NONBLOCK);
}

// 1 wire timings from here:
// http://en.wikipedia.org/wiki/1-Wire
// checked playing with read delays - got stable result for wide range

static void w1_write_0(int mask) {
	io_port_b->PIO_OER = mask; //enable output
	io_port_b->PIO_CODR = mask; //level low
	lib_delay_us(60);
	io_port_b->PIO_ODR = mask; //disable output
	lib_delay_us(4);
}

static void w1_write_1(int mask) {
	io_port_b->PIO_OER = mask; //enable output
	io_port_b->PIO_CODR = mask; //level low
	lib_delay_us(4);
	io_port_b->PIO_ODR = mask; //disable output
	lib_delay_us(60);
}

static unsigned w1_read(int mask) {
	io_port_b->PIO_OER = mask; //enable output
	io_port_b->PIO_CODR = mask; //level low
	lib_delay_us(4);
	io_port_b->PIO_ODR = mask; //disable output
	lib_delay_us(10);
	unsigned bit_value = io_port_b->PIO_PDSR & mask;
	lib_delay_us(45);
	return bit_value;
}

static unsigned w1_read_byte(int mask) {
	unsigned result = 0;
	if(w1_read(mask)) result |= 0x01;
	if(w1_read(mask)) result |= 0x02;
	if(w1_read(mask)) result |= 0x04;
	if(w1_read(mask)) result |= 0x08;
	if(w1_read(mask)) result |= 0x10;
	if(w1_read(mask)) result |= 0x20;
	if(w1_read(mask)) result |= 0x40;
	if(w1_read(mask)) result |= 0x80;
	return result;
}

static void w1_write_byte(int mask, unsigned data) {
	data & 0x01 ? w1_write_1(mask) : w1_write_0(mask);
	data & 0x02 ? w1_write_1(mask) : w1_write_0(mask);
	data & 0x04 ? w1_write_1(mask) : w1_write_0(mask);
	data & 0x08 ? w1_write_1(mask) : w1_write_0(mask);
	data & 0x10 ? w1_write_1(mask) : w1_write_0(mask);
	data & 0x20 ? w1_write_1(mask) : w1_write_0(mask);
	data & 0x40 ? w1_write_1(mask) : w1_write_0(mask);
	data & 0x80 ? w1_write_1(mask) : w1_write_0(mask);
}

static unsigned ds18b20_reset(mask) {
	io_port_b->PIO_OER = mask; //enable output
	io_port_b->PIO_CODR = mask; //level low
	lib_delay_us(500);
	io_port_b->PIO_ODR = mask; //disable output
	lib_delay_us(60);
	int data_bit = io_port_b->PIO_PDSR;
	usleep(1000);
	return data_bit & mask;
}

static int ds18b20_read_temp(float* temp, int mask) {
	int res = 0;
	//start conversion
	io_port_b->PIO_ODR = mask; //disable output
	ds18b20_reset(mask);
	w1_write_byte(mask, DS18B20_SKIP_ROM);
	w1_write_byte(mask, DS18B20_CONVERT_T);
	res = usleep(750000);
	//reading
	ds18b20_reset(mask);
	w1_write_byte(mask, DS18B20_SKIP_ROM);
	w1_write_byte(mask, DS18B20_READ_SCRATCHPAD);
	*temp = (w1_read_byte(mask) | (w1_read_byte(mask) << 8)) / 16;
	//skip reading the rest of scratchpad bytes
	io_port_b->PIO_OER = mask; //enable output
	io_port_b->PIO_CODR = mask; //level low
	return res;
}

//=============================================================================
static int dev_ds18b20(int argc, char* argv[]) {
	//... ds18b20 <pin> <lun>
	if(argc != 3) {
		fprintf(stderr, "Usage: ... %s <pin> <lun>\n", argv[0]);
		return ERR_ARGC;
	}
	fprintf(stderr, "ds18b20===>>\n");
	int pin = lib_piob_from_pin(atoi(argv[1]));
	if(pin < 0) return ERR_PIN;
	int mask = 1 << pin;
	union CMD_BUFF data = {
		.sd.cmd = CMD_SDATA,
		.sd.dev = DEV_DS18B20,
		.sd.lun = atoi(argv[2])
	};
	io_map_base = lib_open_base((off_t)AT91C_BASE_AIC);
	io_port_b = PIO_B(io_map_base);
	io_port_b->PIO_PER = mask;
	io_port_b->PIO_PPUER = mask; //enable pull up
	if(ds18b20_reset(mask)) {
		fprintf(stderr, "DS18B20 not detected.\n");
		return 0;
	}
	float old, temp;
	ds18b20_read_temp(&old, mask);
	while(g_run && !ds18b20_read_temp(&temp, mask)) {
		temp += old;
		temp /= 2;
		if(temp - old > 0.0675 || old - temp > 0.0675) {
			data.sd.time = time(0);
			data.sd.val = temp;
			write(1, &data, sizeof(data));
			old = temp;
		}
	}
	fprintf(stderr, "ds18b20===<<\n");
	lib_close_base(io_map_base);
	return 0;
}

//=============================================================================
#if 0
static int counter_main(int argc, char* argv[]) {
	if(argc != 2) {
		fprintf(stderr, "Usage: ... %s name\n", argv[0]);
		return ERR_CMD;
	}
	PMC(io_map_base)->PMC_PCER = (1 << AT91C_ID_TC0); //start periferial clock
	volatile AT91S_TCB* tcb_base = lib_open_base((off_t)AT91C_BASE_TC0);
	if(!tcb_base) {
		return ERR_MMAP;
	}
	tcb_base->TCB_TC0.TC_IDR = 0xFF;//disable all interrupts for TC0
	tcb_base->TCB_TC0.TC_CMR = AT91C_TC_CLKS_XC1 | AT91C_TC_ETRGEDG_RISING; //XC1 as clock, rising edge
	tcb_base->TCB_BMR = AT91C_TCB_TC1XC1S_TCLK1; //connect XC1 to TCLK1 (pin 9)
	tcb_base->TCB_TC0.TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG; //enable clock, reset counter
	fprintf(stderr, "%s -- %s ====>\n", argv[0], argv[1]);
	float old = -9999999;
	while(g_run && !sleep(1)) {
		lib_check_print_float(argv[1], (float)tcb_base->TCB_TC0.TC_CV, &old, "counter");
	}
	lib_close_base(tcb_base);
	return ERR_OK;
}
#endif

static int dev_lm75(int argc, char* argv[]) {
	//... <i2c addr> <lun>
	if(argc < 3) {
		fprintf(stderr, "Usage: ... %s <i2c addr> <lun>\n", argv[0]);
		return ERR_ARGC;
	}
	fprintf(stderr, "lm75   ===>>\n");
	int fd = open("/dev/i2c-0", O_RDWR);
	if(fd < 0) {
		perror("/dev/i2c-0");
		return ERR_I2COPEN;
	}
	int addr_i = -1;
	sscanf(argv[1], "%i", &addr_i);
	if(ioctl(fd, I2C_SLAVE, addr_i) < 0) {
		perror("Failed to acquire slave address\n");
		close(fd);
		return ERR_I2CADDR;
	}
	union CMD_BUFF data = {
		.sd.cmd = CMD_SDATA,
		.sd.dev = DEV_LM75,
		.sd.lun = atoi(argv[2])
	};
	float old = 0;
	int res = ERR_OK;
	while(g_run && !usleep(300000)) {
		char buff[2] = {0};
		if(1 != write(fd, buff, 1) || 2 != read(fd, buff, 2)) {
			fprintf(stderr, "lm75 temperature read failed\n");
			res = -1;
			break;
		}
		float temp = (float)((short)buff[0] << 8 | buff[1]) / 256;
		temp += old;
		temp /= 2;
		if(temp - old > 0.5 || old - temp > 0.5) continue;
		data.sd.time = time(0);
		data.sd.val = temp;
		write(1, data.bt, sizeof(data.bt));
		old = temp;
	}
	close(fd);
	fprintf(stderr, "lm75   ===<<\n");
	return 0;
}


//=============================================================================
//ADC FUNCTIONS

static int adc_enable() {
	char fname[64];
	sprintf(fname, "/sys/class/misc/adc/ch%d_enable", ADC_CHANNEL);
	int fd = open(fname, O_WRONLY);
	if(fd == -1) {
		perror(fname);
		return -1;
	}
	char cmd = '1';
	int ret = write(fd, &cmd, 1);
	close(fd);
	return 1 == ret ? 0 : -1;
}

static int adc_read(float* val) {
	char fname[64];
	sprintf(fname, "/sys/class/misc/adc/ch%d_value", ADC_CHANNEL);
	int fd = open(fname, O_RDONLY);
	if(fd == -1) {
		perror(fname);
		return -1;
	}
	char buff[6] = {0};
	int ret = read(fd, buff, sizeof(buff));
	close(fd);
	if(0 > ret) {
		perror("read ADC value");
		return -1;
	}
	sscanf(buff, "%g", val);
	return 0;
}

static int dev_adc(int argc, char* argv[]) {
	//... adc <channel> <period ms>
	if(argc < 3) {
		fprintf(stderr, "Usage: ... %s 0|1|2|3 <period ms>\n", argv[0]);
		return ERR_ARGC;
	}
	//argv[1] = channel
	//argv[2] = period
	fprintf(stderr, "adc    ===>>\n");
	int period = 1000 * atoi(argv[2]);
	if(period < 10000) period = 10000;
	if(adc_enable(argv[1])) {
		return ERR_ADCCHAN;
	}
	float old;
	if(adc_read(&old)) {
		return ERR_READADC;
	}
	union CMD_BUFF data = {
		.sd.cmd = CMD_SDATA,
		.sd.dev = DEV_ADC,
		.sd.lun = atoi(argv[1])
	};
	int ret = ERR_OK;
	while(g_run && !usleep(period)) {
		float val = 0;
		if(adc_read(&val)) {
			ret = ERR_READADC;
			break;
		}
		val += old;
		val /= 2;
		if(abs(val - old) < 1) continue;
		data.sd.time = time(0);
		data.sd.val = val;
		write(1, data.bt, sizeof(data.bt));
		old = val;
	}
	fprintf(stderr, "adc    ===<<\n");
	return ret;
}

//=============================================================================
//COUNTER FUNCTIONS
static int dev_counter(int argc, char* argv[]) {
	//... counter <pin> <lun>
	if(argc < 3) {
		fprintf(stderr, "Usage: ... %s <pin> <lun> [period ms]\n", argv[0]);
		return ERR_ARGC;
	}
	fprintf(stderr, "counter===>>\n");
	int pin = lib_piob_from_pin(atoi(argv[1]));
	if(pin < 0) return ERR_PIN;
	int mask = 1 << pin;
	io_map_base = lib_open_base((off_t)AT91C_BASE_AIC);
	io_port_b = PIO_B(io_map_base);
	io_port_b->PIO_PER = mask;
	io_port_b->PIO_ODR = mask;
	lib_export_gpiob(pin);
	lib_setdir_gpiob(pin, 0);
	lib_setedge_gpiob(pin);
	int fd = lib_open_gpiob_read(pin);
	int period = argc > 3 ? atoi(argv[3]) : -1;
	int count = 0;
	int old = -1;
	int old_state = 0;
	while(g_run) {
		struct pollfd fdset = {0};
		fdset.fd = fd;
		fdset.events = POLLPRI;
		int rc = poll(&fdset, 1, period);
		if(rc < 0) {
			break;
		}
		if(rc > 0 && (fdset.revents & POLLPRI)) {
			char tmp[4];
			read(fd, tmp, sizeof(tmp));//to clean up queue
			int state = !!(io_port_b->PIO_PDSR & mask);
			count += (state == 1 && !old_state);
			old_state = state;
			if(period == -1) rc = 0;
		}
		if(!rc && count != old) {
			union CMD_BUFF data = {
				.sd.cmd = CMD_SDATA,
				.sd.time = time(0),
				.sd.dev = DEV_COUNTER,
				.sd.lun = atoi(argv[2]),
				.sd.val = (float)count
			};
			write(1, data.bt, sizeof(data.bt));
			old = count;
		}
	}
	close(fd);
	lib_unexport_gpiob(pin);
	lib_close_base(io_map_base);
	fprintf(stderr, "counter===<<\n");
	return 0;
}

//=============================================================================
//MAGNETOMETER FUNCTIONS
static int dev_magnet(int argc, char* argv[]) {
	//... counter <pin> <lun>
	if(argc < 1) {
		fprintf(stderr, "Usage: ... %s\n", argv[0]);
		return ERR_ARGC;
	}
	fprintf(stderr, "magnet====>>\n");
	io_map_base = lib_open_base((off_t)AT91C_BASE_AIC);
	io_port_b = PIO_B(io_map_base);
	io_port_b->PIO_PER = MAGNET_MASK;
	io_port_b->PIO_ODR = MAGNET_MASK;
	lib_export_gpiob(13);
	lib_setdir_gpiob(13, 0);
	lib_setedge_gpiob(13);
	int fd = lib_open_gpiob_read(13);
	int period = -1;
	int old_strobe = 0;
	int state = 0;
	unsigned uinfo = 0;
	unsigned utiming = 0;
	while(g_run) {
		struct pollfd fdset = {0};
		fdset.fd = fd;
		fdset.events = POLLPRI;
		int rc = poll(&fdset, 1, period);
		if(rc < 0) {
			break;
		}
		if(!rc) {//timeout, reset state
			state = 0;
			period = -1;
			continue;
		}
		if(rc > 0 && (fdset.revents & POLLPRI)) {
			char tmp[4];
			read(fd, tmp, sizeof(tmp));//to clean up queue
			int strobe = !!(io_port_b->PIO_PDSR & (1 << 13));
			if(strobe == 1 && !old_strobe) { 
				unsigned uval = io_port_b->PIO_PDSR & 0xFFFF0000;
				uval >>= 16;
				switch(state) {
					case 0:
						state ++;
						period = 500;
						uinfo = uval << 16;
						break;
					case 1:
						state ++;
						uinfo |= uval; 
						break;
					case 2:
						state = 0;
						period = -1;
						utiming = uval;
						fprintf(stderr, "===Magnet - Info: %u, Timing: %u\n", uinfo, utiming);
						break;
				}
			}
			old_strobe = strobe;
		}
		/*
		if(!rc && count != old) {
			union CMD_BUFF data = {
				.sd.cmd = CMD_SDATA,
				.sd.time = time(0),
				.sd.dev = DEV_COUNTER,
				.sd.lun = atoi(argv[2]),
				.sd.val = (float)count
			};
			write(1, data.bt, sizeof(data.bt));
			old = count;
		}*/
	}
	close(fd);
	lib_unexport_gpiob(13);
	lib_close_base(io_map_base);
	fprintf(stderr, "magnet====<<\n");
	return 0;
}

//=============================================================================
//SHT1X FUNCTIONS
static void sht1x_shift_in(int clk_mask) {
	io_port_b->PIO_SODR = clk_mask; //clock up
	lib_delay_us(10);
	io_port_b->PIO_CODR = clk_mask; //clock down
}

static int sht1x_shift_out(int clk_mask, int data_mask) {
	io_port_b->PIO_SODR = clk_mask;
	int bit = io_port_b->PIO_PDSR & data_mask;
	io_port_b->PIO_CODR = clk_mask;
	lib_delay_us(10);
	return !!bit;
}

static int sht1x_read_byte(int clk_mask, int data_mask) {
	io_port_b->PIO_ODR = data_mask; //data is input
	int data = sht1x_shift_out(clk_mask, data_mask);
	data <<= 1;
	data |= sht1x_shift_out(clk_mask, data_mask);
	data <<= 1;
	data |= sht1x_shift_out(clk_mask, data_mask);
	data <<= 1;
	data |= sht1x_shift_out(clk_mask, data_mask);
	data <<= 1;
	data |= sht1x_shift_out(clk_mask, data_mask);
	data <<= 1;
	data |= sht1x_shift_out(clk_mask, data_mask);
	data <<= 1;
	data |= sht1x_shift_out(clk_mask, data_mask);
	data <<= 1;
	data |= sht1x_shift_out(clk_mask, data_mask);
	return data;
}

static void sht1x_send_ack(int clk_mask, int data_mask) {
	io_port_b->PIO_OER = data_mask; //data output
	io_port_b->PIO_CODR = data_mask; //data down
	io_port_b->PIO_SODR = clk_mask; //clock up
	lib_delay_us(10);
	io_port_b->PIO_CODR = clk_mask; //clock down
	lib_delay_us(10);
}

static void sht1x_skip_ack(int clk_mask, int data_mask) {
	io_port_b->PIO_OER = data_mask; //data output
	io_port_b->PIO_SODR = data_mask; //data up
	io_port_b->PIO_SODR = clk_mask; //clock up
	lib_delay_us(10);
	io_port_b->PIO_CODR = clk_mask; //clock down
	lib_delay_us(10);
}


static void sht1x_init(int clk_mask, int data_mask) {
	//init ports
	io_port_b->PIO_PER = clk_mask | data_mask; //enable clock and data
	io_port_b->PIO_OER = clk_mask; //clock always output
	io_port_b->PIO_CODR = clk_mask; //clock down
	io_port_b->PIO_OER = data_mask; //data initially output
	io_port_b->PIO_PPUER = data_mask; //pullup on data

	//init sensor
	io_port_b->PIO_SODR = data_mask; //data up
	io_port_b->PIO_SODR = clk_mask; //clock up
	io_port_b->PIO_CODR = data_mask; //data down
	io_port_b->PIO_CODR = clk_mask; //clock down
	lib_delay_us(10);
	io_port_b->PIO_SODR = clk_mask; //clock up
	io_port_b->PIO_SODR = data_mask; //data up
	io_port_b->PIO_CODR = clk_mask; //clock down
	lib_delay_us(10);
}

static int sht1x_read_temp(float* temp, int clk_mask, int data_mask) {
	int res = 0;
	sht1x_init(clk_mask, data_mask);
	//address 000
	io_port_b->PIO_CODR = data_mask; //data down
	sht1x_shift_in(clk_mask);
	sht1x_shift_in(clk_mask);
	sht1x_shift_in(clk_mask);

	//command 00011
	sht1x_shift_in(clk_mask);
	sht1x_shift_in(clk_mask);
	sht1x_shift_in(clk_mask);
	io_port_b->PIO_SODR = data_mask; //data up
	sht1x_shift_in(clk_mask);
	sht1x_shift_in(clk_mask);

	//read ACK
	io_port_b->PIO_ODR = data_mask; //data is input
	sht1x_shift_out(clk_mask, data_mask);

	res = usleep(500000);
	//read 1st byte
	int data = sht1x_read_byte(clk_mask, data_mask);
	sht1x_send_ack(clk_mask, data_mask);
	//read 2nd byte
	data <<= 8;
	data |= sht1x_read_byte(clk_mask, data_mask);

	//don't send to stop sending CRC
	sht1x_skip_ack(clk_mask, data_mask);

	*temp = (float)data;
	(*temp) *= 0.01;
	(*temp) -= 39.7;
	return res;
}

static int dev_sht1x(int argc, char* argv[]) {
	if(argc != 4) {
		fprintf(stderr, "Usage: ... %s <clock pin> <data pin> <lun>\n", argv[0]);
		return ERR_ARGC;
	}
	fprintf(stderr, "sht1x=====>>\n");
	int clk_pin = lib_piob_from_pin(atoi(argv[1]));
	int data_pin = lib_piob_from_pin(atoi(argv[2]));
	if(clk_pin < 0 || data_pin < 0 || clk_pin == data_pin) return ERR_PIN;
	int clk_mask = 1 << clk_pin;
	int data_mask = 1 << data_pin;
	union CMD_BUFF data = {
		.sd.cmd = CMD_SDATA,
		.sd.dev = DEV_SHT1XTEMP,
		.sd.lun = atoi(argv[3])
	};
	io_map_base = lib_open_base((off_t)AT91C_BASE_AIC);
	io_port_b = PIO_B(io_map_base);
	float old, temp;
	sht1x_read_temp(&old, clk_mask, data_mask);
	while(g_run && !sht1x_read_temp(&temp, clk_mask, data_mask)) {
		temp += old;
		temp /= 2;
		if(temp - old > 0.1 || old - temp > 0.1) {
			data.sd.time = time(0),
				data.sd.val = temp;
			write(1, data.bt, sizeof(data.bt));
		};
		old = temp;
	}
	fprintf(stderr, "sht1x=====<<\n");
	lib_close_base(io_map_base);
	return ERR_OK;
}

static int dev_main(int argc, char* argv[]) {
	//... <dev type> ...
	if(argc < 2) {
		fprintf(stderr, "Usage: ... %s ds18b20|lm75|adc|counter|magnet|sht1x\n", argv[0]);
		return ERR_ARGC;
	}
	argc --;
	argv ++;
	if(!strcmp("ds18b20", *argv)) return dev_ds18b20(argc, argv);
	if(!strcmp("lm75", *argv)) return dev_lm75(argc, argv);
	if(!strcmp("adc", *argv)) return dev_adc(argc, argv);
	if(!strcmp("counter", *argv)) return dev_counter(argc, argv);
	if(!strcmp("magnet", *argv)) return dev_magnet(argc, argv);
	if(!strcmp("sht1x", *argv)) return dev_sht1x(argc, argv);
	fprintf(stderr, "Unknown subcommand: %s\n", *argv);
	return ERR_CMD;
}

//=============================================================================
//CLOCK FUNCTIONS
static int clock_main(int argc, char* argv[]) {
	//... clock <period> <type>
	if(argc < 3) {
		fprintf(stderr, "Usage: ... %s <period sec> flush\n", argv[0]);
		return ERR_ARGC;
	}
	int period = atoi(argv[1]);
	if(period < 1) period = 1;
	if(strcmp("flush", argv[2])) {
		fprintf(stderr, "Invalid argument: %s. Must be: flush\n", argv[2]);
		return ERR_CMD;
	}
	fprintf(stderr, "clock.flush  ===>>\n");
	union CMD_BUFF data = {.fd.cmd = CMD_FLUSH};
	while(g_run && !sleep(period)) {
		data.fd.time = time(0);
		write(1, data.bt, sizeof(data.bt));
	}
	data.fd.time = time(0);
	write(1, data.bt, sizeof(data.bt));
	fprintf(stderr, "clock.flush  ===<<\n");
	return 0;
}

//=============================================================================
//FILTER FUNCTIONS

static const char* filter_dev_name(unsigned char dev_id) {
	if(dev_id >= DEV_LAST) {
		return "Unknown";
	}
	static const char* name_arr[] = {
		"ds18b20",
		"lm75",
		"adc",
		"counter",
		"magnetometer",
		"magnetometer-time",
		"sht1x-temp",
		"sht1x-humidity"
	};
	return name_arr[dev_id];
}

static const char* filter_dev_type(unsigned char dev_id) {
	if(dev_id >= DEV_LAST) {
		return "Unknown";
	}
	static const char* type_arr[] = {
		"temp",
		"temp",
		"analog",
		"count",
		"magnet",
		"magtime",
		"temp",
		"humidity"
	};
	return type_arr[dev_id];
}

static void filter_sql_sd(int* data_avail, const char* key, const char* tbl, union CMD_BUFF* data) {
	time_t t0 = data->sd.time;
	struct tm* t1 = gmtime(&t0);
	io_port_b->PIO_SODR = LED0_MASK;
	if(!*data_avail) {
		(*data_avail) ++;
		printf("begin transaction;\n");
	}
	printf("insert into %s values(\'%04d-%02d-%02d %02d:%02d:%02d\', \'%s-%d\', \'%g\', \'%s\', \'%s\');\n",
			tbl, t1->tm_year + 1900, t1->tm_mon, t1->tm_mday, t1->tm_hour, t1->tm_min, t1->tm_sec,
			filter_dev_name(data->sd.dev), data->sd.lun, data->sd.val, key, filter_dev_type(data->sd.dev));
	io_port_b->PIO_CODR = LED0_MASK;
}

static void filter_sql_fd(int* data_avail, const char* key, const char* tbl, union CMD_BUFF* data, const char* outdir) {
	if(*data_avail) {
		io_port_b->PIO_SODR = LED1_MASK;
		(*data_avail) = 0;
		printf("commit;\n");
		fflush(stdout);
		if(strcmp("-", outdir)) {
			close(1);
			char fname[32];
			sprintf(fname, "%s/%08lX", outdir, data->fd.time);
			rename(".data", fname);
			dup2(open(".data", O_CREAT | O_WRONLY), 1);
		}
		io_port_b->PIO_CODR = LED1_MASK;
	}
}

static int filter_sql(int argc, char* argv[]) {
	//... sql <key> <table name>
	static void (*arr[])() ={
		filter_sql_sd,
		filter_sql_fd
	};
	if(argc != 4) {
		fprintf(stderr, "Usage: ... %s <key> <table> <out dir>\n", argv[0]);
		return ERR_ARGC;
	}
	if(strcmp(argv[3], "-")) {
		if(chdir(argv[3])) {
			perror(argv[3]);
			return ERR_OUTDIR;
		}
		dup2(open(".data", O_CREAT | O_WRONLY), 1);
	}
	fprintf(stderr, "filter.sql   ===>>\n");
	io_map_base = lib_open_base((off_t)AT91C_BASE_AIC);
	io_port_b = PIO_B(io_map_base);
	io_port_b->PIO_PER = LED0_MASK | LED1_MASK;
	io_port_b->PIO_OER = LED0_MASK | LED1_MASK;
	int data_avail = 0;
	union CMD_BUFF data = {{0}};
	while(0 < read(0, data.bt, sizeof(data.bt))) {
		if(data.cmd >= CMD_LAST) continue;
		io_port_b->PIO_SODR = LED0_MASK;
		arr[data.cmd](&data_avail, argv[1], argv[2], &data, argv[3]);
		io_port_b->PIO_CODR = LED0_MASK;
	}
	printf("commit;\n");
	fflush(stdout);
	lib_close_base(io_map_base);
	fprintf(stderr, "filter.sql   ===<<\n");
	return ERR_OK;
}

static int filter_main(int argc, char* argv[]) {
	//... filter <type> ...
	signal(SIGINT, SIG_IGN);
	if(argc < 2) {
		fprintf(stderr, "Usage: ... %s sql ...\n", argv[0]);
		return ERR_ARGC;
	}
	argc --;
	argv ++;
	if(!strcmp("sql", *argv)) return filter_sql(argc, argv);
	fprintf(stderr, "Unknown subcommand: %s\n", *argv);
	return ERR_CMD;
}

int main(int argc, char* argv[]) {
	if(argc < 2) {
		fprintf(stderr, "Usage: %s dev|filter|clock ...\n", argv[0]);
		return ERR_ARGC;
	}
	argc --;
	argv ++;
	signal(SIGINT, ctrl_c);
	if(!strcmp("dev", *argv)) return dev_main(argc, argv);
	if(!strcmp("filter", *argv)) return filter_main(argc, argv);
	if(!strcmp("clock", *argv)) return clock_main(argc, argv);
	fprintf(stderr, "Unknown subcommand: %s\n", *argv);
	return ERR_CMD;
}

