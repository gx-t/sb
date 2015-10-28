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
#include <math.h>

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


static volatile unsigned char* io_base = 0; 
#define PIOB_PER					(*(volatile unsigned*)&io_base[0x600])
#define PIOB_OER					(*(volatile unsigned*)&io_base[0x610])
#define PIOB_ODR					(*(volatile unsigned*)&io_base[0x614])
#define PIOB_SODR					(*(volatile unsigned*)&io_base[0x630])
#define PIOB_CODR					(*(volatile unsigned*)&io_base[0x634])
#define PIOB_PDSR					(*(volatile unsigned*)&io_base[0x63C])
#define PIOB_PPUER					(*(volatile unsigned*)&io_base[0x664])

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

//=============================================================================
static void ctrl_c(int sig) {
	(void)sig;
	g_run = 0;
}

static unsigned char* lib_open_base(off_t offset) {
	unsigned char* map_base = MAP_FAILED;
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if(fd == -1) {
		perror("/dev/mem");
		return 0;
	}
	map_base = (unsigned char*)mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
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
	PIOB_OER = mask; //enable output
	PIOB_CODR = mask; //level low
	lib_delay_us(60);
	PIOB_ODR = mask; //disable output
	lib_delay_us(4);
}

static void w1_write_1(int mask) {
	PIOB_OER = mask; //enable output
	PIOB_CODR = mask; //level low
	lib_delay_us(4);
	PIOB_ODR = mask; //disable output
	lib_delay_us(60);
}

static unsigned w1_read(int mask) {
	PIOB_OER = mask; //enable output
	PIOB_CODR = mask; //level low
	lib_delay_us(4);
	PIOB_ODR = mask; //disable output
	lib_delay_us(10);
	unsigned bit_value = PIOB_PDSR & mask;
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
	PIOB_OER = mask; //enable output
	PIOB_CODR = mask; //level low
	lib_delay_us(500);
	PIOB_ODR = mask; //disable output
	lib_delay_us(60);
	int data_bit = PIOB_PDSR;
	usleep(1000);
	return data_bit & mask;
}

static int ds18b20_read_temp(float* temp, int mask) {
	int res = 0;
	//start conversion
	PIOB_ODR = mask; //disable output
	ds18b20_reset(mask);
	w1_write_byte(mask, DS18B20_SKIP_ROM);
	w1_write_byte(mask, DS18B20_CONVERT_T);
	res = usleep(750000);
	//reading
	ds18b20_reset(mask);
	w1_write_byte(mask, DS18B20_SKIP_ROM);
	w1_write_byte(mask, DS18B20_READ_SCRATCHPAD);
	*temp = (w1_read_byte(mask) | (w1_read_byte(mask) << 8));
	(*temp) /= 16;
	//skip reading the rest of scratchpad bytes
	PIOB_OER = mask; //enable output
	PIOB_CODR = mask; //level low
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
	io_base = lib_open_base(0xFFFFF000);
	PIOB_PER = mask;
	PIOB_PPUER = mask; //enable pull up
	if(ds18b20_reset(mask)) {
		fprintf(stderr, "DS18B20 not detected.\n");
		return 0;
	}
	float old = -999, temp;
	while(g_run && !ds18b20_read_temp(&temp, mask)) {
		if(temp - old < 0.1 && old - temp < 0.1) continue;
		data.sd.time = time(0);
		data.sd.val = temp;
		write(1, &data, sizeof(data));
		old = temp;
	}
	fprintf(stderr, "ds18b20===<<\n");
	lib_close_base(io_base);
	return 0;
}

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
	float old = -999;
	int res = ERR_OK;
	while(g_run && !usleep(300000)) {
		char buff[2] = {0};
		if(1 != write(fd, buff, 1) || 2 != read(fd, buff, 2)) {
			fprintf(stderr, "lm75 temperature read failed\n");
			res = -1;
			break;
		}
		float temp = (float)((short)buff[0] << 8 | buff[1]) / 256;
		if(temp - old < 0.5 && old - temp < 0.5) continue;
		data.sd.time = time(0);
		data.sd.val = temp;
		write(1, data.bt, sizeof(data.bt));
		old = temp;
	}
	close(fd);
	fprintf(stderr, "lm75   ===<<\n");
	return res;
}


//=============================================================================
//ADC FUNCTIONS

static int adc_enable(char* chan) {
	char fname[64];
	sprintf(fname, "/sys/class/misc/adc/ch%s_enable", chan);
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

static int adc_read(float* val, char* chan) {
	char fname[64];
	sprintf(fname, "/sys/class/misc/adc/ch%s_value", chan);
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
	if(adc_read(&old, argv[1])) {
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
		if(adc_read(&val, argv[1])) {
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
	io_base = lib_open_base(0xFFFFF000);
	PIOB_PER = mask;
	PIOB_ODR = mask;
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
			int state = !!(PIOB_PDSR & mask);
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
	lib_close_base(io_base);
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
	io_base = lib_open_base(0xFFFFF000);
	PIOB_PER = MAGNET_MASK;
	PIOB_ODR = MAGNET_MASK;
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
			int strobe = !!(PIOB_PDSR & (1 << 13));
			if(strobe == 1 && !old_strobe) { 
				unsigned uval = PIOB_PDSR & 0xFFFF0000;
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
	lib_close_base(io_base);
	fprintf(stderr, "magnet====<<\n");
	return 0;
}

//=============================================================================
//SHT1X FUNCTIONS
static void sht1x_shift_in(int clk_mask) {
	PIOB_SODR = clk_mask; //clock up
	PIOB_CODR = clk_mask; //clock down
}

static void sht1x_shift_out(int clk_mask, int data_mask, int* data) {
	PIOB_SODR = clk_mask;
	lib_delay_us(10);//pull-up moves input slowly, needs delay
	int bit = PIOB_PDSR & data_mask;
	PIOB_CODR = clk_mask;
	if(!data) return;
	(*data) <<= 1;
	(*data) |= !!bit;
}

static void sht1x_read_byte(int clk_mask, int data_mask, int* data, int skip_ack) {
	PIOB_ODR = data_mask; //data is input
	sht1x_shift_out(clk_mask, data_mask, data);
	sht1x_shift_out(clk_mask, data_mask, data);
	sht1x_shift_out(clk_mask, data_mask, data);
	sht1x_shift_out(clk_mask, data_mask, data);
	sht1x_shift_out(clk_mask, data_mask, data);
	sht1x_shift_out(clk_mask, data_mask, data);
	sht1x_shift_out(clk_mask, data_mask, data);
	sht1x_shift_out(clk_mask, data_mask, data);
	PIOB_OER = data_mask; //data output
	if(skip_ack) {
		PIOB_SODR = data_mask; //data up
	}
	else {
		PIOB_CODR = data_mask; //data down
	}
	PIOB_SODR = clk_mask; //clock up
	PIOB_CODR = clk_mask; //clock down
}

static void sht1x_init(int clk_mask, int data_mask) {
	//init ports
	PIOB_PER = clk_mask | data_mask; //enable clock and data
	PIOB_OER = clk_mask; //clock always output
	PIOB_CODR = clk_mask; //clock down
	PIOB_OER = data_mask; //data initially output
	PIOB_PPUER = data_mask; //pullup on data

	//init sensor
	PIOB_SODR = data_mask; //data up
	PIOB_SODR = clk_mask; //clock up
	PIOB_CODR = data_mask; //data down
	PIOB_CODR = clk_mask; //clock down
	PIOB_SODR = clk_mask; //clock up
	PIOB_SODR = data_mask; //data up
	PIOB_CODR = clk_mask; //clock down
}

static int sht1x_read_sensor(float* val, int clk_mask, int data_mask, int temp) {
	int res = 0;
	sht1x_init(clk_mask, data_mask);
	//address 000
	PIOB_CODR = data_mask; //data down
	sht1x_shift_in(clk_mask);
	sht1x_shift_in(clk_mask);
	sht1x_shift_in(clk_mask);

	//command 00011
	sht1x_shift_in(clk_mask);
	sht1x_shift_in(clk_mask);
	if(temp) {
		sht1x_shift_in(clk_mask);
		PIOB_SODR = data_mask; //data up
		sht1x_shift_in(clk_mask);
		sht1x_shift_in(clk_mask);
	}
	else {
		PIOB_SODR = data_mask; //data up
		sht1x_shift_in(clk_mask);
		PIOB_CODR = data_mask; //data down
		sht1x_shift_in(clk_mask);
		PIOB_SODR = data_mask; //data up
		sht1x_shift_in(clk_mask);
	}

	//read ACK
	PIOB_ODR = data_mask; //data is input
	sht1x_shift_out(clk_mask, data_mask, 0);

	res = usleep(500000);
	int data = 0;
	//read 1st byte
	sht1x_read_byte(clk_mask, data_mask, &data, 0);
	//read 2nd byte
	sht1x_read_byte(clk_mask, data_mask, &data, 1);

	*val = (float)data;
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
	io_base = lib_open_base(0xFFFFF000);
	float old_temp = -999, temp, old_hum = -999, hum;
	while(g_run
			&& !sht1x_read_sensor(&temp, clk_mask, data_mask, 1)
			&& g_run
			&& !sht1x_read_sensor(&hum, clk_mask, data_mask, 0)) {
		temp *= 0.01;
		temp -= 40.1;
		hum = (temp - 25.0) * (0.01 + 0.00008 * hum) + 0.0367 * hum - 1.5955e-6 * hum * hum - 2.0468;
		data.sd.time = time(0);
		if(temp - old_temp > 0.5 || old_temp - temp > 0.5) {
			data.sd.dev = DEV_SHT1XTEMP;
			data.sd.val = temp;
			write(1, data.bt, sizeof(data.bt));
			old_temp = temp;
		}
		if(hum - old_hum > 1 || old_hum - hum > 1) {
			data.sd.dev = DEV_SHT1XHUM;
			data.sd.val = hum;
			write(1, data.bt, sizeof(data.bt));
			old_hum = hum;
		}
	}
	PIOB_CODR = data_mask; //data down
	PIOB_ODR = data_mask; //data is input
	PIOB_CODR = clk_mask; //clock down
	fprintf(stderr, "sht1x=====<<\n");
	lib_close_base(io_base);
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
		"sht1x-humidity",
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
		"humidity",
	};
	return type_arr[dev_id];
}

static void filter_sql_sd(int* data_avail, const char* key, const char* tbl, union CMD_BUFF* data) {
	time_t t0 = data->sd.time;
	struct tm* t1 = gmtime(&t0);
	PIOB_SODR = LED0_MASK;
	if(!*data_avail) {
		(*data_avail) ++;
	}
	printf("insert into %s values(\'%04d-%02d-%02d %02d:%02d:%02d\', \'%s-%d\', \'%g\', \'%s\', \'%s\');\n",
			tbl, t1->tm_year + 1900, t1->tm_mon, t1->tm_mday, t1->tm_hour, t1->tm_min, t1->tm_sec,
			filter_dev_name(data->sd.dev), data->sd.lun, data->sd.val, key, filter_dev_type(data->sd.dev));
	PIOB_CODR = LED0_MASK;
}

static void filter_sql_fd(int* data_avail, const char* key, const char* tbl, union CMD_BUFF* data, const char* outdir) {
	if(*data_avail) {
		PIOB_SODR = LED1_MASK;
		(*data_avail) = 0;
		fflush(stdout);
		if(strcmp("-", outdir)) {
			close(1);
			char fname[32];
			sprintf(fname, "%s/%08lX", outdir, data->fd.time);
			rename(".data", fname);
			dup2(open(".data", O_CREAT | O_WRONLY), 1);
		}
		PIOB_CODR = LED1_MASK;
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
	io_base = lib_open_base(0xFFFFF000);
	PIOB_PER = LED0_MASK | LED1_MASK;
	PIOB_OER = LED0_MASK | LED1_MASK;
	int data_avail = 0;
	union CMD_BUFF data = {{0}};
	while(0 < read(0, data.bt, sizeof(data.bt))) {
		if(data.cmd >= CMD_LAST) continue;
		PIOB_SODR = LED0_MASK;
		arr[data.cmd](&data_avail, argv[1], argv[2], &data, argv[3]);
		PIOB_CODR = LED0_MASK;
	}
	fflush(stdout);
	lib_close_base(io_base);
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

