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
#include "at91sam9g20.h"

enum {
	ERR_OK = 0,
	ERR_ARGC,
	ERR_PIPE,
	ERR_CMD,
	ERR_MMAP,
	ERR_PIN,
	ERR_VAL,
	ERR_RESET,
	ERR_I2COPEN,
	ERR_I2CADDR,
	ERR_OPENDEV,
	ERR_NOSENSOR,
	ERR_READI2C,
	ERR_ADCCHAN,
	ERR_READADC
};

enum {
	DEV_DS18B20,
	DEV_LM75,
	DEV_ADC,

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
#define DS18B20_MASK				(1 << 12)
#define LM75_ADDR					0x4F
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

//=============================================================================
static int dev_ds18b20(int argc, char* argv[]) {
	//... ds18b20 <pin> <lun>
	if(argc < 3) {
		fprintf(stderr, "Usage: ... %s <pin> <lun>\n", argv[0]);
		return ERR_ARGC;
	}
	fprintf(stderr, "ds18b20===>>\n");
	int pin = lib_piob_from_pin(atoi(argv[1]));
	if(pin < 0) return ERR_PIN;
	int mask = 1 << pin;
	union CMD_BUFF data = {{0}};
	data.sd.cmd = CMD_SDATA;
	data.sd.dev = DEV_DS18B20;
	data.sd.lun = atoi(argv[2]);
	io_map_base = lib_open_base((off_t)AT91C_BASE_AIC);
	io_port_b = PIO_B(io_map_base);
	io_port_b->PIO_PER = mask;
	io_port_b->PIO_PPUER = mask; //enable pull up
	if(ds18b20_reset(mask)) {
		fprintf(stderr, "DS18B20 not detected.\n");
		return 0;
	}
	float old = -99999999;
	//start conversion
	ds18b20_reset(mask);
	w1_write_byte(mask, DS18B20_SKIP_ROM);
	w1_write_byte(mask, DS18B20_CONVERT_T);
	while(g_run && !usleep(750000)) {
		//reading
		ds18b20_reset(mask);
		w1_write_byte(mask, DS18B20_SKIP_ROM);
		w1_write_byte(mask, DS18B20_READ_SCRATCHPAD);
		float temp = w1_read_byte(mask) | (w1_read_byte(mask) << 8);
		//skip reading the rest of scratchpad bytes
		io_port_b->PIO_OER = mask; //enable output
		io_port_b->PIO_CODR = mask; //level low
		temp /= 16;
		if(temp != old) {
			io_port_b->PIO_SODR = LED0_MASK;
			data.sd.time = time(0);
			data.sd.val = temp;
			write(1, &data, sizeof(data));
			io_port_b->PIO_CODR = LED0_MASK;
			old = temp;
		}
		io_port_b->PIO_ODR = mask; //disable output

		//start conversion
		ds18b20_reset(mask);
		w1_write_byte(mask, DS18B20_SKIP_ROM);
		w1_write_byte(mask, DS18B20_CONVERT_T);
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
	io_map_base = lib_open_base((off_t)AT91C_BASE_AIC);
	io_port_b = PIO_B(io_map_base);
	io_port_b->PIO_PER = LED0_MASK;
	io_port_b->PIO_OER = LED0_MASK;
	union CMD_BUFF data = {{0}};
	data.sd.cmd = CMD_SDATA;
	data.sd.dev = DEV_LM75;
	data.sd.lun = atoi(argv[2]);
	float old = -9999999;
	int res = ERR_OK;
	while(g_run && !usleep(300000)) {
		char buff[2] = {0};
		if(1 != write(fd, buff, 1) || 2 != read(fd, buff, 2)) {
			fprintf(stderr, "lm75 temperature read failed\n");
			res = -1;
			break;
		}
		float temp = (float)((short)buff[0] << 8 | buff[1]) / 256;
		if(temp == old) continue;
		io_port_b->PIO_SODR = LED0_MASK;
		data.sd.time = time(0);
		data.sd.val = temp;
		write(1, data.bt, sizeof(data.bt));
		io_port_b->PIO_CODR = LED0_MASK;
		old = temp;
	}
	close(fd);
	fprintf(stderr, "lm75   ===<<\n");
	lib_close_base(io_map_base);
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
	float old = -1;
	io_map_base = lib_open_base((off_t)AT91C_BASE_AIC);
	io_port_b = PIO_B(io_map_base);
	io_port_b->PIO_PER = LED0_MASK;
	io_port_b->PIO_OER = LED0_MASK;
	union CMD_BUFF data = {{0}};
	data.sd.cmd = CMD_SDATA;
	data.sd.dev = DEV_ADC;
	data.sd.lun = atoi(argv[1]);
	while(g_run && !usleep(period)) {
		float val = 0;
		if(adc_read(&val)) {
			break;
		}
		val += old;
		val /= 2;
		if(abs(val - old) < 1) continue;
		io_port_b->PIO_SODR = LED0_MASK;
		data.sd.time = time(0);
		data.sd.val = val;
		write(1, data.bt, sizeof(data.bt));
		io_port_b->PIO_CODR = LED0_MASK;
		old = val;
	}
	fprintf(stderr, "adc    ===<<\n");
	lib_close_base(io_map_base);
	return ERR_OK;
}


static int dev_main(int argc, char* argv[]) {
	//... <dev type> ...
	if(argc < 2) {
		fprintf(stderr, "Usage: ... %s ds18b20|lm75|adc\n", argv[0]);
		return ERR_ARGC;
	}
	argc --;
	argv ++;
	if(!strcmp("ds18b20", *argv)) return dev_ds18b20(argc, argv);
	if(!strcmp("lm75", *argv)) return dev_lm75(argc, argv);
	if(!strcmp("adc", *argv)) return dev_adc(argc, argv);
	return ERR_CMD;
#if 0
	io_map_base = lib_open_base((off_t)AT91C_BASE_AIC);
	tcb_base = lib_open_base((off_t)AT91C_BASE_TC0);
	io_port_b = PIO_B(io_map_base);
	io_port_b->PIO_PER = LED0_MASK | LED1_MASK;
	io_port_b->PIO_OER = LED0_MASK | LED1_MASK;
	int res = send_clock(atoi(argv[3])) && !sleep(1) && ds18b20() && lm75() && adc();
	while(res && -1 != wait(0));
	lib_close_base(tcb_base);
	lib_close_base(io_map_base);
	return ERR_OK;
#endif
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
	union CMD_BUFF data = {{0}};
	data.fd.cmd = CMD_FLUSH;
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
		"adc"
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
		"analog"
	};
	return type_arr[dev_id];
}

static void filter_sql_sd(const char* key, const char* tbl, union CMD_BUFF* data) {
	time_t t0 = data->sd.time;
	struct tm* t1 = gmtime(&t0);
	printf("insert into %s values(\'%04d-%02d-%02d %02d:%02d:%02d\', \'%s-%d\', \'%g\', \'%s\', \'%s\');\n",
		tbl, t1->tm_year + 1900, t1->tm_mon, t1->tm_mday, t1->tm_hour, t1->tm_min, t1->tm_sec,
		filter_dev_name(data->sd.dev), data->sd.lun, data->sd.val, key, filter_dev_type(data->sd.dev));
}

static void filter_sql_fd(const char* key, const char* tbl, union CMD_BUFF* data) {
	printf("===CMD_FLUSH: 0x%08X, 0x%08lX\n", data->fd.cmd, data->fd.time);
}

static int filter_sql(int argc, char* argv[]) {
	//... sql <key> <table name>
	static void (*arr[])(const char*, const char*, union CMD_BUFF*) ={
		filter_sql_sd,
		filter_sql_fd
	};
	if(argc != 3) {
		fprintf(stderr, "Usage: ... %s <key> <table>\n", argv[0]);
		return ERR_ARGC;
	}
	fprintf(stderr, "filter.sql   ===>>\n");
	union CMD_BUFF data = {{0}};
	while(0 < read(0, data.bt, sizeof(data.bt))) {
		if(data.cmd >= CMD_LAST) continue;
		arr[data.cmd](argv[1], argv[2], &data);
	}
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
	return ERR_CMD;
}

