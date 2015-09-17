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
	ERR_I2CADDR,
	ERR_OPENDEV,
	ERR_NOSENSOR,
	ERR_READI2C,
	ERR_ADCCHAN,
	ERR_READADC
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
static volatile AT91S_TCB* tcb_base  = 0;

static const char* key  = 0;
static const char* tbl  = 0;
static const char* name = 0;
static const char* type = 0;

//=============================================================================
static void ctrl_c(int sig) {
	(void)sig;
	g_run = 0;
}

static void lib_check_print_float(float val, float* old) {
	if(val == *old) return;
	//LED on
	io_port_b->PIO_SODR = LED0_MASK;
	time_t t0;
	time(&t0);
	struct tm* t1 = gmtime(&t0);
	printf("insert into %s values(\'%04d-%02d-%02d %02d:%02d:%02d\', \'%s\', \'%g\', \'%s\', \'%s\');\n",
	tbl, t1->tm_year + 1900, t1->tm_mon, t1->tm_mday, t1->tm_hour, t1->tm_min, t1->tm_sec,
	name, val, key, type);
	fflush(stdout);
	//LED off
	io_port_b->PIO_CODR = LED0_MASK;
	//new value => old value
	*old = val;
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

// calibrated with good accuracy for GESBC-9G20u board
static void lib_delay_us(unsigned us) {
	volatile unsigned i = 79 * us / 2;
	while(i --);
}

// 1 wire timings from here:
// http://en.wikipedia.org/wiki/1-Wire
// checked playing with read delays - got stable result for wide range

static void w1_write_0() {
	io_port_b->PIO_OER = DS18B20_MASK; //enable output
	io_port_b->PIO_CODR = DS18B20_MASK; //level low
	lib_delay_us(60);
	io_port_b->PIO_ODR = DS18B20_MASK; //disable output
	lib_delay_us(4);
}

static void w1_write_1() {
	io_port_b->PIO_OER = DS18B20_MASK; //enable output
	io_port_b->PIO_CODR = DS18B20_MASK; //level low
	lib_delay_us(4);
	io_port_b->PIO_ODR = DS18B20_MASK; //disable output
	lib_delay_us(60);
}

static unsigned w1_read() {
	io_port_b->PIO_OER = DS18B20_MASK; //enable output
	io_port_b->PIO_CODR = DS18B20_MASK; //level low
	lib_delay_us(4);
	io_port_b->PIO_ODR = DS18B20_MASK; //disable output
	lib_delay_us(10);
	unsigned bit_value = io_port_b->PIO_PDSR & DS18B20_MASK;
	lib_delay_us(45);
	return bit_value;
}

static unsigned w1_read_byte() {
	unsigned result = 0;
	if(w1_read()) result |= 0x01;
	if(w1_read()) result |= 0x02;
	if(w1_read()) result |= 0x04;
	if(w1_read()) result |= 0x08;
	if(w1_read()) result |= 0x10;
	if(w1_read()) result |= 0x20;
	if(w1_read()) result |= 0x40;
	if(w1_read()) result |= 0x80;
	return result;
}

static void w1_write_byte(unsigned data) {
	data & 0x01 ? w1_write_1() : w1_write_0();
	data & 0x02 ? w1_write_1() : w1_write_0();
	data & 0x04 ? w1_write_1() : w1_write_0();
	data & 0x08 ? w1_write_1() : w1_write_0();
	data & 0x10 ? w1_write_1() : w1_write_0();
	data & 0x20 ? w1_write_1() : w1_write_0();
	data & 0x40 ? w1_write_1() : w1_write_0();
	data & 0x80 ? w1_write_1() : w1_write_0();
}

static unsigned ds18b20_reset() {
	io_port_b->PIO_OER = DS18B20_MASK; //enable output
	io_port_b->PIO_CODR = DS18B20_MASK; //level low
	lib_delay_us(500);
	io_port_b->PIO_ODR = DS18B20_MASK; //disable output
	lib_delay_us(60);
	int data_bit = io_port_b->PIO_PDSR;
	usleep(1000);
	return data_bit & DS18B20_MASK;
}

//=============================================================================
static int ds18b20() {
	if(fork()) return 1;
	fprintf(stderr, "ds18b20===>>\n");
	name = "ds18b20";
	type = "temp";
	io_port_b->PIO_PER = DS18B20_MASK;;
	io_port_b->PIO_PPUER = DS18B20_MASK; //enable pull up
	if(ds18b20_reset()) {
		fprintf(stderr, "DS18B20 not detected.\n");
		return 0;
	}
	float old = -99999999;
	//start conversion
	ds18b20_reset();
	w1_write_byte(DS18B20_SKIP_ROM);
	w1_write_byte(DS18B20_CONVERT_T);
	while(g_run && !usleep(750000)) {
		//reading
		ds18b20_reset();
		w1_write_byte(DS18B20_SKIP_ROM);
		w1_write_byte(DS18B20_READ_SCRATCHPAD);
		float temp = w1_read_byte() | (w1_read_byte() << 8);
		//skip reading the rest of scratchpad bytes
		io_port_b->PIO_OER = DS18B20_MASK; //enable output
		io_port_b->PIO_CODR = DS18B20_MASK; //level low
		temp /= 16;
		lib_check_print_float(temp, &old);
		usleep(750000);
		io_port_b->PIO_ODR = DS18B20_MASK; //disable output

		//start conversion
		ds18b20_reset();
		w1_write_byte(DS18B20_SKIP_ROM);
		w1_write_byte(DS18B20_CONVERT_T);
	}
	fprintf(stderr, "ds18b20===<<\n");
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

static int lm75() {
	if(fork()) return 1;
	fprintf(stderr, "lm75   ===>>\n");
	name = "lm75";
	type = "temp";
	int fd = open("/dev/i2c-0", O_RDWR);
	if(fd < 0) {
		perror("/dev/i2c-0");
	}
	if(ioctl(fd, I2C_SLAVE, LM75_ADDR) < 0) {
		perror("Failed to acquire slave address\n");
		close(fd);
		return 0;
	}
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
		lib_check_print_float(temp, &old);
	}
	close(fd);
	fprintf(stderr, "lm75   ===<<\n");
	return 0;
}

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

static int adc() {
	if(fork()) return 1;
	fprintf(stderr, "adc    ===>>\n");
	name = "adc";
	type = "light";
	if(adc_enable()) {
		return 0;
	}
	float old = -1;
	while(g_run && !usleep(500000)) {
		float val = 0;
		if(adc_read(&val)) {
			break;
		}
		lib_check_print_float(val, &old);
	}
	fprintf(stderr, "adc    ===<<\n");
	return 0;
}

static int send_clock(int period) {
	if(fork()) return 1;
	if(period < 1) period = 1;
	fprintf(stderr, "clock  ===>>\n");
	printf("begin transaction;\n"
			".timeout 1000\n"
			"create table if not exists data(time timestamp, name text, val text, key text, type text);\n");
	fflush(stdout);
	while(g_run && !sleep(period)) {
		io_port_b->PIO_SODR = LED1_MASK;
		printf("commit;\n");
		fflush(stdout);
		printf("begin transaction;\n");
		fflush(stdout);
		io_port_b->PIO_CODR = LED1_MASK;
	}
	usleep(200000);
	printf("commit;\n");
	fflush(stdout);
	fprintf(stderr, "clock  ===<<\n");
	return 0;
}

static int dev_main(int argc, char* argv[]) {
	signal(SIGINT, ctrl_c);
	key = argv[1];
	tbl = argv[2];
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
}

static int read_main(int argc, char* argv[]) {
	signal(SIGINT, SIG_IGN);
	chdir("outbox");
	char buff[0x1000], fname[0x32];
	FILE* pf = 0;
	while(fgets(buff, sizeof(buff), stdin)) {
		if(!strcmp("begin transaction;\n", buff)) {
			printf("=======>>\n");
			pf = fopen(".data", "w");
			fprintf(pf, "%s", buff);
			continue;
		}
		if(!strcmp("commit;\n", buff)) {
			printf("<<=======\n");
			fprintf(pf, "%s", buff);
			fclose(pf);
			sprintf(fname, "%08lx", time(0));
			rename(".data", fname);
			continue;
		}
		fprintf(pf, "%s", buff);
	}
	return ERR_OK;
}

int main(int argc, char* argv[]) {
	if(argc < 2) return ERR_ARGC;
	argc --;
	argv ++;
	if(!strcmp("dev", *argv)) return dev_main(argc, argv);
	if(!strcmp("read", *argv)) return read_main(argc, argv);
	return ERR_CMD;
}

