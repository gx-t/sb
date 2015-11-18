#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <string.h>

enum {
	ERR_OK = 0,
	ERR_ARGC,
	ERR_CMD,
	ERR_DEV,
};


static volatile unsigned char* reg_base = 0;

#define REG_BASE					0x10000000
#define MAP_SIZE					4096UL

#define PIOB_PDSR					(*(volatile unsigned*)&reg_base[0x620])
#define PIOB_OER					(*(volatile unsigned*)&reg_base[0x624])
#define PIOB_PPR					(*(volatile unsigned*)&reg_base[0x628])
#define PIOB_SODR					(*(volatile unsigned*)&reg_base[0x62C])
#define PIOB_CODR					(*(volatile unsigned*)&reg_base[0x630])
#define PIOB_TODR					(*(volatile unsigned*)&reg_base[0x634])

#define PIOA_PDSR					(*(volatile unsigned*)&reg_base[0x670])
#define PIOA_OER					(*(volatile unsigned*)&reg_base[0x674])
#define PIOA_PPR					(*(volatile unsigned*)&reg_base[0x678])
#define PIOA_SODR					(*(volatile unsigned*)&reg_base[0x67C])
#define PIOA_CODR					(*(volatile unsigned*)&reg_base[0x680])
#define PIOA_TODR					(*(volatile unsigned*)&reg_base[0x684])

static int g_run = 1;

//=============================================================================
static void ctrl_c(int sig) {
	g_run = 0;
}

static void lib_open_reg_base() {
	reg_base = 0;
	static const char* dev_name = "/dev/mem";
	int fd = open(dev_name, O_RDWR | O_SYNC);
	if(fd == -1) {
		perror(dev_name);
		return;
	}
	reg_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, REG_BASE);
	close(fd);
	if(MAP_FAILED == reg_base) {
		perror("mmap");
		reg_base = 0;
	}
}

static inline void lib_close_reg_base() {
	munmap((void*)reg_base, MAP_SIZE);
}

// calibrated with good accuracy for VoCore1.0 board
static void lib_delay_us(unsigned us) {
	volatile unsigned i = 60 * us - 7;
	while(i --);
}

//=============================================================================
//DEV FUNCTIONS
static int dev_sht1x(int argc, char* argv[]) {
#if 0
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
#endif
	lib_open_reg_base();
	if(!reg_base) return ERR_DEV;
#if 0
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
#endif
	lib_close_reg_base();
	return ERR_OK;
}

static int dev_main(int argc, char* argv[]) {
	//... <dev type> ...
	if(argc < 2) {
		fprintf(stderr, "Usage: ... %s sht1x\n", argv[0]);
		return ERR_ARGC;
	}
	argc --;
	argv ++;
	if(!strcmp("sht1x", *argv)) return dev_sht1x(argc, argv);
	fprintf(stderr, "Unknown subcommand: %s\n", *argv);
	return ERR_CMD;
}

//=============================================================================
//FILTER FUNCTIONS

static int filter_main(int argc, char* argv[]) {
	//... filter <type> ...
	signal(SIGINT, SIG_IGN);
#if 0
	if(argc < 2) {
		fprintf(stderr, "Usage: ... %s sql ...\n", argv[0]);
		return ERR_ARGC;
	}
#endif
	argc --;
	argv ++;
#if 0
	if(!strcmp("sql", *argv)) return filter_sql(argc, argv);
#endif
	fprintf(stderr, "Unknown subcommand: %s\n", *argv);
	return ERR_CMD;
}

int main(int argc, char* argv[]) {
	if(argc < 2) {
		fprintf(stderr, "Usage: %s dev|filter ...\n", argv[0]);
		return ERR_ARGC;
	}
	argc --;
	argv ++;
	signal(SIGINT, ctrl_c);
	if(!strcmp("dev", *argv)) return dev_main(argc, argv);
	if(!strcmp("filter", *argv)) return filter_main(argc, argv);
	fprintf(stderr, "Unknown subcommand: %s\n", *argv);
	return ERR_CMD;

#if 0
	int fd = open("/dev/mem", O_RDWR);
	io_base = (volatile unsigned char*)mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x10000000);
	close(fd);
//	IOREG(0x624) |= (1 << 0); //GPIO26 - output
//	IOREG(0x628) = 0; //All GPIOs polarity - normal
//	while((IOREG(0x62c) |= (1 << 0), !usleep(100000)) && (IOREG(0x630) |= (1 << 0), !usleep(100000)));
//	IOREG(0x630) |= (1 << 0); //GPIO26 - low
	PIOB_OER |= (1 << 0); //GPIO22 - output
	PIOB_PPR = 0; //All GPIOs polarity - normal
//	while((IOREG(0x67c) |= (1 << 4), !usleep(100000)) && (IOREG(0x680) |= (1 << 4), !usleep(100000)));
	PIOB_CODR |= (1 << 0); //GPIO26 - low
	while(g_run) {
		int i = 1000;
		while(g_run && i --) {
			usleep(i);
			PIOB_TODR |= (1 << 0); //toggle GPIO
			usleep(1000 - i);
			PIOB_TODR |= (1 << 0); //toggle GPIO
		}
		i = 1000;
		while(g_run && i --) {
			usleep(1000 - i);
			PIOB_TODR |= (1 << 0); //toggle GPIO
			usleep(i);
			PIOB_TODR |= (1 << 0); //toggle GPIO
		}
	}
	PIOB_CODR |= (1 << 0); //GPIO26 - low
	munmap((void*)io_base, 4096);
	fprintf(stderr, "===END===\n");
	return 0;
#endif
}


