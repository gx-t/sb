#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>


static volatile unsigned char* io_base = 0;


#define PIOB_PDSR					(*(volatile unsigned*)&io_base[0x620])
#define PIOB_OER					(*(volatile unsigned*)&io_base[0x624])
#define PIOB_PPR					(*(volatile unsigned*)&io_base[0x628])
#define PIOB_SODR					(*(volatile unsigned*)&io_base[0x62C])
#define PIOB_CODR					(*(volatile unsigned*)&io_base[0x630])
#define PIOB_TODR					(*(volatile unsigned*)&io_base[0x634])

#define PIOA_PDSR					(*(volatile unsigned*)&io_base[0x670])
#define PIOA_OER					(*(volatile unsigned*)&io_base[0x674])
#define PIOA_PPR					(*(volatile unsigned*)&io_base[0x678])
#define PIOA_SODR					(*(volatile unsigned*)&io_base[0x67C])
#define PIOA_CODR					(*(volatile unsigned*)&io_base[0x680])
#define PIOA_TODR					(*(volatile unsigned*)&io_base[0x684])

static int g_run = 1;
static void ctrl_c(int sig) {
	g_run = 0;
}

int main(int argc, char* argv[]) {
	signal(SIGINT, ctrl_c);
	fprintf(stderr, "Test\n");
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
}

