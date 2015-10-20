#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>


static volatile unsigned char* reg_base = 0;

#define IOREG(offset)		(*(volatile unsigned*)&reg_base[offset])
//reg_base + 0x24; //dir
//reg_base + 0x28; //polarity
//reg_base + 0x2C; //set data
//reg_base + 0x30; //clear data
//reg_base + 0x34; //toggle
//
//
//reg_base + 0x70; //data
//reg_base + 0x74; //dir
//reg_base + 0x78; //polarity
//reg_base + 0x7C; //set data
//reg_base + 0x80; //clear data
//reg_base + 0x84; //toggle
//
static int g_run = 1;
static void ctrl_c(int sig) {
	g_run = 0;
}

int main(int argc, char* argv[]) {
	signal(SIGINT, ctrl_c);
	fprintf(stderr, "Test\n");
	int fd = open("/dev/mem", O_RDWR);
	reg_base = (volatile unsigned char*)mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x10000000);
	close(fd);
//	IOREG(0x624) |= (1 << 0); //GPIO26 - output
//	IOREG(0x628) = 0; //All GPIOs polarity - normal
//	while((IOREG(0x62c) |= (1 << 0), !usleep(100000)) && (IOREG(0x630) |= (1 << 0), !usleep(100000)));
//	IOREG(0x630) |= (1 << 0); //GPIO26 - low
	IOREG(0x624) |= (1 << 0); //GPIO22 - output
	IOREG(0x628) = 0; //All GPIOs polarity - normal
//	while((IOREG(0x67c) |= (1 << 4), !usleep(100000)) && (IOREG(0x680) |= (1 << 4), !usleep(100000)));
	IOREG(0x630) |= (1 << 0); //GPIO26 - low
	while(g_run) {
		int i = 1000;
		while(g_run && i --) {
			usleep(i);
			IOREG(0x634) |= (1 << 0); //toggle GPIO
			usleep(1000 - i);
			IOREG(0x634) |= (1 << 0); //toggle GPIO
		}
		i = 1000;
		while(g_run && i --) {
			usleep(1000 - i);
			IOREG(0x634) |= (1 << 0); //toggle GPIO
			usleep(i);
			IOREG(0x634) |= (1 << 0); //toggle GPIO
		}
	}
	IOREG(0x630) |= (1 << 0); //GPIO26 - low
	munmap((void*)reg_base, 4096);
	fprintf(stderr, "===END===\n");
	return 0;
}

