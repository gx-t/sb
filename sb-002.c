#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <signal.h>
#include <sys/file.h>
#include <math.h>

enum {
	ERR_OK = 0,
	ERR_I2COPEN,
	ERR_IOCTL,
	ERR_WRITE_REG,
	ERR_CALIB,
	ERR_READING,
};

#define BMP180_ADDRESS		0x77
#define BMP180_CALIBREG		0xAA
#define BMP180_CTRLREG		0xF4
#define BMP180_CTRL_TEMP	0x2E
#define BMP180_DATAREG		0xF6

static int i2c_fd = -1;
static int g_run = 0;

static int i2c_read_reg(uint8_t reg, uint8_t* buff, uint8_t count) {
	int res = -1;
	if(1 == write(i2c_fd, &reg, 1)) {
		res = read(i2c_fd, buff, count);
	}
	return res;
}

static int bmp180_main() {
	uint8_t buff[32];
	if(0 > ioctl(i2c_fd, I2C_SLAVE, BMP180_ADDRESS)) {
		perror("BMP180 Set I2C Address");
		return ERR_IOCTL;
	}

//read calibration data
	int cnt = i2c_read_reg(0xAA, buff, 22);
	if(cnt != 22) {
		fprintf(stderr, "BMP180: Invalid Calibration Data\n");
		return ERR_CALIB;
	}
	short ac1, ac2, ac3, b1, b2, mc, md;
	unsigned short ac4, ac5, ac6;
	ac1 = buff[0]  << 8 | buff[1];
	ac2 = buff[2]  << 8 | buff[3];
	ac3 = buff[4]  << 8 | buff[5];
	ac4 = buff[6]  << 8 | buff[7];
	ac5 = buff[8]  << 8 | buff[9];
	ac6 = buff[10] << 8 | buff[11];
	b1  = buff[12] << 8 | buff[13];
	b2  = buff[14] << 8 | buff[15];
	mc  = buff[18] << 8 | buff[19];
	md  = buff[20] << 8 | buff[21];

	float pf = 0;
	float tf = 0;
	while(g_run && 0 == sleep(1)) {
		//init temp. measurement
		buff[0] = 0xF4;
		buff[1] = 0x2e;
		if(2 != write(i2c_fd, buff, 2)) {
			fprintf(stderr, "BMP180: Cannot Initiate Temperature Measurement\n");
			return ERR_WRITE_REG;
		}

		//wait for ADC to complete measurement
		if(0 != usleep(5000)) {
			//interrupted by ctrl+C
			return ERR_OK;
		}

		//read uncompensated temperarure
		cnt = i2c_read_reg(0xF6, buff, 2);
		if(cnt != 2) {
			fprintf(stderr, "BMP180: Invalid Reading\n");
			return ERR_READING;
		}
		long ut = buff[0] << 8 | buff[1];

		//init pres. measurement
		buff[0] = 0xF4;
		buff[1] = 0xF4;
		if(2 != write(i2c_fd, buff, 2)) {
			fprintf(stderr, "BMP180: Cannot Initiate Pressure Measurement\n");
			return ERR_WRITE_REG;
		}

		//wait for ADC to complete measurement
		if(0 != usleep(26000)) {
			//interrupted by ctrl+C
			return ERR_OK;
		}

		//read uncompensated pressure
		cnt = i2c_read_reg(0xF6, buff, 3);
		if(cnt != 3) {
			fprintf(stderr, "BMP180: Invalid Reading\n");
			return ERR_READING;
		}

		long up = (buff[0] << 16 | buff[1] << 8 | buff[2]) >> 5;

		//calculate true temperature
		long x1, x2, x3, b3, b5, b6;
		unsigned long b4, b7;
		x1 = (ut - 	ac6) * ac5 >> 15;
		x2 = (mc << 11) / (x1 + md);
		b5 = x1 + x2;
		float t = (b5 + 8) >> 4;
		t /= 10;

		//calculate true pressure

		b6 = b5 - 4000;
		x1 = (b2 * (b6 * b6 >> 12)) >> 11;
		x2 = ac2 * b6 >> 11;
		x3 = x1 + x2;

		b3 = (((ac1 * 4 + x3) << 3) + 2) >> 2;
		x1 = ac3 * b6 >> 13;
		x2 = (b1 * (b6 * b6 >> 12)) >> 16;
		x3 = (x1 + x2 + 2) >> 2;
		b4 = (ac4 * (x3 + 32768)) >> 15;
		b7 = (up - b3) * (50000 >> 3);

		long p;
		if(b7 < 0x80000000)
			p = (b7 << 1) / b4;
		else
			p = (b7 / b4) << 1;

		x1 = (p >> 8) * (p >> 8);
		x1 = (x1 * 3038) >> 16;
		x2 = (-7357 * p) >> 16;
		p += ((x1 + x2 + 3791) >> 4);

		tf += t;;
		tf /= 2;

		pf += p;
		pf /= 2;

		float hg = pf * 0.00750062;
		float hf = 44330 * (1 - pow(pf / 101325, 1.0 / 5.255));

		printf("T=%.1f °C, P=%.2f hPa (%.2f mm, %.1f m)\n", tf, pf / 100, hg, hf);
	}

	return ERR_OK;
}

static void ctrl_c(int sig) {
	g_run = 0;
	signal(SIGINT, ctrl_c);
}

int main() {
	static const char* i2c_name = "/dev/i2c-0";
	i2c_fd = open(i2c_name, O_RDWR);
	if(i2c_fd < 0) {
		perror(i2c_name);
		return ERR_I2COPEN;
	}
	g_run = 1;
	signal(SIGINT, ctrl_c);
	int res = bmp180_main();
	close(i2c_fd);
	return res;
}

