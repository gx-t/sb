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

enum {
	ERR_OK = 0,
	ERR_I2COPEN,
	ERR_IOCTL,
	ERR_CALIB,
};

#define BMP180_ADDRESS		0x77
#define BMP180_CALIBREG		0xAA
#define BMP180_CTRLREG		0xF4
#define BMP180_CTRL_TEMP	0x2E
#define BMP180_DATAREG		0xF6

static int i2c_fd = -1;

static int bmp180_read_calib() {
	uint8_t buff[32];
	uint8_t reg = 0xAA;
	uint8_t cnt = 0;
	if(1 == write(i2c_fd, &reg, 1)) {
		cnt = read(i2c_fd, buff, 22);
	}
	if(cnt != 22) {
		fprintf(stderr, "BMP180: Invalid Calibration Data\n");
		return -1;
	}
	int i;
	printf("==============================\n");
	for (i = 0; i < cnt; i++) {
		printf("\t0x%02X\n", buff[i]);
	}
	return ERR_OK;
}

static int bmp180_temp_start() {
	uint8_t data[2] = {
		0xF4, 0x2E
	};
	return write(i2c_fd, data, 2);
}

static int bmp180_read() {
	uint8_t buff[32];
	uint8_t reg = 0xF6;
	uint8_t cnt = 0;
	if(1 == write(i2c_fd, &reg, 1)) {
		cnt = read(i2c_fd, buff, 3);
	}
	if(cnt < 0) {
		fprintf(stderr, "BMP180: Invalid Reading\n");
		return -1;
	}
	int i;
	printf("++++++++++++++++++++++++ %d\n", cnt);
	for (i = 0; i < cnt; i++) {
		printf("\t0x%02X\n", buff[i]);
	}
	return ERR_OK;
}

static int bmp180_pres_start() {
	uint8_t data[2] = {
		0xF4, 0xF4
	};
	return write(i2c_fd, data, 2);
}

static int bmp180_main() {
//	union i2c_smbus_data data;
//	struct i2c_smbus_ioctl_data args;
//	
//	args.read_write = I2C_SMBUS_READ;
//	args.data		= &data;
//	args.command	= BMP180_CALIBREG;
//	args.size		= I2C_SMBUS_I2C_BLOCK_DATA;

	if(0 > ioctl(i2c_fd, I2C_SLAVE, BMP180_ADDRESS)) {
		perror("BMP180 Set I2C Address");
		return ERR_IOCTL;
	}
	bmp180_read_calib();
	bmp180_temp_start();
	usleep(100000);
	bmp180_read();
	bmp180_pres_start();
	usleep(100000);
	bmp180_read();
//	if(0 > ioctl(i2c_fd, I2C_SMBUS, &args)) {
//		perror("BMP180 Read Calibration Data");
//		return ERR_IOCTL;
//	}
//
//	if(data.block[0] != 22) {
//		fprintf(stderr, "BMP180 Invalid Calibration Info\n");
//		return ERR_CALIB;
//	}
//
//	int i;
//	printf("++++%d\n", data.block[0]);
//	for (i = 1; i <= data.block[0]; i++) {
//		printf("\t0x%02X\n", data.block[i]);
//	}
//
//	args.read_write = I2C_SMBUS_WRITE;
//	args.command	= BMP180_CTRLREG;
//	args.size		= I2C_SMBUS_BYTE_DATA;
//	data.byte		= BMP180_CTRL_TEMP;
//	if(0 > ioctl(i2c_fd, I2C_SMBUS, &args)) {
//		perror("BMP180 Start Temp. Measurement");
//		return ERR_IOCTL;
//	}
//	sleep(1);
#if 0
	while(i2c_fd != -1) {
		do {
			args.read_write = I2C_SMBUS_WRITE;
			args.command	= BMP180_CTRLREG;
			args.size		= I2C_SMBUS_BYTE_DATA;
			data.byte		= BMP180_CTRL_TEMP;
			if(0 > ioctl(i2c_fd, I2C_SMBUS, &args)) {
				perror("BMP180 Start Temp. Measurement");
				res = ERR_IOCTL;
				break;
			}
		} while(0);
		usleep(4500);

		do {
			args.read_write = I2C_SMBUS_READ;
			args.command	= BMP180_DATAREG;
			args.size		= I2C_SMBUS_I2C_BLOCK_DATA;
			if(0 > ioctl(i2c_fd, I2C_SMBUS, &args)) {
				perror("BMP180 Read Temperature");
				res = ERR_IOCTL;
				break;
			}
		} while(0);

		sleep(1);
	}
#endif
	return ERR_OK;
}

static int i2c_init() {
	static const char* i2c_name = "/dev/i2c-0";
	i2c_fd = open(i2c_name, O_RDWR);
	if(i2c_fd < 0) {
		perror(i2c_name);
		return ERR_I2COPEN;
	}
	return ERR_OK;
}

static void i2c_release() {
	if(i2c_fd != -1) {
		close(i2c_fd);
		i2c_fd = -1;
	}
}

static void ctrl_c(int sig) {
	i2c_release();
}

int main() {
	int res = i2c_init();
	if(res) {
		return res;
	}
	signal(SIGINT, ctrl_c);
	res = bmp180_main();
	i2c_release();
	return res;
}

