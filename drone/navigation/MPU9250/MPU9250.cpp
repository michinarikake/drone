#include "MPU9250.hpp"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#define DEV_PATH "/dev/i2c-1"

//0x68 = 110_1000 <-> 0xD0 = 1101_000(R/W=0)
#define I2C_ADDR 0x68

//0x0c = 000_1100 <-> 0x18 = 001_1000(R/W=0)
#define I2C_ADDR_MAG 0x0c

#define SWITCH_TO_INERTIA() ioctl(fd, I2C_SLAVE, I2C_ADDR)
#define SWITCH_TO_MAG() ioctl(fd, I2C_SLAVE, I2C_ADDR_MAG)

MPU9250::MPU9250(){
	Reg_Conf = Reg_AccConf = Reg_AccConf2 = Reg_GyroConf = 0;

	fd = open(DEV_PATH, O_RDWR);
	if( fd < 0){
		fprintf(stderr, "I2C Open Error\n");
		return ;
	}

	//set I2C Addr
	if( ioctl(fd, I2C_SLAVE, I2C_ADDR) < 0){
		fprintf(stderr, "MPU9250: I2c Address Set Failed\n");
		return ;
	}

	if( CheckConnection() ){
		return;
	}else{
		fprintf(stderr, "MPU9250 Connection failed\n");
		return;
	}

}

MPU9250::~MPU9250(){
	close(fd);
}

int MPU9250::CheckConnection(){
	int ret;
	unsigned char dat;

	dat = (unsigned char)i2c_smbus_read_byte_data(fd, MPU9250_WHO_AM_I);

	if(dat == 0x71)	//found
		return 1;
	else			//Not found
		return 0;		
}

int MPU9250::Init(){
	i2c_smbus_write_byte_data(fd, MPU9250_PWR_MGMT_1, 0x00);
	i2c_smbus_write_byte_data(fd, MPU9250_INT_PIN_CFG, 0x02);	//I2C bypass (External I2c is connected to Magnetometer I2C interface)


//Start magnetometer
	SWITCH_TO_MAG();	//I2C addr -> Magnetometer

	i2c_smbus_write_byte_data(fd, MPU9250_MAG_CNTL, 0x12);	//16bit, Continuout mode

	SWITCH_TO_INERTIA();	//I2C addr -> Acc/Gyro/Temp

	return 0;
}


int MPU9250::ReadData(struct timeval* tim, short acc[], short rot[], short *temp){
	int ret;
	ret = ReadData(acc, rot, temp);
	gettimeofday(tim, NULL);
	return ret;
}

//Return data: short acc[3]	
int MPU9250::ReadData(short acc[], short rot[], short *temp){
	int ret;
	unsigned char buf[14];

	ret = i2c_smbus_read_i2c_block_data(fd,
			MPU9250_ACCEL_XOUT_H, 14, buf);

	*((unsigned char*)(acc) + 1) = buf[0];	//acc[0]
	*((unsigned char*)(acc) + 0) = buf[1];
	*((unsigned char*)(acc) + 3) = buf[2];	//acc[1]
	*((unsigned char*)(acc) + 2) = buf[3];
	*((unsigned char*)(acc) + 5) = buf[4];	//acc[2]
	*((unsigned char*)(acc) + 4) = buf[5];

	*((unsigned char*)(temp) + 1) = buf[6];	//temp
	*((unsigned char*)(temp) + 0) = buf[7];
	
	*((unsigned char*)(rot) + 1) = buf[8];	//rot[0]
	*((unsigned char*)(rot) + 0) = buf[9];
	*((unsigned char*)(rot) + 3) = buf[10];	//rot[1]
	*((unsigned char*)(rot) + 2) = buf[11];
	*((unsigned char*)(rot) + 5) = buf[12];	//rot[2]
	*((unsigned char*)(rot) + 4) = buf[13];

	return ret;
}	


//This function is valid only for "single measurement mode" or "external trigger mode"
//e.g. should not used in "continuout measurement mode"
int MPU9250::isDataReady_Mag(){
	int ret;
	unsigned char dat;

	SWITCH_TO_MAG();	//I2C addr -> Magnetometer

	dat = (unsigned char)i2c_smbus_read_byte_data(fd, MPU9250_MAG_ST1);

	SWITCH_TO_INERTIA();	//I2C addr -> Acc/Gyro/Temp

	if( dat & (0x01) ){
		return 1;
	}else{
		return 0;
	}
}


int MPU9250::CheckConnection_Mag(){
	unsigned char dat;

	SWITCH_TO_MAG();	//I2C addr -> Magnetometer

	dat = (unsigned char)i2c_smbus_read_byte_data(fd, MPU9250_MAG_WIA);

	SWITCH_TO_INERTIA();	//I2C addr -> Acc/Gyro/Temp

	if(dat == 0x48)	//found
		return 1;
	else			//Not found
		return 0;		
}

int MPU9250::ReadData_Mag(short mag[] ){
	unsigned char buf[7];
	unsigned char st2;

	SWITCH_TO_MAG();	//I2C addr -> Magnetometer

	i2c_smbus_read_i2c_block_data(fd, MPU9250_MAG_HXL, 7, buf);	//Read magnetometer

	SWITCH_TO_INERTIA();	//I2C addr -> Acc/Gyro/Temp


	*((unsigned char*)(mag) + 1) = buf[0];	//mag[0]
	*((unsigned char*)(mag) + 0) = buf[1];
	*((unsigned char*)(mag) + 3) = buf[2];	//mag[1]
	*((unsigned char*)(mag) + 2) = buf[3];
	*((unsigned char*)(mag) + 5) = buf[4];	//mag[2]
	*((unsigned char*)(mag) + 4) = buf[5];
	
	st2 = buf[6];

	if( st2 & (1<<3) ){	//HOFL
		fprintf(stderr, "***WARNING*** :Manetometer overflowed (HOFL=1)\n");
	}
	return st2;
}

int MPU9250::SetAccFullScale(AccFS_t fs){
	unsigned char b;
	switch(fs){
		case Acc_FS_2G:
			b = 0x00; 	
			break;
		case Acc_FS_4G:
			b = 0x01 << 3;
			break;
		case Acc_FS_8G:
			b = 0x02 << 3;
			break;
		case Acc_FS_16G:
			b = 0x03 << 3;
			break;
		default:
			b = 0;
		break;
	}

	Reg_AccConf &= ~(0x03 << 3);
	Reg_AccConf |= b;
	
	int ret = i2c_smbus_write_byte_data(fd, MPU9250_ACCEL_CONFIG, Reg_AccConf);
	return ret;
}

int MPU9250::SetGyroFullScale(GyroFS_t fs){
	unsigned char b;
	switch(fs){
		case Gyro_FS_250dps:
			b = 0x00; 	
			break;
		case Gyro_FS_500dps:
			b = 0x01 << 3;
			break;
		case Gyro_FS_1000dps:
			b = 0x02 << 3;
			break;
		case Gyro_FS_2000dps:
			b = 0x03 << 3;
			break;
		default:
			b = 0;
		break;
	}

	Reg_GyroConf &= ~(0x03 << 3);
	Reg_GyroConf |= b;
	
	int ret = i2c_smbus_write_byte_data(fd, MPU9250_GYRO_CONFIG, Reg_GyroConf);
	return ret;
}

int MPU9250::SetAccRate( AccRate_t rate ){
	unsigned char b;
	b = 0;
	switch(rate){
		case Acc_BW1130Hz_SR4k:
			b |= (1<<3);
			break;
		case Acc_BW460Hz_SR1k:
			b = 0;
			break;
		case Acc_BW184Hz_SR1k:
			b |= 0x01;
			break;
		case Acc_BW92Hz_SR1k:
			b |= 0x02;
			break;
		case Acc_BW41Hz_SR1k:
			b |= 0x03;
			break;
		case Acc_BW20Hz_SR1k:
			b |= 0x04;
			break;
		case Acc_BW10Hz_SR1k:
			b |= 0x05;
			break;
		case Acc_BW5Hz_SR1k:
			b |= 0x06;
			break;
		default:
			break;
	}
	Reg_AccConf2 = b;

	int ret = i2c_smbus_write_byte_data(fd, MPU9250_ACCEL_CONFIG_2, Reg_AccConf2);
	return ret;
}


int MPU9250::SetGyroRate( GyroRate_t rate ){
	unsigned char b, c;
	b = c = 0;
	switch(rate){
		case Gyro_BW8800Hz_SR32k:
			b =  0x01; //FCHOISE = 0bx0, FCHOISE_b = 0bx1 = 1 or 3
			break;
		case Gyro_BW3600Hz_SR32k:
			b =  0x02;	//FCHOISE = 0b01, FCHOISE_b = 0b10 = 2
			break;
		case Gyro_BW250Hz_SR8k:
			b =  0x00;	//FCHOISE = 0b11, FCHOISE_b = 0b00
			c =  0x00;
			break;
		case Gyro_BW184Hz_SR1k:
			c =  0x01;
			break;
		case Gyro_BW92Hz_SR1k:
			c =  0x02;
			break;
		case Gyro_BW41Hz_SR1k:
			c =  0x03;
			break;
		case Gyro_BW20Hz_SR1k:
			c =  0x04;
			break;
		case Gyro_BW10Hz_SR1k:
			c =  0x05;
			break;
		case Gyro_BW5Hz_SR1k:
			c =  0x06;
			break;
		case Gyro_BW3600Hz_SR8k:
			c =  0x07;
			break;
		default:
			b = 0;
			c = 0;
			break;
	}

	Reg_Conf &= 0x07;
	Reg_Conf |= c;
	Reg_GyroConf &= 0x03;
	Reg_GyroConf |= b;

	int ret = i2c_smbus_write_byte_data(fd, MPU9250_GYRO_CONFIG, Reg_GyroConf);
	ret = i2c_smbus_write_byte_data(fd, MPU9250_CONFIG, Reg_Conf);
	return ret;
}
