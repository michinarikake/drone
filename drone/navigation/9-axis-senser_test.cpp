#include <stdio.h>
#include <string.h>
#include "MPU9250/MPU9250.hpp"
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

double delta_t(struct timeval tim, struct timeval tim0){
	long dt, dt_usec;
	dt_usec = tim.tv_usec - tim0.tv_usec;
	dt = tim.tv_sec - tim0.tv_sec;
	return ( (double)dt + (double)(dt_usec)*1e-6 );
}

int main(){
	int i, ret;
	char str[256];
	short acc[3];
	short rot[3];
	short mag[3];
	short temp;
	int Acc[3];

	MPU9250 sensor;

	struct timeval tim, tim0;
	Acc[0] = Acc[1] = Acc[2] = 0;

//Init MPU9250
//Check connection (Acc/Gyro/Temp)
	ret = sensor.CheckConnection();
	if( ret ){
		printf("MPU9250 detected\n");
	}else{
		printf("MPU9250 not found\n");
		return -1;
	}

	sensor.Init();	//Enable Magnetometer

//Check connection (Magnetometer)
	ret = sensor.CheckConnection();
	ret = sensor.CheckConnection_Mag();
	if( ret ){
		printf("MPU9250(Magnetometer) detected\n");
	}else{
		printf("MPU9250(Magnetometer) not found\n");
		return -1;
	}

//Setting (Acc/Gyro .. FullScale & Sample Rate)
	sensor.SetAccFullScale( Acc_FS_4G );
	sensor.SetGyroFullScale( Gyro_FS_250dps );
	//sensor.SetAccRate( Acc_BW5Hz_SR1k );
	//sensor.SetGyroRate( Gyro_BW5Hz_SR1k );
	sensor.SetAccRate( Acc_BW460Hz_SR1k );
	sensor.SetGyroRate( Gyro_BW250Hz_SR8k );


	ret = sensor.ReadData(&tim0, acc, rot, &temp);

	for(i=0; i<100 ;i++){
		usleep(10*1000);

		ret = sensor.ReadData(&tim, acc, rot, &temp);
		ret = sensor.ReadData_Mag( mag );

		printf("%lf, acc=(%d,%d,%d), rot=(%d,%d,%d), temp=%d, mag=(%d,%d,%d)\n",
			delta_t(tim, tim0),
			acc[0], acc[1], acc[2],
			rot[0], rot[1], rot[2],
			temp,
			mag[0], mag[1], mag[2]
		);

			//sensor.ReadData_Mag( mag );
			//printf("mag=(%d,%d,%d)\n", mag[0], mag[1], mag[2] );


	}
}