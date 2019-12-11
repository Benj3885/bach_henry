#include "motion.h"
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <math.h>


imu::imu(){
    int_data_mtx = new std::mutex;

    char *filename = (char*)"/dev/i2c-1";
	if ((fd = open(filename, O_RDWR)) < 0)
	{
		printf("Failed to open the i2c bus");
		exit(0);
	}
	
	int addr = 0x68; //https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
	if (ioctl(fd, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		exit(0);
	}
    setup_imu();
    calibrate();

    std::thread imuthread(&imu::main, this);
    imuthread.detach();
}

void imu::main(){
    int interval = 1000 / IMU_RATE;
    period = interval / 1000.f;
    auto time_interval = std::chrono::milliseconds(interval);
    auto time = std::chrono::steady_clock::now();
    
    while(1){
        time += time_interval;
        
        read_imu();
        save_imu_data();
        integrate_pos();
        std::this_thread::sleep_until(time);
    }
}

void imu::calibrate(){
    int interval = 1000 / IMU_RATE;
    int samples = 500;
    period = interval / 1000.f;
    auto time_interval = std::chrono::milliseconds(interval);
    auto time = std::chrono::steady_clock::now();

    short int temp[7];

    for(int i = 0; i < samples; i++){
        time += time_interval;
        read_imu();

        for(int i = 0; i < 7; i++)
            temp[i] = (buffer[i*2] << 8) + buffer[i*2+1];
        
        bias[0] += temp[0];
        bias[1] += temp[1];
        bias[2] += temp[2];
        bias[4] += temp[4];
        bias[5] += temp[5];
        bias[6] += temp[6];

        std::this_thread::sleep_until(time);
    }

    for(int i = 0; i < 7; i++)
        bias[i] = bias[i] / samples;

    time = std::chrono::steady_clock::now();
}

void imu::read_imu(){
    buffer[0] = 59;
    length = 1;
    if(write(fd, buffer, length) != length){
        printf("Failed to write to the i2c bus.\n");
    }
    length = 14;	
    if(read(fd, buffer, length) != length){
        printf("Failed to read from the i2c bus.\n");
    }
}

void imu::save_imu_data(){
    short int temp;

    static float filtVal[7] = {0};

    float tempVal;

    for(int i = 0; i < 7; i++){
        temp = (int16_t)(buffer[i*2] << 8) + buffer[i*2+1];
        tempVal = (temp - bias[i]) * unit_per_in[i];
        filtVal[i] = tempVal;
    }

    imu_in.ax = filtVal[0];
    imu_in.ay = filtVal[1];
    imu_in.az = filtVal[2];

    imu_in.temp = filtVal[3] + 21;

    imu_in.gx = -filtVal[4];
    imu_in.gy = filtVal[5];
    imu_in.gz = filtVal[6];
}

void imu::integrate_pos(){
    int_data_mtx->lock();

    pos.rz.f += imu_in.gz * period;

    float refax, refay;

    refax = imu_in.ax * cos(pos.rz.f) - imu_in.ay * sin(pos.rz.f);
    refay = imu_in.ay * sin(pos.rz.f) + imu_in.ay * cos(pos.rz.f);

    pos.vx.f += refax * period;
    pos.vy.f += refay * period;

    pos.x.f += pos.vx.f * period;
    pos.y.f += pos.vy.f * period;

    printf("%10f      %10f      %10f\n", refax, refay, pos.rz.f);

    //printf("%10f      %10f\n", imu_in.gy, pos.ry.f);
    /*printf("%10f      %10f      %10f\n", imu_in.ax, pos.vx.f, pos.x.f);
    printf("%10f      %10f      %10f\n", imu_in.ay, pos.vy.f, pos.y.f);
    printf("\n");*/
    
    pos.temp.f = imu_in.temp;

    int_data_mtx->unlock();
}

integrated_data imu::read_integrated_data(){
    std::lock_guard<std::mutex> lock(*int_data_mtx);
    return pos;
}

void imu::reset_vel(){
    int_data_mtx->lock();

    pos.vx.f = 0;
    pos.vy.f = 0;

    int_data_mtx->unlock();
}

void imu::setup_imu(){
    buffer[0] = 26; // Gyro DLPF
    buffer[1] = 1;
    length = 2;
    if(write(fd, buffer, length) != length){
        printf("Failed to write to the i2c bus.\n");
    }


    buffer[0] = 29; // Accel DLPF
    buffer[1] = 5;
    length = 2;
    if(write(fd, buffer, length) != length){
        printf("Failed to write to the i2c bus.\n");
    }
}

