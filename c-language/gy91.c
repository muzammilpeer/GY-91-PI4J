/*
 * gy91.c
 * 
 * Copyright 2017  <pi@pizero2>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */


#include <stdio.h>
#include <stdlib.h>
#include "MPU9250.h"
#include "BMP280.h"

int main(int argc, char **argv)
{
	int16_t ACCxyz[3];
	int16_t GYRxyz[3];
	int16_t MAGxyz[3];
	float MagDes[3];
	//float CalDes1[3];
	//float CalDes2[3];
	//float SelDes[6];
	
	wiringPiSetup();
	
	BMP280();
	BMP280_read_id();
    BMP280_reg_check();
	
    MPU9250();
    initMPU9250();
    initAK8963(MagDes);
    
    //calibrateMPU9250(CalDes1,CalDes2);
    //MPU9250SelfTest(SelDes);
    
    
    while(1)
    {
        readAccelData(ACCxyz);
        readGyroData(GYRxyz);
		readMagData(MAGxyz);
		bmp280_read();
		
		printf("MPU9250:\r\n");
        printf("ACC:  \tX: %5.4f  \tY: %5.4f  \tZ: %5.4f\r\n",ACCxyz[0]*getAres(),ACCxyz[1]*getAres(),ACCxyz[2]*getAres());
        printf("GYRO: \tX: %7.4f  \tY: %7.4f  \tZ: %7.4f\r\n",GYRxyz[0]*getGres(),GYRxyz[1]*getGres(),GYRxyz[2]*getGres());
        printf("MAG:  \tX: %8.3f  \tY: %8.3f  \tZ: %8.3f\r\n",MAGxyz[0]*getMres(),MAGxyz[1]*getMres(),MAGxyz[2]*getMres());
        printf("Temp: \t%3.1f°C\r\n\r\n",readTempInC());
       
        printf("BMP280:\r\n");
        printf("Temp:\t\t%2.2f `C\r\n", bmp.temperature);
		printf("Pressure:\t%5.4f mbar\r\n", bmp.pressure);
		printf("Altitude:\t%5.3f m\r\n\r\n", bmp.altitude);
        delay(100);
    
    }
    
	
	return 0;
}

