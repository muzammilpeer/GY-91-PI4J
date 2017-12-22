package com.muzammilpeer.sensors.gy91;

import static com.pi4j.wiringpi.Gpio.delay;
import static com.pi4j.wiringpi.Gpio.wiringPiSetup;

public class GY91 {

    public void readDataFromSensors() {
        int[] ACCxyz = new int[3];
        int[] GYRxyz = new int[3];
        int[] MAGxyz = new int[3];
        float[] MagDes = new float[3];
        // float CalDes1[3];
        // float CalDes2[3];
        // float SelDes[6];
        BMP280 bmp280 = new BMP280();
        MPU9250 mpu9250 = new MPU9250();


        wiringPiSetup();

        bmp280.BMP280();
        bmp280.BMP280_read_id();
        bmp280.BMP280_reg_check();

        mpu9250.MPU9250();
        mpu9250.initMPU9250();
        mpu9250.initAK8963(MagDes);
        // calibrateMPU9250(CalDes1,CalDes2);
        // MPU9250SelfTest(SelDes);
        while (true) {
            mpu9250.readAccelData(ACCxyz);
            mpu9250.readGyroData(GYRxyz);
            mpu9250.readMagData(MAGxyz);
            bmp280.bmp280_read();
            System.out.printf("MPU9250:\r\n");
            System.out.printf("ACC:  \tX: %5.4f  \tY: %5.4f  \tZ: %5.4f\r\n", (ACCxyz[0] * mpu9250.getAres()), (ACCxyz[1] * mpu9250.getAres()), (ACCxyz[2] * mpu9250.getAres()));
            System.out.printf("GYRO: \tX: %7.4f  \tY: %7.4f  \tZ: %7.4f\r\n", (GYRxyz[0] * mpu9250.getGres()), (GYRxyz[1] * mpu9250.getGres()), (GYRxyz[2] * mpu9250.getGres()));
            System.out.printf("MAG:  \tX: %8.3f  \tY: %8.3f  \tZ: %8.3f\r\n", (MAGxyz[0] * mpu9250.getMres()), (MAGxyz[1] * mpu9250.getMres()), (MAGxyz[2] * mpu9250.getMres()));
            System.out.printf("Temp: \t%3.1f�C\r\n\r\n", mpu9250.readTempInC());
            System.out.printf("BMP280:\r\n");
            System.out.printf("Temp:\t\t%2.2f `C\r\n", bmp280.bmp.temperature);
            System.out.printf("Pressure:\t%5.4f mbar\r\n", bmp280.bmp.pressure);
            System.out.printf("Altitude:\t%5.3f m\r\n\r\n", bmp280.bmp.altitude);
            delay(100);
        }
    }
}
