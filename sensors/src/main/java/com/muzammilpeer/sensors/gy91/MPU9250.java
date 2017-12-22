package com.muzammilpeer.sensors.gy91;


import com.muzammilpeer.sensors.gy91.enums.Ascale;
import com.muzammilpeer.sensors.gy91.enums.Gscale;
import com.muzammilpeer.sensors.gy91.enums.Mscale;

import static com.muzammilpeer.sensors.gy91.enums.Ascale.AFS_2G;
import static com.muzammilpeer.sensors.gy91.enums.Gscale.GFS_250DPS;
import static com.muzammilpeer.sensors.gy91.enums.Mscale.MFS_16BITS;
import static com.pi4j.wiringpi.Gpio.delay;
import static com.pi4j.wiringpi.I2C.*;
import static java.lang.Math.pow;
import static java.lang.StrictMath.sqrt;


//#define DEBUG
 /*
#define uint8_t unsigned char
#define int8_t char
#define uint16_t unsigned short
#define int16_t short
#define uint32_t unsigned long
#define int32_t long
*/
public class MPU9250 {


    // See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
    final int AK8963_ADDRESS = 0x0C;
    final int WHO_AM_I_AK8963 = 0x00; // should return 0x48
    final int INFO = 0x01;
    final int AK8963_ST1 = 0x02;// data ready status bit 0
    final int AK8963_XOUT_L = 0x03;// data
    final int AK8963_XOUT_H = 0x04;
    final int AK8963_YOUT_L = 0x05;
    final int AK8963_YOUT_H = 0x06;
    final int AK8963_ZOUT_L = 0x07;
    final int AK8963_ZOUT_H = 0x08;
    final int AK8963_ST2 = 0x09; // Data overflow bit 3 and data read error status bit 2
    final int AK8963_CNTL = 0x0A;// Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
    final int AK8963_ASTC = 0x0C;// Self test control
    final int AK8963_I2CDIS = 0x0F; // I2C disable
    final int AK8963_ASAX = 0x10; // Fuse ROM x-axis sensitivity adjustment value
    final int AK8963_ASAY = 0x11;// Fuse ROM y-axis sensitivity adjustment value
    final int AK8963_ASAZ = 0x12;// Fuse ROM z-axis sensitivity adjustment value
    final int SELF_TEST_X_GYRO = 0x00;
    final int SELF_TEST_Y_GYRO = 0x01;
    final int SELF_TEST_Z_GYRO = 0x02;

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

    final int SELF_TEST_X_ACCEL = 0x0D;
    final int SELF_TEST_Y_ACCEL = 0x0E;
    final int SELF_TEST_Z_ACCEL = 0x0F;
    final int SELF_TEST_A = 0x10;
    final int XG_OFFSET_H = 0x13; // User-defined trim values for gyroscope
    final int XG_OFFSET_L = 0x14;
    final int YG_OFFSET_H = 0x15;
    final int YG_OFFSET_L = 0x16;
    final int ZG_OFFSET_H = 0x17;
    final int ZG_OFFSET_L = 0x18;
    final int SMPLRT_DIV = 0x19;
    final int CONFIG = 0x1A;
    final int GYRO_CONFIG = 0x1B;
    final int ACCEL_CONFIG = 0x1C;
    final int ACCEL_CONFIG2 = 0x1D;
    final int LP_ACCEL_ODR = 0x1E;
    final int WOM_THR = 0x1F;
    final int MOT_DUR = 0x20; // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
    final int ZMOT_THR = 0x21;// Zero-motion detection threshold bits [7:0]
    final int ZRMOT_DUR = 0x22;// Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
    final int FIFO_EN = 0x23;
    final int I2C_MST_CTRL = 0x24;
    final int I2C_SLV0_ADDR = 0x25;
    final int I2C_SLV0_REG = 0x26;
    final int I2C_SLV0_CTRL = 0x27;
    final int I2C_SLV1_ADDR = 0x28;
    final int I2C_SLV1_REG = 0x29;
    final int I2C_SLV1_CTRL = 0x2A;
    final int I2C_SLV2_ADDR = 0x2B;
    final int I2C_SLV2_REG = 0x2C;
    final int I2C_SLV2_CTRL = 0x2D;
    final int I2C_SLV3_ADDR = 0x2E;
    final int I2C_SLV3_REG = 0x2F;
    final int I2C_SLV3_CTRL = 0x30;
    final int I2C_SLV4_ADDR = 0x31;
    final int I2C_SLV4_REG = 0x32;
    final int I2C_SLV4_DO = 0x33;
    final int I2C_SLV4_CTRL = 0x34;
    final int I2C_SLV4_DI = 0x35;
    final int I2C_MST_STATUS = 0x36;
    final int INT_PIN_CFG = 0x37;
    final int INT_ENABLE = 0x38;
    final int DMP_INT_STATUS = 0x39; // Check DMP interrupt
    final int INT_STATUS = 0x3A;
    final int ACCEL_XOUT_H = 0x3B;
    final int ACCEL_XOUT_L = 0x3C;
    final int ACCEL_YOUT_H = 0x3D;
    final int ACCEL_YOUT_L = 0x3E;
    final int ACCEL_ZOUT_H = 0x3F;
    final int ACCEL_ZOUT_L = 0x40;
    final int TEMP_OUT_H = 0x41;
    final int TEMP_OUT_L = 0x42;
    final int GYRO_XOUT_H = 0x43;
    final int GYRO_XOUT_L = 0x44;
    final int GYRO_YOUT_H = 0x45;
    final int GYRO_YOUT_L = 0x46;
    final int GYRO_ZOUT_H = 0x47;
    final int GYRO_ZOUT_L = 0x48;
    final int EXT_SENS_DATA_00 = 0x49;
    final int EXT_SENS_DATA_01 = 0x4A;
    final int EXT_SENS_DATA_02 = 0x4B;
    final int EXT_SENS_DATA_03 = 0x4C;
    final int EXT_SENS_DATA_04 = 0x4D;
    final int EXT_SENS_DATA_05 = 0x4E;
    final int EXT_SENS_DATA_06 = 0x4F;
    final int EXT_SENS_DATA_07 = 0x50;
    final int EXT_SENS_DATA_08 = 0x51;
    final int EXT_SENS_DATA_09 = 0x52;
    final int EXT_SENS_DATA_10 = 0x53;
    final int EXT_SENS_DATA_11 = 0x54;
    final int EXT_SENS_DATA_12 = 0x55;
    final int EXT_SENS_DATA_13 = 0x56;
    final int EXT_SENS_DATA_14 = 0x57;
    final int EXT_SENS_DATA_15 = 0x58;
    final int EXT_SENS_DATA_16 = 0x59;
    final int EXT_SENS_DATA_17 = 0x5A;
    final int EXT_SENS_DATA_18 = 0x5B;
    final int EXT_SENS_DATA_19 = 0x5C;
    final int EXT_SENS_DATA_20 = 0x5D;
    final int EXT_SENS_DATA_21 = 0x5E;
    final int EXT_SENS_DATA_22 = 0x5F;
    final int EXT_SENS_DATA_23 = 0x60;
    final int MOT_DETECT_STATUS = 0x61;
    final int I2C_SLV0_DO = 0x63;
    final int I2C_SLV1_DO = 0x64;
    final int I2C_SLV2_DO = 0x65;
    final int I2C_SLV3_DO = 0x66;
    final int I2C_MST_DELAY_CTRL = 0x67;
    final int SIGNAL_PATH_RESET = 0x68;
    final int MOT_DETECT_CTRL = 0x69;
    final int USER_CTRL = 0x6A; // Bit 7 enable DMP, bit 3 reset DMP
    final int PWR_MGMT_1 = 0x6B;// Device defaults to the SLEEP mode
    final int PWR_MGMT_2 = 0x6C;
    final int DMP_BANK = 0x6D; // Activates a specific bank in the DMP
    final int DMP_RW_PNT = 0x6E; // Set read/write pointer to a specific start address in specified DMP bank
    final int DMP_REG = 0x6F; // Register in DMP from which to read or to which to write
    final int DMP_REG_1 = 0x70;
    final int DMP_REG_2 = 0x71;
    final int FIFO_COUNTH = 0x72;
    final int FIFO_COUNTL = 0x73;
    final int FIFO_R_W = 0x74;
    final int WHO_AM_I_MPU9250 = 0x75;// Should return 0x71
    final int XA_OFFSET_H = 0x77;
    final int XA_OFFSET_L = 0x78;
    final int YA_OFFSET_H = 0x7A;
    final int YA_OFFSET_L = 0x7B;
    final int ZA_OFFSET_H = 0x7D;
    final int ZA_OFFSET_L = 0x7E;

    // Using the MSENSR-9250 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
//mbed uses the eight-bit device address, so shift seven-bit addresses left by one!
    final int ADO = 0;
    // Device address when ADO = 1
    // Device address when ADO = 0
    final int MPU9250_ADDRESS = ADO == 1 ? 0x69 : 0x68;

    // Set initial input parameters

    Ascale ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    Gscale gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
    Mscale mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
    //peer
    int Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
    double aRes, gRes, mRes;      // scale resolutions per LSB for the sensors


    // Pin definitions
    int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

    int accelCount[] = new int[3];  // Stores the 16-bit signed accelerometer sensor output
    int gyroCount[] = new int[3];   // Stores the 16-bit signed gyro sensor output
    int magCount[] = new int[3];    // Stores the 16-bit signed magnetometer sensor output
    float magCalibration[] = {0, 0, 0};
    float magbias[] = {0, 0, 0};  // Factory mag calibration and mag bias
    float gyroBias[] = {0, 0, 0};
    float accelBias[] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
    float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
    int tempCount;   // Stores the real internal chip temperature in degrees Celsius
    float temperature;
    float SelfTest[] = new float[6];

    int delt_t = 0; // used to control display output rate
    int count = 0;  // used to control display output rate


    float Kp = 2.0f * 5.0f;// these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
    float Ki = 0.0f;

    float pitch, yaw, roll;
    float deltat = 0.0f;                             // integration interval for both filter schemes
    int lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval                               // used to calculate integration interval
    double q[] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion
    float eInt[] = {0.0f, 0.0f, 0.0f};              // vector to hold integral error for Mahony method

    int fdAK8963;
    int fdMPU9250;
    double beta, zeta;
    float PI, GyroMeasError, GyroMeasDrift;

    void MPU9250() {
        // parameters for 6 DoF sensor fusion calculations
        PI = 3.14159265358979323846f;
        GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
        beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
        GyroMeasDrift = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
        zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

        fdAK8963 = wiringPiI2CSetup(AK8963_ADDRESS);

        fdMPU9250 = wiringPiI2CSetup(MPU9250_ADDRESS);

    }

    void writeByte(int address, int subAddress, int data) {

        if (address == MPU9250_ADDRESS)
            wiringPiI2CWriteReg8(fdMPU9250, subAddress, data);
        if (address == AK8963_ADDRESS)
            wiringPiI2CWriteReg8(fdAK8963, subAddress, data);
    }

    //peer
//    char readByte(int address, int subAddress) {
    int readByte(int address, int subAddress) {
        //peer
//        char data;
        int data = 0;


        if (address == MPU9250_ADDRESS)
            data = wiringPiI2CReadReg8(fdMPU9250, subAddress);
        if (address == AK8963_ADDRESS)
            data = wiringPiI2CReadReg8(fdAK8963, subAddress);

        return data;
    }


    void readBytes(int address, int subAddress, int count, int dest[]) {
        //peer
//        char data;
        int data;
        for (char i = 0; i < count; i++) {
            data = readByte(address, subAddress + i);
            dest[i] = data;
        }
    }


    //peer
//    float getMres() {
    double getMres() {
        switch (mscale) {
            // Possible magnetometer scales (and their register bit settings) are:
            // 14 bit resolution (0) and 16 bit resolution (1)
            case MFS_14BITS:
                mRes = 10.0 * 4219.0 / 8190.0; // Proper scale to return milliGauss
                break;
            case MFS_16BITS:
                mRes = 10.0 * 4219.0 / 32760.0; // Proper scale to return milliGauss
                break;
        }
        return mRes;
    }

    //peer
//    float getGres() {
    double getGres() {
        switch (gscale) {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case GFS_250DPS:
                gRes = 250.0 / 32768.0;
                break;
            case GFS_500DPS:
                gRes = 500.0 / 32768.0;
                break;
            case GFS_1000DPS:
                gRes = 1000.0 / 32768.0;
                break;
            case GFS_2000DPS:
                gRes = 2000.0 / 32768.0;
                break;
        }
        return gRes;
    }

    //peer
//    float getAres() {
    double getAres() {
        switch (ascale) {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case AFS_2G:
                aRes = 2.0 / 32768.0;
                break;
            case AFS_4G:
                aRes = 4.0 / 32768.0;
                break;
            case AFS_8G:
                aRes = 8.0 / 32768.0;
                break;
            case AFS_16G:
                aRes = 16.0 / 32768.0;
                break;
        }
        return aRes;
    }


    void readAccelData(int destination[]) {
        int rawData[] = new int[6];  // x/y/z accel register data stored here
        //peer
//        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawData[0]);  // Read the six raw data registers into data array
        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawData);  // Read the six raw data registers into data array
        destination[0] = (int) (((int) rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = (int) (((int) rawData[2] << 8) | rawData[3]);
        destination[2] = (int) (((int) rawData[4] << 8) | rawData[5]);
    }

    void readGyroData(int destination[]) {
        int rawData[] = new int[6];  // x/y/z gyro register data stored here
//        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawData[0])
        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawData);  // Read the six raw data registers sequentially into data array
        destination[0] = (int) (((int) rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = (int) (((int) rawData[2] << 8) | rawData[3]);
        destination[2] = (int) (((int) rawData[4] << 8) | rawData[5]);
    }

    void readMagData(int destination[]) {
        int rawData[] = new int[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
        // muzammilpeer
//        if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
        if ((readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) == 1) { // wait for magnetometer data ready bit to be set
            //peer
//            readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, & rawData[0])
            readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, rawData);  // Read the six raw data and ST2 registers sequentially into data array
            int c = rawData[6]; // End data read by reading ST2 register
            // muzammilpeer
//            if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
            if (!((c & 0x08) == 1)) { // Check if magnetic sensor overflow set, if not then report data
                destination[0] = (int) (((int) rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
                destination[1] = (int) (((int) rawData[3] << 8) | rawData[2]);  // Data stored as little Endian
                destination[2] = (int) (((int) rawData[5] << 8) | rawData[4]);
            }
        }
    }

    int readTempData() {
        int rawData[] = new int[2];  // x/y/z gyro register data stored here
//        readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, & rawData[0])
        readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, rawData);  // Read the two raw data registers sequentially into data array
        return (int) (((int) rawData[0]) << 8 | rawData[1]);  // Turn the MSB and LSB into a 16-bit value
    }

    //peer
//    float readTempInC() {
    double readTempInC() {
        return (double) readTempData() / 333.87 + 21.0;
    }


    void resetMPU9250() {
        // reset device
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
        delay(100);
    }

    void initAK8963(float destination[]) {
        // First extract the factory calibration for each magnetometer axis
        int rawData[] = new int[3];  // x/y/z gyro calibration data stored here
        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
        delay(10);
        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
        delay(10);
        //peer
//        readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, & rawData[0]);  // Read the x-, y-, and z-axis calibration values
        readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, rawData);  // Read the x-, y-, and z-axis calibration values
        destination[0] = (float) (rawData[0] - 128) / 256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
        destination[1] = (float) (rawData[1] - 128) / 256.0f + 1.0f;
        destination[2] = (float) (rawData[2] - 128) / 256.0f + 1.0f;
        writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
        delay(10);
        // Configure the magnetometer for continuous read and highest resolution
        // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
        // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
        writeByte(AK8963_ADDRESS, AK8963_CNTL, mscale.ordinal() << 4 | Mmode); // Set magnetometer data resolution and sample ODR
        delay(10);
    }


    void initMPU9250() {
        // Initialize MPU9250 device
        // wake up device
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
        delay(10); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

        // get stable time source
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

        // Configure Gyro and Accelerometer
        // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
        // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
        // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
        writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above

        // Set gyroscope full scale range
        // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        int c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | gscale.ordinal() << 3); // Set full scale range for the gyro

        // Set accelerometer configuration
        c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | ascale.ordinal() << 3); // Set full scale range for the accelerometer

        // Set accelerometer sample rate configuration
        // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
        // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
        c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
        // can join the I2C bus and all can be controlled by the Arduino as master
        writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
        writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    }

    // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
    // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
    void calibrateMPU9250(float dest1[], float dest2[]) {
        int data[] = new int[12]; // data array to hold accelerometer and gyro x, y, z, data
        int ii, packet_count, fifo_count;
        int gyro_bias[] = {0, 0, 0}, accel_bias[] = {0, 0, 0};

        // reset device, reset all registers, clear gyro and accelerometer bias registers
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
        delay(100);

        // get stable time source
        // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
        writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
        delay(200);

        // Configure device for bias calculation
        writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
        writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
        writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
        delay(15);

        // Configure MPU9250 gyro and accelerometer for bias calculation
        writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
        writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

        int gyrosensitivity = 131;   // = 131 LSB/degrees/sec
        int accelsensitivity = 16384;  // = 16384 LSB/g

        // Configure FIFO to capture accelerometer and gyro data for bias calculation
        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
        delay(80); // accumulate 40 samples in 80 milliseconds = 480 bytes

        // At end of sample accumulation, turn off FIFO sensor read
        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
        readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, data); // read FIFO sample count
        fifo_count = ((int) data[0] << 8) | data[1];
        packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

        for (ii = 0; ii < packet_count; ii++) {
            int accel_temp[] = {0, 0, 0}, gyro_temp[] = {0, 0, 0};
            //peer
//            readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, & data[0]); // read data for averaging
            readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, data); // read data for averaging
            accel_temp[0] = (int) (((int) data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
            accel_temp[1] = (int) (((int) data[2] << 8) | data[3]);
            accel_temp[2] = (int) (((int) data[4] << 8) | data[5]);
            gyro_temp[0] = (int) (((int) data[6] << 8) | data[7]);
            gyro_temp[1] = (int) (((int) data[8] << 8) | data[9]);
            gyro_temp[2] = (int) (((int) data[10] << 8) | data[11]);

            accel_bias[0] += (int) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            accel_bias[1] += (int) accel_temp[1];
            accel_bias[2] += (int) accel_temp[2];
            gyro_bias[0] += (int) gyro_temp[0];
            gyro_bias[1] += (int) gyro_temp[1];
            gyro_bias[2] += (int) gyro_temp[2];

        }
        accel_bias[0] /= (int) packet_count; // Normalize sums to get average count biases
        accel_bias[1] /= (int) packet_count;
        accel_bias[2] /= (int) packet_count;
        gyro_bias[0] /= (int) packet_count;
        gyro_bias[1] /= (int) packet_count;
        gyro_bias[2] /= (int) packet_count;

        if (accel_bias[2] > 0L) {
            accel_bias[2] -= (int) accelsensitivity;
        }  // Remove gravity from the z-axis accelerometer bias calculation
        else {
            accel_bias[2] += (int) accelsensitivity;
        }

        // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
        data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
        data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
        data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
        data[3] = (-gyro_bias[1] / 4) & 0xFF;
        data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
        data[5] = (-gyro_bias[2] / 4) & 0xFF;

        /// Push gyro biases to hardware registers
		/*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
		writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
		writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
		writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
		writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
		writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
		*/
        dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
        dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
        dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

        // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
        // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
        // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
        // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
        // the accelerometer biases calculated above must be divided by 8.

        int accel_bias_reg[] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
//        readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, & data[0]); // Read factory accelerometer trim values
        readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, data); // Read factory accelerometer trim values
        accel_bias_reg[0] = (int) ((int) data[0] << 8) | data[1];
//        readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, & data[0]);
        readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, data);
        accel_bias_reg[1] = (int) ((int) data[0] << 8) | data[1];
        readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, data);
        accel_bias_reg[2] = (int) ((int) data[0] << 8) | data[1];


        int mask = 1; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
        int mask_bit[] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

        for (ii = 0; ii < 3; ii++) {
            //muzammilpeer
//            if ((accel_bias_reg[ii] & mask) == 1) {
            if ((accel_bias_reg[ii] & mask) == 1) {
                mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
            }
        }

        // Construct total accelerometer bias, including calculated average accelerometer bias from above
        accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
        accel_bias_reg[1] -= (accel_bias[1] / 8);
        accel_bias_reg[2] -= (accel_bias[2] / 8);

        data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
        data[1] = (accel_bias_reg[0]) & 0xFF;
        data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
        data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
        data[3] = (accel_bias_reg[1]) & 0xFF;
        data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
        data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
        data[5] = (accel_bias_reg[2]) & 0xFF;
        data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

        // Apparently this is not working for the acceleration biases in the MPU-9250
        // Are we handling the temperature correction bit properly?
        // Push accelerometer biases to hardware registers
		/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
		writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
		writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
		writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
		writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
		writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
		*/
        // Output scaled accelerometer biases for manual subtraction in the main program
        dest2[0] = (float) accel_bias[0] / (float) accelsensitivity;
        dest2[1] = (float) accel_bias[1] / (float) accelsensitivity;
        dest2[2] = (float) accel_bias[2] / (float) accelsensitivity;
    }


    // Accelerometer and gyroscope self test; check calibration wrt factory settings
    void MPU9250SelfTest(double destination[]) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
    {
        int rawData[] = {0, 0, 0, 0, 0, 0};
        int selfTest[] = new int[6];
        int gAvg[] = new int[3];
        int aAvg[] = new int[3];
        int aSTAvg[] = new int[3];
        int gSTAvg[] = new int[3];
        double factoryTrim[] = new double[6];
        int FS = 0;

        writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
        writeByte(MPU9250_ADDRESS, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

        for (int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

            //peer
//            readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, & rawData[0])
            readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawData); // Read the six raw data registers into data array
            aAvg[0] += (int) (((int) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
            aAvg[1] += (int) (((int) rawData[2] << 8) | rawData[3]);
            aAvg[2] += (int) (((int) rawData[4] << 8) | rawData[5]);

            //peer
//            readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, & rawData[0])
            readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawData); // Read the six raw data registers sequentially into data array
            gAvg[0] += (int) (((int) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
            gAvg[1] += (int) (((int) rawData[2] << 8) | rawData[3]);
            gAvg[2] += (int) (((int) rawData[4] << 8) | rawData[5]);
        }

        for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
            aAvg[ii] /= 200;
            gAvg[ii] /= 200;
        }

        // Configure the accelerometer for self-test
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
        delay(25); // Delay a while to let the device stabilize

        for (int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

            //peer
//            readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, & rawData[0])
            readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawData); // Read the six raw data registers into data array
            aSTAvg[0] += (int) (((int) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
            aSTAvg[1] += (int) (((int) rawData[2] << 8) | rawData[3]);
            aSTAvg[2] += (int) (((int) rawData[4] << 8) | rawData[5]);

            //peer
//            readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, & rawData[0])
            readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawData); // Read the six raw data registers sequentially into data array
            gSTAvg[0] += (int) (((int) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
            gSTAvg[1] += (int) (((int) rawData[2] << 8) | rawData[3]);
            gSTAvg[2] += (int) (((int) rawData[4] << 8) | rawData[5]);
        }

        for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
            aSTAvg[ii] /= 200;
            gSTAvg[ii] /= 200;
        }

        // Configure the gyro and accelerometer for normal operation
        writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
        writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
        delay(25); // Delay a while to let the device stabilize

        // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
        selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
        selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
        selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
        selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO); // X-axis gyro self-test results
        selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
        selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

        // Retrieve factory self-test value from self-test code reads
        factoryTrim[0] = (double) (2620 / 1 << FS) * (pow(1.01, ((float) selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
        factoryTrim[1] = (double) (2620 / 1 << FS) * (pow(1.01, ((float) selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
        factoryTrim[2] = (double) (2620 / 1 << FS) * (pow(1.01, ((float) selfTest[2] - 1.0))); // FT[Za] factory trim calculation
        factoryTrim[3] = (double) (2620 / 1 << FS) * (pow(1.01, ((float) selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
        factoryTrim[4] = (double) (2620 / 1 << FS) * (pow(1.01, ((float) selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
        factoryTrim[5] = (double) (2620 / 1 << FS) * (pow(1.01, ((float) selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

        // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
        // To get percent, must multiply by 100
        for (int i = 0; i < 3; i++) {
            destination[i] = 100.0 * ((double) (aSTAvg[i] - aAvg[i])) / factoryTrim[i]; // Report percent differences
            destination[i + 3] = 100.0 * ((double) (gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3]; // Report percent differences
        }

    }


    // Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
    // (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
    // which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
    // device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
    // The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
    // but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
    void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
        double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
        double norm;
        double hx, hy, _2bx, _2bz;
        double s1, s2, s3, s4;
        double qDot1, qDot2, qDot3, qDot4;

        // Auxiliary variables to avoid repeated arithmetic
        double _2q1mx;
        double _2q1my;
        double _2q1mz;
        double _2q2mx;
        double _4bx;
        double _4bz;
        double _2q1 = 2.0f * q1;
        double _2q2 = 2.0f * q2;
        double _2q3 = 2.0f * q3;
        double _2q4 = 2.0f * q4;
        double _2q1q3 = 2.0f * q1 * q3;
        double _2q3q4 = 2.0f * q3 * q4;
        double q1q1 = q1 * q1;
        double q1q2 = q1 * q2;
        double q1q3 = q1 * q3;
        double q1q4 = q1 * q4;
        double q2q2 = q2 * q2;
        double q2q3 = q2 * q3;
        double q2q4 = q2 * q4;
        double q3q3 = q3 * q3;
        double q3q4 = q3 * q4;
        double q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        _2q1mx = 2.0f * q1 * mx;
        _2q1my = 2.0f * q1 * my;
        _2q1mz = 2.0f * q1 * mz;
        _2q2mx = 2.0f * q2 * mx;
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
        norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
        norm = 1.0f / norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
        qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
        qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
        qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

        // Integrate to yield quaternion
        q1 += qDot1 * deltat;
        q2 += qDot2 * deltat;
        q3 += qDot3 * deltat;
        q4 += qDot4 * deltat;
        norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
        norm = 1.0f / norm;
        q[0] = q1 * norm;
        q[1] = q2 * norm;
        q[2] = q3 * norm;
        q[3] = q4 * norm;

    }


    // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
    // measured ones.
    void MahonyQuaternionUpdate(double ax, double ay, double az, double gx, double gy, double gz, double mx, double my, double mz) {
        double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
        double norm;
        double hx, hy, bx, bz;
        double vx, vy, vz, wx, wy, wz;
        double ex, ey, ez;
        double pa, pb, pc;

        // Auxiliary variables to avoid repeated arithmetic
        double q1q1 = q1 * q1;
        double q1q2 = q1 * q2;
        double q1q3 = q1 * q3;
        double q1q4 = q1 * q4;
        double q2q2 = q2 * q2;
        double q2q3 = q2 * q3;
        double q2q4 = q2 * q4;
        double q3q3 = q3 * q3;
        double q3q4 = q3 * q4;
        double q4q4 = q4 * q4;

        // Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;        // use reciprocal for division
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalise magnetometer measurement
        norm = sqrt(mx * mx + my * my + mz * mz);
        if (norm == 0.0f) return; // handle NaN
        norm = 1.0f / norm;        // use reciprocal for division
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
        hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
        bx = sqrt((hx * hx) + (hy * hy));
        bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

        // Estimated direction of gravity and magnetic field
        vx = 2.0f * (q2q4 - q1q3);
        vy = 2.0f * (q1q2 + q3q4);
        vz = q1q1 - q2q2 - q3q3 + q4q4;
        wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
        wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
        wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

        // Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
        if (Ki > 0.0f) {
            eInt[0] += ex;      // accumulate integral error
            eInt[1] += ey;
            eInt[2] += ez;
        } else {
            eInt[0] = 0.0f;     // prevent integral wind up
            eInt[1] = 0.0f;
            eInt[2] = 0.0f;
        }

        // Apply feedback terms
        gx = gx + Kp * ex + Ki * eInt[0];
        gy = gy + Kp * ey + Ki * eInt[1];
        gz = gz + Kp * ez + Ki * eInt[2];

        // Integrate rate of change of quaternion
        pa = q2;
        pb = q3;
        pc = q4;
        q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
        q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
        q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
        q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

        // Normalise quaternion
        norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        norm = 1.0f / norm;
        q[0] = q1 * norm;
        q[1] = q2 * norm;
        q[2] = q3 * norm;
        q[3] = q4 * norm;

    }


}
