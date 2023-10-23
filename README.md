# Ublox_NEO-MPU6050
### Description  


This project is the attempt to combine MPU6050 and Ublox NEO-M9N. The I2C bus transmits position, speed, accuracy and acceleration data from MPU6050 and Ublox NEO-M9N to the microcontroller. Based on this data, the microcontroller calculates the current coordinates and transmits them to the receiver itself or allows the transfer of position data to receiver.  

![Scheme of MPU6050 and NEO-M9N](/img/scheme_mpu.png)  

### Libraries
Code uses the modified [Simple_MPU6050](https://github.com/ZHomeSlice/Simple_MPU6050) and [SparkFun_u-blox_GNSS_Arduino_Library v2](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library)

### Problem
An MPU6050 has very low accuracy for use, even after additional calibrations. Who knows, maybe someone will be able to improve the code and make a useful solution