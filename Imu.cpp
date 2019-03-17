#include "Imu.h"

#define addr 0x0D //I2C Address for The HMC5883

Imu::Imu()
{
    init();
}

boolean Imu::init() 
{

    Wire.begin();
    bool ret;
    
    accelerometer.initialize();
    ret = accelerometer.testConnection();
    if(!ret)
        return false;

    gyroscope.initialize();
    ret = gyroscope.testConnection();
    if(!ret)
        return false;
    
    Wire.beginTransmission(addr); //start talking
    Wire.write(0x09); // Set the Register
    Wire.write(0x1D); // Tell the HMC5883 to Continuously Measure
    Wire.endTransmission();

    return true;
}

geometry_msgs::Vector3 Imu::readAccelerometer()
{
    geometry_msgs::Vector3 accel;
    int16_t ax, ay, az;
    
    accelerometer.getAcceleration(&ax, &ay, &az);

    accel.x = ax * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.y = ay * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.z = az * (double) ACCEL_SCALE * G_TO_ACCEL;
    
    return accel;
}


geometry_msgs::Vector3 Imu::readGyroscope()
{
    geometry_msgs::Vector3 gyro;
    int16_t gx, gy, gz;

    gyroscope.getRotation(&gx, &gy, &gz);

    gyro.x = gx * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.y = gy * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.z = gz * (double) GYRO_SCALE * DEG_TO_RAD;

    return gyro;
}

geometry_msgs::Vector3 Imu::readMagnetometer()
{
    geometry_msgs::Vector3 mag;
    int16_t mx, my, mz;

    //Tell the HMC what regist to begin writing data into
    Wire.beginTransmission(addr);
    Wire.write(0x00); //start with register 3.
    Wire.endTransmission();
  
 
    //Read the data.. 2 bytes for each axis.. 6 total bytes
    Wire.requestFrom(addr, 6);
    if(6<=Wire.available()){
        mx = Wire.read()<<8; //MSB  x 
        mx |= Wire.read(); //LSB  x
        mz = Wire.read()<<8; //MSB  z
        mz |= Wire.read(); //LSB z
        my = Wire.read()<<8; //MSB y
        my |= Wire.read(); //LSB y
    }

    mag.x = mx * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.y = my * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.z = mz * (double) MAG_SCALE * UTESLA_TO_TESLA;

    return mag;
}
