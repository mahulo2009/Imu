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

    magnetometer.init();

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
    int mx, my, mz, mt;
    mx = my = mz = 0;

    
    int r = magnetometer.read(&mx,&my,&mz);
    
    //MGUAS TO GAUS - GAUS TO TESLA
    mag.x = mx *0.001 * 0.0001;
    mag.y = my *0.001 * 0.0001;
    mag.z = mz *0.001 * 0.0001;

    return mag;
}
