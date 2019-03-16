#ifndef Imu_H
#define Imu_H


#include "ImuBase.h"

#include "Arduino.h"
#include "Wire.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "QMC5883L.h"

#define G_TO_ACCEL 9.81
#define MGAUSS_TO_UTESLA 0.1
#define UTESLA_TO_TESLA 0.000001

#define ACCEL_SCALE 1 / 256 // LSB/g
#define GYRO_SCALE 1 / 14.375 // LSB/(deg/s)
#define MAG_SCALE 0.92 * MGAUSS_TO_UTESLA // uT/LSB


class Imu : public ImuBase {

    public:

	    Imu();			
	
        boolean init();

        virtual geometry_msgs::Vector3 readAccelerometer();
        virtual geometry_msgs::Vector3 readGyroscope();
        virtual geometry_msgs::Vector3 readMagnetometer();
	
    private:
    
        ADXL345 accelerometer;
        ITG3200 gyroscope;
        QMC5883L magnetometer;

};
#endif