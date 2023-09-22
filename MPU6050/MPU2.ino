#include "I2Cdev.h"
#include "MPU6050.h"

#define TO_DEG 57.3 // 180/pi = 57.295779513082320876798154814105
#define T_OUT 200 // каждый 100 миллисекунд будем проводить вычисления 
#define KF 0.1 // коэффициент комплементарного фильтра

MPU6050 accelgyro;

unsigned long int t;
int16_t ax_raw, ay_raw, az_raw, ax_raw_g, ay_raw_g, az_raw_g;
int16_t gx_raw, gy_raw, gz_raw;
float ax, ay, az, ax_g, ay_g, az_g, ax_n, ay_n, az_n;
float gx, gy, gz;
float angle_ax, angle_ay, angle_gx = 0, angle_gy = 0, angle_gz = 0, angle_x = 0, angle_y = 0, angle_z = 0;
float angle_x_deg, angle_y_deg;
float psi, theta, gamma;
float cos_psi, sin_psi, cos_theta, sin_theta, cos_gamma, sin_gamma;

void setup() {
    Serial.begin(9600);
    
    accelgyro.initialize();
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
}

void loop() {
    if( millis()-t > T_OUT ){
        t = millis();
        
        //////////////////////////////////////Считывание показаний с датчика///////////////////////////////////
        accelgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);
        //printSerial(ax_raw, ay_raw, az_raw);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////Вычисление углов//////////////////////////////////////////////
        // ограничиваем +-1g
        ax_raw_g = constrain(ax_raw, -16384, 16384);
        ay_raw_g = constrain(ay_raw, -16384, 16384);
        az_raw_g = constrain(az_raw, -16384, 16384);
        //printSerial(ax_raw_g, ay_raw_g, az_raw_g);
        
        // переводим в +-1.0
        ax_g = ax_raw_g / 16384.0;
        ay_g = ay_raw_g / 16384.0;
        az_g = az_raw_g / 16384.0;
        //printSerial(ax_g, ay_g, az_g);

        // угол наклона по акселерометру
        angle_ax = 90.0 - (acos(ay_g))*TO_DEG;
        angle_ay = (acos(ax_g))*TO_DEG - 90.0;
        //printSerial(angle_ax, angle_ay);

        // преобразование сырых данных гироскопа в град/сек 250 град/сек = 32768/250
        gx = gx_raw / 32768.0 * 250;
        gy = gy_raw / 32768.0 * 250;
        gz = gz_raw / 32768.0 * 250;
        //printSerial(gx, gy, gz);

        // угол наклона по гироскопу
        angle_gx = angle_gx + gx * T_OUT/1000.0;
        angle_gy = angle_gy + gy * T_OUT/1000.0;
        angle_gz = angle_gz + gz * T_OUT/1000.0;
        //printSerial(angle_gx, angle_gy, angle_gz);

        // угол наклона по акселерометру и гироскопу
        angle_x = (1-KF)*(angle_x+gx*T_OUT/1000.0) + KF*angle_ax;
        angle_y = (1-KF)*(angle_y+gy*T_OUT/1000.0) + KF*angle_ay;
        angle_z = 0;
        //printSerial(angle_x, angle_y, angle_z);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////Вычисление ускорений////////////////////////////////////////
        // преобразование сырых данных акселерометра в G 2G = 32768/2
        ax = ax_raw / 32768.0 * 2;
        ay = ay_raw / 32768.0 * 2;
        az = az_raw / 32768.0 * 2;
        //printSerial(ax, ay, az);
        
        angle_x_deg = angle_x/TO_DEG;
        angle_y_deg = angle_y/TO_DEG;
                
        // преобразование ускорений из связанной СК в нормальную СК 
        psi = angle_y_deg;
        theta = 0;
        gamma = angle_x_deg;
        
        cos_psi = cos(psi);
        sin_psi = sin(psi);
        cos_theta = 1; // cos(0)  =1
        sin_theta = 0; // sin(0) = 0
        cos_gamma = cos(gamma);
        sin_gamma = sin(gamma);

        // ускорения датчика в связанной СК (без учета ускорения свободного падения)
        ax = ax + 1*sin_psi;
        ay = ay - 1*sin_gamma;
        az = az - 1*cos_psi*cos_gamma;
        // printSerial(ax, ay, az);
          
        ax_n = (cos_psi*cos_theta)*ax + (sin_psi*sin_gamma-cos_psi*sin_theta*cos_gamma)*ay + (sin_psi*cos_gamma+cos_psi*sin_theta*sin_gamma)*az;
        ay_n = (sin_theta)*ax + (cos_theta*cos_gamma)*ay + (-cos_theta*sin_gamma)*az;
        az_n = (-sin_psi*cos_theta)*ax + (cos_psi*sin_gamma+sin_psi*sin_theta*cos_gamma)*ay + (cos_psi*cos_gamma-sin_psi*sin_theta*sin_gamma)*az;
        // printSerial(ax_n, ay_n, az_n);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
    }
}

template <typename Type> Type printSerial(Type x, Type y, Type z)
{
    Serial.print(x);Serial.print("\t");
    Serial.print(y);Serial.print("\t");
    Serial.print(z);Serial.println("\t");  
}

template <typename Type> Type printSerial(Type x, Type y)
{
    Serial.print(x);Serial.print("\t");
    Serial.print(y);Serial.println("\t");
}


