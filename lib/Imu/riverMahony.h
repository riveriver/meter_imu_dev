//=====================================================================================================
// MahonyAHRS.c
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
/*
在MahonyAHRS算法中，Kp和Ki是用于控制姿态估计的增益参数。
Kp（比例增益参数）用于调节陀螺仪测量误差的影响。较大的Kp值会增加陀螺仪测量的影响，从而使姿态估计更加灵敏和响应速度更快。
然而，如果Kp设置得太大，可能会引起震荡或不稳定的估计结果。
Ki（积分增益参数）主要用于消除陀螺仪的漂移。它通过对姿态估计的误差进行积分来补偿陀螺仪的漂移。
较大的Ki值会增加陀螺仪漂移的补偿效果，但如果设置得太大，可能会导致过度补偿和不稳定。

1. 初始值设定：可以将Kp和Ki设置为较小的值，例如0.1或0.01。
2. 实验数据收集：在实际应用中，收集传感器数据，包括陀螺仪和加速度计的读数，以及参考的真实姿态（如果可用）。
3. 参数调整：根据实验数据的分析，逐步调整Kp和Ki的值。增加Kp可以提高响应速度，但过大的值可能导致不稳定。增加Ki可以补偿陀螺仪漂移，但过大的值可能会引入过度补偿。
4. 观察和评估：观察姿态估计的性能，包括稳定性、准确性和响应速度。根据需要进行进一步的参数调整。
5.迭代优化：根据观察结果，不断迭代调整Kp和Ki的值，直到满足应用需求。

*/
// Header files
#include <math.h>
#define twoKpDef	(2.0f * 300.0f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 300.0f)	// 2 * integral gain
class Mahony{
private:
    float sampleFreq = 200.0f;
    volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
    volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
    volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
    volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki
    float roll;
    float pitch;
    float yaw;
    char anglesComputed;
public:
    void begin(float freq ) { sampleFreq = freq; }
    float invSqrt(float x) {
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        y = y * (1.5f - (halfx * y * y));
        return y;
    }

    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
        float recipNorm;
        float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
        float hx, hy, bx, bz;
        float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
        float halfex, halfey, halfez;
        float qa, qb, qc;

        // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
        if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
            updateIMU(gx, gy, gz, ax, ay, az);
            return;
        }

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;     

            // Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;   

            // Auxiliary variables to avoid repeated arithmetic
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;   

            // Reference direction of Earth's magnetic field
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = sqrt(hx * hx + hy * hy);
            bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            // Estimated direction of gravity and magnetic field
            halfvx = q1q3 - q0q2;
            halfvy = q0q1 + q2q3;
            halfvz = q0q0 - 0.5f + q3q3;
            halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
        
            // Error is sum of cross product between estimated direction and measured direction of field vectors
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

            // Compute and apply integral feedback if enabled
            if(twoKi > 0.0f) {
                integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
                integralFBy += twoKi * halfey * (1.0f / sampleFreq);
                integralFBz += twoKi * halfez * (1.0f / sampleFreq);
                gx += integralFBx;	// apply integral feedback
                gy += integralFBy;
                gz += integralFBz;
            }
            else {
                integralFBx = 0.0f;	// prevent integral windup
                integralFBy = 0.0f;
                integralFBz = 0.0f;
            }

            // Apply proportional feedback
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }
        
        // Integrate rate of change of quaternion
        gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
        gy *= (0.5f * (1.0f / sampleFreq));
        gz *= (0.5f * (1.0f / sampleFreq));
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx); 
        
        // Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
        anglesComputed = 0;
    }

    //---------------------------------------------------------------------------------------------------
    // IMU algorithm update
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
        float recipNorm;
        float halfvx, halfvy, halfvz;
        float halfex, halfey, halfez;
        float qa, qb, qc;

        // 检查加速度计的测量值是否有效（非零），以避免在加速度计标准化过程中产生NaN值
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

            // 对加速度计的测量值进行标准化，即将其转化为单位向量
            // 将其转化为单位向量。这是通过计算加速度计测量值的模，并将其倒数存储在recipNorm变量中。
            // 然后，将加速度计的每个分量乘以recipNorm，以获得标准化后的加速度计测量值
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;        

            // 计算重力的估计方向
            // halfvx、halfvy、halfvz所表示的向量就是重力在设备局部坐标系中的估计方向
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5f + q3 * q3;
        
            // 计算估计方向和测量重力方向之间的误差项
            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            // 应用积分反馈
            if(twoKi > 0.0f) {
                integralFBx += twoKi * halfex * (1.0f / sampleFreq); // integral error scaled by Ki
                integralFBy += twoKi * halfey * (1.0f / sampleFreq);
                integralFBz += twoKi * halfez * (1.0f / sampleFreq);
                gx += integralFBx;	// apply integral feedback
                gy += integralFBy;
                gz += integralFBz;
            }
            else {
                integralFBx = 0.0f;	// prevent integral windup
                integralFBy = 0.0f;
                integralFBz = 0.0f;
            }

            // 应用比例反馈 
            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }
        
        // 对四元数的变化率进行积分，以更新四元数的值
        gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
        gy *= (0.5f * (1.0f / sampleFreq));
        gz *= (0.5f * (1.0f / sampleFreq));
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx); 
        
        // 对四元数进行标准化，以确保其模为1
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
        anglesComputed = 0;
    }

    void computeAngles()
    {
        roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
        pitch = asinf(-2.0f * (q1*q3 - q0*q2));
        yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
        anglesComputed = 1;
    }

    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f;
        // return yaw * 57.29578f;
    }
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
};
