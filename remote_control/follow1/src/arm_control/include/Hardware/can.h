#ifndef _CAN_H_
#define _CAN_H_

#include <stdint.h>
#include <string.h>
#include <iostream>
#include "../../include/Hardware/motor.h"
#include "../../include/Ecat/ecat_base.hpp"
#include "../../include/Ecat/ecat_typedef.hpp"
#include "../../include/utility.h"

#include <ros/ros.h>
#include <ros/package.h>
#include "utility.h"

typedef struct
{
    float imu[3];  //单位 rad    顺序yaw pitch roll
    float last_imu[3]; //上一时刻的imu角度数据
    float gyro[3]; //单位 rad/s  顺序pitch  roll  yaw
} IMU_Float_t;



typedef struct{
    int8_t   temperature;
    int16_t	 speed_rpm;
    int16_t  real_current;
    uint16_t position;
    int8_t   round_cnt;
    float    total_angle;
    float    total_angle_last;
	
}m_rmd_t;


class can
{
public:
    can();
    ~can();
    float target_pos_temp1[7]={0};
    void can0_ReceiveFrame(ecat::EcatBase ecat_base);
    void Enable_Moto(ecat::EcatBase ecat_base,uint16_t ID);
    void Send_moto_Cmd1(ecat::EcatBase ecat_base, uint16_t motor_id, float kp, float kd, float pos, float spd, float tor);
    void Send_moto_Cmd2(ecat::EcatBase ecat_base, uint16_t motor_id, float kp, float kd, float pos, float spd, float tor);
    float control(float*target_pos,bool& motor_control_cmd,bool& is_teach_mode,bool& current_normal,float*joint_torque ,ecat::EcatBase ecat_base,float* current_pos);
    float ramp(float goal, float current, float ramp_k);

};

#endif