#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include "utility.h"
#include "Hardware/can.h"
#include "Hardware/motor.h"
#include "Hardware/teleop.h"
#include "App/arm_control.h"
#include "App/arm_control.cpp"
#include "App/keyboard.h"
#include "App/play.h"
#include "App/solve.h"
#include "App/arx_s.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <atomic>
#include <signal.h>
#include "Ecat/ecat_base.hpp"
#include "Ecat/ecat_typedef.hpp"
#include "arm_control/JointControl.h"
#include "arm_control/JointInformation.h"
#include "arm_control/PosCmd.h"

char phy[] = "enx00e04c36117b";
int CONTROL_MODE=0; 
command cmd;

bool app_stopped = false;
void sigint_handler(int sig);
void safe_stop();

ecat::EcatBase Ethercat(1);
can CAN_Handlej;


float calc_cur[7]={};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm2"); 
    ros::NodeHandle node;
    Teleop_Use()->teleop_init(node);

    arx_arm ARX_ARM((int) CONTROL_MODE);

    ros::Subscriber sub_joint = node.subscribe<sensor_msgs::JointState>("/master/joint_right", 100, 
                                  [&ARX_ARM](const sensor_msgs::JointState::ConstPtr& msg) 
                                  {
                               
                                      ARX_ARM.ros_control_pos_t[0] = msg->position[0];
                                      ARX_ARM.ros_control_pos_t[1] = msg->position[1];
                                      ARX_ARM.ros_control_pos_t[2] = msg->position[2];
                                      ARX_ARM.ros_control_pos_t[3] = msg->position[3];
                                      ARX_ARM.ros_control_pos_t[4] = msg->position[4];
                                      ARX_ARM.ros_control_pos_t[5] = msg->position[5];
                                      ARX_ARM.ros_control_pos_t[6] = msg->position[6];

                                  });
    
     ros::Publisher pub_joint02 = node.advertise<sensor_msgs::JointState>("/puppet/joint_right", 100);

    arx5_keyboard ARX_KEYBOARD;

    ros::Rate loop_rate(200);

    std::thread keyThread(&arx5_keyboard::detectKeyPress, &ARX_KEYBOARD);
    sleep(1);
    Ethercat.EcatStart(phy);
    CAN_Handlej.Enable_Moto(Ethercat,1);
    CAN_Handlej.Enable_Moto(Ethercat,1);
    CAN_Handlej.Enable_Moto(Ethercat,2);
    CAN_Handlej.Enable_Moto(Ethercat,4);
    CAN_Handlej.Enable_Moto(Ethercat,5);
    CAN_Handlej.Enable_Moto(Ethercat,6);
    CAN_Handlej.Enable_Moto(Ethercat,7);
    CAN_Handlej.Enable_Moto(Ethercat,8);
    Ethercat.EcatSyncMsg(); CAN_Handlej.can0_ReceiveFrame(Ethercat); 

    while(ros::ok())
    { 

        Ethercat.packet_tx[0].LED = 1;
        Ethercat.packet_tx[0].LED = 1;
        Ethercat.EcatSyncMsg();

        char key = ARX_KEYBOARD.keyPress.load();
        ARX_ARM.getKey(key);

        ARX_ARM.get_curr_pos();
        if(!ARX_ARM.is_starting){
             cmd = ARX_ARM.get_cmd();
        }
        ARX_ARM.update_real(Ethercat,cmd);

        sensor_msgs::JointState msg_joint01;
        msg_joint01.header.stamp = ros::Time::now();
  
        size_t num_joint = 7;
        msg_joint01.name.resize(num_joint);
        msg_joint01.velocity.resize(num_joint);
        msg_joint01.position.resize(num_joint);
        msg_joint01.effort.resize(num_joint);
        
        for (size_t i=0; i < 7; i++)
        {   
            msg_joint01.name[i] = "joint" + std::to_string(i);
            msg_joint01.position[i] = ARX_ARM.current_pos[i];
            msg_joint01.velocity[i] = ARX_ARM.current_vel[i];
            msg_joint01.effort[i] = ARX_ARM.current_torque[i];
        }
        pub_joint02.publish(msg_joint01);    
        ros::spinOnce();
        loop_rate.sleep();
        arx_1();

    }

    arx_2(CAN_Handlej,Ethercat);

    return 0;
}