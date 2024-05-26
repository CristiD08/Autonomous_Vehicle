#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "MPU6050.h"
#include "config.h"
//-----------------includ mesajele ROS-----------------------
#include "mbs_msgs/Velocities.h"
#include "mbs_msgs/RawImu.h"
#include "mbs_msgs/PID.h"

#include <Wire.h>
#include <TimerFive.h>
#include "Mecanum_Car.hpp"

#define ENCODER_OPTIMIZE_INTERRUPTS

#define IMU_PUBLISH_RATE 10 //hz
#define VEL_PUBLISH_RATE 10 //hz
#define COMMAND_RATE 10 //hz

DC_Motor   LF_Wheel_Motor(PORT_LF), RF_Wheel_Motor(PORT_RF),LR_Wheel_Motor(PORT_LR), RR_Wheel_Motor(PORT_RR);
Mecanum_Car  Robot(&LF_Wheel_Motor, &RF_Wheel_Motor, &LR_Wheel_Motor, &RR_Wheel_Motor);
MPU6050 Mpu6050;

double g_req_angular_vel_z = 0;
double g_req_linear_vel_x = 0;
double g_req_linear_vel_y = 0;

unsigned long g_prev_command_time = 0;
unsigned long g_prev_control_time = 0;
unsigned long g_publish_vel_time = 0;
unsigned long g_prev_imu_time = 0;


bool g_is_first = true;

// functii callback
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const mbs_msgs::PID& pid);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<mbs_msgs::PID> pid_sub("pid", PIDCallback);

mbs_msgs::Imu  raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

mbs_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void PID_control()
{
  Robot.Increment_PID();  
}

void setup()
{
    nh.initNode();
    nh.getHardware()->setBaud(115200);
    nh.subscribe(pid_sub);
    nh.subscribe(cmd_sub);
    nh.advertise(raw_vel_pub);
    nh.advertise(raw_imu_pub);
    Mpu6050.initialize();           
    Timer5.initialize(0);
    Timer5.attachInterrupt(PID_control); 
    Robot.Init();
    nh.spinOnce();

    nh.loginfo("Ros Connected!");
    
    Wire.begin();
    delay(5);

}


void loop()
{

    if ((millis() - g_prev_control_time) >= (1000 / COMMAND_RATE))
    {
        Robot.ROS_MoveBase(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
        g_prev_control_time = millis();  
    }
    
    if ((millis() - g_prev_command_time) >= 4000)
    {
        stopBase();
    }
    
    if ((millis() - g_publish_vel_time) >= (1000 / VEL_PUBLISH_RATE))
    {
        publishVelocities();
        g_publish_vel_time = millis();
    }
    
    if ((millis() - g_prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
    {
        if (g_is_first)
        {
            Robot.ClearOdom();
            g_is_first = false;
        }
        else
        {
            publishIMU();
        }
        g_prev_imu_time = millis();
    }
    
    nh.spinOnce();
    
}

void PIDCallback(const mbs_msgs::PID& pid)
{

    Robot.Update_PID(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
  
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = cmd_msg.linear.y;
    g_req_angular_vel_z = cmd_msg.angular.z;

    g_prev_command_time = millis();
}

void stopBase()
{
    g_req_linear_vel_x = 0.0;
    g_req_linear_vel_y = 0.0;
    g_req_angular_vel_z = 0.0;
}

void publishVelocities()
{
    long LF_Wheel_Spd = LF_Wheel_Motor.getMotorRPM();
    long RF_Wheel_Spd = RF_Wheel_Motor.getMotorRPM();
    long LR_Wheel_Spd = LR_Wheel_Motor.getMotorRPM();
    long RR_Wheel_Spd = RR_Wheel_Motor.getMotorRPM();
    
    double average_rpm_x = (LF_Wheel_Spd + LR_Wheel_Spd + RF_Wheel_Spd + RR_Wheel_Spd) / 4;
    raw_vel_msg.linear_x = (average_rpm_x * (0.068 * PI)); 
    
    double average_rpm_y = (-LF_Wheel_Spd + LR_Wheel_Spd + RF_Wheel_Spd - RR_Wheel_Spd) / 4;
    raw_vel_msg.linear_y = (average_rpm_y * (0.068 * PI)); 
    
    double average_rpm_a = (-LF_Wheel_Spd + LR_Wheel_Spd - RF_Wheel_Spd + RR_Wheel_Spd) / 4;
    raw_vel_msg.angular_z =  (average_rpm_a * (0.068 * PI)) / 0.26;
    
    
    raw_vel_pub.publish(&raw_vel_msg);
}

void publishIMU()
{
  int16_t ax, ay, az, gx, gy, gz;             
  
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  

  raw_imu_msg.header.stamp = nh.now();
  raw_imu_msg.header.frame_id = "imu_link";

  raw_imu_msg.raw_linear_acceleration.x = (float)ax / 682.072;
  raw_imu_msg.raw_linear_acceleration.y = (float)ay / 682.072;
  raw_imu_msg.raw_linear_acceleration.z = (float)az / 682.072;

  raw_imu_msg.raw_angular_velocity.x = (float)gx / 1024.0;
  raw_imu_msg.raw_angular_velocity.y = (float)gy / 1024.0;
  raw_imu_msg.raw_angular_velocity.z = (float)gz / 1024.0;

  raw_imu_pub.publish(&raw_imu_msg);

  }


