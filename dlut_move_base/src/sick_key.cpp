/*********************************************************************
 * Software License Agreement (BSD License)
 * Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. and Intelligent Robotics 
 *        Lab, DLUT nor the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific prior written 
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#include <ros/ros.h>
#include <dlut_move_base/Velocity.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <std_msgs/Bool.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20
#define KEYCODE_0 0x30
#define KEYCODE_ENTER 0x0A

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop(dlut_move_base::Velocity &vel);

private:  
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Publisher is_run_pub1_,is_run_pub2_;
  std_msgs::Bool is_motor_run_;  
};

TeleopTurtle::TeleopTurtle():
  linear_(0.0),
  angular_(0.0),
  l_scale_(0.1),
  a_scale_(0.1)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<dlut_move_base::Velocity>("cmd_velocity", 1);
  is_run_pub1_ = nh_.advertise<std_msgs::Bool>("is_run", 1000);
  is_run_pub2_ = nh_.advertise<std_msgs::Bool>("is_run_pch", 1000);
}

int g_kfd = 0;
struct termios g_cooked, g_raw;

void quit(int sig)
{
  tcsetattr(g_kfd, TCSANOW, &g_cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_key");
  TeleopTurtle teleop_turtle;
  dlut_move_base::Velocity vel;
  vel.angular = 0.0;
  vel.linear = 0.0;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop(vel);
  
  return(0);
}

void TeleopTurtle::keyLoop(dlut_move_base::Velocity &vel)
{
  char c;                         //the value of the key pressed
  bool is_control_command = false;//is control command or not
  bool is_data_command = false;   //is data command or not

  // get the console in g_raw mode                                                              
  tcgetattr(g_kfd, &g_cooked);
  memcpy(&g_raw, &g_cooked, sizeof(struct termios));
  g_raw.c_lflag &= ~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  g_raw.c_cc[VEOL] = 1;
  g_raw.c_cc[VEOF] = 2;
  tcsetattr(g_kfd, TCSANOW, &g_raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");

  //the robot stopped moving or not
  bool is_stop_move = 0;

  //the robot begin getting the laser data or not
  int begin_get_data = 0; 

  //the robot stop getting the laser data or not
  int stop_get_data = 0;  

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(g_kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_ = angular_ = 0.0;
    ROS_DEBUG("value: 0x%02X\n", c);
 
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        is_control_command = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        is_control_command = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 1.0;
        is_control_command = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        is_control_command = true;
	break;
      case KEYCODE_SPACE:
	ROS_DEBUG("STOP");
	is_stop_move = 1;
        is_control_command = true;
        break;
      case KEYCODE_0:
	ROS_DEBUG("BEGIN");
	begin_get_data = 1;
        is_data_command = true;
        break;
      case KEYCODE_ENTER:
	ROS_DEBUG("END");
	stop_get_data = 1;
        is_data_command = true;
        break;
      default :
	break;	
    }

    vel.angular = vel.angular + a_scale_*angular_;
    vel.linear = vel.linear + l_scale_*linear_;

    //add the following two if in case the function gcvt convert 0 to a numner very near to 0
    if(vel.angular > -0.001 && vel.angular < 0.001)
    {
      vel.angular = 0.0;	
    }

    if(vel.linear > -0.001 && vel.linear < 0.001)//the robot runs faster at 0.05 than 								at 0.1,it's abnormal
    {
      vel.linear = 0.0;	
    }

    if(is_stop_move == 1)
    {
      vel.angular = 0.0;
      vel.linear = 0.0;	
      is_stop_move = 0;	
    }

    if(begin_get_data == 1)
    {
      is_motor_run_.data = true;
      begin_get_data = 0;
    }

    if(stop_get_data == 1)
    {
      is_motor_run_.data = false;
      stop_get_data = 0;
    }

    if(is_control_command == true)//is control command, so publish it
    {
      vel_pub_.publish(vel); 
      is_control_command = false;
    }

    if(is_data_command == true)
    {
      is_run_pub1_.publish(is_motor_run_);//tell the motor under the sick to rotate
      is_run_pub2_.publish(is_motor_run_);//publish the signal to save the laser data or not
      is_data_command = false;
    }
  }

  return;
}
