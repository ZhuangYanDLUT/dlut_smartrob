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

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include "time.h" 
#include <math.h>

#include <ros/ros.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <dlut_motor_hokuyo/motor.h>

#define DEFAULT_PORT 1
#define DEFAULT_BAUDRATE 19200
#define DEFAULT_DIRECTION 0 //counterclockwise
#define DEFAULT_FRE 1000
#define DEFAULT_ANGEL 720

using namespace std;
class SickMotor
{
public:
  SickMotor();
  ~SickMotor();
  int openPort(int fd_, int comport);//open the serial port
  int setOpt(int fd_,int nspeed,int nbits,char nevent,int nstop);//config the serial port
  int getPulseNum(int _degree);//calculate the number of the pulse according to the rotate degree of the motor
  //void getDirection(char* _dir,char &motor_dir_);
  void delayms(long int i);//the delay function,the unit is ms
  int comInit();

  void sickMotorCallBack(const std_msgs::Bool &is_run);
  ros::NodeHandle sick_motor_nh_;
  char motor_dir_;//indicate the rotating direction of the sick

  std_msgs::Bool is_motor_run_;

  ros::Subscriber is_motor_run_sub_;
  ros::Publisher motor_pub_;
private:
  int port_,baudrate_;
  int fd_;
  int nwrite_; 

  char command_buff_[30];	
};

SickMotor::SickMotor()
{
  sick_motor_nh_.param("Port",port_,DEFAULT_PORT);//get the value of port_ from parameter server
  sick_motor_nh_.param("BaudRate",baudrate_,DEFAULT_BAUDRATE);
  comInit();
  is_motor_run_sub_ = sick_motor_nh_.subscribe("is_run", 1, &SickMotor::sickMotorCallBack,this);
  motor_pub_ = sick_motor_nh_.advertise<dlut_motor_hokuyo::motor>("motor_param",1000);
  motor_dir_ = '0';//counterclockwise rotating direction
  is_motor_run_.data = false;
}

SickMotor::~SickMotor()
{
    close(fd_);
}

//open the serial port
int SickMotor::openPort(int fd_, int comport)
{
  if(comport == 1)//port 1
  {
    fd_ = open("/dev/ttyS0",O_RDWR|O_NOCTTY|O_NDELAY);
    
    if(-1  ==  fd_)
    {
      ROS_INFO("failed to open port 1！");
      return(-1);
    }
  }
  else if(comport == 2)//port 2
  {
    fd_ = open("/dev/ttyS1",O_RDWR|O_NOCTTY|O_NDELAY);
    if(-1 == fd_)
    {
      ROS_INFO("failed to open port 2！");
      return(-1);
    }
  }
  else if(comport == 3)//port 3
  {
    fd_ = open("/dev/ttyS2",O_RDWR|O_NOCTTY|O_NDELAY);
	
    if(-1 == fd_)
    {
      ROS_INFO("failed to open port 3！");
      return(-1);
    }
  }

  if(fcntl(fd_,F_SETFL,0) < 0)
    ROS_INFO("fcntl failed!");
  else
    ROS_INFO("fcntl=%d.",fcntl(fd_,F_SETFL,0));

  if(isatty(STDIN_FILENO) == 0)
    ROS_INFO("standsrd input is not a terminal device.");
  else
    ROS_INFO("is a tty sucess.");

  ROS_INFO("the return value of the serial open function=%d，!=-1,indicates succeed to open the serial.",fd_);
	
  return fd_;
}

//setup the serial port
int SickMotor::setOpt(int fd_,int nspeed,int nbits,char nevent,int nstop)
{
  struct termios newtio,oldtio;

  //check the parameter of the serial port to see whether there is error or not
  if(tcgetattr(fd_,&oldtio) != 0)
  {
    ROS_INFO("failed to setup the serial，failed to save the serial value!");
    return -1;	
  }

  bzero(&newtio,sizeof(newtio));

  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  switch(nbits)
  {
    case 7:
      newtio.c_cflag |= CS7;
      break;
    case 8:
      newtio.c_cflag |= CS8;
      break;	
  }

  switch(nevent)
  {
    case 'O': 
      newtio.c_cflag |= PARENB;
      newtio.c_cflag |= PARODD;
      newtio.c_iflag |= (INPCK | ISTRIP);
      break;
    case 'E':
      newtio.c_iflag |= (INPCK | ISTRIP);
      newtio.c_cflag |= PARENB ;
      newtio.c_cflag &= ~ PARODD;
      break;
    case 'N':
      newtio.c_cflag &= ~ PARENB;
      break;		
  }

  //setup the baud rate
  switch(nspeed)
  {
    case 2400:
      cfsetispeed(&newtio,B2400);
      cfsetospeed(&newtio,B2400);
      break;		
    case 4800:
      cfsetispeed(&newtio,B4800);
      cfsetospeed(&newtio,B4800);
      break;	
    case 9600:
      cfsetispeed(&newtio,B9600);
      cfsetospeed(&newtio,B9600);
      break;	
    case 19200:
      cfsetispeed(&newtio,B19200);
      cfsetospeed(&newtio,B19200);
      break;	
    case 38400:
      cfsetispeed(&newtio,B38400);
      cfsetospeed(&newtio,B38400);
      break;
    case 57600:
      cfsetispeed(&newtio,B57600);
      cfsetospeed(&newtio,B57600);
      break;		
    case 115200:
      cfsetispeed(&newtio,B115200);
      cfsetospeed(&newtio,B115200);
      break;	
    case 460800:
      cfsetispeed(&newtio,B460800);
      cfsetospeed(&newtio,B460800);
      break;	
    default:
      cfsetispeed(&newtio,B9600);
      cfsetospeed(&newtio,B9600);
      break;		
  }

  //setup the stop bit
  if(nstop == 1)
    newtio.c_cflag &= ~ CSTOPB;
  else if(nstop == 2)
    newtio.c_cflag |= CSTOPB;
	
  //setup the waitting time and the minimum amount of the characters received 
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  //deal with the characters which were not received.
  tcflush(fd_,TCIFLUSH);

  //activate the new serial setup
  if((tcsetattr(fd_,TCSANOW,&newtio)) != 0)
  {
    ROS_INFO("failed to activate the new serial setup！");
    return -1;
  }
	
  ROS_INFO("serial setup success!\n");

  return 0;
}

int SickMotor::getPulseNum(int _degree)
{
   return (_degree*8240/360);

}
/*
void SickMotor::getDirection(char* _dir,char &motor_dir_)
{
  if(strcmp(_dir,"0") == 0)
  {
    motor_dir_ = '0';
  }
  
  if(strcmp(_dir,"1") == 0)
  {
    motor_dir_ = '1';
  }
}
*/
void SickMotor::delayms(long int i)
{
  usleep(1000*i);
}

int SickMotor::comInit()
{
  int i;

  if((fd_ = openPort(fd_,port_)) < 0)
  {
    ROS_INFO("failed to setup the serial！");
    return 0;
  }

  if((i = setOpt(fd_,baudrate_,8,'N',1)) < 0)
  {
    ROS_INFO("failed to setup the serial！");
    return 0;
  }

  ROS_INFO("the serial openned，setup the serial successed。the file operator = %d\n",fd_); 
  
  return 0;
}

void SickMotor::sickMotorCallBack(const std_msgs::Bool &is_run)
{
  is_motor_run_.data = is_run.data;
  if(is_motor_run_.data == true)
  {
    int motor_pulseNum = getPulseNum(DEFAULT_ANGEL); 
    sprintf(command_buff_,"M%c%06d%06d",motor_dir_,DEFAULT_FRE,motor_pulseNum);//generate the control command
    ROS_INFO("%s\n",command_buff_);
    nwrite_ = write(fd_,command_buff_,strlen(command_buff_));//send the control command to the serial port
    if(nwrite_ < 0)
    {
      ROS_INFO("serial send data error！\n");	
    }
  }
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "SickMotor");   
  SickMotor sm;

  sleep(1);  
  dlut_motor_hokuyo::motor motor_param;
  motor_param.motor_freq.data = DEFAULT_FRE;
  motor_param.num_of_pulse_per_circle.data = 8240;
  sm.motor_pub_.publish(motor_param);

  ros::spin();	
  
  return 0;
}
