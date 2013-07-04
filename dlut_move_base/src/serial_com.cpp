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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dlut_move_base/Velocity.h"
#include "dlut_move_base/Twist.h"

//the serial port
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>

#define DEFAULT_PORT 2
#define DEFAULT_BAUDRATE 9600

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <fstream>

class SerialCom
{
public:
  SerialCom();
  ~SerialCom();
  int openPort(int fd, int comport);
  int setOpt(int fd,int nspeed,int nbits,char nevent,int nstop);
  int packSend(int fd,char ptr,char buff[]);
  int packGet(int fd,char ptr);
  void comCallBack(const dlut_move_base::Velocity &vel);//send the control cmd to the port by Velocity
  void velSend(const dlut_move_base::Twist &tw);//send the control cmd to the port by Twist
  int comInit();
 
  dlut_move_base::Twist twist_;//the velocity we get from the robot.
  void timerPackGet(const ros::TimerEvent &);
  ros::NodeHandle serial_node_handle_;

private:
  int fd_;//the file discriptor of the serial port

  ros::Subscriber sub_cmd_vel_,sub_cmd_twist_; 
  ros::Publisher vel_pub_;//get the v and w from the serial of the robot.
  ros::Publisher odom_pub_;//output the odom of the robot by v and w got from the serial

  tf::TransformBroadcaster odom_broadcaster_;//publish the odom transform

  int port_,baudrate_;

  //the initial pose of the robot
  double x_;
  double y_;
  double th_;

  //the v and w of the robot
  double vx_;
  double vy_;
  double vth_;

  ros::Time current_time_,last_time_;
};

SerialCom::SerialCom()
{
  sub_cmd_vel_ = serial_node_handle_.subscribe("cmd_velocity", 1000, &SerialCom::comCallBack,this);
  sub_cmd_twist_ = serial_node_handle_.subscribe("cmd_twist", 1000, &SerialCom::velSend,this);
  vel_pub_ = serial_node_handle_.advertise<dlut_move_base::Twist>("robot_velocity", 1);

  serial_node_handle_.param("Port",port_,DEFAULT_PORT);
  serial_node_handle_.param("BaudRate",baudrate_,DEFAULT_BAUDRATE);

  twist_.angular.z=0.0;
  twist_.linear.x=0.0;

  comInit();
 
  x_ = 0.0;
  y_ = 0.0;
  th_ = 0.0;

  vx_ = 0.0;
  vy_ = 0.0;
  vth_ = 0.0;

  current_time_ = ros::Time::now();
  last_time_ = ros::Time::now();
  
  odom_pub_ = serial_node_handle_.advertise<nav_msgs::Odometry>("odom", 50);
}

SerialCom::~SerialCom()
{
  close(fd_);
}

//open the serial port
int SerialCom::openPort(int fd, int comport)
{
  if(comport == 1)//port 1
  {
    fd = open("/dev/ttyS0",O_RDWR|O_NOCTTY|O_NDELAY);
    
    if(-1  ==  fd)
    {
      ROS_INFO("failed to open port 1！");
      return(-1);
    }
  }
  else if(comport == 2)//port 2
  {
    fd = open("/dev/ttyS1",O_RDWR|O_NOCTTY|O_NDELAY);
    if(-1 == fd)
    {
      ROS_INFO("failed to open port 2！");
      return(-1);
    }
  }
  else if(comport == 3)//port 3
  {
    fd = open("/dev/ttyS2",O_RDWR|O_NOCTTY|O_NDELAY);

    if(-1 == fd)
    {
      ROS_INFO("failed to open port 3！");
      return(-1);
    }
  }

  if(fcntl(fd,F_SETFL,0)<0)
    ROS_INFO("fcntl failed!");
  else
    ROS_INFO("fcntl=%d.",fcntl(fd,F_SETFL,0));

  if(isatty(STDIN_FILENO) == 0)
    ROS_INFO("standsrd input is not a terminal device.");
  else
    ROS_INFO("is a tty sucess.");

  ROS_INFO("the return value of the serial open function=%d，!=-1,indicates succeed to open the serial.",fd);

  return fd;
}

//setup the serial port
int SerialCom::setOpt(int fd,int nspeed,int nbits,char nevent,int nstop)
{
  struct termios newtio,oldtio;

  //check the parameter of the serial port to see whether there is error or not
  if(tcgetattr(fd,&oldtio) != 0)
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
  tcflush(fd,TCIFLUSH);

  //activate the new serial setup
  if((tcsetattr(fd,TCSANOW,&newtio))!=0)
  {
    ROS_INFO("failed to activate the new serial setup！");
    return -1;
  }

  ROS_INFO("serial setup success!\n");

  return 0;
}
/*
a pack of data consists of four parts:
1.the start sign: "*"
2.the data sign: 
  "l":the velocity of the left wheel
  "r":the velocity of the right wheel
  "v":the velocity of the robot
  "w":the angular velocity of the robot
3.the length of the data:
4.the data
*/
int SerialCom::packSend(int fd,char ptr,char buff[])
{
  int nwrite;	

  //write the start sign
  char begin[2];
  begin[0] = '*';
  nwrite = write(fd,begin,1);
  begin[nwrite] = '\0';
 
  //write the data sign
  char sign[2];//the sign of the pack，'l'、'r'、'v' or 'w'
  if(ptr == 'l' || ptr == 'r' || ptr == 'v' || ptr == 'w')
  {
    sign[0] = ptr;
    sign[1] = '\0';
    nwrite = write(fd,sign,1);		
  }
  else
  {
    ROS_INFO("wrong parameter!");
    return 0;
  }

  //write the length of the data
  char len[2];
  len[0] = strlen(buff)+'0';
  len[1] = '\0';
  if(buff[0] != '-')//if the data is minus,add 1 to the length
  {
    len[0]++;
  }
  nwrite = write(fd,len,1);
  ROS_INFO("len[0]=%d.\n",len[0]);

  //write the data to the port
  if(buff[0] != '-')
  {
    nwrite = write(fd,"+",1);//写串口
  }
  nwrite = write(fd,buff,strlen(buff));//写串口
  buff[nwrite] = '\0';
  
  return 0;
}

/*
get a pack of data which contains v and w of the robot from the serial port
*/
int SerialCom::packGet(int fd,char ptr)
{
  int nwrite,nread;
  char buff[10]="\0";
  if(ptr == 'v')
  {
    nwrite=write(fd,"?v",2);
  }
  else if(ptr == 'w')
  {
    nwrite=write(fd,"?w",2);  
  }
  else if(ptr == 'l')
  {
    nwrite=write(fd,"?l",2); 
  }
  else if(ptr == 'r')
  {
    nwrite=write(fd,"?r",2);    
  }
  else if(ptr == 'a')
  {
    nwrite=write(fd,"?v",2);
    nwrite=write(fd,"?w",2);
    nwrite=write(fd,"?l",2);
    nwrite=write(fd,"?r",2);    
  }
  else
  {
    ROS_INFO("the parameter of the function packGet must be ‘v’、‘w’、‘l’、‘r’ or 'all'!");
    return false;
  }
  
  if(nwrite < 0)
  {
    ROS_INFO("send data error！");	
  }
  
  while(1)
  {
    if((nread = read(fd,buff,8)) == 8)
    {
      buff[8]='\0';
      if(buff[0] != '*' || buff[2] != '5')
      {
        ROS_INFO("The pack we get from the robot isn't right!(the start sign or the length of the data is not right!)");
      }
      else if(buff[1] == 'v' || buff[1] == 'w' || buff[1] == 'l' || buff[1] == 'r')
      {
        switch(buff[1])
        {
	  case 'v':
            ROS_INFO("the velocity of the robot V=");
            twist_.linear.x=atof(&buff[3]);
            break;
          case 'w':
            ROS_INFO("the angle velocity of the robot W=");
            twist_.angular.z=atof(&buff[3]);
            break;
          case 'l':
            ROS_INFO("the velocity of the left wheel=");
            break;
          case 'r':
            ROS_INFO("the velocity of the right wheel=");
            break;
          default:
            ROS_INFO("the sign is wrong!");	
        }

        ROS_INFO("%s.",&buff[3]);	
        break;				
      }
    }
  }

  return true;
}

void SerialCom::comCallBack(const dlut_move_base::Velocity &vel)
{
  char pl[10];
  char pa[10];

  ROS_INFO("linear: [%f] angular: [%f].",vel.linear,vel.angular);	
  ROS_INFO("linear: [%s] angular: [%s].",gcvt(vel.linear,6,pl),gcvt(vel.angular,6,pa));

  packSend(fd_,'v',pl);//send the value of the velocity
  packSend(fd_,'w',pa);//send the value of the angular velocity
}

void SerialCom::velSend(const dlut_move_base::Twist &tw)
{
  char pl[10];
  char pa[10];	

  double vel_linear,vel_angular;
  vel_linear = tw.linear.x;
  vel_angular = tw.angular.z;

  ROS_INFO("linear: [%f] angular: [%f].",vel_linear,vel_angular);

  ROS_INFO("linear: [%s].",gcvt(vel_linear,6,pl));
  ROS_INFO("angular: [%s].",gcvt(vel_angular,6,pa));

  packSend(fd_,'v',pl);//send the value of the velocity
  packSend(fd_,'w',pa);//send the value of the angular velocity
  ROS_INFO("velSend function called!");
}

int SerialCom::comInit()
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

  ROS_INFO("the serial openned，setup the serial successed。the file operator=%d",fd_); 
  
  return 0;
}
void SerialCom::timerPackGet(const ros::TimerEvent &)
{
  ROS_INFO("timerPackGet function called!");	
  packGet(fd_,'v');//get the value of the velocity from the serial port
  packGet(fd_,'w');//get the value of the angular velocity from the serial port	
  vel_pub_.publish(twist_);//in fact,we can delete this line,but we can publish the v and w if not.

  vx_ = twist_.linear.x;
  vy_ = 0.0;
  vth_ = twist_.angular.z;
  ros::Time current_time_ = ros::Time::now();

  double dt = (current_time_ - last_time_).toSec();
  double delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
  double delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
  double delta_th = vth_ * dt;

  x_ += delta_x;
  y_ += delta_y;
  th_ += delta_th;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time_;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster_.sendTransform(odom_trans);
  //odom_broadcasterPtr->sendTransform(odom_trans);

  nav_msgs::Odometry odom;
  odom.header.stamp = current_time_;
  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx_;
  odom.twist.twist.linear.y = vy_;
  odom.twist.twist.angular.z = vth_;

  ROS_INFO("the pose of the robot =%f,%f,%f.",x_,y_,th_);
  ROS_INFO("the v and of the robot =%f.",vx_);
  ROS_INFO("the w and of the robot =%f.",vth_);
       
  odom_pub_.publish(odom);

  last_time_ = current_time_;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serial_com");

  SerialCom serial;
  ros::Timer timer1=serial.serial_node_handle_.createTimer(ros::Duration(0.025),&SerialCom::timerPackGet,&serial);

  ros::spin();

  return 0;
}

