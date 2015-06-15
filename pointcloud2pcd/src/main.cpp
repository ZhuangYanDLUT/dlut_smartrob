#include <ros/ros.h>
#include <iostream>
#include <strstream>

#include <nav_msgs/Odometry.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
using namespace std;

class myPC
{
    public:
        myPC()
        {
            sub = n_.subscribe("hokuyo_point_cloud2", 10, &myPC::pointCloud2Callback,this);
            sub_stereo_odom_ = n_.subscribe("/stereo_odometer/odometry", 10, &myPC::stereoCallback,this);
            timer = n_.createTimer(ros::Duration(0.5),&myPC::timerCallback,this);
        }
        ~myPC()
        {
            writer.write("robot.pcd", sum_robot);
            writer.write("stereo.pcd", sum_stereo);
        }
    private:
        void pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void stereoCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void timerCallback(const ros::TimerEvent&);
        ros::NodeHandle n_;
        ros::Subscriber sub;
        ros::Subscriber sub_stereo_odom_;
        ros::Timer timer;
        tf::TransformListener mylistener;
        tf::TransformListener mylistener1;
        tf::StampedTransform Laser2robot;
        tf::StampedTransform Laser2multisense;
        tf::StampedTransform Laser2multisense_odom;

        pcl::PointCloud<pcl::PointXYZ> sum_robot;
        pcl::PointCloud<pcl::PointXYZ> sum_stereo;
        pcl::PCDWriter writer;
};

void myPC::stereoCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    /*
       tf::Transform vo_meas;
       poseMsgToTF(msg->pose.pose,Laser2multisense_odom);

       try{
       (mylistener1.waitForTransform("Laser","odom",ros::Time::now()-ros::Duration(0.1),ros::Duration(0.2)));
       {
       mylistener1.lookupTransform("Laser","odom",ros::Time::now()-ros::Duration(0.1),Laser2multisense);
    //ROS_WARN_THROTTLE(2.0,"Laser2robot: %f,%f,%f",Laser2multisense.getOrigin().x(),Laser2multisense.getOrigin().y(),Laser2multisense.getOrigin().z());
    }
    }
    catch(tf::TransformException ex)
    {
    ROS_WARN_THROTTLE(10.0, "Laser2robot. ex.what:%s",ex.what());
    }
    Laser2multisense_odom *= Laser2multisense.inverse();

    ROS_WARN_THROTTLE(1.0,"Laser2multisense_odom Tran: %f,%f,%f",Laser2multisense_odom.getOrigin().getX(),Laser2multisense_odom.getOrigin().getY(),Laser2multisense_odom.getOrigin().getZ());
    double r,p,y;
    Laser2multisense_odom.getBasis().getEulerYPR(y,p,r);
    ROS_WARN_THROTTLE(1.0,"Laser2multisense_odom RPY: %f,%f,%f",y,p,r);
    */
}

void myPC::pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    std::string robot_name,stereo_name;
    strstream ss;
    ss << ros::Time::now().toNSec();
    ss >> robot_name;
    stereo_name = robot_name;
    robot_name += string("robot.pcd");
    stereo_name += string("stereo.pcd");

    sensor_msgs::PointCloud2 robot_pointcloud;
    sensor_msgs::PointCloud2 stereo_pointcloud;
    pcl_ros::transformPointCloud("Laser", Laser2robot.inverse(), *msg, robot_pointcloud);
    pcl_ros::transformPointCloud("Laser", Laser2multisense.inverse(), *msg, stereo_pointcloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_robot(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_stereo(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(robot_pointcloud,*cloud_robot);
    pcl::fromROSMsg(stereo_pointcloud,*cloud_stereo);

    sum_robot += *cloud_robot;
    sum_stereo += *cloud_stereo;
    writer.write(robot_name, *cloud_robot);
    writer.write(stereo_name, *cloud_stereo);
}

void myPC::timerCallback(const ros::TimerEvent&)
{
    try{
        (mylistener.waitForTransform("Laser_based_on_robot","Laser",ros::Time::now()-ros::Duration(0.1),ros::Duration(0.2)));
        {
            mylistener.lookupTransform("Laser_based_on_robot","Laser",ros::Time::now()-ros::Duration(0.1),Laser2robot);
            ROS_WARN_THROTTLE(2.0,"Laser2robot: %f,%f,%f",Laser2robot.getOrigin().x(),Laser2robot.getOrigin().y(),Laser2robot.getOrigin().z());
        }
    }
    catch(tf::TransformException ex)
    {
        ROS_WARN_THROTTLE(10.0, "Laser2robot. ex.what:%s",ex.what());
    }

    try{
        (mylistener1.waitForTransform("Laser_based_on_stereo","Laser",ros::Time::now()-ros::Duration(0.8),ros::Duration(0.2)));
        {
            mylistener1.lookupTransform("Laser_based_on_stereo","Laser", ros::Time::now()-ros::Duration(0.8),Laser2multisense);

            ROS_WARN_THROTTLE(1.0,"Laser2multisense Tran: %f,%f,%f",Laser2multisense.getOrigin().getX(),Laser2multisense.getOrigin().getY(),Laser2multisense.getOrigin().getZ());
            double r,p,y;
            Laser2multisense.getBasis().getEulerYPR(y,p,r);
            ROS_WARN_THROTTLE(1.0,"Laser2multisense RPY: %f,%f,%f",y,p,r);
        }
    }
    catch(tf::TransformException ex)
    {
        ROS_WARN_THROTTLE(10.0, "Laser2robot. ex.what:%s",ex.what());
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud2pcd");
    ros::NodeHandle n;
    myPC myPC;
    ros::spin();

    return 0;
}
