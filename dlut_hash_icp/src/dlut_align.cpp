/*************************************************************************
  > File Name: dlut_hash_icp.cpp
  > Author: 吴乃亮
  > Mail: wunailiang@gmail.com
  > Created Time: Wed 21 May 2014 10:13:53 AM CST
 ************************************************************************/
#include <strstream>
#include "ros/ros.h"
//用于打开pcd
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include "src/icp.h"
#include "src/net_transfor.h"
#include "boost/thread.hpp"
#include <queue>

using namespace std;
boost::mutex io_mutex;
typedef boost::unordered_map<unordered_map_voxel,un_key> umap;
std::queue<pcl::PointCloud<pcl::PointXYZ> > point_queue;
pcl::PCDWriter writer;

void pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    cout<<"into callback"<<endl;
    static int i = 0;
    std::string temp_filename;
    strstream ss;
    ss << i;
    ss >> temp_filename;
    temp_filename+=".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_robot(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg,*cloud_robot);
    writer.write(temp_filename, *cloud_robot);
    {
        boost::mutex::scoped_lock lock(io_mutex);
        point_queue.push(*cloud_robot);
    }
    i++;
}

void get_data()
{
    Voxelize* voxelize1 = new Voxelize;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local(new pcl::PointCloud<pcl::PointXYZ>);//local map. contain a 360' pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local_temp(new pcl::PointCloud<pcl::PointXYZ>);//local map. contain a 360' pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global(new pcl::PointCloud<pcl::PointXYZ>);//global map
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global_raw(new pcl::PointCloud<pcl::PointXYZ>);//global map

    int marker = 0;
    int num_of_scan = 0;
    boost::unordered_map<unordered_map_voxel,un_key> m1;
    boost::unordered_map<unordered_map_voxel,un_key> m2;
    Eigen::Matrix4f tf_sum;
    tf_sum<<1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,0;
    while(ros::ok())
    {
        if(point_queue.empty())
        {
            continue;
        }
        //live time
        {
            boost::mutex::scoped_lock lock(io_mutex);
            *cloud = point_queue.front();
            point_queue.pop();
        }

        if(marker==0)
        {
            marker=1;

            pcl::copyPointCloud(*cloud,*cloud_global);
            pcl::copyPointCloud(*cloud,*cloud_global_raw);
            swap(*cloud,*cloud_local);
            cloud->clear();
            voxelize1->generateUmap(*cloud_local,0.3,m2);
            continue;
        }
        *cloud_global_raw += *cloud;
        m1.clear();

        voxelize1->generateUmap(*cloud,0.3,m1);
        Icp icp(m1,m2,*cloud,0);
        Eigen::Matrix4f tf_mat;
        tf_mat = icp.icpFit();
        tf_sum+=tf_mat;
        num_of_scan++;
        pcl::transformPointCloud(*cloud,*cloud,tf_mat);
        *cloud_local_temp += *cloud;

        cloud_local->swap(*cloud_local_temp);
        cloud_local_temp->clear();
        m2.clear();
        voxelize1->generateUmap(*cloud_local,0.3,m2);
        *cloud_global += *cloud;

    }

    pcl::PCDWriter writer;
    if(cloud_global->size()!=0)
    {
        writer.write("global_map.pcd",*cloud_global);
        writer.write("global_map_raw.pcd",*cloud_global_raw);
    }
    delete voxelize1;
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"dlut_align");
    ros::NodeHandle n;
    ros::Subscriber sub;
    sub = n.subscribe("hokuyo_point_cloud2", 10, pointCloud2Callback);

    boost::thread getdata(&get_data);

    ros::spin();
    sleep(4);
    cout << "hello icp" <<endl;
    return 0;
}
