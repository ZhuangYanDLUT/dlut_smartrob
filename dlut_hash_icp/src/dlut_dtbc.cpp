/*************************************************************************
  > File Name: dlut_hash_icp.cpp
  > Author: 吴乃亮
  > Mail: wunailiang@gmail.com
  > Created Time: Wed 21 May 2014 10:13:53 AM CST
 ************************************************************************/

#include "ros/ros.h"
//用于打开pcd
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include "src/icp.h"
#include "boost/thread.hpp"
#include <queue>
#include "src/net_transfor.h"

using namespace std;
boost::mutex io_mutex;
typedef boost::unordered_map<unordered_map_voxel,un_key> umap;

void getdata(std::queue<pcl::PointCloud<pcl::PointXYZ> > *p_q)
{
    Net_transfor n1(0);

    while(ros::ok())
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_copy;
        cout <<"in thread\n";
        //cloud->clear();
        if(n1.receive(cloud_copy)==false)
            break;

        {
            boost::mutex::scoped_lock lock(io_mutex);
            p_q->push(cloud_copy);
        }
        cout <<"p_q.size="<<p_q->size()<<endl;
    }
    n1.close_receive();
    cout <<"exit thread\n";
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"dlut_dtbc");
    ros::NodeHandle n;
    double voxel_size,filter_voxel_size;
    n.param("voxel_size",voxel_size,0.3);
    n.param("filter_voxel_size",filter_voxel_size,0.03);

    Voxelize* voxelize1;
    voxelize1 = new Voxelize;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local_temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global_raw(new pcl::PointCloud<pcl::PointXYZ>);
    std::queue<pcl::PointCloud<pcl::PointXYZ> > point_queue;
    boost::thread get_data(boost::bind(getdata,&point_queue));
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
            voxelize1->generateUmap(*cloud_local,voxel_size,m2);
            continue;
        }
        *cloud_global_raw += *cloud;
        m1.clear();

        voxelize1->generateUmap(*cloud,voxel_size,m1);
        Icp icp(m1,m2,*cloud,voxel_size);
        Eigen::Matrix4f tf_mat;
        tf_mat = icp.icpFit();
        tf_sum+=tf_mat;

        num_of_scan++;
        pcl::transformPointCloud(*cloud,*cloud,tf_mat);
        *cloud_local_temp += *cloud;
        cloud_local->swap(*cloud_local_temp);
        cloud_local_temp->clear();
        m2.clear();
        voxelize1->generateUmap(*cloud_local,voxel_size,m2);
        *cloud_global += *cloud;

    }

    pcl::PCDWriter writer;
    if(cloud_global->size()!=0)
    {
        writer.write("global_map.pcd",*cloud_global);
        writer.write("global_map_raw.pcd",*cloud_global_raw);
    }
    delete voxelize1;
    return 0;
}
