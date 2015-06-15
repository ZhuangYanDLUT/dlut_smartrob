/*************************************************************************
  > File Name: dlut_hash_icp.cpp
  > Author: 吴乃亮
  > Mail: wunailiang@gmail.com
  > Created Time: Wed 21 May 2014 10:13:53 AM CST
 ************************************************************************/

#include<iostream>
#include"ros/ros.h"
//用于打开pcd
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>



#include "src/voxel.h"
#include "src/icp.h"


#include "src/net_transfor.h"
using namespace std;

typedef boost::unordered_map<unordered_map_voxel,un_key> umap;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"dlut_hash_icp");
    ros::NodeHandle n;
    double voxel_size,filter_voxel_size;
    n.param("voxel_size",voxel_size,0.3);
    n.param("filter_voxel_size",filter_voxel_size,0.03);

    Voxelize* voxelize1;
    voxelize1 = new Voxelize;
    pcl::PCDWriter writer;

    //打开两个pcd文件并且分别进行栅格化处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloud_fil;
    pcl::PointCloud<pcl::PointXYZ> cloud_fil1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unmerged(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>(argv[1],*cloud);

    /*
     *滤波，过滤掉无效激光点
     */
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
    indices.clear();

    boost::unordered_map<unordered_map_voxel,un_key> m1;

    /*
     *滤波，用于精简数据加速算法
     */
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filter_voxel_size,filter_voxel_size,filter_voxel_size);
    sor.filter(cloud_fil);
    writer.write(std::string(argv[1])+"_filtered.pcd",cloud_fil);
    cout<<argv[1]<<"'s size is "<<cloud_fil.size()<<endl;

    /*
     * 做一个旋转平移矩阵打乱匹配点云
     */

    /*
     *Eigen::Matrix4f tf_distored=Eigen::Matrix4f::Identity();
     *double theta=M_PI*10/180;
     *tf_distored(0,0)=cos(theta);
     *tf_distored(0,1)=-sin(theta);
     *tf_distored(1,0)=sin(theta);
     *tf_distored(1,1)=cos(theta);
     *tf_distored(0,3)=0;
     *tf_distored(1,3)=0;
     *pcl::transformPointCloud(cloud_fil,cloud_fil,tf_distored);
     */


    voxelize1->generateUmap(cloud_fil,voxel_size,m1);

    reader.read<pcl::PointXYZ>(argv[2],*cloud1);
    pcl::removeNaNFromPointCloud(*cloud1,*cloud1, indices);

    sor.setInputCloud(cloud1);
    sor.setLeafSize(filter_voxel_size,filter_voxel_size,filter_voxel_size);
    sor.filter(cloud_fil1);
    writer.write(std::string(argv[2])+"_filtered.pcd",cloud_fil1);
    cout<<argv[2]<<"'s size is "<<cloud_fil1.size()<<endl;

    *cloud_unmerged=(cloud_fil+cloud_fil1);

    /*
     *存储未做任何处理的点云
     */

    writer.write("un_merged.pcd",*cloud_unmerged);

    boost::unordered_map<unordered_map_voxel,un_key> m2;
    voxelize1->generateUmap(cloud_fil1,voxel_size,m2);

    Icp testicp(m1,m2,cloud_fil, voxel_size);
    Eigen::Matrix4f tf_mat=testicp.icpFit();
    pcl::transformPointCloud(cloud_fil,cloud_fil,tf_mat);
    cloud->clear();
    *cloud=(cloud_fil+cloud_fil1);
    writer.write("merged.pcd",*cloud);

    delete voxelize1;
    return 0;
}
