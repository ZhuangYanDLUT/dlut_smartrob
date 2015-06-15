/*************************************************************************
  > File Name: voxel.h
  > Author: 吴乃亮
  > Mail: wunailiang@gmail.com
  > Created Time: Wed 21 May 2014 09:06:12 AM CST
 ************************************************************************/

#ifndef voxel_h
#define voxel_h

#include <boost/unordered_map.hpp> //不需要排序的话最好用这个hash_map
#include "unordered_map_voxel.h"
#include <pcl/io/io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef boost::unordered_map<unordered_map_voxel,un_key> umap;

//对点云场景进行生成栅格和处理栅格、搜索最近邻的类
class Voxelize
{
    public:
        Voxelize(void){};
        ~Voxelize(){};
        void generateUmap(pcl::PointCloud<pcl::PointXYZ>& cloud,double resolution ,umap &m);//遍历每个点，生成栅格。然后计算每个栅格的参数
        pair<umap::iterator,bool> neighbor_search( umap &m, umap &m1,const unordered_map_voxel &tempp );//搜索最近邻
};
#endif
