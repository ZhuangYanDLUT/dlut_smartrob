/*************************************************************************
  > File Name: src/unordered_map_voxel.h
  > Author: 吴乃亮
  > Mail: wunailiang@gmail.com
  > Created Time: Mon 28 Apr 2014 04:22:04 PM CST
 ************************************************************************/

#ifndef Unordered_map_h
#define Unordered_map_h

#include <iostream>
#include <vector>
#include <boost/unordered_map.hpp>
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace boost;
using namespace Eigen;

class unordered_map_voxel
{
    public:
        unordered_map_voxel(double x,double y,double z,double resolution): //用于从点的三坐标得到栅格坐标
            _resolution(resolution)
    {
        if(x>=0)
            _x= x/resolution +1;
        else
            _x = x/resolution -1;
        if(y>=0)
            _y = y/resolution +1;
        else
            _y = y/resolution -1;
        if(z>=0)
            _z = z/resolution +1;
        else
            _z = z/resolution -1;
    }
        unordered_map_voxel(int x,int y,int z):
            _x(x),
            _y(y),
            _z(z)
    {}

        //用于从点的三坐标得到栅格坐标
        bool operator== (const unordered_map_voxel& v) const // 判断是否相同
        {
            return _x == v._x && _y == v._y && _z == v._z;
        }

        const int x() const
        {
            return _x;
        }
        const int y() const
        {
            return _y;
        }
        const int z() const
        {
            return _z;
        }
    private:
        double _resolution;
        int _x; // 三坐标
        int _y;
        int _z;
};

size_t hash_value(const unordered_map_voxel &v);


class un_key
{
    public:
        int cout;
        double c;
        double p;
        vector<int> vec_index_point;
        Vector3d u;
        VectorXcd v1;
        VectorXcd v3;
        Matrix<double,9,1> vector_9D;
        Matrix3d matS;
};

//这个类用于对特征向量进行排序
class eigen_sort
{
    public:
        eigen_sort()
        {

        }
        eigen_sort(double value,VectorXcd vector):
            eigen_value(value),
            eigen_vector(vector)
    {}

        bool operator < (const eigen_sort &e) const
        {
            return eigen_value < e.eigen_value;
        }
        double eigen_value;
        VectorXcd eigen_vector;
};
#endif
