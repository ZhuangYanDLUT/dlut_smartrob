/*************************************************************************
  > File Name: icp.cpp
  > Author: 吴乃亮
  > Mail: wunailiang@gmail.com
  > Created Time: Thu 22 May 2014 09:14:34 AM CST
 ************************************************************************/

#include<math.h>
#include<vector>
#include "icp.h"
using namespace std;

Icp::Icp(umap &m1, umap &m2,pcl::PointCloud<pcl::PointXYZ> &cloud,double voxel_size):
  m_1(m1),
  m_2(m2),
  cloud_source(cloud),
  first_iter(true),
  _is_Fit(false)
{
  this->voxel_size = voxel_size;
  m_1_copy = m_1;
  R = Matrix3d::Identity();
  t.setZero();
  jd_x = 0;
  jd_y = 0;
  jd_z = 0;
  RT.open("RT.txt");
}

void Icp::least_Square()
{
  MatrixXd A(mat_param.size(),6);
  VectorXd b(mat_param.size());
  for(int i=0;i<mat_param.size();i++)
  {
    double param1 = mat_param[i].u_b(0);
    double param2 = mat_param[i].u_b(1);
    double param3 = mat_param[i].u_b(2);
    Matrix3d skew_mat;//Ub
    skew_mat<<0,-param3,param2,param3,0,-param1,-param2,param1,0;
    RowVector3d r1 = mat_param[i].v1.transpose();//CT
    RowVector3d r2 = r1*skew_mat;//-CT*Ub

    A(i,0) = -r2(0);
    A(i,1) = -r2(1);
    A(i,2) = -r2(2);
    A(i,3) = r1(0);
    A(i,4) = r1(1);
    A(i,5) = r1(2);

    b(i) = r1*(mat_param[i].u_a - mat_param[i].u_b);
  }

  /*
   *SVD解线性方程，解就是需要的旋转平移信息
   */
  VectorXd x = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

  Matrix3d result_R;

  result_R << cos(x(1))*cos(x(2)) ,-sin(x(2))*cos(x(0)) + cos(x(2))*sin(x(1))*sin(x(0))  , sin(x(2))*sin(x(0))+cos(x(2))*sin(x(1))*cos(x(0))
    , sin(x(2))*cos(x(1)) , cos(x(2))*cos(x(0))+sin(x(2))*sin(x(1))*sin(x(0)) , -cos(x(2))*sin(x(0)) + sin(x(2))*sin(x(1))*cos(x(0))
    , -sin(x(1)) , cos(x(1))*sin(x(0)) , cos(x(1))*cos(x(0));

  MatrixXd _result_R = result_R.transpose();
  Vector3d result_t;

  result_t << -x(3),-x(4),-x(5);
  R = _result_R*R;
  t = _result_R*t +result_t;

  jd_x+=x(0);
  jd_y+=x(1);
  jd_z+=x(2);
  cout <<"Angle computed：=\t"<<jd_x<<"\t"<<jd_y<<"\t"<<jd_z<<endl;

  RT<<"Rotation Matrix:\n"<<R<<endl;
  RT<<"t=\n"<<t<<endl;
  _is_Fit=is_Fit(x(0),x(1),x(2),result_t);
}
bool Icp::voxel_Merge(unordered_map_voxel &v1,unordered_map_voxel &v2)
{
  umap::iterator m1_iter = m_1_copy.find(v1);
  umap::iterator m2_iter = m_2.find(v2);

  temp_ls.S_merged = (m1_iter->second.matS + m2_iter->second.matS);

  temp_ls.u_a = m1_iter->second.u;
  temp_ls.u_b = m2_iter->second.u;
  Vector3d u_average=(temp_ls.u_a+temp_ls.u_b)/2;
  vector<eigen_sort> vector1(3);//用于排序

  EigenSolver<MatrixXd> es(temp_ls.S_merged);
  VectorXcd eivals = es.eigenvalues();
  MatrixXcd eigenvectors = es.eigenvectors();
  vector1[0]=eigen_sort(eivals(0).real(),eigenvectors.col(0));
  vector1[1]=eigen_sort(eivals(1).real(),eigenvectors.col(1));
  vector1[2]=eigen_sort(eivals(2).real(),eigenvectors.col(2));

  sort(vector1.begin(),vector1.end());

  double lamada1 = vector1[0].eigen_value;
  double lamada2 = vector1[1].eigen_value;
  double lamada3 = vector1[2].eigen_value;

  Vector3d tempVector=Vector3d(0,0,0)-u_average;
  if(Vector3d(vector1[0].eigen_vector(0).real(),vector1[0].eigen_vector(1).real(),vector1[0].eigen_vector(2).real()).dot(tempVector)<0)
  {
    vector1[0].eigen_vector(2).real() = -vector1[0].eigen_vector(2).real();
  }
  if(vector1[2].eigen_vector(2).real()<0)
  {
    vector1[2].eigen_vector(2).real() = -vector1[2].eigen_vector(2).real();
  }

  temp_ls.v1 << vector1[0].eigen_vector(0).real(),vector1[0].eigen_vector(1).real(),vector1[0].eigen_vector(2).real();
  temp_ls.lamada1 = lamada1;
  mat_param.push_back(temp_ls);
  return true;
}

bool Icp::linear_System()
{
  return true;
}

bool Icp::correction()
{
  /*
  //更新栅格参数，包括u,v1,v3,9D,S
  for(umap::iterator iter=m_1_copy.begin();iter!=m_1_copy.end();iter++)
  {
  iter->second.u = R*(iter->second.u)+t;
  iter->second.v1.real() = R*(iter->second.v1.real());
  iter->second.v3.real() = R*(iter->second.v3.real());
  iter->second.vector_9D << iter->second.u,iter->second.v1.real(),iter->second.v3.real();
  iter->second.matS = R*iter->second.matS*R.transpose();
  }
  */
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  Eigen::Matrix4f transformation_matrix;
  transformation_matrix.setZero();
  //构造旋转平移矩阵T
  transformation_matrix (0,0) = R(0,0);transformation_matrix (0,1) = R(0,1);transformation_matrix (0,2) = R(0,2);
  transformation_matrix (1,0) = R(1,0);transformation_matrix (1,1) = R(1,1);transformation_matrix (1,2) = R(1,2);
  transformation_matrix (2,0) = R(2,0);transformation_matrix (2,1) = R(2,1);transformation_matrix (2,2) = R(2,2);
  transformation_matrix (0,3) = t(0);transformation_matrix (1,3) = t(1);transformation_matrix (2,3) = t(2);

  pcl::transformPointCloud (cloud_source, transformed_cloud, transformation_matrix);
  Voxelize* voxelize1;
  voxelize1 = new Voxelize;
  m_1_copy.clear();
  if(first_iter)
  {
    m_1_copy = m_1;
    first_iter= false;
  }
  else
    voxelize1->generateUmap(transformed_cloud,voxel_size,m_1_copy);
  delete voxelize1;
  return true;
}

Eigen::Matrix4f Icp::icpFit()
{

  double t0 = ros::Time::now().toSec();
  cout<<t0<<endl;
  int iter_count = 0;//迭代次数
  Voxelize* voxelize2;
  voxelize2 = new Voxelize;

  int iteration_time=13;
  vector<bool> iteration_Result;
  Eigen::Matrix4f error_Return=Eigen::Matrix4f::Identity();
  error_Return(3,3)=0;
  for(int count =0;count<iteration_time;count ++)
  {
    //找匹配对
    mat_param.clear();
    correction();
    double t1 = ros::Time::now().toSec();
    pair<umap::iterator,bool> search_result;
    for(umap::iterator iter=m_2.begin();iter!=m_2.end();iter++)
    {
      search_result = voxelize2->neighbor_search(m_1_copy,m_2,iter->first);

      if(iter->second.p<0.5)
      {
        continue;
      }

      if(search_result.second)
      {
        unordered_map_voxel v1(search_result.first->first.x(),search_result.first->first.y(),search_result.first->first.z());
        unordered_map_voxel v2(iter->first.x(),iter->first.y(),iter->first.z());
        voxel_Merge(v1,v2);
      }
    }

    double t2 = ros::Time::now().toSec();
    if(mat_param.size()<6)
      break;
    linear_System();
    least_Square();
    iteration_Result.push_back(_is_Fit);
    if(end_of_iteration(iteration_Result)==true)
    {
      break;
    }
    if(iter_count>14)
    {
      return error_Return;
    }
    iter_count++;

  }
  delete voxelize2;

  Eigen::Matrix4f transformation_matrix;
  transformation_matrix.setZero();
  //构造旋转平移矩阵T
  transformation_matrix (0,0) = R(0,0);transformation_matrix (0,1) = R(0,1);transformation_matrix (0,2) = R(0,2);
  transformation_matrix (1,0) = R(1,0);transformation_matrix (1,1) = R(1,1);transformation_matrix (1,2) = R(1,2);
  transformation_matrix (2,0) = R(2,0);transformation_matrix (2,1) = R(2,1);transformation_matrix (2,2) = R(2,2);
  transformation_matrix (0,3) = t(0);transformation_matrix (1,3) = t(1);transformation_matrix (2,3) = t(2);
  cout <<"transformation_matrix=\n"<<transformation_matrix<<endl;
  double t4 = ros::Time::now().toSec();
  cout <<"匹配时间为："<<(t4-t0)<<endl;
  return transformation_matrix;
}

bool Icp::is_Fit(double angle_x,double angle_y,double angle_z,Vector3d shift_t)
{
  double criterion_angle=3.1415926*5/180;
  double criterion_shift=0.1;
  double angle=(fabs(angle_x)+fabs(angle_y)+fabs(angle_z));
  double shift=(fabs(shift_t(0))+fabs(shift_t(1))+fabs(shift_t(2)));
  cout<<"Total Angle："<<angle*180/3.1415926<<"\t"<<"Total shift："<<shift<<"\t";

  if((angle<criterion_angle)&&(shift<criterion_shift))
    return true;
  else
    return false;
}

bool Icp::end_of_iteration(const vector<bool>& result)
{
  if(result.size()<3)
    return false;
  int n=result.size()-1;
  if(result[n]&&result[n-1]&&(result.size()>3))
    return true;
}
