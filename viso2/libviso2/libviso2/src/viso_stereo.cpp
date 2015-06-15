/*
   Copyright 2011. All rights reserved.
   Institute of Measurement and Control Systems
   Karlsruhe Institute of Technology, Germany

   This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA
*/

#include "viso_stereo.h"
#include "P3p.h"
#include <eigen3/Eigen/Dense>
#include "ceres/ceres.h"
#include "glog/logging.h"

#include <TooN/TooN.h>
#include <algorithm>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::CauchyLoss;
using ceres::ArctanLoss;
using namespace std;

//ceres solver
struct ExponentialResidual {
    ExponentialResidual(double u, double v, double xp, double yp ,double zp)
        : u_(u), v_(v), xp_(xp), yp_(yp), zp_(zp)
    {}

    template <typename T> bool operator()(
            const T* const rx,
            const T* const ry,
            const T* const rz,
            const T* const tx,
            const T* const ty,
            const T* const tz,
            T* residual) const
    {
        // precompute sine/cosine
        T sx = sin(rx[0]); T cx = cos(rx[0]); T sy = sin(ry[0]);
        T cy = cos(ry[0]); T sz = sin(rz[0]); T cz = cos(rz[0]);

        // compute rotation matrix and derivatives
        T r00    = +cy*cz;          T r01    = -cy*sz;          T r02    = +sy;
        T r10    = +sx*sy*cz+cx*sz; T r11    = -sx*sy*sz+cx*cz; T r12    = -sx*cy;
        T r20    = -cx*sy*cz+sx*sz; T r21    = +cx*sy*sz+sx*cz; T r22    = +cx*cy;

        T point_Predicted_x = r00*xp_ + r01*yp_ + r02*zp_ + tx[0];
        T point_Predicted_y = r10*xp_ + r11*yp_ + r12*zp_ + ty[0];
        T point_Predicted_z = r20*xp_ + r21*yp_ + r22*zp_ + tz[0];

        T u_Predicted = 582.508667*point_Predicted_x/point_Predicted_z + 520.5;
        T v_Predicted = 582.508667*point_Predicted_y/point_Predicted_z + 506.5;
        residual[0] = (u_-u_Predicted);
        residual[1] = (v_-v_Predicted);
        return true;
    }

    private:
    const double u_;
    const double v_;
    const double xp_;
    const double yp_;
    const double zp_;
};

VisualOdometryStereo::VisualOdometryStereo (parameters param) : param(param), VisualOdometry(param) {
    rx_old = 0.0;
    ry_old = 0.0;
    rz_old = 0.0;
    tx_old = 0.0;
    ty_old = 0.0;
    tz_old = 0.0;
    matcher->setIntrinsics(param.calib.f,param.calib.cu,param.calib.cv,param.base);
}

VisualOdometryStereo::~VisualOdometryStereo() {}

bool VisualOdometryStereo::process (uint8_t *I1,uint8_t *I2,int32_t* dims,bool replace) {
    matcher->pushBack(I1,I2,dims,replace);
    if (Tr_valid) matcher->matchFeatures(2,&Tr_delta);
    else          matcher->matchFeatures(2);
    p_matched = matcher->getMatches();
    int n = p_matched.size();
    p_matched.clear();
    matcher->bucketFeatures(param.bucket.max_features,param.bucket.bucket_width,param.bucket.bucket_height);
    int width = param.bucket.bucket_width;
    int height = param.bucket.bucket_height;
    int number_of_matches = 350;
    while(n > number_of_matches)
    {
        matcher->bucketFeatures(param.bucket.max_features, width, height);
        n = matcher->getMatches().size();
        width++;
        height++;
    }

    p_matched = matcher->getMatches();
    return updateMotion();
}

//justice if 2 matches fit the constraint of maxclique
bool VisualOdometryStereo::justice2Matches(Matcher::p_match &m1,Matcher::p_match &m2)
{
    double d1p = max(m1.u1p - m1.u2p,0.0001f);
    double d1c = max(m1.u1c - m1.u2c,0.0001f);
    double x1p = (m1.u1p-param.calib.cu)*param.base/d1p;
    double y1p = (m1.v1p-param.calib.cv)*param.base/d1p;
    double z1p = param.calib.f*param.base/d1p;
    double x1c = (m1.u1c-param.calib.cu)*param.base/d1c;
    double y1c = (m1.v1c-param.calib.cv)*param.base/d1c;
    double z1c = param.calib.f*param.base/d1c;

    double d2p = max(m2.u1p - m2.u2p,0.0001f);
    double d2c = max(m2.u1c - m2.u2c,0.0001f);
    double x2p = (m2.u1p-param.calib.cu)*param.base/d2p;
    double y2p = (m2.v1p-param.calib.cv)*param.base/d2p;
    double z2p = param.calib.f*param.base/d2p;
    double x2c = (m2.u1c-param.calib.cu)*param.base/d2c;
    double y2c = (m2.v1c-param.calib.cv)*param.base/d2c;
    double z2c = param.calib.f*param.base/d2c;

    double dist1 = sqrt(pow(x1p-x2p,2)+pow(y1p-y2p,2)+pow(z1p-z2p,2));
    double dist2 = sqrt(pow(x1c-x2c,2)+pow(y1c-y2c,2)+pow(z1c-z2c,2));
    double diff = fabs(dist1-dist2);
    if(diff< 0.25 && dist1<5.0)
        return true;
    else
        return false;
}

//Creat the adjmatrix, which is used in maxclique
void VisualOdometryStereo::creatAdjMatrix(vector<Matcher::p_match>& p_matched,bool** &conn)
{
    int size = p_matched.size();
    conn = new bool*[size];
    for (int i=0; i < size; i++) {
        conn[i] = new bool[size];
        memset(conn[i], 0, size * sizeof(bool));
    }
    for(int i=0;i<size;++i)
        for(int j=0;j<=i;++j)
            if(justice2Matches(p_matched[i],p_matched[j]))
            {
                conn[i][j]=true;
                conn[j][i]=true;
            }
}

//estimate motion with Maxclique
vector<double> VisualOdometryStereo::estimateMotionMaxClique (vector<Matcher::p_match> p_matched)
{
    /*//MaxClique +RANSAC + p3p
    int64_t N  = p_matched.size();
    double rx=rx_old,ry=ry_old,rz=rz_old,tx=tx_old,ty=ty_old,tz=tz_old;
    Problem problem;

    // loop variables
    TooN::Matrix<3,4> tr_delta,tr_delta_curr;
    tr_delta = TooN::Data(0,1,0,0,
            0,0,1,0,
            0,0,0,1);
    tr_delta_curr = TooN::Data(0,1,0,0,
            0,0,1,0,
            0,0,0,1);

    if (N<6)
        return vector<double>();
    bool** conn;
    int* qmax,qsize;
    creatAdjMatrix(p_matched,conn);

    Maxclique md(conn,N,0.1);
    md.mcqdyn(qmax, qsize);

    std::vector<int32_t> inliers_maxclique;
    for (int i = 0; i < qsize; i++)
        inliers_maxclique.push_back(qmax[i]);
    bool success=true;

    std::vector<Matcher::p_match> p_matched_maxclique;
    for(int i = 0; i<inliers_maxclique.size(); ++i)
    {
        p_matched_maxclique.push_back(p_matched[inliers_maxclique[i] ]);
    }
    int32_t N_ = p_matched_maxclique.size();
    p_matched.clear();
    p_matched = p_matched_maxclique;
    p_matched1 = p_matched_maxclique;
    // allocate dynamic memory
    X          = new double[N_];
    Y          = new double[N_];
    Z          = new double[N_];
    XC         = new double[N_];
    YC         = new double[N_];
    ZC         = new double[N_];
    J          = new double[4*N_*6];
    p_predict  = new double[4*N_];
    p_observe  = new double[4*N_];
    p_residual = new double[4*N_];

    for (int32_t i=0; i<N_; i++) {
        double d = max(p_matched_maxclique[i].u1p - p_matched_maxclique[i].u2p,0.0001f);
        X[i] = (p_matched_maxclique[i].u1p-param.calib.cu)*param.base/d;
        Y[i] = (p_matched_maxclique[i].v1p-param.calib.cv)*param.base/d;
        Z[i] = param.calib.f*param.base/d;
        XC[i] = (p_matched_maxclique[i].u1c-param.calib.cu)*param.base/d;
        YC[i] = (p_matched_maxclique[i].v1c-param.calib.cv)*param.base/d;
        ZC[i] = param.calib.f*param.base/d;
    }
    inliers.clear();
    // initial RANSAC estimate
    for (int32_t k=0;k<param.ransac_iters;k++) {

        // draw random sample set
        vector<int32_t> active = getRandomSample(N_,3);

        // minimize reprojection errors
        VisualOdometryStereo::result result = UPDATED;
        result = updateParametersP3p(p_matched1,active,tr_delta_curr,1,1e-6);
        if(result == CONVERGED)
        {
            vector<int32_t> inliers_curr = getInlier(p_matched1,tr_delta_curr);
            if (inliers_curr.size()>inliers.size()) {
                inliers = inliers_curr;
                tr_delta = tr_delta_curr;
            }
        }
    }

    vector<double> tr_delta_vec;
    tr_delta_vec.resize(6);
    tr_delta_vec[0] = tr_delta[2][2];
    tr_delta_vec[1] = tr_delta[2][1];
    tr_delta_vec[2] = tr_delta[1][1];
    tr_delta_vec[3] = tr_delta[0][0];
    tr_delta_vec[4] = tr_delta[0][1];
    tr_delta_vec[5] = tr_delta[0][2];

    std::cout<<"inliers of P3P-MaxClique:\t"<<inliers.size()<<"\tp_matched N N_:"<<N<<" "<<N_<<"\t"<<(double)inliers.size()*100/N_<<std::endl;
    rx = tr_delta_vec[0]; ry = tr_delta_vec[1]; rz = tr_delta_vec[2];
    tx = tr_delta_vec[3]; ty = tr_delta_vec[4]; tz = tr_delta_vec[5];

    for(int i=0;i<inliers.size();++i)
    {
        problem.AddResidualBlock(
                new AutoDiffCostFunction<ExponentialResidual, 2, 1, 1,1,1,1,1>(
                    new ExponentialResidual(p_matched_maxclique[inliers[i]].u1c,p_matched_maxclique[inliers[i]].v1c,X[inliers[i]],Y[inliers[i]],Z[inliers[i]])),
                new CauchyLoss(0.5),
                //NULL,
                &rx, &ry,&rz,&tx,&ty,&tz);
    }

    Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    if(summary.IsSolutionUsable()==false)
    {
        success = false;
        std::cout <<"UN CONVERGED!!!\n" <<summary.BriefReport() << "\n";
    }

    tr_delta_vec[0] = rx;
    tr_delta_vec[1] = ry;
    tr_delta_vec[2] = rz;
    tr_delta_vec[3] = tx;
    tr_delta_vec[4] = ty;
    tr_delta_vec[5] = tz;

    for (int i=0;i<N;i++)
        delete [] conn[i];
    delete [] conn;
    delete [] qmax;
    // release dynamic memory
    delete[] X;
    delete[] Y;
    delete[] Z;
    delete[] XC;
    delete[] YC;
    delete[] ZC;
    delete[] J;
    delete[] p_predict;
    delete[] p_observe;
    delete[] p_residual;
    // parameter estimate succeeded?
    if (success) return tr_delta_vec;
    else         return vector<double>();*/

    //MaxClique +RANSAC + ceres
    int32_t N  = p_matched.size();
    double rx=rx_old,ry=ry_old,rz=rz_old,tx=tx_old,ty=ty_old,tz=tz_old;
    Problem problem;

    // loop variables
    vector<double> tr_delta;
    tr_delta.resize(6);
    vector<double> tr_delta_curr;
    tr_delta_curr.resize(6);

    if (N<6)
        return vector<double>();
    bool** conn;
    int* qmax,qsize;
    creatAdjMatrix(p_matched,conn);

    Maxclique md(conn,N);
    md.mcq(qmax, qsize);

    std::vector<int32_t> inliers_maxclique;
    for (int i = 0; i < qsize; i++)
        inliers_maxclique.push_back(qmax[i]);
    bool success=true;

    std::vector<Matcher::p_match> p_matched_maxclique;
    for(int i = 0; i<inliers_maxclique.size(); ++i)
    {
        p_matched_maxclique.push_back(p_matched[inliers_maxclique[i] ]);
    }
    int32_t N_ = p_matched_maxclique.size();
    p_matched.clear();
    p_matched = p_matched_maxclique;
    p_matched1 = p_matched_maxclique;
    // allocate dynamic memory
    X          = new double[N_];
    Y          = new double[N_];
    Z          = new double[N_];
    XC         = new double[N_];
    YC         = new double[N_];
    ZC         = new double[N_];
    J          = new double[4*N_*6];
    p_predict  = new double[4*N_];
    p_observe  = new double[4*N_];
    p_residual = new double[4*N_];

    for (int32_t i=0; i<N_; i++) {
        double d = max(p_matched_maxclique[i].u1p - p_matched_maxclique[i].u2p,0.0001f);
        X[i] = (p_matched_maxclique[i].u1p-param.calib.cu)*param.base/d;
        Y[i] = (p_matched_maxclique[i].v1p-param.calib.cv)*param.base/d;
        Z[i] = param.calib.f*param.base/d;
        XC[i] = (p_matched_maxclique[i].u1c-param.calib.cu)*param.base/d;
        YC[i] = (p_matched_maxclique[i].v1c-param.calib.cv)*param.base/d;
        ZC[i] = param.calib.f*param.base/d;
    }
    inliers.clear();
    // initial RANSAC estimate
    for (int32_t k=0;k<param.ransac_iters;k++) {

        // draw random sample set
        vector<int32_t> active = getRandomSample(N_,3);

        // clear parameter vector
        for (int32_t i=0; i<6; i++)
            tr_delta_curr[i] = 0;

        // minimize reprojection errors
        VisualOdometryStereo::result result = UPDATED;
        int32_t iter=0;
        while (result==UPDATED) {
            result = updateParameters(p_matched_maxclique,active,tr_delta_curr,1,1e-8);
            if (iter++ > 100 || result==CONVERGED)
                break;
        }
        // overwrite best parameters if we have more inliers
        if (result==CONVERGED) {
            vector<int32_t> inliers_curr = getInlier(p_matched_maxclique,tr_delta_curr);
            if (inliers_curr.size()>inliers.size()) {
                inliers = inliers_curr;
                tr_delta = tr_delta_curr;
            }
        }
    }
    rx = tr_delta[0]; ry = tr_delta[1]; rz = tr_delta[2];
    tx = tr_delta[3]; ty = tr_delta[4]; tz = tr_delta[5];
    std::cout<<"inliers of MaxClique:\t"<<inliers.size()<<"\tp_matched N N_:"<<N<<" "<<N_<<"\t"<<(double)inliers.size()*100/N_<<std::endl;


    for(int i=0;i<inliers.size();++i)
    {
        problem.AddResidualBlock(
                new AutoDiffCostFunction<ExponentialResidual, 2, 1, 1,1,1,1,1>(
                    new ExponentialResidual(p_matched_maxclique[inliers[i]].u1c,p_matched_maxclique[inliers[i]].v1c,X[inliers[i]],Y[inliers[i]],Z[inliers[i]])),
                new CauchyLoss(0.5),
                //NULL,
                &rx, &ry,&rz,&tx,&ty,&tz);
    }

    Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    if(summary.IsSolutionUsable()==false)
    {
        success = false;
        std::cout <<"UN CONVERGED!!!\n" <<summary.BriefReport() << "\n";
    }

    tr_delta[0] = rx;
    tr_delta[1] = ry;
    tr_delta[2] = rz;
    tr_delta[3] = tx;
    tr_delta[4] = ty;
    tr_delta[5] = tz;

    for (int i=0;i<N;i++)
        delete [] conn[i];
    delete [] conn;
    delete [] qmax;
    // release dynamic memory
    delete[] X;
    delete[] Y;
    delete[] Z;
    delete[] XC;
    delete[] YC;
    delete[] ZC;
    delete[] J;
    delete[] p_predict;
    delete[] p_observe;
    delete[] p_residual;
    // parameter estimate succeeded?
    if (success) return tr_delta;
    else         return vector<double>();

    //newton-gauss and Maxclique
    /*
     *        if(p_matched.size()>200)
     *        {
     *            partial_sort(p_matched.begin(),p_matched.begin()+200,p_matched.end());
     *            p_matched.resize(200);
     *        }
     *
     *    [>printf("after filter, the p_matched's size is %d\n",p_matched.size());<]
     *    int32_t N  = p_matched.size();
     *    // allocate dynamic memory
     *    X          = new double[N];
     *    Y          = new double[N];
     *    Z          = new double[N];
     *    XC         = new double[N];
     *    YC         = new double[N];
     *    ZC         = new double[N];
     *    J          = new double[4*N*6];
     *    p_predict  = new double[4*N];
     *    p_observe  = new double[4*N];
     *    p_residual = new double[4*N];
     *
     *    // project matches of previous image into 3d
     *    for (int32_t i=0; i<N; i++) {
     *        double d = max(p_matched[i].u1p - p_matched[i].u2p,0.0001f);
     *        X[i] = (p_matched[i].u1p-param.calib.cu)*param.base/d;
     *        Y[i] = (p_matched[i].v1p-param.calib.cv)*param.base/d;
     *        Z[i] = param.calib.f*param.base/d;
     *        XC[i] = (p_matched[i].u1c-param.calib.cu)*param.base/d;
     *        YC[i] = (p_matched[i].v1c-param.calib.cv)*param.base/d;
     *        ZC[i] = param.calib.f*param.base/d;
     *    }
     *
     *    // loop variables
     *    vector<double> tr_delta;
     *    tr_delta.resize(6);
     *    vector<double> tr_delta_curr;
     *    tr_delta_curr.resize(6);
     *
     *    if (N<6)
     *        return vector<double>();
     *    bool** conn;
     *    int* qmax,qsize;
     *    creatAdjMatrix(p_matched,conn);
     *    Maxclique md(conn,N,0.025);
     *    md.mcqdyn(qmax, qsize);
     *    inliers.clear();
     *    for (int i = 0; i < qsize; i++)
     *        inliers.push_back(qmax[i]);
     *    bool success=true;
     *
     *    [>printf("after Max Clique, the inliers's size is %d\n",inliers.size());<]
     *    if (inliers.size()>=6) {
     *        int32_t iter=0;
     *        VisualOdometryStereo::result result = UPDATED;
     *        while (result==UPDATED) {
     *            result = updateParameters(p_matched,inliers,tr_delta,1,1e-8);
     *            if (iter++ > 100 || result==CONVERGED)
     *                break;
     *        }
     *
     *        // not converged
     *        if (result!=CONVERGED)
     *            success = false;
     *
     *        // not enough inliers
     *    } else {
     *        success = false;
     *    }
     *
     *    for (int i=0;i<N;i++)
     *        delete [] conn[i];
    *    delete [] conn;
    *    delete [] qmax;
    *    // release dynamic memory
        *    delete[] X;
    *    delete[] Y;
    *    delete[] Z;
    *    delete[] XC;
    *    delete[] YC;
    *    delete[] ZC;
    *    delete[] J;
    *    delete[] p_predict;
    *    delete[] p_observe;
    *    delete[] p_residual;
    *
        *    // parameter estimate succeeded?
        *    if (success) return tr_delta;
    *    else         return vector<double>();
    */
}
vector<double> VisualOdometryStereo::estimateMotionLinear (vector<Matcher::p_match> p_matched)
{
    if(p_matched.size()<6)
        return vector<double> ();
    TooN::Matrix<3,4> tr_delta,tr_delta_curr;
    tr_delta = TooN::Data(0,1,0,0,
            0,0,1,0,
            0,0,0,1);
    tr_delta_curr = TooN::Data(0,1,0,0,
            0,0,1,0,
            0,0,0,1);

    // return value
    bool success = true;

    // compute minimum distance for RANSAC samples
    double width=0,height=0;
    for (vector<Matcher::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
        if (it->u1c>width)  width  = it->u1c;
        if (it->v1c>height) height = it->v1c;
    }
    double min_dist = min(width,height)/3.0;

    // get number of matches
    int32_t N  = p_matched.size();
    if (N<6)
        return vector<double>();

    // allocate dynamic memory
    X          = new double[N];
    Y          = new double[N];
    Z          = new double[N];
    XC         = new double[N];
    YC         = new double[N];
    ZC         = new double[N];
    J          = new double[4*N*6];
    p_predict  = new double[4*N];
    p_observe  = new double[4*N];
    p_residual = new double[4*N];

    // project matches of previous image into 3d
    for (int32_t i=0; i<N; i++) {
        double d = max(p_matched[i].u1p - p_matched[i].u2p,0.0001f);
        X[i] = (p_matched[i].u1p-param.calib.cu)*param.base/d;
        Y[i] = (p_matched[i].v1p-param.calib.cv)*param.base/d;
        Z[i] = param.calib.f*param.base/d;
        XC[i] = (p_matched[i].u1c-param.calib.cu)*param.base/d;
        YC[i] = (p_matched[i].v1c-param.calib.cv)*param.base/d;
        ZC[i] = param.calib.f*param.base/d;
    }

    inliers.clear();
    // initial RANSAC estimate
    for (int32_t k=0;k<param.ransac_iters;k++) {

        // draw random sample set
        vector<int32_t> active = getRandomSample(N,3);

        // clear parameter vector

        // minimize reprojection errors
        VisualOdometryStereo::result result = UPDATED;
        result = updateParametersP3p(p_matched,active,tr_delta_curr,1,1e-6);
        if(result == CONVERGED)
        {
            vector<int32_t> inliers_curr = getInlier(p_matched,tr_delta_curr);
            if (inliers_curr.size()>inliers.size()) {
                inliers = inliers_curr;
                tr_delta = tr_delta_curr;
            }
        }
    }

    vector<double> tr_delta_vec;
    tr_delta_vec.resize(6);
    tr_delta_vec[0] = tr_delta[2][2];
    tr_delta_vec[1] = tr_delta[2][1];
    tr_delta_vec[2] = tr_delta[1][1];
    tr_delta_vec[3] = tr_delta[0][0];
    tr_delta_vec[4] = tr_delta[0][1];
    tr_delta_vec[5] = tr_delta[0][2];


    std::cout<<"inliers of P3p:\t"<<inliers.size()<<"\tp_matched :\t"<<p_matched.size()<<"\t"<<(double)inliers.size()*100/p_matched.size()<<std::endl;

    double rx=rx_old,ry=ry_old,rz=rz_old,tx=tx_old,ty=ty_old,tz=tz_old;
    rx = tr_delta_vec[0]; ry = tr_delta_vec[1]; rz = tr_delta_vec[2];
    tx = tr_delta_vec[3]; ty = tr_delta_vec[4]; tz = tr_delta_vec[5];
    Problem problem;
    for(int i=0;i<inliers.size();++i)
    {
        problem.AddResidualBlock(
                new AutoDiffCostFunction<ExponentialResidual, 2, 1, 1,1,1,1,1>(
                    new ExponentialResidual(p_matched[inliers[i]].u1c,p_matched[inliers[i]].v1c,X[inliers[i]],Y[inliers[i]],Z[inliers[i]])),
                new CauchyLoss(2.0),
                //NULL,
                &rx, &ry,&rz,&tx,&ty,&tz);
    }

    Solver::Options options;
    options.max_num_iterations = 20;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    Solver::Summary summary;
    Solve(options, &problem, &summary);
    if(summary.IsSolutionUsable()==false)
    {
        success = false;
        std::cout <<"UN CONVERGED!!!\n" <<summary.BriefReport() << "\n";
    }

    tr_delta_vec[0] = rx;
    tr_delta_vec[1] = ry;
    tr_delta_vec[2] = rz;
    tr_delta_vec[3] = tx;
    tr_delta_vec[4] = ty;
    tr_delta_vec[5] = tz;

    p_matched1 = p_matched;
    // parameter estimate succeeded?
    if (success) return tr_delta_vec;
    else         return vector<double>();
    delete[] X;
    delete[] Y;
    delete[] Z;
    delete[] XC;
    delete[] YC;
    delete[] ZC;
    delete[] J;
    delete[] p_predict;
    delete[] p_observe;
    delete[] p_residual;
    return vector<double>();
}

vector<double> VisualOdometryStereo::estimateMotion (vector<Matcher::p_match> p_matched) {

    // return value
    bool success = true;

    // compute minimum distance for RANSAC samples
    double width=0,height=0;
    for (vector<Matcher::p_match>::iterator it=p_matched.begin(); it!=p_matched.end(); it++) {
        if (it->u1c>width)  width  = it->u1c;
        if (it->v1c>height) height = it->v1c;
    }
    double min_dist = min(width,height)/3.0;

    // get number of matches
    int32_t N  = p_matched.size();
    if (N<6)
        return vector<double>();
    double rx=rx_old,ry=ry_old,rz=rz_old,tx=tx_old,ty=ty_old,tz=tz_old;
    // allocate dynamic memory
    X          = new double[N];
    Y          = new double[N];
    Z          = new double[N];
    XC         = new double[N];
    YC         = new double[N];
    ZC         = new double[N];
    J          = new double[4*N*6];
    p_predict  = new double[4*N];
    p_observe  = new double[4*N];
    p_residual = new double[4*N];

    // project matches of previous image into 3d
    for (int32_t i=0; i<N; i++) {
        double d = max(p_matched[i].u1p - p_matched[i].u2p,0.0001f);
        X[i] = (p_matched[i].u1p-param.calib.cu)*param.base/d;
        Y[i] = (p_matched[i].v1p-param.calib.cv)*param.base/d;
        Z[i] = param.calib.f*param.base/d;
        XC[i] = (p_matched[i].u1c-param.calib.cu)*param.base/d;
        YC[i] = (p_matched[i].v1c-param.calib.cv)*param.base/d;
        ZC[i] = param.calib.f*param.base/d;
    }

    // loop variables
    vector<double> tr_delta;
    vector<double> tr_delta_curr;
    tr_delta_curr.resize(6);

    // clear parameter vector
    inliers.clear();

    // initial RANSAC estimate
    for (int32_t k=0;k<param.ransac_iters;k++) {

        // draw random sample set
        vector<int32_t> active = getRandomSample(N,3);

        // clear parameter vector
        for (int32_t i=0; i<6; i++)
            tr_delta_curr[i] = 0;

        // minimize reprojection errors
        VisualOdometryStereo::result result = UPDATED;
        int32_t iter=0;
        while (result==UPDATED) {
            result = updateParameters(p_matched,active,tr_delta_curr,1,1e-6);
            if (iter++ > 20 || result==CONVERGED)
                break;
        }
        // overwrite best parameters if we have more inliers
        if (result==CONVERGED) {
            vector<int32_t> inliers_curr = getInlier(p_matched,tr_delta_curr);
            if (inliers_curr.size()>inliers.size()) {
                inliers = inliers_curr;
                tr_delta = tr_delta_curr;
            }
        }
    }
    /*
       std::cout<<"inliers of Gauss-Newton:"<<inliers.size()<<"p_matched :"<<p_matched.size()<<"\t"<<(double)100*inliers.size()/p_matched.size()<<std::endl;
       if (inliers.size()>=6) {
       int32_t iter=0;
       VisualOdometryStereo::result result = UPDATED;
       while (result==UPDATED) {
       result = updateParameters(p_matched,inliers,tr_delta,1,1e-8);
       if (iter++ > 100 || result==CONVERGED)
       break;
       }

// not converged
if (result!=CONVERGED)
success = false;

// not enough inliers
} else {
success = false;
}
*/

rx = tr_delta[0]; ry = tr_delta[1]; rz = tr_delta[2];
tx = tr_delta[3]; ty = tr_delta[4]; tz = tr_delta[5];
std::cout<<"inliers of newton-gauss:\t"<<inliers.size()<<"\tp_matched :\t"<<p_matched.size()<<"\t"<<(double)inliers.size()*100/p_matched.size()<<std::endl;
Problem problem;

for(int i=0;i<inliers.size();++i)
{
    problem.AddResidualBlock(
            new AutoDiffCostFunction<ExponentialResidual, 2, 1, 1,1,1,1,1>(
                new ExponentialResidual(p_matched[inliers[i]].u1c,p_matched[inliers[i]].v1c,X[inliers[i]],Y[inliers[i]],Z[inliers[i]])),
            new CauchyLoss(0.5),
            //NULL,
            &rx, &ry,&rz,&tx,&ty,&tz);
}

Solver::Options options;
options.max_num_iterations = 100;
options.linear_solver_type = ceres::DENSE_QR;
options.minimizer_progress_to_stdout = false;

Solver::Summary summary;
Solve(options, &problem, &summary);
if(summary.IsSolutionUsable()==false)
{
    success = false;
    std::cout <<"UN CONVERGED!!!\n" <<summary.BriefReport() << "\n";
}

tr_delta[0] = rx;
tr_delta[1] = ry;
tr_delta[2] = rz;
tr_delta[3] = tx;
tr_delta[4] = ty;
tr_delta[5] = tz;

// release dynamic memory
delete[] X;
delete[] Y;
delete[] Z;
delete[] XC;
delete[] YC;
delete[] ZC;
delete[] J;
delete[] p_predict;
delete[] p_observe;
delete[] p_residual;

p_matched1 = p_matched;
// parameter estimate succeeded?
if (success) return tr_delta;
else         return vector<double>();
}

vector<int32_t> VisualOdometryStereo::getInlier(vector<Matcher::p_match> &p_matched,vector<double> &tr) {

    // mark all observations active
    vector<int32_t> active;
    for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
        active.push_back(i);

    // extract observations and compute predictions
    computeObservations(p_matched,active);
    computeResidualsAndJacobian(tr,active);

    // compute inliers
    vector<int32_t> inliers;
    for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
        if (pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2) +
                pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2) < param.inlier_threshold*param.inlier_threshold)
            inliers.push_back(i);
    return inliers;
}

vector<int32_t> VisualOdometryStereo::getInlier(vector<Matcher::p_match> &p_matched,TooN::Matrix<3,4> &tr)
{
    // mark all observations active
    vector<int32_t> active;
    for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
        active.push_back(i);

    // extract observations and compute predictions
    computeObservations(p_matched,active);
    computeResidualsAndJacobianP3p(tr,active);

    // compute inliers
    vector<int32_t> inliers;
    for (int32_t i=0; i<(int32_t)p_matched.size(); i++)
        if (pow(p_observe[4*i+0]-p_predict[4*i+0],2)+pow(p_observe[4*i+1]-p_predict[4*i+1],2) +
                pow(p_observe[4*i+2]-p_predict[4*i+2],2)+pow(p_observe[4*i+3]-p_predict[4*i+3],2) < param.inlier_threshold*param.inlier_threshold)
            inliers.push_back(i);
    return inliers;
}
VisualOdometryStereo::result VisualOdometryStereo::updateParametersP3p(vector<Matcher::p_match> &p_matched,vector<int32_t> &active,TooN::Matrix<3,4> &tr,double step_size,double eps)
{
    if(active.size()!=3)
        return FAILED;
    TooN::Matrix<3,3> features;
    TooN::Matrix<3,3> worldPoints;
    TooN::Matrix<3,16> solutions;
    for(int i=0;i<3;++i)
    {
        double normPc = sqrt(pow(X[active[i]],2)+pow(Y[active[i]],2)+pow(Z[active[i]],2));
        worldPoints[0][i] = XC[active[i]];
        worldPoints[1][i] = YC[active[i]];
        worldPoints[2][i] = ZC[active[i]];

        features[0][i] = X[active[i]]/normPc;
        features[1][i] = Y[active[i]]/normPc;
        features[2][i] = Z[active[i]]/normPc;
    }
    P3p myP3p;
    if(myP3p.computePoses(features,worldPoints,solutions) != 0)
        return FAILED;
    for(int i=0;i<4;++i)
    {
        if(fabs(solutions[0][4*i])<0.3 && fabs(solutions[1][4*i])<0.3 && fabs(solutions[2][4*i])<0.3)
        {
            tr = solutions.slice(0,4*i,3,4);
            //std::cout<<tr[2][2]<<"\t"<<-tr[2][1]<<"\t"<<tr[1][1]<<"\t"<<tr[0][0]<<"\t"<<tr[1][0]<<"\t"<<tr[2][0]<<std::endl;
            //std::cout<<tr<<std::endl;
            return CONVERGED;
        }
    }
    return FAILED;
}
VisualOdometryStereo::result VisualOdometryStereo::updateParametersLinear(vector<Matcher::p_match> &p_matched,vector<int32_t> &active,vector<double> &tr,double step_size,double eps)
{
    //Linear Version one
    /*
     *    if(active.size()<3)
     *        return FAILED;
     *    Eigen::MatrixXd A(3*active.size(),6);
     *    Eigen::VectorXd b(3*active.size());
     *    for(int i=0;i<active.size();++i)
     *    {
     *        if((Z[active[i]]>8)||(ZC[active[i]]>8))
     *            return FAILED;
     *        A(i*3+0,0) = 0;     A(i*3+1,0) = -Z[active[i]];      A(i*3+2,0) = Y[active[i]];
     *        A(i*3+0,1) = -Z[active[i]];     A(i*3+1,1) = 0;      A(i*3+2,1) = X[active[i]];
     *        A(i*3+0,2) = -Y[active[i]]; A(i*3+1,2) = X[active[i]];A(i*3+2,2) = 0;
     *        A(i*3+0,3) = 1; A(i*3+1,3) = 0; A(i*3+2,3) = 0;
     *        A(i*3+0,4) = 0; A(i*3+1,4) = 1; A(i*3+2,4) = 0;
     *        A(i*3+0,5) = 0; A(i*3+1,5) = 0; A(i*3+2,5) = 1;
     *        b(i*3+0) = XC[active[i]]-X[active[i]];
     *        b(i*3+1) = YC[active[i]]-Y[active[i]];
     *        b(i*3+2) = ZC[active[i]]-Z[active[i]];
     *    }
     *
     *    Eigen::VectorXd x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
     *    for(int i=0;i<6;++i)
     *        tr[i] = x(i);
     *    return CONVERGED;
     */
    /*
     *Matrix A(6,6);
     *Matrix B(6,1);
     */
    /*
     *    Eigen::MatrixXd A(2*active.size(),6);
     *    Eigen::VectorXd b(2*active.size());
     *    double f = param.calib.f;
     *
     *    for(int i=0;i<active.size();++i)
     *    {
     *        int uc = p_matched[active[i]].u1c;
     *        int vc = p_matched[active[i]].v1c;
     *        int up = p_matched[active[i]].u1p;
     *        int vp = p_matched[active[i]].v1p;
     *        double disparity = max(p_matched[i].u1p - p_matched[i].u2p,0.0001f);
     *        A(i*2+0,0) = uc*vp;
     *        A(i*2+0,1) = up*uc+f*f;
     *        A(i*2+0,2) = f*vp;
     *        A(i*2+0,3) = -f*disparity/param.base;
     *        A(i*2+0,4) = 0;
     *        A(i*2+0,5) = uc*disparity/param.base;
     *
     *        A(i*2+1,0) = vp*vc;
     *        A(i*2+1,1) = up*vc;
     *        A(i*2+1,2) = -f*up;
     *        A(i*2+1,3) = 0;
     *        A(i*2+1,4) = -f*disparity/param.base;
     *        A(i*2+1,5) = vc*disparity/param.base;
     *
     *        b(i*2+0) = f*(up-uc);
     *        b(i*2+1) = f*(vp-vc);
     */
    /*
     *
     *        A.val[i*2+0][0] = (double)uc*vp;
     *        A.val[i*2+0][1] = up*uc+f*f;
     *        A.val[i*2+0][2]= f*vp;
     *        A.val[i*2+0][3] = -f*disparity/param.base;
     *        A.val[i*2+0][4] = 0;
     *        A.val[i*2+0][5] = uc*disparity/param.base;
     *
     *        A.val[i*2+1][0] = vp*vc;
     *        A.val[i*2+1][1] = up*vc;
     *        A.val[i*2+1][2] = -f*up;
     *        A.val[i*2+1][3]= 0;
     *        A.val[i*2+1][4] = -f*disparity/param.base;
     *        A.val[i*2+1][5] = vc*disparity/param.base;
     *
     *        B.val[i*2+0][0] = f*(up-uc);
     *        B.val[i*2+1][0] = f*(vp-vc);
     */
    /*
     * if (B.solve(A)) {
     *    bool converged = true;
     *    for (int32_t m=0; m<6; m++) {
     *        tr[m] += step_size*B.val[m][0];
     *        if (fabs(B.val[m][0])>eps)
     *            converged = false;
     *    }
     *    if (converged)
     *        return CONVERGED;
     *    else
     *        return UPDATED;
     *} else {
     *    return FAILED;
     *}
     */
}

VisualOdometryStereo::result VisualOdometryStereo::updateParameters(vector<Matcher::p_match> &p_matched,vector<int32_t> &active,vector<double> &tr,double step_size,double eps) {

    // we need at least 3 observations
    if (active.size()<3)
        return FAILED;

    // extract observations and compute predictions
    computeObservations(p_matched,active);
    computeResidualsAndJacobian(tr,active);
    // init
    Matrix A(6,6);
    Matrix B(6,1);
    // fill matrices A and B
    for (int32_t m=0; m<6; m++) {
        for (int32_t n=0; n<6; n++) {
            double a = 0;
            for (int32_t i=0; i<4*(int32_t)active.size(); i++) {
                a += J[i*6+m]*J[i*6+n];
            }
            A.val[m][n] = a;
        }
        double b = 0;
        for (int32_t i=0; i<4*(int32_t)active.size(); i++) {
            b += J[i*6+m]*(p_residual[i]);
        }
        B.val[m][0] = b;
    }

    // perform elimination
    if (B.solve(A)) {
        bool converged = true;
        for (int32_t m=0; m<6; m++) {
            tr[m] += step_size*B.val[m][0];
            if (fabs(B.val[m][0])>eps)
                converged = false;
        }
        if (converged)
            return CONVERGED;
        else
            return UPDATED;
    } else {
        return FAILED;
    }
}

void VisualOdometryStereo::computeObservations(vector<Matcher::p_match> &p_matched,vector<int32_t> &active) {


    // set all observations
    for (int32_t i=0; i<(int32_t)active.size(); i++) {
        p_observe[4*i+0] = p_matched[active[i]].u1c; // u1
        p_observe[4*i+1] = p_matched[active[i]].v1c; // v1
        p_observe[4*i+2] = p_matched[active[i]].u2c; // u2
        p_observe[4*i+3] = p_matched[active[i]].v2c; // v2
    }
}

void VisualOdometryStereo::computeResidualsAndJacobianP3p(TooN::Matrix<3,4> &tr,vector<int32_t> &active)
{
    // extract motion parameters
    double tx = tr[0][0]; double ty = tr[1][0]; double tz = tr[2][0];

    // compute rotation matrix and derivatives
    double r00 =  tr[0][1]; double r01 =  tr[0][2]; double r02 =  tr[0][3];
    double r10 =  tr[1][1]; double r11 =  tr[1][2]; double r12 =  tr[1][3];
    double r20 =  tr[2][1]; double r21 =  tr[2][2]; double r22 =  tr[2][3];

    // loop variables
    double X1p,Y1p,Z1p;
    double X1c,Y1c,Z1c,X2c;
    double X1cd,Y1cd,Z1cd;

    // for all observations do
    for (int32_t i=0; i<(int32_t)active.size(); i++) {

        // get 3d point in previous coordinate system
        X1p = X[active[i]];
        Y1p = Y[active[i]];
        Z1p = Z[active[i]];

        // compute 3d point in current left coordinate system
        X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
        Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
        Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;

        // weighting
        // 越靠近中心权重越大
        double weight = 1.0;
        if (param.reweighting)
            weight = 1.0/(fabs(p_observe[4*i+0]-param.calib.cu)/fabs(param.calib.cu) + 0.05);

        // compute 3d point in current right coordinate system
        X2c = X1c-param.base;

        // for all paramters do

        // set prediction (project via K)
        p_predict[4*i+0] = param.calib.f*X1c/Z1c+param.calib.cu; // left u
        p_predict[4*i+1] = param.calib.f*Y1c/Z1c+param.calib.cv; // left v
        p_predict[4*i+2] = param.calib.f*X2c/Z1c+param.calib.cu; // right u
        p_predict[4*i+3] = param.calib.f*Y1c/Z1c+param.calib.cv; // right v

        // set residuals
        p_residual[4*i+0] = weight*(p_observe[4*i+0]-p_predict[4*i+0]);
        p_residual[4*i+1] = weight*(p_observe[4*i+1]-p_predict[4*i+1]);
        p_residual[4*i+2] = weight*(p_observe[4*i+2]-p_predict[4*i+2]);
        p_residual[4*i+3] = weight*(p_observe[4*i+3]-p_predict[4*i+3]);
    }
}

void VisualOdometryStereo::computeResidualsAndJacobian(vector<double> &tr,vector<int32_t> &active) {

    // extract motion parameters
    double rx = tr[0]; double ry = tr[1]; double rz = tr[2];
    double tx = tr[3]; double ty = tr[4]; double tz = tr[5];

    // precompute sine/cosine
    double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
    double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);

    // compute rotation matrix and derivatives
    double r00    = +cy*cz;          double r01    = -cy*sz;          double r02    = +sy;
    double r10    = +sx*sy*cz+cx*sz; double r11    = -sx*sy*sz+cx*cz; double r12    = -sx*cy;
    double r20    = -cx*sy*cz+sx*sz; double r21    = +cx*sy*sz+sx*cz; double r22    = +cx*cy;
    double rdrx10 = +cx*sy*cz-sx*sz; double rdrx11 = -cx*sy*sz-sx*cz; double rdrx12 = -cx*cy;
    double rdrx20 = +sx*sy*cz+cx*sz; double rdrx21 = -sx*sy*sz+cx*cz; double rdrx22 = -sx*cy;
    double rdry00 = -sy*cz;          double rdry01 = +sy*sz;          double rdry02 = +cy;
    double rdry10 = +sx*cy*cz;       double rdry11 = -sx*cy*sz;       double rdry12 = +sx*sy;
    double rdry20 = -cx*cy*cz;       double rdry21 = +cx*cy*sz;       double rdry22 = -cx*sy;
    double rdrz00 = -cy*sz;          double rdrz01 = -cy*cz;
    double rdrz10 = -sx*sy*sz+cx*cz; double rdrz11 = -sx*sy*cz-cx*sz;
    double rdrz20 = +cx*sy*sz+sx*cz; double rdrz21 = +cx*sy*cz-sx*sz;

    // loop variables
    double X1p,Y1p,Z1p;
    double X1c,Y1c,Z1c,X2c;
    double X1cd,Y1cd,Z1cd;

    // for all observations do
    for (int32_t i=0; i<(int32_t)active.size(); i++) {

        // get 3d point in previous coordinate system
        X1p = X[active[i]];
        Y1p = Y[active[i]];
        Z1p = Z[active[i]];

        // compute 3d point in current left coordinate system
        X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
        Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
        Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;

        // weighting
        // 越靠近中心权重越大
        double weight = 1.0;
        if (param.reweighting)
            weight = 1.0/(fabs(p_observe[4*i+0]-param.calib.cu)/fabs(param.calib.cu) + 0.05);

        // compute 3d point in current right coordinate system
        X2c = X1c-param.base;

        // for all paramters do
        for (int32_t j=0; j<6; j++) {

            // derivatives of 3d pt. in curr. left coordinates wrt. param j
            switch (j) {
                case 0: X1cd = 0;
                        Y1cd = rdrx10*X1p+rdrx11*Y1p+rdrx12*Z1p;
                        Z1cd = rdrx20*X1p+rdrx21*Y1p+rdrx22*Z1p;
                        break;
                case 1: X1cd = rdry00*X1p+rdry01*Y1p+rdry02*Z1p;
                        Y1cd = rdry10*X1p+rdry11*Y1p+rdry12*Z1p;
                        Z1cd = rdry20*X1p+rdry21*Y1p+rdry22*Z1p;
                        break;
                case 2: X1cd = rdrz00*X1p+rdrz01*Y1p;
                        Y1cd = rdrz10*X1p+rdrz11*Y1p;
                        Z1cd = rdrz20*X1p+rdrz21*Y1p;
                        break;
                case 3: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
                case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
                case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
            }

            // set jacobian entries (project via K)
            J[(4*i+0)*6+j] = weight*param.calib.f*(X1cd*Z1c-X1c*Z1cd)/(Z1c*Z1c); // left u'
            J[(4*i+1)*6+j] = weight*param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // left v'
            J[(4*i+2)*6+j] = weight*param.calib.f*(X1cd*Z1c-X2c*Z1cd)/(Z1c*Z1c); // right u'
            J[(4*i+3)*6+j] = weight*param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // right v'
        }

        // set prediction (project via K)
        p_predict[4*i+0] = param.calib.f*X1c/Z1c+param.calib.cu; // left u
        p_predict[4*i+1] = param.calib.f*Y1c/Z1c+param.calib.cv; // left v
        p_predict[4*i+2] = param.calib.f*X2c/Z1c+param.calib.cu; // right u
        p_predict[4*i+3] = param.calib.f*Y1c/Z1c+param.calib.cv; // right v

        // set residuals
        p_residual[4*i+0] = weight*(p_observe[4*i+0]-p_predict[4*i+0]);
        p_residual[4*i+1] = weight*(p_observe[4*i+1]-p_predict[4*i+1]);
        p_residual[4*i+2] = weight*(p_observe[4*i+2]-p_predict[4*i+2]);
        p_residual[4*i+3] = weight*(p_observe[4*i+3]-p_predict[4*i+3]);
    }
}

