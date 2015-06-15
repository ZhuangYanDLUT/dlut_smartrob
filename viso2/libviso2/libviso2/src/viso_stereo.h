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

#ifndef VISO_STEREO_H
#define VISO_STEREO_H

#include "viso.h"
#include "mcqd.h"
#include <TooN/TooN.h>
class VisualOdometryStereo : public VisualOdometry {

public:

  // stereo-specific parameters (mandatory: base)
  struct parameters : public VisualOdometry::parameters {
    double  base;             // baseline (meters)
    int32_t ransac_iters;     // number of RANSAC iterations
    double  inlier_threshold; // fundamental matrix inlier threshold
    bool    reweighting;      // lower border weights (more robust to calibration errors)
    parameters () {
      base             = 1.0;
      ransac_iters     = 100;
      inlier_threshold = 2.0;
      reweighting      = true;
    }
  };

  // constructor, takes as inpute a parameter structure
  VisualOdometryStereo (parameters param);

  // deconstructor
  ~VisualOdometryStereo ();

  // process a new images, push the images back to an internal ring buffer.
  // valid motion estimates are available after calling process for two times.
  // inputs: I1 ........ pointer to rectified left image (uint8, row-aligned)
  //         I2 ........ pointer to rectified right image (uint8, row-aligned)
  //         dims[0] ... width of I1 and I2 (both must be of same size)
  //         dims[1] ... height of I1 and I2 (both must be of same size)
  //         dims[2] ... bytes per line (often equal to width)
  //         replace ... replace current images with I1 and I2, without copying last current
  //                     images to previous images internally. this option can be used
  //                     when small/no motions are observed to obtain Tr_delta wrt
  //                     an older coordinate system / time step than the previous one.
  // output: returns false if an error occured
  bool process (uint8_t *I1,uint8_t *I2,int32_t* dims,bool replace=false);

  using VisualOdometry::process;
	std::vector<Matcher::p_match> getMatches () { return p_matched1; }


private:

  std::vector<double>  estimateMotion (std::vector<Matcher::p_match> p_matched);
  std::vector<double>  estimateMotionLinear (std::vector<Matcher::p_match> p_matched);
  std::vector<double>  estimateMotionMaxClique (std::vector<Matcher::p_match> p_matched);
  enum                 result { UPDATED, FAILED, CONVERGED };
  result               updateParameters(std::vector<Matcher::p_match> &p_matched,std::vector<int32_t> &active,std::vector<double> &tr,double step_size,double eps);
  result               updateParametersLinear(std::vector<Matcher::p_match> &p_matched,std::vector<int32_t> &active,std::vector<double> &tr,double step_size,double eps);
  result               updateParametersP3p(std::vector<Matcher::p_match> &p_matched,std::vector<int32_t> &active,TooN::Matrix<3,4> &tr,double step_size,double eps);
  void                 computeObservations(std::vector<Matcher::p_match> &p_matched,std::vector<int32_t> &active);
  void                 computeResidualsAndJacobian(std::vector<double> &tr,std::vector<int32_t> &active);
  void                 computeResidualsAndJacobianP3p(TooN::Matrix<3,4> &tr,std::vector<int32_t> &active);
  std::vector<int32_t> getInlier(std::vector<Matcher::p_match> &p_matched,std::vector<double> &tr);
  std::vector<int32_t> getInlier(std::vector<Matcher::p_match> &p_matched,TooN::Matrix<3,4> &tr);
  
  bool justice2Matches(Matcher::p_match &m1,Matcher::p_match &m2);
  void creatAdjMatrix(std::vector<Matcher::p_match>& p_matched,bool** &conn);

  double *X,*Y,*Z,*XC,*YC,*ZC;    // 3d points
  double *p_residual; // residuals (p_residual=p_observe-p_predict)
  double rx_old,ry_old,rz_old,tx_old,ty_old,tz_old;
  std::vector<Matcher::p_match>  p_matched1;
  // parameters
  parameters param;
};

#endif // VISO_STEREO_H

