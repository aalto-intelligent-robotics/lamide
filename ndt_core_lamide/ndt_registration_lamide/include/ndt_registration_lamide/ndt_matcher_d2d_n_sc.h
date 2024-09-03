/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AASS Research Center nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#pragma once
#include "stdio.h"
#include <ndt_registration_lamide/ndt_matcher_d2d.h>
#include "ndt_registration_lamide/ndt_matcher_d2d_n.h"
#include <ndt_registration_lamide/ndt_matcher_d2d_sc.h>
namespace perception_oru
{
/**
 * This class extends the NDT / NDT registration to also incorporate a soft constraint using an egomotion estimate
 */
class NDTMatcherD2DNSC : public perception_oru::NDTMatcherD2DN
{
public:
    NDTMatcherD2DNSC() : NDTMatcherD2DN()
    {
      nb_match_calls = 0;
      nb_success_reg = 0;
    }
    NDTMatcherD2DNSC(bool _isIrregularGrid,
                    bool useDefaultGridResolutions, std::vector<double> _resolutions) : NDTMatcherD2DN(_isIrregularGrid,useDefaultGridResolutions,_resolutions)
    {
      nb_match_calls = 0;
      nb_success_reg = 0;
    }


  /**
     * Registers a point cloud to an NDT structure.
     * \param  fixed
     *   Reference data.
     * \param  moving
     *   The output transformation registers this point cloud to \c fixed.
     * \param  T
     *   This is an input/output parameter. The initial value of \c T
     *   gives the initial pose estimate of \c moving. When the
     *   algorithm terminates, \c T holds the registration result.
     * \param Tcov
     *   Covariance of the input pose parameter. This will be used to form
     *   an additional cost in the objective.
     */
    bool match( const std::vector<NDTMap*>& target,
                std::vector<NDTMap*>& source,
                Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
                const Eigen::MatrixXd& Tcov);


    double lineSearchMTSCN(Eigen::Matrix<double,6,1> &increment,
                                           std::vector<std::vector<NDTCell*> > &sourceNDT,
                                           const std::vector<NDTMap*> &targetNDT,
                                           const Eigen::Matrix<double,6,1> &localpose,
                                           const Eigen::MatrixXd &Q,
                                           const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T);




  double a;
  int info;

protected:



};
} // namespace



