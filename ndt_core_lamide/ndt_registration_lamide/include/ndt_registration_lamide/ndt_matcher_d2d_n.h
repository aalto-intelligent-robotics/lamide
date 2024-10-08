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
#ifndef NDTMATCHERD2DN_HH
#define NDTMATCHERD2DN_HH

#include <ndt_registration_lamide/ndt_matcher_d2d.h>
namespace perception_oru
{
/**
 * This class extends the NDT / NDT registration to also incorporate a soft constraint using an egomotion estimate
 */
class NDTMatcherD2DN : public perception_oru::NDTMatcherD2D
{
public:
    NDTMatcherD2DN() : NDTMatcherD2D()
    {
      nb_match_calls = 0;
      nb_success_reg = 0;
      only_xy_motion = false;
      lock_zrp_motion = false;
    }
    NDTMatcherD2DN(bool lock_zrp_mot) : NDTMatcherD2D()
    {
      nb_match_calls = 0;
      nb_success_reg = 0;
      only_xy_motion = false;
      lock_zrp_motion = lock_zrp_mot;
    }
    NDTMatcherD2DN(bool _isIrregularGrid,
                    bool useDefaultGridResolutions, std::vector<double> _resolutions) : NDTMatcherD2D(_isIrregularGrid,useDefaultGridResolutions,_resolutions)
    {
      nb_match_calls = 0;
      nb_success_reg = 0;
      only_xy_motion = false;
      lock_zrp_motion = false;
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
              bool useInitialGuess = false);


  double derivativesNDTN(
      const std::vector<std::vector<NDTCell*> > &source,
      const std::vector<NDTMap*> &target,
      Eigen::MatrixXd &score_gradient,
      Eigen::MatrixXd &Hessian,
      bool computeHessian
  );

  //perform line search to find the best descent rate (Mohre&Thuente)
  double lineSearchMTN(Eigen::Matrix<double,6,1> &increment,
                        std::vector<std::vector<NDTCell*> > &sourceNDT,
                        const std::vector<NDTMap*> &targetNDT);


  bool only_xy_motion;
  bool lock_zrp_motion;
  double a;
  int info;

protected:


};
} // namespace


#endif
