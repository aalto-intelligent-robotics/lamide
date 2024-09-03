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
#ifndef NDTMATCHERD2DSC_HH
#define NDTMATCHERD2DSC_HH

#include <ndt_registration_lamide/ndt_matcher_d2d.h>
namespace perception_oru
{
/**
 * This class extends the NDT / NDT registration to also incorporate a soft constraint using an egomotion estimate
 */
class NDTMatcherD2DSC : public perception_oru::NDTMatcherD2D
{
public:
    NDTMatcherD2DSC() : NDTMatcherD2D(), Hessian(6,6),score_gradient(6,1),Cov(6,6) //column vectors, pose_increment_v(6,1)
    {
      nb_match_calls = 0;
      nb_success_reg = 0;
      only_xy_motion = false;
      lock_zrp_motion = false;
    }
    NDTMatcherD2DSC(bool lock_zrp_mot) : NDTMatcherD2D(), Hessian(6,6),score_gradient(6,1), Cov(6,6)
    {
      nb_match_calls = 0;
      nb_success_reg = 0;
      only_xy_motion = false;
      lock_zrp_motion = lock_zrp_mot;
    }
    NDTMatcherD2DSC(bool _isIrregularGrid,
                    bool useDefaultGridResolutions, std::vector<double> _resolutions) : NDTMatcherD2D(_isIrregularGrid,useDefaultGridResolutions,_resolutions), Hessian(6,6),score_gradient(6,1), Cov(6,6)
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
    bool match( const NDTMap& target,
                const NDTMap& source,
                Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T,
                const Eigen::MatrixXd& Tcov);

    //perform line search to find the best descent rate (Mohre&Thuente)
    double lineSearchMTSC(Eigen::Matrix<double,6,1> &increment,
                          std::vector<NDTCell*> &sourceNDT,
                          const NDTMap &targetNDT,
                          const Eigen::Matrix<double,6,1> &localpose,
                          const Eigen::MatrixXd &Q,
                          const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T);



    // compare the difference in scores for different local pose offsets. The assumption is that the T and TCov is expressed in the same frame
    void scoreComparision( const NDTMap& targetNDT,
                           const NDTMap& scoureNDT,
                           const Eigen::Affine3d& T ,
                           const Eigen::MatrixXd& Tcov,
                           double &score_NDT,
                           double &score_NDT_SC,
                           const Eigen::Affine3d &offset,
                           const Eigen::Affine3d &odom_offset,
                           double alpha);


  bool only_xy_motion;
  bool lock_zrp_motion;
  double a;
  int info;
  Eigen::MatrixXd Hessian, score_gradient, Cov;


protected:




};


double computeScoreMahalanobis(const Eigen::Matrix<double,6,1> &x, const Eigen::MatrixXd &Q);

Eigen::Matrix<double,6,1> computeGradientMahalanobis(const Eigen::Matrix<double,6,1> &x, const Eigen::MatrixXd &Q);

Eigen::MatrixXd computeHessianMahalanobis(const Eigen::MatrixXd &Q);
} // namespace


#endif
