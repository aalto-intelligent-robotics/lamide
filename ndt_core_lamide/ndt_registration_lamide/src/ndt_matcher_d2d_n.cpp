#include <ndt_map_lamide/ndt_cell.h>
#include <ndt_map_lamide/lazy_grid.h>
#include <ndt_map_lamide/pointcloud_utils.h>
#include <ndt_registration_lamide/ndt_matcher_d2d_n.h>
#include <ndt_generic_lamide/eigen_utils.h>

#include <Eigen/Eigen>
#include <fstream>
#include <omp.h>
#include <sys/time.h>


namespace perception_oru
{

double NDTMatcherD2DN::lineSearchMTN(Eigen::Matrix<double,6,1> &increment,
                                     std::vector<std::vector<NDTCell*> > &sourceNDT,
                                     const std::vector<NDTMap*> &targetNDT)
{
//  return lineSearchMT(increment, sourceNDT[0], *(targetNDT[0]));


  // default params
  double stp = 1.0; //default step
  double recoverystep = 0.1;
  double dginit = 0.0;
  double ftol = 0.11111; //epsilon 1
  double gtol = 0.99999; //epsilon 2
  double stpmax = 4.0;
  double stpmin = 0.001;
  int maxfev = 40; //max function evaluations
  double xtol = 0.01; //window of uncertainty around the optimal step

  //my temporary variables
  std::vector<std::vector<NDTCell*> > sourceNDTHere(sourceNDT.size());
  double score_init = 0.0;

  Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> ps;
  ps.setIdentity();

  Eigen::Matrix<double,6,1> scg_here;
  Eigen::MatrixXd pincr(6,1), score_gradient_here(6,1);
  Eigen::MatrixXd pseudoH(6,6);
  Eigen::Vector3d eulerAngles;
  /////

  int info = 0;			// return code
  int infoc = 1;		// return code for subroutine cstep

  // Compute the initial gradient in the search direction and check
  // that s is a descent direction.

  //we want to maximize s, so we should minimize -s
  //score_init = scoreNDT(sourceNDT,targetNDT);

  //gradient directions are opposite for the negated function
  //score_gradient_init = -score_gradient_init;

  //  cout<<"score_init "<<score_init<<endl;
  //  cout<<"score_gradient_init "<<score_gradient_init.transpose()<<endl;
  //  cout<<"increment "<<increment.transpose()<<endl;
  score_gradient_here.setZero();
  score_init = derivativesNDTN(sourceNDT,targetNDT,score_gradient_here,pseudoH,false);
  scg_here = score_gradient_here;
  dginit = increment.dot(scg_here);

  //  cout<<"dginit "<<dginit<<endl;

  if (dginit >= 0.0)
  {
    std::cout << "MoreThuente::cvsrch - wrong direction (dginit = " << dginit << ")" << std::endl;

    increment = -increment;
    dginit = -dginit;

    if (dginit >= 0.0)
    {
      for (size_t j = 0; j < sourceNDTHere.size(); j++) {
        for(unsigned int i=0; i<sourceNDTHere[j].size(); i++)
        {
          if(sourceNDTHere[j][i]!=NULL)
            delete sourceNDTHere[j][i];
        }
        sourceNDTHere[j].clear();
      }
      return recoverystep;
    }
  }
  else
  {
    //     cout<<"correct direction (dginit = " << dginit << ")" << endl;
  }

  // Initialize local variables.

  bool brackt = false;		// has the soln been bracketed?
  bool stage1 = true;		// are we in stage 1?
  int nfev = 0;			// number of function evaluations
  double dgtest = ftol * dginit; // f for curvature condition
  double width = stpmax - stpmin; // interval width
  double width1 = 2 * width;	// ???

  //cout<<"dgtest "<<dgtest<<endl;
  // initial function value
  double finit = 0.0;
  finit = score_init;

  // The variables stx, fx, dgx contain the values of the step,
  // function, and directional derivative at the best step.  The
  // variables sty, fy, dgy contain the value of the step, function,
  // and derivative at the other endpoint of the interval of
  // uncertainty.  The variables stp, f, dg contain the values of the
  // step, function, and derivative at the current step.

  double stx = 0.0;
  double fx = finit;
  double dgx = dginit;
  double sty = 0.0;
  double fy = finit;
  double dgy = dginit;

  // Get the linear solve tolerance for adjustable forcing term
  //double eta_original = -1.0;
  //double eta = 0.0;
  //eta = eta_original;

  // Start of iteration.

  double stmin, stmax;
  double fm, fxm, fym, dgm, dgxm, dgym;

  while (1)
  {
    // Set the minimum and maximum steps to correspond to the present
    // interval of uncertainty.
    if (brackt)
    {
      stmin = MoreThuente::min(stx, sty);
      stmax = MoreThuente::max(stx, sty);
    }
    else
    {
      stmin = stx;
      stmax = stp + 4 * (stp - stx);
    }

    // Force the step to be within the bounds stpmax and stpmin.
    stp = MoreThuente::max(stp, stpmin);
    stp = MoreThuente::min(stp, stpmax);

    // If an unusual termination is to occur then let stp be the
    // lowest point obtained so far.

    if ((brackt && ((stp <= stmin) || (stp >= stmax))) ||
        (nfev >= maxfev - 1) || (infoc == 0) ||
        (brackt && (stmax - stmin <= xtol * stmax)))
    {
      stp = stx;
    }

    // Evaluate the function and gradient at stp
    // and compute the directional derivative.
    ///////////////////////////////////////////////////////////////////////////

    pincr = stp*increment;

    ps = Eigen::Translation<double,3>(pincr(0),pincr(1),pincr(2))*
        Eigen::AngleAxisd(pincr(3),Eigen::Vector3d::UnitX())*
        Eigen::AngleAxisd(pincr(4),Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(pincr(5),Eigen::Vector3d::UnitZ());

    for (size_t j = 0; j < sourceNDTHere.size(); j++) {
      for(unsigned int i=0; i<sourceNDTHere[j].size(); i++)
      {
        if(sourceNDTHere[j][i]!=NULL)
          delete sourceNDTHere[j][i];
      }
      sourceNDTHere[j].clear();
    }

    for (size_t j = 0; j < sourceNDT.size(); j++) {
      for(unsigned int i=0; i<sourceNDT[j].size(); i++)
      {
        NDTCell *cell = sourceNDT[j][i];
        if(cell!=NULL){
         Eigen::Vector3d mean = cell->getMean();
         Eigen::Matrix3d cov = cell->getCov();
         mean = ps*mean;
         cov = ps.rotation()*cov*ps.rotation().transpose();
         NDTCell* nd = (NDTCell*)cell->copy();
         nd->setMean(mean);
         nd->setCov(cov);
         sourceNDTHere[j].push_back(nd);
        }
      }
    }

    double f = 0.0;
    score_gradient_here.setZero();

    /*f = scoreNDT(sourceNDT,targetNDT,ps);
        derivativesNDT(sourceNDT,targetNDT,ps,score_gradient_here,pseudoH,false);
        std::cout<<"scg1  " <<score_gradient_here.transpose()<<std::endl;
        */

    //option 2:
    //f = scoreNDT(sourceNDTHere,targetNDT);
    f = derivativesNDTN(sourceNDTHere,targetNDT,score_gradient_here,pseudoH,false);
    //std::cout<<"scg2  " <<score_gradient_here.transpose()<<std::endl;


    //cout<<"incr " <<pincr.transpose()<<endl;
    //cout<<"score (f) "<<f<<endl;

    double dg = 0.0;
    scg_here = score_gradient_here;
    dg = increment.dot(scg_here);

    //VALGRIND_CHECK_VALUE_IS_DEFINED(dg);
    //cout<<"dg = "<<dg<<endl;
    nfev ++;

    ///////////////////////////////////////////////////////////////////////////

    //cout<<"consider step "<<stp<<endl;
    // Armijo-Goldstein sufficient decrease
    double ftest1 = finit + stp * dgtest;
    //cout<<"ftest1 is "<<ftest1<<endl;

    // Test for convergence.

    if ((brackt && ((stp <= stmin) || (stp >= stmax))) || (infoc == 0))
      info = 6;			// Rounding errors

    if ((stp == stpmax) && (f <= ftest1) && (dg <= dgtest))
      info = 5;			// stp=stpmax

    if ((stp == stpmin) && ((f > ftest1) || (dg >= dgtest)))
      info = 4;			// stp=stpmin

    if (nfev >= maxfev)
      info = 3;			// max'd out on fevals

    if (brackt && (stmax-stmin <= xtol*stmax))
      info = 2;			// bracketed soln

    // RPP sufficient decrease test can be different
    bool sufficientDecreaseTest = false;
    sufficientDecreaseTest = (f <= ftest1);  // Armijo-Golstein

    //cout<<"ftest2 "<<gtol*(-dginit)<<endl;
    //cout<<"sufficientDecrease? "<<sufficientDecreaseTest<<endl;
    //cout<<"curvature ok? "<<(fabs(dg) <= gtol*(-dginit))<<endl;
    if ((sufficientDecreaseTest) && (fabs(dg) <= gtol*(-dginit)))
      info = 1;			// Success!!!!

    if (info != 0) 		// Line search is done
    {
      if (info != 1) 		// Line search failed
      {
        // RPP add
        // counter.incrementNumFailedLineSearches();

        //if (recoveryStepType == Constant)
        stp = recoverystep;

        //newgrp.computeX(oldgrp, dir, stp);

        //message = "(USING RECOVERY STEP!)";

      }
      else 			// Line search succeeded
      {
        //message = "(STEP ACCEPTED!)";
      }

      //print.printStep(nfev, stp, finit, f, message);

      // Returning the line search flag
      //cout<<"LineSearch::"<<message<<" info "<<info<<endl;
      for (size_t j = 0; j < sourceNDTHere.size(); j++) {
        for(unsigned int i=0; i<sourceNDTHere[j].size(); i++)
        {
          if(sourceNDTHere[j][i]!=NULL)
            delete sourceNDTHere[j][i];
        }
        sourceNDTHere[j].clear();
      }
      //      std::cout<<"nfev = "<<nfev<<std::endl;
      return stp;

    } // info != 0

    // RPP add
    //counter.incrementNumIterations();

    // In the first stage we seek a step for which the modified
    // function has a nonpositive value and nonnegative derivative.

    if (stage1 && (f <= ftest1) && (dg >= MoreThuente::min(ftol, gtol) * dginit))
    {
      stage1 = false;
    }

    // A modified function is used to predict the step only if we have
    // not obtained a step for which the modified function has a
    // nonpositive function value and nonnegative derivative, and if a
    // lower function value has been obtained but the decrease is not
    // sufficient.

    if (stage1 && (f <= fx) && (f > ftest1))
    {

      // Define the modified function and derivative values.

      fm = f - stp * dgtest;
      fxm = fx - stx * dgtest;
      fym = fy - sty * dgtest;
      dgm = dg - dgtest;
      dgxm = dgx - dgtest;
      dgym = dgy - dgtest;

      // Call cstep to update the interval of uncertainty
      // and to compute the new step.

      //VALGRIND_CHECK_VALUE_IS_DEFINED(dgm);
      infoc = MoreThuente::cstep(stx,fxm,dgxm,sty,fym,dgym,stp,fm,dgm,
                                 brackt,stmin,stmax);

      // Reset the function and gradient values for f.

      fx = fxm + stx*dgtest;
      fy = fym + sty*dgtest;
      dgx = dgxm + dgtest;
      dgy = dgym + dgtest;

    }

    else
    {

      // Call cstep to update the interval of uncertainty
      // and to compute the new step.

      //VALGRIND_CHECK_VALUE_IS_DEFINED(dg);
      infoc = MoreThuente::cstep(stx,fx,dgx,sty,fy,dgy,stp,f,dg,
                                 brackt,stmin,stmax);

    }

    // Force a sufficient decrease in the size of the
    // interval of uncertainty.

    if (brackt)
    {
      if (fabs(sty - stx) >= 0.66 * width1)
        stp = stx + 0.5 * (sty - stx);
      width1 = width;
      width = fabs(sty-stx);
    }

  } // while-loop
}


//compute the score gradient of a point cloud + transformation to an NDT
double NDTMatcherD2DN::derivativesNDTN(
    const std::vector<std::vector<NDTCell*> > &sourceNDT,
    const std::vector<NDTMap*> &targetNDT,
    Eigen::MatrixXd &score_gradient,
    Eigen::MatrixXd &Hessian,
    bool computeHessian
    )
{
  double score = 0;
  Eigen::MatrixXd Hessian_(6,6), score_gradient_(6,1);

  if (sourceNDT.empty()) {

    return 0.;
  }
  if (sourceNDT.size() != targetNDT.size()) {

    return 0.;
  }

  for (size_t j = 0; j < sourceNDT.size(); j++) {
    if (j == 0) {
      score += derivativesNDT(sourceNDT[j], *(targetNDT[j]), score_gradient, Hessian, computeHessian);
    }
    else {
      score += derivativesNDT(sourceNDT[j], *(targetNDT[j]), score_gradient_, Hessian_, computeHessian);
      score_gradient += score_gradient_;
      Hessian += Hessian_;
    }
  }
  return score;
}


bool NDTMatcherD2DN::match( const std::vector<NDTMap*>& targetNDT,
                             std::vector<NDTMap*>& sourceNDT,
                             Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor>& T ,
                             bool useInitialGuess)
{
  //locals
  bool convergence = false;
  //double score=0;
  double score_best = INT_MAX;
  //double DELTA_SCORE = 0.0005;
  //double NORM_MAX = current_resolution, ROT_MAX = M_PI/10; //
  int itr_ctr = 0;
  //double alpha = 0.95;
  double step_size = 1;
  Eigen::Matrix<double,6,1>  pose_increment_v, scg, pose_local_v, pose_increment_v_Tframe;
  Eigen::MatrixXd Hessian(6,6), score_gradient(6,1); //column vectors, pose_increment_v(6,1)
//  Eigen::MatrixXd Q = Tcov.inverse();


  Eigen::Matrix<double,6,1> X;
  Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> TR, Tbest;
  Eigen::Vector3d transformed_vec, mean;
  bool ret = true;

  if(!useInitialGuess)
  {
    T.setIdentity();
  }

  Eigen::Affine3d Tinit = T;

  Tbest = T;
  pose_local_v.setZero();

  Eigen::Array<double,6,1> weights;
  std::vector<std::vector<NDTCell*> > nextNDT;
  nextNDT.resize(sourceNDT.size());

  for (size_t j = 0; j < sourceNDT.size(); j++)
  {

    nextNDT[j] = sourceNDT[j]->pseudoTransformNDT(T);
  }

  int iter = 0;
  int best_iter;
  this->nb_match_calls++;
  bool found_improvement = false;

  double alpha = 0.0; //

  while(!convergence)
  {
    iter++;
    TR.setIdentity();
    Hessian.setZero();
    score_gradient.setZero();

    double score_here = derivativesNDTN(nextNDT,targetNDT,score_gradient,Hessian,true);
    X = pose_local_v;
    //std::cout << "D2DN - score_here : " << score_here << std::endl;
    //std::cout << "D2DN - score_gradient : " << score_gradient << std::endl;

    scg = score_gradient;
    if(score_here < score_best)
    {
      Tbest = T;
      score_best = score_here;
      if (iter > 1) {
        if (!found_improvement) {
          this->nb_success_reg++;
          found_improvement = true;
          alpha = 1.;
        }
        std::cout << "[" << iter << "] : " << pose_increment_v.transpose() << " -- " << score_here << std::endl;
      }
    }
    else {
      //            std::cerr << "X : " << X.transpose() << std::flush;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6> > Sol (Hessian);
    Eigen::Matrix<double,6,1> evals = Sol.eigenvalues().real();
    double minCoeff = evals.minCoeff();
    double maxCoeff = evals.maxCoeff();
    if(minCoeff < 0)  //|| evals.minCoeff()) // < 10e-5*evals.maxCoeff())
    {
      if(regularize) {
        Eigen::Matrix<double,6,6> evecs = Sol.eigenvectors().real();
        double regularizer = score_gradient.norm();
        regularizer = regularizer + minCoeff > 0 ? regularizer : 0.001*maxCoeff - minCoeff;
        //double regularizer = 0.001*maxCoeff - minCoeff;
        Eigen::Matrix<double,6,1> reg;
        //ugly
        reg<<regularizer,regularizer,regularizer,regularizer,regularizer,regularizer;

        evals += reg;
        Eigen::Matrix<double,6,6> Lam;
        Lam = evals.asDiagonal();
        Hessian = evecs*Lam*(evecs.transpose());
      } else {
        if(score_here > score_best)
        {
          T = Tbest;
        }
        //de-alloc nextNDT

        for (size_t j = 0; j < nextNDT.size(); j++) {
          for(unsigned int i=0; i<nextNDT[j].size(); i++)
          {
            if(nextNDT[j][i]!=NULL)
              delete nextNDT[j][i];
          }
        }
        status_=SUCCESS;
        return true;
      }
    }

    if (score_gradient.norm()<= DELTA_SCORE)
    {
      if(score_here > score_best)
      {
        T = Tbest;
      }
      //de-alloc nextNDT

      for (size_t j = 0; j < nextNDT.size(); j++) {
        for(unsigned int i=0; i<nextNDT[j].size(); i++)
        {
          if(nextNDT[j][i]!=NULL)
            delete nextNDT[j][i];
        }
      }
      status_=SUCCESS;
      return true;
    }

    //        std::cerr << " score_here : " << score_here << " score_gradient : " << score_gradient.transpose() << std::endl;

    pose_increment_v = -Hessian.ldlt().solve(score_gradient);

    double dginit = pose_increment_v.dot(scg);
    if(dginit > 0)
    {
      //de-alloc nextNDT
      if(score_here > score_best)
      {
        T = Tbest;
      }
      //de-alloc nextNDT

      for (size_t j = 0; j < nextNDT.size(); j++) {
        for(unsigned int i=0; i<nextNDT[j].size(); i++)
        {
          if(nextNDT[j][i]!=NULL)
            delete nextNDT[j][i];
        }
      }
      return true;
    }
    //check direction here:

    if(step_control) {
      step_size = lineSearchMTN(pose_increment_v,nextNDT,targetNDT);
    } else {
      step_size = 1;
    }

    pose_increment_v = step_size*pose_increment_v;

    // const double max_step = 1.0;
    // if (pose_increment_v.norm() > max_step) {
    //   pose_increment_v.normalize();
    //   pose_increment_v *= max_step;
    // }

    TR.setIdentity();
    TR =  Eigen::Translation<double,3>(pose_increment_v(0),pose_increment_v(1),pose_increment_v(2))*
        Eigen::AngleAxis<double>(pose_increment_v(3),Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxis<double>(pose_increment_v(4),Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxis<double>(pose_increment_v(5),Eigen::Vector3d::UnitZ()) ;

    // Make sure that the pose_local_v is updated with the right frame(!)
    Eigen::Affine3d T_previous = T;
    T = TR*T;
    Eigen::Affine3d T_current = T;
    Eigen::Vector3d Tposition_diff = T_current.translation() - T_previous.translation();

    for (size_t j = 0; j < nextNDT.size(); j++) {
      for(unsigned int i=0; i<nextNDT[j].size(); i++)
      {
        //TRANSFORM
        Eigen::Vector3d meanC = nextNDT[j][i]->getMean();
        Eigen::Matrix3d covC = nextNDT[j][i]->getCov();
        meanC = TR*meanC;
        covC = TR.rotation()*covC*TR.rotation().transpose();
        nextNDT[j][i]->setMean(meanC);
        nextNDT[j][i]->setCov(covC);
      }
    }

    if(itr_ctr>0)
    {
      convergence = ((pose_increment_v.norm()) < DELTA_SCORE);
    }
    if(itr_ctr>ITR_MAX)
    {
      convergence = true;
      ret = found_improvement; //false;
      status_=MAX_ITR;
    }
    itr_ctr++;
  }

  score_gradient.setZero();
  double score_here = derivativesNDTN(nextNDT,targetNDT,score_gradient,Hessian,false);
  if(score_here > score_best)
  {
    T = Tbest;
  }

  //de-alloc nextNDT
  for (size_t j = 0; j < nextNDT.size(); j++) {
    for(unsigned int i=0; i<nextNDT[j].size(); i++)
    {
      if(nextNDT[j][i]!=NULL)
        delete nextNDT[j][i];
    }
  }
  return ret;
}
} // namespace



