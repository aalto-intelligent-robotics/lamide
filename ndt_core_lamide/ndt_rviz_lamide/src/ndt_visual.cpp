#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <rviz/ogre_helpers/shape.h>
#include "ndt_rviz_lamide/ndt_visual.hpp"

namespace perception_oru
{
NDTVisual::NDTVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
{
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
    NDT_elipsoid_.reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
}

NDTVisual::~NDTVisual()
{
    scene_manager_->destroySceneNode(frame_node_);
}
void makeRightHanded(Eigen::Matrix3d& eigenvectors, Eigen::Vector3d& eigenvalues)
{
    // Note that sorting of eigenvalues may end up with left-hand coordinate system.
    // So here we correctly sort it so that it does end up being righ-handed and normalised.
    Eigen::Vector3d c0 = eigenvectors.block<3, 1>(0, 0);
    c0.normalize();
    Eigen::Vector3d c1 = eigenvectors.block<3, 1>(0, 1);
    c1.normalize();
    Eigen::Vector3d c2 = eigenvectors.block<3, 1>(0, 2);
    c2.normalize();
    Eigen::Vector3d cc = c0.cross(c1);
    if (cc.dot(c2) < 0)
    {
        eigenvectors << c1, c0, c2;
        double e = eigenvalues[0];
        eigenvalues[0] = eigenvalues[1];
        eigenvalues[1] = e;
    }
    else
    {
        eigenvectors << c0, c1, c2;
    }
}

void NDTVisual::setCell(ndt_map_lamide::NDTCellRGBMsg cell, double resolution)
{
    Ogre::Vector3 position(cell.mean_x, cell.mean_y, cell.mean_z);
    // eigen values aka size of the elipsoid
    Eigen::Matrix3d cov;
    int m_itr = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cov(i, j) = cell.cov_matrix[m_itr];
            m_itr++;
        }
    }
    /*Eigen::Matrix3d m_eigVec = Eigen::Matrix3d::Zero(3,3);
      Eigen::Matrix3d m_eigVal = Eigen::Matrix3d::Zero(3,3);
      Eigen::EigenSolver<Eigen::Matrix3d> es(cov);*/

    /*m_eigVal = es.pseudoEigenvalueMatrix();
      m_eigVec = es.pseudoEigenvectors();
      m_eigVal = m_eigVal.cwiseSqrt();*/
    Eigen::Vector3d eigenvalues(Eigen::Vector3d::Identity());
    Eigen::Matrix3d eigenvectors(Eigen::Matrix3d::Zero());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
    if (eigensolver.info() == Eigen::Success)
    {
        eigenvalues = eigensolver.eigenvalues();
        eigenvectors = eigensolver.eigenvectors();
    }
    else
    {
        ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values for position. Is the "
                             "covariance matrix correct?");
        eigenvalues =
            Eigen::Vector3d::Zero(); // Setting the scale to zero will hide it on the screen
        eigenvectors = Eigen::Matrix3d::Identity();
    }
    makeRightHanded(eigenvectors, eigenvalues);
    Eigen::Quaterniond orient_eig = Eigen::Quaterniond(eigenvectors);
    Eigen::Vector3d scale_eig;
    scale_eig(0) = 2 * std::sqrt(eigenvalues[0]);
    scale_eig(1) = 2 * std::sqrt(eigenvalues[1]);
    scale_eig(2) = 2 * std::sqrt(eigenvalues[2]);
    /*Eigen::Quaternion<double> q(m_eigVec);
      Ogre::Vector3 scale(3*m_eigVal(0,0),3*m_eigVal(1,1),3*m_eigVal(2,2));
      Ogre::Quaternion orient(q.w(),q.x(),q.y(),q.z());*/

    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> Sol (cov);
    // Eigen::Matrix3d evecs;
    // Eigen::Vector3d evals;
    // evecs = Sol.eigenvectors().real();
    // evals = Sol.eigenvalues().real();
    // Eigen::Quaternion<double> q(evecs);
    // Ogre::Vector3 scale(30*evals(0),30*evals(1),30*evals(2));
    // Ogre::Quaternion orient(q.w(),q.x(),q.y(),q.z());
    Ogre::Vector3 scale(2 * std::sqrt(eigenvalues[0]), 2 * std::sqrt(eigenvalues[1]),
                        2 * std::sqrt(eigenvalues[2]));
    Ogre::Quaternion orient(orient_eig.w(), orient_eig.x(), orient_eig.y(), orient_eig.z());

    NDT_elipsoid_->setScale(scale);
    NDT_elipsoid_->setPosition(position);
    NDT_elipsoid_->setOrientation(orient);
}

void NDTVisual::setCellRGB(ndt_map_lamide::NDTCellRGBMsg cell, double resolution)
{
    Ogre::Vector3 position(cell.mean_x, cell.mean_y, cell.mean_z);
    // eigen values aka size of the elipsoid
    Eigen::Matrix3d cov;
    int m_itr = 0;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cov(i, j) = cell.cov_matrix[m_itr];
            m_itr++;
        }
    }

    Eigen::Vector3d eigenvalues(Eigen::Vector3d::Identity());
    Eigen::Matrix3d eigenvectors(Eigen::Matrix3d::Zero());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
    if (eigensolver.info() == Eigen::Success)
    {
        eigenvalues = eigensolver.eigenvalues();
        eigenvectors = eigensolver.eigenvectors();
    }
    else
    {
        ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values for position. Is the "
                             "covariance matrix correct?");
        eigenvalues =
            Eigen::Vector3d::Zero(); // Setting the scale to zero will hide it on the screen
        eigenvectors = Eigen::Matrix3d::Identity();
    }
    makeRightHanded(eigenvectors, eigenvalues);
    Eigen::Quaterniond orient_eig = Eigen::Quaterniond(eigenvectors);
    Eigen::Vector3d scale_eig;
    scale_eig(0) = 2 * std::sqrt(eigenvalues[0]);
    scale_eig(1) = 2 * std::sqrt(eigenvalues[1]);
    scale_eig(2) = 2 * std::sqrt(eigenvalues[2]);

    Ogre::Vector3 scale(2 * std::sqrt(eigenvalues[0]), 2 * std::sqrt(eigenvalues[1]),
                        2 * std::sqrt(eigenvalues[2]));
    Ogre::Quaternion orient(orient_eig.w(), orient_eig.x(), orient_eig.y(), orient_eig.z());

    NDT_elipsoid_->setScale(scale);
    NDT_elipsoid_->setPosition(position);
    NDT_elipsoid_->setOrientation(orient);
}

void NDTVisual::setFramePosition(const Ogre::Vector3& position)
{
    frame_node_->setPosition(position);
}

void NDTVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
    frame_node_->setOrientation(orientation);
}

void NDTVisual::setColor(float r, float g, float b, float a)
{
    NDT_elipsoid_->setColor(r, g, b, a);
}
} // namespace perception_oru
