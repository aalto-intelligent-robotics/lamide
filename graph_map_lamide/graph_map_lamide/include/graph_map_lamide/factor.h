#ifndef GRAPHFACTOR_H
#define GRAPHFACTOR_H
#include "Eigen/Dense"
#include "boost/serialization/serialization.hpp"
#include "graph_map_lamide/map_node.h"
#include "graph_map_lamide/map_type.h"
#include "graphfactory.h"
#include "ndt_generic_lamide/serialization.h"
namespace perception_oru
{
namespace graph_map
{
typedef enum factoryype
{
    observationFactor = 0,
    poseFactor = 1
} factorType;
class factor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    factor();

    factor(MapNodePtr prev, NodePtr next, const Eigen::Affine3d& diff, const Matrix6d& cov);

    virtual NodePtr GetPrev()
    {
        return prev_;
    }

    virtual NodePtr GetNext()
    {
        return next_;
    }

    virtual bool Connects(NodePtr node);

    virtual void GetNodes(NodePtr& prev, NodePtr& next)
    {
        prev = prev_;
        next = next_;
    }

    Eigen::Affine3d GetTransform(NodePtr prev, NodePtr next, bool& status);

    void UpdateFactor(const Eigen::Affine3d& diff)
    {
        diff_ = diff;
    }

    virtual Eigen::Affine3d GetDiff() const
    {
        return diff_;
    }

    virtual Matrix6d GetCovariance() const
    {
        return covar_;
    }

protected:
    // factorType factortype_;
    Eigen::Affine3d diff_;
    NodePtr prev_, next_;
    Matrix6d covar_;

private:
    friend class GraphFactory;
    friend class boost::serialization::access;
    template <class Archive> void serialize(Archive& ar, const unsigned int version)
    {
        // ar & factortype_;
        ar& diff_;
        ar& prev_;
        ar& next_;
        ar& covar_;
    }
};
} // namespace graph_map
} // namespace perception_oru

#endif // FACTOR_H
