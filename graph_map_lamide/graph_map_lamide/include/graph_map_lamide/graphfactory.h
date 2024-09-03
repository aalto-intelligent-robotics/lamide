#ifndef GRAPHFACTORY_H
#define GRAPHFACTORY_H
#include "Eigen/Dense"
#include "boost/shared_ptr.hpp"

#include <boost/algorithm/string.hpp>
#include <stdio.h>
#include <string>
#include <vector>

namespace perception_oru
{
namespace graph_map
{

using Eigen::Affine3d;
using Eigen::Vector3d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> EigenAffineVector;
const Matrix6d unit_covar = (Eigen::Matrix<double, 6, 6>() << 0.1,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.1,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.1,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.01,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.01,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.0,
                             0.01)
                                .finished();

/*--------------------------- TEMPLATE FOR USAGE OF THE GRAPH LIBRARY
 * ---------------------------------*/
/*!
 * The following types with name template_map_type are presented here as an example for how to
 * create your own map and registration types
 */

/*!
 * \brief templateMapType implements the map or acts as a wrapper to your existing map classes
 * \brief templateMapTypePtr is the general way the map type is passed around, based om shared
 * pointers to ensure memory no memory losses
 */
// class TemplateMapType;
// typedef boost::shared_ptr<TemplateMapType> TemplateMapTypePtr;

/*!
 * \brief templateMapParam implements the parameters for the map type
 * \brief templateMapParamPtr
 */
// class TemplateMapParam;
// typedef boost::shared_ptr<TemplateMapParam> TemplateMapParamPtr;

/*!
 * \brief templateRegType implements the registration or act as a wrapper to your existing
 * registration types. \brief templateRegTypePtr
 */
// class TemplateRegType;
// typedef boost::shared_ptr<TemplateRegType> TemplateRegTypePtr;

/*!
 * \brief templateRegTypeParam implements the parameters for the registration type <templateRegType>
 * \brief regParamPtr
 */
// class TemplateRegTypeParam;
// typedef boost::shared_ptr<TemplateRegTypeParam> TemplateRegTypeParamPtr;

/*--------------------------------END OF TEMPLATE
 * -------------------------------------------------------*/

// class NarfMapType;
// typedef boost::shared_ptr<NarfMapType> NarfMapTypePtr;

/*!
 * \brief templateMapParam implements the parameters for the map type
 * \brief templateMapParamPtr
 */
// class NarfMapParam;
// typedef boost::shared_ptr<NarfMapParam> NarfMapParamPtr;

/*!
 * \brief templateRegType implements the registration or act as a wrapper to your existing
 * registration types. \brief templateRegTypePtr
 */
// class NarfRegType;
// typedef boost::shared_ptr<NarfRegType> NarfRegTypePtr;

/*!
 * \brief templateRegTypeParam implements the parameters for the registration type <templateRegType>
 * \brief regParamPtr
 */
// class NarfRegTypeParam;
// typedef boost::shared_ptr<NarfRegTypeParam> NarfRegTypeParamPtr;

// class OctomapMapType;
// typedef boost::shared_ptr<OctomapMapType> OctomapMapTypePtr;

/*!
 * \brief templateMapParam implements the parameters for the map type
 * \brief templateMapParamPtr
 */
// class OctomapMapTypeParam;
// typedef boost::shared_ptr<OctomapMapTypeParam> OctomapMapTypeParamPtr;

// class ScanMapType;
// typedef boost::shared_ptr<ScanMapType> ScanMapTypePtr;

// class ScanMapParam;
// typedef boost::shared_ptr<ScanMapParam> ScanMapParamPtr;

// class ScanRegParam;
// typedef boost::shared_ptr<ScanRegParam> ScanRegParamPtr;

// class ScanRegType;
// typedef boost::shared_ptr<ScanRegType> ScanRegTypePtr;

class NDTMapType;
typedef boost::shared_ptr<NDTMapType> NDTMapPtr;

class NDTMapParam;
typedef boost::shared_ptr<NDTMapParam> NDTMapParamPtr;

class NDTD2DRegParam;
typedef boost::shared_ptr<NDTD2DRegParam> NDTD2DRegParamPtr;

class NDTLamideRegParam;
typedef boost::shared_ptr<NDTLamideRegParam> NDTLamideRegParamPtr;

class NDTD2DRegType;
typedef boost::shared_ptr<NDTD2DRegType> NDTD2DRegTypePtr;

class NDTLamideRegType;
typedef boost::shared_ptr<NDTLamideRegType> NDTLamideRegTypePtr;

class NDTDLMapType;
typedef boost::shared_ptr<NDTDLMapType> NDTDLMapPtr;

class NDTDLMapParam;
typedef boost::shared_ptr<NDTDLMapParam> NDTDLMapParamPtr;

class NDTDLRegParam;
typedef boost::shared_ptr<NDTDLRegParam> NDTDLRegParamPtr;

class NDTDLRegType;
typedef boost::shared_ptr<NDTDLRegType> NDTDLRegTypePtr;

class factor;
typedef boost::shared_ptr<factor> FactorPtr;

/*!
 *\brief registrationType is an abstract class for registration
 *\brief registrationParameters provides paramerters to the registration
 */
class registrationType;
typedef boost::shared_ptr<registrationType> RegTypePtr;

class registrationParameters;
typedef boost::shared_ptr<registrationParameters> RegParamPtr;

class MapType;
typedef boost::shared_ptr<MapType> MapTypePtr;

class MapParam;
typedef boost::shared_ptr<MapParam> MapParamPtr;

class Node;
typedef boost::shared_ptr<Node> NodePtr;

class MapNode;
typedef boost::shared_ptr<MapNode> MapNodePtr;

class GraphMap;
typedef boost::shared_ptr<GraphMap> GraphMapPtr;

class GraphMapNavigator;
typedef boost::shared_ptr<GraphMapNavigator> GraphMapNavigatorPtr;

class GraphMapParam;
typedef boost::shared_ptr<GraphMapParam> GraphMapParamPtr;

class GraphMapNavigatorParam;
typedef boost::shared_ptr<GraphMapNavigatorParam> GraphMapNavigatorParamPtr;

/*!
 * ... Abstract class to implement map parameters.  ...
 */
class GraphFactory
{
public:
    static MapParamPtr CreateMapParam(const std::string& MapType);
    static MapTypePtr CreateMapType(const MapParamPtr& mapparam);

    static GraphMapParamPtr CreateGraphParam(const std::string& name);
    static GraphMapPtr CreateGraph(const Eigen::Affine3d& nodepose,
                                   MapParamPtr& mapparam,
                                   GraphMapParamPtr graphparam);

    static MapNodePtr CreateMapNode(const Eigen::Affine3d& pose, const MapParamPtr& mapparam, const std::string& path);

    static FactorPtr CreateObservationFactor(MapNodePtr mapPose,
                                             NodePtr observationPose,
                                             const Eigen::Affine3d& diff,
                                             const Matrix6d& covar);
    static FactorPtr CreateMapNodeFactor(MapNodePtr prevMapPose,
                                         MapNodePtr nextMapPose,
                                         const Eigen::Affine3d& diff,
                                         const Matrix6d& covar);

    static RegTypePtr CreateRegistrationType(RegParamPtr regparam);
    static RegParamPtr CreateRegParam(std::string regType);

    static void SetCoutOptions(bool disable_output)
    {
        disable_output_ = disable_output;
    }

private:
    static bool disable_output_;
    GraphFactory()
    {
    }
};

} // namespace graph_map

} // namespace perception_oru
#endif // GRAPHFACTORY_H
