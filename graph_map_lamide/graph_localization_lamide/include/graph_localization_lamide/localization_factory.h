#ifndef LOCALISATIONFACTORY_H
#define LOCALISATIONFACTORY_H
#include "Eigen/Dense"
#include "boost/shared_ptr.hpp"
#include "iostream"
#include "stdio.h"
#include "string.h"

#include <boost/algorithm/string.hpp>
#include <string>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
using namespace std;
namespace perception_oru
{
namespace graph_localization
{

/*!
 * \brief templateMapType implements the map or acts as a wrapper to your existing map classes
 * \brief templateMapTypePtr is the general way the map type is passed around, based om shared
 * pointers to ensure memory no memory losses
 */
class TemplateLocalisationType;
typedef boost::shared_ptr<TemplateLocalisationType> TemplateLocalisationTypePtr;

/*!
 * \brief templateMapParam implements the parameters for the map type
 * \brief templateMapParamPtr
 */
class TemplateLocalisationParam;
typedef boost::shared_ptr<TemplateLocalisationParam> TemplateLocalisationParamPtr;

class MCLNDTType;
typedef boost::shared_ptr<MCLNDTType> MCLNDTTypePtr;

class MCLNDTParam;
typedef boost::shared_ptr<MCLNDTParam> MCLNDTParamPtr;

//note the custom typedefs
class MCLNDTLamideType;
typedef boost::shared_ptr<MCLNDTLamideType> MCLNDTLamideTypePtr;

class MCLNDTLamideParam;
typedef boost::shared_ptr<MCLNDTLamideParam> MCLNDTLamideParamPtr;
//note end

class SubmapMCLType;
typedef boost::shared_ptr<SubmapMCLType> SubmapMCLTypePtr;

class SubmapMCLParam;
typedef boost::shared_ptr<SubmapMCLParam> SubmapMCLParamPtr;

class MCLNDTDLType;
typedef boost::shared_ptr<MCLNDTDLType> MCLNDTDLTypePtr;

class MCLNDTDLParam;
typedef boost::shared_ptr<MCLNDTDLParam> MCLNDTDLParamPtr;

class UKFNDTType;
typedef boost::shared_ptr<UKFNDTType> UKFNDTTypePtr;

class UKFNDTParam;
typedef boost::shared_ptr<UKFNDTParam> UKFNDTParamPtr;

class UKFRegType;
typedef boost::shared_ptr<UKFRegType> UKFRegTypePtr;

class UKFRegParam;
typedef boost::shared_ptr<UKFRegParam> UKFRegParamPtr;

class RegLocalisationType;
typedef boost::shared_ptr<RegLocalisationType> RegLocalisationTypePtr;

class RegLocalisationParam;
typedef boost::shared_ptr<RegLocalisationParam> RegLocalisationParamPtr;

class LocalizationType;
typedef boost::shared_ptr<LocalizationType> LocalisationTypePtr;

class LocalisationParam;
typedef boost::shared_ptr<LocalisationParam> LocalisationParamPtr;

class LocalisationFactory
{
public:
    static LocalisationParamPtr CreateLocalisationParam(string localisationType);
    static LocalisationTypePtr CreateLocalisationType(LocalisationParamPtr param);

private:
    LocalisationFactory()
    {
    }
};

} // namespace graph_localization
} // namespace perception_oru
#endif // LOCALISATIONFACTORY_H
