#include "graph_localization_lamide/localization_factory.h" //must be included first

#include "graph_localization_lamide/localization_type.h"
#include "graph_localization_lamide/mcl_ndt/mcl_ndt.h"
#include "graph_localization_lamide/mcl_ndt/mcl_ndt_lamide.h"
#include "graph_localization_lamide/mcl_ndt/mcl_ndtdl.h"
#include "graph_localization_lamide/mcl_ndt/submap_mcl.h"
#include "graph_localization_lamide/reg_localization_type/reg_localization_type.h"
#include "graph_localization_lamide/ukf_ndt/ukf_ndt.h"
#include "graph_localization_lamide/ukf_ndt/ukf_reg.h"

string ndt_mcl_name = "ndt_mcl";
string ndt_mcl_lamide_name = "ndt_mcl_lamide";
string submap_mcl_name = "submap_mcl";
string ndtdl_mcl_name = "mcl_ndtdl";
string ndt_ukf_name = "ukf_ndt";
string ukf_reg_name = "ukf_reg";
string reg_localisation_name = "reg_localisation_type";
using namespace std;
namespace perception_oru
{
namespace graph_localization
{
LocalisationParamPtr LocalisationFactory::CreateLocalisationParam(string localisationType)
{

    if (localisationType.compare(ndt_mcl_name) == 0)
    {
        cout << "LocalisationFactory: : \"" << ndt_mcl_name << "\" parameters" << endl;
        ;
        return MCLNDTParamPtr(new MCLNDTParam());
    }
    // note This is the custom type
    else if (localisationType.compare(ndt_mcl_lamide_name) == 0)
    {
        cout << "LocalisationFactory: : \"" << ndt_mcl_lamide_name << "\" parameters" << endl;
        return MCLNDTLamideParamPtr(new MCLNDTLamideParam());
    }
    // note end
    else if (localisationType.compare(submap_mcl_name) == 0)
    {
        cout << "LocalisationFactory: : \"" << submap_mcl_name << "\" parameters" << endl;
        return SubmapMCLParamPtr(new SubmapMCLParam());
    }
    else if (localisationType.compare(ndtdl_mcl_name) == 0)
    {
        cout << "LocalisationFactory: \"" << ndtdl_mcl_name << "\"";
        return MCLNDTDLParamPtr(
            new MCLNDTDLParam()); // Currently the same parameters are used for NDTDL and NDT.
    }
    else if (localisationType.compare(ndt_ukf_name) == 0)
    {
        cout << "LocalisationFactory: \"" << ndt_ukf_name << "\"";
        return UKFNDTParamPtr(new UKFNDTParam());
    }
    else if (localisationType.compare(ukf_reg_name) == 0)
    {
        cout << "LocalisationFactory: \"" << ukf_reg_name << "\"";
        return UKFRegParamPtr(new UKFRegParam());
    }
    else if (localisationType.compare(reg_localisation_name) == 0)
    {
        cout << "LocalisationFactory: Creating parameters for localisation type: \""
             << reg_localisation_name << "\"" << endl;
        return RegLocalisationParamPtr(new RegLocalisationParam());
    }
    else
    {
        std::cerr << "No localisation parameter type exists with name: \"" << localisationType
                  << "\"" << endl;
        exit(0);
        return NULL;
    }
}

LocalisationTypePtr LocalisationFactory::CreateLocalisationType(LocalisationParamPtr param)
{
    if (MCLNDTParamPtr mcl_ndt_par = boost::dynamic_pointer_cast<MCLNDTParam>(param))
    {
        cout << "LocalisationFactory: creating object of type: \"" << ndt_mcl_name << "\"" << endl;
        return MCLNDTTypePtr(new MCLNDTType(mcl_ndt_par));
    }
    //note this is the custom type
    else if (MCLNDTLamideParamPtr mcl_ndt_lamide_par =
                 boost::dynamic_pointer_cast<MCLNDTLamideParam>(param))
    {
        cout << "LocalisationFactory: creating object of type: \"" << ndt_mcl_lamide_name << "\""
             << endl;
        return MCLNDTLamideTypePtr(new MCLNDTLamideType(mcl_ndt_lamide_par));
    }
    //note end
    else if (SubmapMCLParamPtr mcl_ndt_par = boost::dynamic_pointer_cast<SubmapMCLParam>(param))
    {
        cout << "LocalisationFactory: creating object of type: \"" << submap_mcl_name << "\""
             << endl;
        return SubmapMCLTypePtr(new SubmapMCLType(mcl_ndt_par));
    }
    else if (MCLNDTDLParamPtr mcl_ndtdl_par = boost::dynamic_pointer_cast<MCLNDTDLParam>(param))
    {
        cout << "LocalisationFactory: creating object of type: \"" << ndtdl_mcl_name << "\""
             << endl;
        return MCLNDTDLTypePtr(new MCLNDTDLType(mcl_ndtdl_par));
    }
    else if (UKFNDTParamPtr ukf_ndt_par = boost::dynamic_pointer_cast<UKFNDTParam>(param))
    {
        cout << "LocalisationFactory: creating object of type: \"" << ndt_ukf_name << "\"" << endl;
        return UKFNDTTypePtr(new UKFNDTType(ukf_ndt_par));
    }
    else if (UKFRegParamPtr ukf_ndt_par = boost::dynamic_pointer_cast<UKFRegParam>(param))
    {
        cout << "LocalisationFactory: creating object of type: \"" << ukf_reg_name << "\"" << endl;
        return UKFRegTypePtr(new UKFRegType(ukf_ndt_par));
    }
    else if (RegLocalisationParamPtr localisation_reg_ptr =
                 boost::dynamic_pointer_cast<RegLocalisationParam>(param))
    {
        cout << "LocalisationFactory: Creating object of type: \"" << reg_localisation_name << "\""
             << endl;
        return RegLocalisationTypePtr(new RegLocalisationType(localisation_reg_ptr));
    }
    else
    {
        std::cerr << "LocalisationFactory: No localisation type exists for parameters" << endl;
        exit(0);
        return NULL;
    }
}
} // namespace graph_localization
} // namespace perception_oru
