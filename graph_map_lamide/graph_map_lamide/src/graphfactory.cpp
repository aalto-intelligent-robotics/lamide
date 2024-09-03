#include "graph_map_lamide/graphfactory.h"

#include "Eigen/Geometry"
#include "graph_map_lamide/factor.h"
#include "graph_map_lamide/graph_map.h"
#include "graph_map_lamide/graph_map_navigator.h"
#include "graph_map_lamide/map_node.h"
#include "graph_map_lamide/map_type.h"
#include "graph_map_lamide/narf/narf_map_type.h"
#include "graph_map_lamide/narf/narf_reg_type.h"
#include "graph_map_lamide/ndt/ndt_map_param.h"
#include "graph_map_lamide/ndt/ndt_map_type.h"
#include "graph_map_lamide/ndt/ndtd2d_reg_type.h"
#include "graph_map_lamide/ndt/ndtd2d_reg_type_lamide.h"
#include "graph_map_lamide/ndt_dl/ndtdl_map_param.h"
#include "graph_map_lamide/ndt_dl/ndtdl_map_type.h"
#include "graph_map_lamide/ndt_dl/ndtdl_reg_type.h"
#include "graph_map_lamide/octomap/octomap_map_type.h"
#include "graph_map_lamide/reg_type.h"
#include "graph_map_lamide/template/template_map_type.h"
#include "graph_map_lamide/template/template_reg_type.h"
//#include "graph_map_lamide/scanmap/scan_map_type.h"
//#include "graph_map_lamide/scanmap/scan_reg_type.h"

namespace perception_oru
{
namespace graph_map
{

bool GraphFactory::disable_output_ = false;
//!
//! \brief GraphFactory::CreateMapParam Creates parameter type based on string input
//! \param mapname
//! \return
//!
//!
MapParamPtr GraphFactory::CreateMapParam(const std::string& mapname)
{
    if (mapname.compare(ndt_map_type_name) == 0)
    {
        if (!disable_output_)
            cout << "Graphfactory: Created parameters for map type: \"" << ndt_map_type_name << "\""
                 << endl;
        NDTMapParamPtr paramPtr(new NDTMapParam());
        return paramPtr;
    }
    else if (mapname.compare(ndtdl_map_type_name) == 0)
    {
        if (!disable_output_)
            cout << "Graphfactory: Created parameters for map type: \"" << ndtdl_map_type_name
                 << "\"" << endl;
        NDTDLMapParamPtr paramPtr(new NDTDLMapParam());
        return paramPtr;
    }

    /*  else if(mapname.compare(template_map_name)==0){
      TemplateMapParamPtr templatePar(new TemplateMapParam());
      if(!disable_output_)
        cerr<<"Graphfactory:  Template, no map parameter instance created"<<endl;
      return templatePar;
    }
    else if(mapname.compare(narf_map_name)==0){
      NarfMapParamPtr narf_map_par(new NarfMapParam());
      if(!disable_output_)
        cerr<<"Graphfactory:  narf map parameters created"<<endl;
      return narf_map_par;
    }
    else if(mapname.compare(octomap_map_name)==0){
      OctomapMapTypeParamPtr octomap_map_par(new OctomapMapTypeParam());
      if(!disable_output_)
        cerr<<"Graphfactory:  octomap map parameters created"<<endl;
      return octomap_map_par;
    }*/
    else
    {
        if (!disable_output_)
            cerr << "No map type exists with name: \"" << mapname << "\"" << endl;
        return NULL;
    }
}

MapTypePtr GraphFactory::CreateMapType(const MapParamPtr& mapparam)
{
    if (NDTMapParamPtr ndt2MapParam = boost::dynamic_pointer_cast<NDTMapParam>(mapparam))
    { // perform typecast and check if not null conversion
        if (!disable_output_)
            //cout << "Graphfactory: Created map of type: \"" << ndt_map_type_name << "\"" << endl;
        return MapTypePtr(new NDTMapType(ndt2MapParam));
    }
    else if (NDTDLMapParamPtr ndtdlMapParam = boost::dynamic_pointer_cast<NDTDLMapParam>(mapparam))
    { // perform typecast and check if not null conversion
        if (!disable_output_)
            //cout << "Graphfactory: Created map of type: \"" << ndtdl_map_type_name << "\"" << endl;
        return MapTypePtr(new NDTDLMapType(ndtdlMapParam));
    }

    /*else if(TemplateMapParamPtr
    template_par=boost::dynamic_pointer_cast<TemplateMapParam>(mapparam)){ if(!disable_output_)
        cerr<<"Graphfactory: no map exists for \"template\""<<endl;
      return NULL;
    }
    else if(  NarfMapParamPtr narfMapParam = boost::dynamic_pointer_cast< NarfMapParam >(mapparam)
    ){ //perform typecast and check if not null conversion if(!disable_output_) cout<<"Graphfactory:
    Created map of type: \""<<narf_map_name<<"\""<<endl; return  MapTypePtr(new
    NarfMapType(narfMapParam));
      }
      else if(  OctomapMapTypeParamPtr octomap_param = boost::dynamic_pointer_cast<
    OctomapMapTypeParam >(mapparam) ){ //perform typecast and check if not null conversion
      if(!disable_output_)
        cout<<"Graphfactory: Created map of type: \""<<narf_map_name<<"\""<<endl;
      return  MapTypePtr(new OctomapMapType(octomap_param));
    }*/
    else
    {
        if (!disable_output_)
            cerr << "Graphfactory: No map type exists for map parameters" << endl;
        return NULL;
    }
}

GraphMapPtr GraphFactory::CreateGraph(const Eigen::Affine3d& nodepose,
                                      MapParamPtr& mapparam,
                                      GraphMapParamPtr graphparam)
{

    if (mapparam != NULL)
    {
        if (!disable_output_)
            cout << "Graphfactory: Creating graph" << endl;
        GraphMapPtr graphPtr = GraphMapPtr(new GraphMapNavigator(nodepose, mapparam, graphparam));
        return graphPtr;
    }
    else
        cerr << "Graphfactory: mapp parameters are NULL, no graph created" << endl;
}

MapNodePtr GraphFactory::CreateMapNode(const Eigen::Affine3d& pose, const MapParamPtr& mapparam, const std::string& path)
{ // Create a node
    return boost::shared_ptr<MapNode>(new MapNode(pose, mapparam, path));
}

RegTypePtr GraphFactory::CreateRegistrationType(RegParamPtr regparam)
{

    if (NDTD2DRegParamPtr ndt_reg_ptr = boost::dynamic_pointer_cast<NDTD2DRegParam>(regparam))
    {
        cout << "Graphfactory: created registration type:" << ndt_d2d_reg_type_name << endl;
        return NDTD2DRegTypePtr(new NDTD2DRegType(ndt_reg_ptr));
    }
    else if (NDTLamideRegParamPtr ndt_reg_ptr = boost::dynamic_pointer_cast<NDTLamideRegParam>(regparam))
    { // perform typecast and check if not null conversion

        cout << "Graphfactory: created registration type:" << ndt_lamide_reg_type_name << endl;
        return NDTLamideRegTypePtr(new NDTLamideRegType(ndt_reg_ptr));
    }
    else if (NDTDLRegParamPtr ndt_reg_ptr = boost::dynamic_pointer_cast<NDTDLRegParam>(regparam))
    { // perform typecast and check if not null conversion

        cout << "Graphfactory: created registration type:" << ndt_dl_reg_type_name << endl;
        return NDTDLRegTypePtr(new NDTDLRegType(ndt_reg_ptr));
    }
    cerr << "Failed to create object of registration type" << endl;
    return NULL;
}

RegParamPtr GraphFactory::CreateRegParam(string regType)
{
    if (regType.compare(ndt_d2d_reg_type_name) == 0)
    {
        cout << "Graphfactory: Creating parameters for registration type: \""
             << ndt_d2d_reg_type_name << "\"" << endl;
        return NDTD2DRegParamPtr(new NDTD2DRegParam());
    }
    else if (regType.compare(ndt_lamide_reg_type_name) == 0)
    {
        if (!disable_output_)
            cout << "Graphfactory: Creating parameters for registration type: \""
                 << ndt_lamide_reg_type_name << "\"" << endl;

        return NDTLamideRegParamPtr(new NDTLamideRegParam());
    }
    else if (regType.compare(ndt_dl_reg_type_name) == 0)
    {

        if (!disable_output_)
            cout << "Graphfactory: Creating parameters for registration type: \""
                 << ndt_dl_reg_type_name << "\"" << endl;

        return NDTDLRegParamPtr(new NDTDLRegParam());
    }
    else
    {
        cerr << "No registration type with name: \"" << regType << "\"" << endl;
        return NULL;
    }
}

FactorPtr GraphFactory::CreateMapNodeFactor(MapNodePtr prevMapPose,
                                            MapNodePtr nextMapPose,
                                            const Eigen::Affine3d& diff,
                                            const Matrix6d& covar)
{
    FactorPtr factorptr =
        boost::shared_ptr<factor>(new factor(prevMapPose, nextMapPose, diff, covar));
    return factorptr;
}

} // namespace graph_map
} // namespace perception_oru
