#ifndef NDT_VISUAL_H
#define NDT_VISUAL_H

#include <ndt_map_lamide/NDTMapMsg.h>
#include <ndt_map_lamide/NDTCellMsg.h>
#include <ndt_map_lamide/NDTMapRGBMsg.h>
#include <ndt_map_lamide/NDTCellRGBMsg.h>

namespace Ogre{
  class Vector3;
  class Quaternion;
}

namespace rviz{
  class Shape;
}
namespace perception_oru{

  class NDTVisual{
  public:
    NDTVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
    virtual ~NDTVisual();
    void setCell(ndt_map_lamide::NDTCellRGBMsg msg, double resolution);
    void setCellRGB(ndt_map_lamide::NDTCellRGBMsg msg, double resolution);
    void setFramePosition(const Ogre::Vector3& position);
    void setFrameOrientation(const Ogre::Quaternion& orientation);
    void setColor( float r, float g, float b, float a );
  private:

    boost::shared_ptr<rviz::Shape> NDT_elipsoid_;
    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;
  };
}
#endif
