#ifndef NDT_VECTORMAPS_DISPLAY_H_19032018
#define NDT_VECTORMAPS_DISPLAY_H_19032018

#include <boost/circular_buffer.hpp>

#include <ndt_map_lamide/NDTVectorMapMsg.h>
#include <ndt_map_lamide/NDTMapMsg.h>
#include <rviz/message_filter_display.h>

namespace Ogre {
class SceneNode;
}

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace perception_oru {
class NDTLineVisual;
}

namespace perception_oru {
namespace ndt_rviz {

class NDTVectorMapDisplay
    : public rviz::MessageFilterDisplay<ndt_map_lamide::NDTVectorMapMsg> {
  Q_OBJECT
public:
  NDTVectorMapDisplay();
  virtual ~NDTVectorMapDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateHistoryLength();

private:
  void processMessage(const ndt_map_lamide::NDTVectorMapMsg::ConstPtr &msg);

  std::deque<boost::shared_ptr<perception_oru::NDTLineVisual>> visuals_;

  rviz::ColorProperty *color_property_;
  rviz::FloatProperty *alpha_property_;
  rviz::IntProperty *history_length_property_;
};
}
}

#endif
