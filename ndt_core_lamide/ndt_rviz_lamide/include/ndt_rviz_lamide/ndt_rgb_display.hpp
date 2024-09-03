#ifndef NDT_RGB_DISPLAY_H
#define NDT_RGB_DISPLAY_H

#include <boost/circular_buffer.hpp>

#include <ndt_map_lamide/NDTMapRGBMsg.h>
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

class NDTVisual;

class NDTRGBDisplay : public rviz::MessageFilterDisplay<ndt_map_lamide::NDTMapRGBMsg> {
  Q_OBJECT
public:
  NDTRGBDisplay();
  virtual ~NDTRGBDisplay();

protected:
  virtual void onInitialize();

  virtual void reset();

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateHistoryLength();

private:
  /**
   * @brief transform a NDTmap msg into a vector of NDTVisual, one per cell. If
   * history is set to one or lower, it only display the last NDTMap. Otherwise,
   * it keeps all cells through time
   */
  void processMessage(const ndt_map_lamide::NDTMapRGBMsg::ConstPtr &msg);

  std::vector<boost::shared_ptr<NDTVisual>> visuals_;

  rviz::ColorProperty *color_property_;
  rviz::FloatProperty *alpha_property_;
  rviz::IntProperty *history_length_property_;
};
}

#endif
