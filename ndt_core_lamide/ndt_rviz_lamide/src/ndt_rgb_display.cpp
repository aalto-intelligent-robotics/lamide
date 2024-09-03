#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include <rviz/frame_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/visualization_manager.h>

#include "ndt_rviz_lamide/ndt_visual.hpp"

#include "ndt_rviz_lamide/ndt_rgb_display.hpp"

namespace perception_oru
{

NDTRGBDisplay::NDTRGBDisplay()
{
    color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204),
                                              "Color to draw the acceleration arrows.", this,
                                              SLOT(updateColorAndAlpha()));

    alpha_property_ =
        new rviz::FloatProperty("Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
                                SLOT(updateColorAndAlpha()));

    history_length_property_ =
        new rviz::IntProperty("History Length", 1, "Number of prior measurements to display.", this,
                              SLOT(updateHistoryLength()));
    history_length_property_->setMin(1);
    history_length_property_->setMax(100000);
}
void NDTRGBDisplay::onInitialize()
{
    MFDClass::onInitialize();
}

NDTRGBDisplay::~NDTRGBDisplay()
{
}
void NDTRGBDisplay::reset()
{
    MFDClass::reset();
    visuals_.clear();
}
void NDTRGBDisplay::updateColorAndAlpha()
{
    float alpha = alpha_property_->getFloat();
    // Ogre::ColourValue color = color_property_->getOgreColor();
    // for (size_t i = 0; i < visuals_.size(); i++)
    // {
    //     visuals_[i]->setColor(color.r, color.g, color.b, alpha);
    // }
}

void NDTRGBDisplay::updateHistoryLength()
{
    ROS_INFO_STREAM("history received: " << this->history_length_property_->getInt());
}

void NDTRGBDisplay::processMessage(const ndt_map_lamide::NDTMapRGBMsg::ConstPtr& msg)
{
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;

    if (history_length_property_->getInt() <= 1)
    {
        visuals_.clear();
    }

    if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp,
                                                   position, orientation))
    {
        ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
                  qPrintable(fixed_frame_));
        return;
    }
    for (int itr = 0; itr < msg->cells.size(); itr++)
    {
        // for(int itr=0;itr<10;itr++){
        if (msg->cells[itr].hasGaussian_ == true)
        {
            boost::shared_ptr<NDTVisual> visual;
            visual.reset(new NDTVisual(context_->getSceneManager(), scene_node_));
            if (!(msg->x_cell_size == msg->y_cell_size && msg->y_cell_size == msg->z_cell_size))
            {
                ROS_ERROR("SOMETHING HAS GONE VERY WRONG YOUR VOXELL IS NOT A CUBE");
                // return false;
            }

            visual->setCellRGB(msg->cells[itr], msg->x_cell_size);
            visual->setFramePosition(position);
            visual->setFrameOrientation(orientation);
            float alpha = alpha_property_->getFloat();
            Ogre::ColourValue color = color_property_->getOgreColor();
            float r = msg->cells[itr].red;
            float g = msg->cells[itr].green;
            float b = msg->cells[itr].blue;
            visual->setColor(r, g, b, alpha);
            visuals_.push_back(visual);
        }
    }
}
} // namespace perception_oru

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(perception_oru::NDTRGBDisplay, rviz::Display)
