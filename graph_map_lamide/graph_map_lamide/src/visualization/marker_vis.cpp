#include "graph_map_lamide/visualization/marker_vis.h"
namespace perception_oru
{
namespace graph_map
{

marker_vis::marker_vis(const std::string& topic) : nh_("~")
{
    pub = nh_.advertise<visualization_msgs::MarkerArray>(topic, 10);
}
visualization_msgs::Marker marker_vis::GetDefaultMarker()
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "prepare_localization";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 3;
    marker.pose.position.y = 3;
    marker.pose.position.z = 3;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = .0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    return marker;
}
std_msgs::ColorRGBA marker_vis::GetColor(double h, double min, double max)
{
    std_msgs::ColorRGBA rgba;
    if (h > 1.0)
        h = 1.0;

    double hnorm = (h * (max - min) + min) * 360;
    hsv hs = {.h = hnorm, .s = 1.0, .v = 1.0};
    rgb color = hsv2rgb(hs);
    rgba.r = color.r;
    rgba.g = color.g;
    rgba.b = color.b;
    rgba.a = 1.0;
    return rgba;
}
void marker_vis::PlotLabeledPoses(
    const std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>& poses,
    const std::vector<unsigned int>& idx,
    double size)
{

    if (idx.empty() || idx.size() != poses.size())
    {
        std::cerr << "Cannot plot Labeled Poses" << std::endl;
        return;
    }

    visualization_msgs::MarkerArray m_arr;
    geometry_msgs::Pose pose;
    visualization_msgs::Marker marker = GetDefaultMarker();
    marker.scale.x = marker.scale.y = marker.scale.z = size;
    double max = *std::max_element(std::begin(idx), std::end(idx)) + 1.0;

    for (int i = 0; i < poses.size(); i++)
    {
        tf::poseEigenToMsg(poses[i], pose);
        // std::cout<<poses[i].translation().transpose()<<std::endl;
        marker.pose.position = pose.position;
        marker.id = i;
        marker.text = std::to_string(idx[i]);
        double h = (double)idx[i] / (double)max;
        // std::cout<<"h= "<<h<<", ";
        marker.color = GetColor(h);
        m_arr.markers.push_back(marker);
    }
    pub.publish(m_arr);
}

// void marker_vis::PlotMatrix(Eigen::MatrixXd &m){

/*  unsigned char* image_pointer_red;
unsigned char* image_pointer_green;
unsigned char* image_pointer_blue;

uint8_t *uint8_pointer_red    = reinterpret_cast<uint8_t*>((unsigned
char*)channel_red_pointer[0].L()); uint8_t *uint8_pointer_green  =
reinterpret_cast<uint8_t*>((unsigned char*)channel_green_pointer[0].L()); uint8_t
*uint8_pointer_blue   = reinterpret_cast<uint8_t*>((unsigned char*)channel_blue_pointer[0].L());

int height =m.rows(); //atoi(height_pointer.ToString().Text());
int width = m.cols(); //atoi(width_pointer.ToString().Text());

sensor_msgs::Image output_image;
output_image.header.stamp     = ros::Time::now();
output_image.height           = m.rows();
output_image.width            = m.cols();
output_image.encoding         = "rgb8";
output_image.is_bigendian     = false;
output_image.step             = 3 * height;
double max_coeff = m.maxCoeff();
hsv c_hsv= {.h= 1,.s=1,.v=1};
rgb c_rgb = hsv2rgb(c_hsv);
for(int i=0; i<m.rows();i++)
{for(int j=0; j<m.cols();i++)
  {
    unsigned int idx=i*m.cols()+j;
  c_hsv.h=m(i,j)/max_coeff*2/3;
  c_rgb=hsv2rgb(c_hsv);
  output_image.data.push_back(uint8_pointer_red[idx]);
  output_image.data.push_back(uint8_pointer_green[idx]);
  output_image.data.push_back(uint8_pointer_blue[idx]);
  }
}
image_publisher.publish(output_image);



std::cout<<"r: "<<c_rgb.r<<"g: "<<c_rgb.g<<"b: "<<c_rgb.b<<std::endl;*/

} // namespace graph_map
} // namespace perception_oru
