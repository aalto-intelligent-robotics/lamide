#include "eigen_conversions/eigen_msg.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"

#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
int counter, stops = 0;
Eigen::Affine3d pose;
Eigen::Vector3d speed;
ros::Publisher* pub_;

void imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
    static ros::Time Tprevmsg = msg->header.stamp;
    static tf::TransformBroadcaster br;
    Eigen::Vector3d offset;
    Eigen::Quaterniond q;
    Eigen::Vector3d accel;

    tf::quaternionMsgToEigen(msg->orientation, q);
    tf::vectorMsgToEigen(msg->linear_acceleration, accel);
    if (counter++ == 0)
    {
        pose.linear() = q.toRotationMatrix();
        pose.translation() = Eigen::Vector3d(0, 0, 0);
        speed << 0, 0, 0;
        // tf::vectorMsgToEigen( msg->linear_acceleration,offset);
    }
    else
    {
        tf::vectorMsgToEigen(msg->linear_acceleration, accel);

        double dt = (msg->header.stamp - Tprevmsg).toSec();
        pose.translation() = pose.translation() + dt * speed;
        speed = speed + dt * (accel - offset);
        pose.linear() = q.toRotationMatrix();
        if (speed.norm() > 1)
        {
            std::cout << "accel:" << accel << ", norm:" << accel.norm() << std::endl;
            //  exit(0);
        }
    }
    std::cout << "speed:" << speed.transpose() << std::endl;
    std::cout << "pos:" << pose.translation().transpose() << std::endl;
    // std::cout<<"pose:\n"<<pose.rotation()<<std::endl;
    tf::Transform transform;
    tf::poseEigenToTF(pose, transform);
    Tprevmsg = msg->header.stamp;
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "xsens"));
    // if(counter%1000==0)
    // speed<<0,0,0;
}

int main(int argc, char** argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "listener");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    ros::Subscriber sub = n.subscribe("imu/data", 1000, imu_callback);

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}
