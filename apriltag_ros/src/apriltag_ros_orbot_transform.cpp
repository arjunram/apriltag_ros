#include "apriltag_ros/common_functions.h"

#include "geometry_msgs/PoseStamped.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"



class TransformToBase
{
public:
  TransformToBase() :
    tf2_(buffer_),  target_frame_("j2n6s300_link_base"),
    tf2_filter_(pose_sub_, buffer_, target_frame_, 10, 0)
  {
    apriltag_sub_ = n_.subscribe("tag_detections", 10, &TransformToBase::apriltagCallback, this);
    pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("tag_pose", 100);
    pose_pub_global = n_.advertise<geometry_msgs::PoseStamped>("tag_pose_base_frame", 100);
    pose_sub_.subscribe(n_, "tag_pose", 10);
    tf2_filter_.registerCallback( boost::bind(&TransformToBase::poseCallback, this, _1) );
  }

  void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tag)
  {
    geometry_msgs::PoseStamped tag_pose;
    try
    {
      if(tag->detections.size())
      {
        tag_pose.header = tag->detections[0].pose.header;
        tag_pose.pose = tag->detections[0].pose.pose.pose;
      }
      pose_pub_.publish(tag_pose);
      
    }
    catch (std::exception& e)
    {
    std::cerr << "Exception caught : " << e.what() << std::endl;
    }
  }

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& pose_ptr)
  {
    geometry_msgs::PoseStamped pose_out;
    
    try 
    {
      buffer_.transform(*pose_ptr, pose_out, target_frame_);
      
      ROS_INFO("point of turtle 3 in frame of turtle 1 Position(x:%f y:%f z:%f)\n", 
             pose_out.pose.position.x,
             pose_out.pose.position.y,
             pose_out.pose.position.z);
      pose_pub_global.publish(pose_out);

    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
  }


private:
  ros::Publisher pose_pub_;
  ros::Publisher pose_pub_global;
  ros::Subscriber apriltag_sub_;
  ros::NodeHandle n_;
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub_;
  tf2_ros::MessageFilter<geometry_msgs::PoseStamped> tf2_filter_;

};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "orbot_transform"); //Init ROS
  TransformToBase pd; //Construct class
  ros::spin(); // Run until interupted 
  return 0;
};

