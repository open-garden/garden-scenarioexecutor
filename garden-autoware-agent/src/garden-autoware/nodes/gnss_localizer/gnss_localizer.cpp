#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

static ros::Time previous_time;
static pose previous_pose;
static ros::Publisher estimate_twist_pub;

static pose convertPoseIntoRelativeCoordinate(const pose &target_pose, const pose &reference_pose)
{
  tf::Quaternion target_q;
  target_q.setRPY(target_pose.roll, target_pose.pitch, target_pose.yaw);
  tf::Vector3 target_v(target_pose.x, target_pose.y, target_pose.z);
  tf::Transform target_tf(target_q, target_v);

  tf::Quaternion reference_q;
  reference_q.setRPY(reference_pose.roll, reference_pose.pitch, reference_pose.yaw);
  tf::Vector3 reference_v(reference_pose.x, reference_pose.y, reference_pose.z);
  tf::Transform reference_tf(reference_q, reference_v);

  tf::Transform trans_target_tf = reference_tf.inverse() * target_tf;

  pose trans_target_pose;
  trans_target_pose.x = trans_target_tf.getOrigin().getX();
  trans_target_pose.y = trans_target_tf.getOrigin().getY();
  trans_target_pose.z = trans_target_tf.getOrigin().getZ();
  tf::Matrix3x3 tmp_m(trans_target_tf.getRotation());
  tmp_m.getRPY(trans_target_pose.roll, trans_target_pose.pitch, trans_target_pose.yaw);

  return trans_target_pose;
}

static void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  // current time
  ros::Time current_time = input->header.stamp;

  // current pose
  pose current_pose;
  current_pose.x = input->pose.position.x;
  current_pose.y = input->pose.position.y;
  current_pose.z = input->pose.position.z;
  tf::Quaternion current_q(input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z,
                           input->pose.orientation.w);
  tf::Matrix3x3 m(current_q);
  m.getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);

  // TF "/base_link" to "/map"
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  transform.setRotation(current_q);
  br.sendTransform(tf::StampedTransform(transform, current_time, "/map", "/base_link"));

  // diff time
  double diff_time = (current_time - previous_time).toSec();

  // diff pose
  double diff_x = current_pose.x - previous_pose.x;
  double diff_y = current_pose.y - previous_pose.y;
  double diff_z = current_pose.z - previous_pose.z;
  double diff_yaw = current_pose.yaw - previous_pose.yaw;
  double diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

  // current velocity
  pose trans_current_pose = convertPoseIntoRelativeCoordinate(current_pose, previous_pose);
  double current_velocity = (diff_time > 0) ? (diff / diff_time) : 0;
  current_velocity =  (trans_current_pose.x >= 0) ? current_velocity : -current_velocity;

  // current angular velocity
  double angular_velocity = (diff_time > 0) ? (diff_yaw / diff_time) : 0;

  // publish /estimate_twist
  geometry_msgs::TwistStamped estimate_twist_msg;
  estimate_twist_msg.header.stamp = current_time;
  estimate_twist_msg.header.frame_id = "/base_link";
  estimate_twist_msg.twist.linear.x = current_velocity;
  estimate_twist_msg.twist.linear.y = 0.0;
  estimate_twist_msg.twist.linear.z = 0.0;
  estimate_twist_msg.twist.angular.x = 0.0;
  estimate_twist_msg.twist.angular.y = 0.0;
  estimate_twist_msg.twist.angular.z = angular_velocity;

  estimate_twist_pub.publish(estimate_twist_msg);

  previous_time = current_time;
  previous_pose = current_pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gnss_localizer");
  ros::NodeHandle nh;

  estimate_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 10);

  ros::Subscriber gnss_sub = nh.subscribe("/gnss_pose", 10, gnss_callback);

  ros::spin();

  return 0;
}

