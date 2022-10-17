#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>
#include <string>
#include <cmath>

void bundleSelection(std::vector<geometry_msgs::TransformStamped> transforms_found, std::vector<geometry_msgs::TransformStamped>& transforms_selected);
void bundleFusion(std::vector<geometry_msgs::TransformStamped> transforms_selected, geometry_msgs::Pose& uav_pose_world);
void odomPublish(geometry_msgs::Pose uav_pose_world);
void optiTrackCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void stallUAV(double time);
bool outlierCheck(geometry_msgs::Pose uav_pose_world);
geometry_msgs::Pose poseTransform(geometry_msgs::Pose pose, std::string source_frame, std::string target_frame);
geometry_msgs::Pose averagePose(std::vector<geometry_msgs::Pose> all_poses_wrt_map);

std::vector<std::string> bundle_names;// = {"tag_bundle_right_bottom", "tag_bundle_center_bottom", "tag_bundle_left_bottom",
//                                        "tag_bundle_left_center", "tag_bundle_center_center", "tag_bundle_right_center", 
//                                       "tag_bundle_right_top", "tag_bundle_center_top", "tag_bundle_left_top"};
float opti_i, opti_j, opti_k, opti_w;
geometry_msgs::Pose last_uav_pose_world;
bool use_optiTrack = true; // use optiTrack's orientation
double epsilon = 0.3; // distance jump threshold
int max_bundle_num = 2; // maximum number of bundle to consider
double stall_time = 0.5;

ros::NodeHandle nh;
ros::Publisher odom_pub;
ros::Subscriber opti_sub;
ros::Rate rate(100);

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_to_odom");

  ros::V_string args;
	ros::removeROSArgs(argc, argv, args);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  odom_pub = nh.advertise<nav_msgs::Odometry>("/camera/pose_d", 100);
  opti_sub = nh.subscribe("/vrpn_client_node/MAV1/pose", 1000, optiTrackCallback);
  bundle_names = args;
  
  last_uav_pose_world.position.x = 0;
  last_uav_pose_world.position.y = 0;
  last_uav_pose_world.position.z = 0;
  last_uav_pose_world.orientation.x = 0;
  last_uav_pose_world.orientation.y = 0;
  last_uav_pose_world.orientation.z = 0;
  last_uav_pose_world.orientation.w = 1.0;

  while (nh.ok()) {
    std::vector<geometry_msgs::TransformStamped> transforms_found, transforms_selected;
    geometry_msgs::Pose uav_pose_world;

    // Read all Tf broadcasted by apriltag_ros
    for(int i = 0; i < bundle_names.size(); i++){
      try {
        // camera pose under bundle frame
        transforms_found.push_back(tfBuffer.lookupTransform(bundle_names[i], "camera_link", ros::Time(0))); // maybe camera_link is optical frame? //camera pose in bundle frame
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
    }

    // Select the <max_bundle_number> closest bundle to the camera 
    bundleSelection(transforms_found, transforms_selected);
    // Transform the camera pose in tag frame to map frame and averaging all measurement
    bundleFusion(transforms_selected, uav_pose_world);

    // check if an uav pose in world frame exists or not
    if(transforms_selected.size()>0){
      // check if the computed pose is valid or not (steer jumping)
      if(outlierCheck(uav_pose_world)){
        odomPublish(uav_pose_world);
        last_uav_pose_world = uav_pose_world;
      }else{
        // stall the UAV to capture better detection
        stallUAV(stall_time);
      }
    }else{
      // stall the UAV to capture better detection
      stallUAV(stall_time);
    }

    ros::spinOnce();
    rate.sleep();
  }
}

// Selecting <max_bundle_number> bundles with the smallest distance
void bundleSelection(std::vector<geometry_msgs::TransformStamped> transforms_found, std::vector<geometry_msgs::TransformStamped>& transforms_selected){
  std::vector<float> dist; // distance of camera and bundle
  std::vector<int> sorted_bundle; // sort the bundles according to their distance
  geometry_msgs::TransformStamped transformStamped;

  // iterate through all detected bundles and sort them in the order of their distance with camera
  for(int i = 0; i < transforms_found.size(); i++){
    float tmp;

    transformStamped = transforms_found[i];
    tmp = pow(transformStamped.transform.translation.x,2) + pow(transformStamped.transform.translation.y,2) + pow(transformStamped.transform.translation.z,2);

    if(i == 0){
      dist.push_back(tmp);
      sorted_bundle.push_back(i);

    }else{
      for(int j = 0; j < dist.size(); j++){
        if(tmp < dist[j]){
          dist.insert(dist.begin()+j,tmp);
          sorted_bundle.insert(sorted_bundle.begin()+j,i);
          break;
        }else if(j == (dist.size()-1)){
          dist.push_back(tmp);
          sorted_bundle.push_back(i);
          break;
        }
      }
    }
  }

  for(int k, cnt; (k < sorted_bundle.size()) || (cnt <= max_bundle_num); k++){
    // check if the z is jumping to negative, if so, then ignore it and find the next smallest
    if(transforms_found[sorted_bundle[k]].transform.translation.z >= 0){
      transforms_selected.push_back(transforms_found[sorted_bundle[k]]);
      cnt++;
    }
  }
}

// Averaging the Pose from previously selected bundles
void bundleFusion(std::vector<geometry_msgs::TransformStamped> transforms_selected, geometry_msgs::Pose& uav_pose_world){
  // 1. find the inverse of transforms_selected to obtain camera pose w.r.t bundle frame (seems duplicated so didn't implemented)
  // 2. use the map to bundle static tf to obtain camera pose w.r.t map frame
  // 3. averaging all obtained position and orientation(quaternion can not be averaged)
  std::vector<geometry_msgs::Pose> all_poses_wrt_map;

  if(transforms_selected.size()>0){
    for(int i=0; i < transforms_selected.size(); i++){
      geometry_msgs::Pose bundle_pose_wrt_camera, robot_pose_wrt_camera, robot_pose_wrt_bundle, robot_pose_wrt_map;

      bundle_pose_wrt_camera.position.x = transforms_selected[i].transform.translation.x;
      bundle_pose_wrt_camera.position.y = transforms_selected[i].transform.translation.y;
      bundle_pose_wrt_camera.position.z = transforms_selected[i].transform.translation.z;
      bundle_pose_wrt_camera.orientation.x = transforms_selected[i].transform.rotation.x;
      bundle_pose_wrt_camera.orientation.y = transforms_selected[i].transform.rotation.y;
      bundle_pose_wrt_camera.orientation.z = transforms_selected[i].transform.rotation.z;
      bundle_pose_wrt_camera.orientation.w = transforms_selected[i].transform.rotation.w;

      // transform from camera_link to base_link
      robot_pose_wrt_camera = poseTransform(bundle_pose_wrt_camera, "camera_link", "base_link");

      // transform from base link frame to bundle frame
      robot_pose_wrt_bundle = poseTransform(robot_pose_wrt_camera, "base_link", transforms_selected[i].child_frame_id);

      // transform the pose from bundle frame to map frame
      robot_pose_wrt_map = poseTransform(robot_pose_wrt_bundle, transforms_selected[i].child_frame_id, "map"); // not sure if the child frame id is the bundle's id???

      all_poses_wrt_map.push_back(robot_pose_wrt_map);
    }
    uav_pose_world = averagePose(all_poses_wrt_map);
  }else{
    return;
  }
}

geometry_msgs::Pose poseTransform(geometry_msgs::Pose pose_wrt_source_frame, std::string source_frame, std::string target_frame){
  geometry_msgs::Pose pose_wrt_target_frame;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped source2target_transform;

  source2target_transform = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0));
  tf2::doTransform(pose_wrt_source_frame, pose_wrt_target_frame, source2target_transform);

  return pose_wrt_target_frame;
}

geometry_msgs::Pose averagePose(std::vector<geometry_msgs::Pose> all_poses_wrt_map){
  float sum_x, sum_y, sum_z;
  geometry_msgs::Pose avg_pose;

  for(int i = 0; i < all_poses_wrt_map.size(); i++){
    sum_x += all_poses_wrt_map[i].position.x;
    sum_y += all_poses_wrt_map[i].position.y;
    sum_z += all_poses_wrt_map[i].position.z;
  }

  avg_pose.position.x = sum_x / all_poses_wrt_map.size();
  avg_pose.position.y = sum_y / all_poses_wrt_map.size();
  avg_pose.position.z = sum_z / all_poses_wrt_map.size();

  if(use_optiTrack){
    avg_pose.orientation.x = opti_i;
    avg_pose.orientation.y = opti_j;
    avg_pose.orientation.z = opti_k;
    avg_pose.orientation.w = opti_w;
  }else{
    // maybe slerp averaging can be applied, currently picking the closest detection's orientation
    avg_pose.orientation.x = all_poses_wrt_map[0].orientation.x;
    avg_pose.orientation.y = all_poses_wrt_map[0].orientation.y;
    avg_pose.orientation.z = all_poses_wrt_map[0].orientation.z;
    avg_pose.orientation.w = all_poses_wrt_map[0].orientation.w;
  }
}

void odomPublish(geometry_msgs::Pose uav_pose_world){
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = uav_pose_world.position.x;
    odom.pose.pose.position.y = uav_pose_world.position.y;
    odom.pose.pose.position.z = uav_pose_world.position.z;
    odom.pose.pose.orientation.w = uav_pose_world.orientation.w;
    odom.pose.pose.orientation.x = uav_pose_world.orientation.x;
    odom.pose.pose.orientation.y = uav_pose_world.orientation.y;
    odom.pose.pose.orientation.z = uav_pose_world.orientation.z;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";

    odom_pub.publish(odom);
}

void optiTrackCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
	opti_i = (*msg).pose.orientation.x;
	opti_j = (*msg).pose.orientation.y;
  opti_k = (*msg).pose.orientation.z;
  opti_w = (*msg).pose.orientation.w;
}

void stallUAV(double time){
  geometry_msgs::Pose uav_pose_world;

  ROS_WARN("====No Valid Transform is Found====");
  ROS_WARN("====Stopping to capture better prediction====");
  // need to change to position command given to uav
  
  uav_pose_world.position.x = 0;
  uav_pose_world.position.y = 0;
  uav_pose_world.position.z = 0;
  uav_pose_world.orientation.x = 0;
  uav_pose_world.orientation.y = 0;
  uav_pose_world.orientation.z = 0;
  uav_pose_world.orientation.w = 1.0;

  // stall the uav by illucinated it being at destination
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time); // Timeout of 0.5 seconds
  while(ros::Time::now() - start_time < timeout) {
    odomPublish(uav_pose_world);
    ros::spinOnce();
  }
}

bool outlierCheck(geometry_msgs::Pose uav_pose_world){
  float delta_x, delta_y;

  delta_x = abs(last_uav_pose_world.position.x - uav_pose_world.position.x);
  delta_y = abs(last_uav_pose_world.position.x - uav_pose_world.position.x);
  
  if(delta_x > epsilon || delta_y > epsilon){
    return false;
  }else{
    return true;
  }
  
}
/*void predictPose(){

}*/

