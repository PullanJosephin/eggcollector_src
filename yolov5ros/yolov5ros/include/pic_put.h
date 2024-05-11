#ifndef __PIC_PUT_H_
#define __PIC_PUT_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <tf/transform_listener.h>
enum ArmState {
IDLE,
WORKING,
NAVIGATING};
class CollectProcess
{
  private:
	/**
 	 * @brief NodeHandle of the current node
 	 */
	ros::NodeHandle nh_;
	/**
     * @brief target_pose_1 is the pose to moveit!
     */
	ros::Time empty_frame_start_time;//when there is no targets
	geometry_msgs::Pose target_pose1;
	/**
     * @brief armgroup moveit interface for arm
     */
	moveit::planning_interface::MoveGroupInterface armgroup;
	/**
     * @brief grippegroup moveit interface for gripper
     */
	moveit::planning_interface::MoveGroupInterface grippergroup;
	/**
     * @brief it_ takes care of message to image conversion
     */
	ArmState arm_state;
	/**
     * @brief state of arm
     */
    ros::Subscriber target_positions_Sub,status_sub,navigate_goal_sub;
	/**
     * @brief boolean to control the grasping movements
     */
	int box_num,search_num,grab_attempt;
	//change num to switch egg box 
	//change the search num to search different area
	std::vector<std::string> eggboxs,search_area;
	bool grasp_running;
	/*
     * @brief vMng_ is the instance of the library for object detection
     */
	void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& ms);
	/*
	 * @brief navigate status callback function
	 */
	void updateTransform();
	
	/**
	 * @brief get the latest transform of camera to base_link
	 */
	void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	tf::StampedTransform camera_to_robot_;
	/**
	 * @brief tf_camera_to_robot is an instance of tf_listener
	 */

	tf::TransformListener tf_camera_to_robot;
	/**
	 * @brief obj_camera_frame, obj_robot_frame are instance of tf::Vector
	 */

	tf::Vector3 obj_camera_frame, obj_robot_frame;
	/**
	 * @brief homePose is StampedPose keeping Home position of arm
	 */

	geometry_msgs::PoseStamped homePose;
	/**
	 * @brief my_plan is an instance of moveit! planning interface
	 */

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	/**
	 * @brief pregrasp_x, pregrasp_y and pregrasp_z is the pregrasp position of arm
	 */

	float pregrasp_x, pregrasp_y, pregrasp_z;
	/**
	 * @brief      attainPosition achieved the given position of the arm
	 *
	 * @param[in]  x     x-position of gripping frame	
	 * @param[in]  y     y-position of gripping frame	
	 * @param[in]  z     z-position of gripping frame
	 */

	void attainPosition(float x, float y, float z);
	/**
	 * @brief      attainObject tries to get the gripper next to object
	 */
	moveit::planning_interface::MoveItErrorCode attainObject();
	/**
	 * @brief      grasp executes the grasping action
	 */
	void grasp();
	/**
	 * @brief      lift attempts to lift the object
	 */
	void lift();

  public:
	/**
 	 * @brief      GraspingDemo behaviour Constructor
 	 *
 	 * @param[in]  n_          ros_NodeHandle
 	 */
	CollectProcess(ros::NodeHandle n_);
	ros::Publisher targetpos_pub;
	/**
	 * @brief      imageCb is called when a new image is received from the camera
	 *
	 * @param[in]  msg   Image received as a message
	 */

	void target_pointCb(const sensor_msgs::PointCloudConstPtr &msg);
	/**
	 * @brief      initiateGrasping initiates the grasping behaviour
	 */
	void initiateGrasping();
	/**
	 * @brief      Function brings the arm back to home configuration
	 */
	void goHome();
};
#endif
