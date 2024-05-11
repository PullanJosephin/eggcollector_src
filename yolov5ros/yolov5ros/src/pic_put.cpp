#include <pic_put.h>

CollectProcess::CollectProcess(ros::NodeHandle n_) :
    armgroup("arm"), 
    grippergroup("claw")
{
  targetpos_pub=n_.advertise<geometry_msgs::PointStamped>("targetpos_points",10);
  this->nh_ = n_;
  this->box_num=0;
  this->search_num=0;
  this->grab_attempt=0;
  this->eggboxs={"egg_box1","egg_box2","egg_box3","egg_box4","egg_box5","egg_box6"};//预定义的位置
  this->search_area={"search_eggs","search_eggs2","search_eggs3"};//预定义的搜索区域
  this->arm_state=IDLE;
  try
  {
    this->tf_camera_to_robot.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(50.0));
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  try
  {
    this->tf_camera_to_robot.lookupTransform("/base_link", "/camera_link", ros::Time(0), (this->camera_to_robot_));
  }

  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
  }

  grasp_running = false; 

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(5.0).sleep();
  ROS_INFO_STREAM("Getting into the Grasping Position....");


  // Subscribe to targetpositions published by yolo
  target_positions_Sub = n_.subscribe("target_positions", 10, &CollectProcess::target_pointCb, this);
  status_sub=n_.subscribe("/move_base/status", 10, &CollectProcess::statusCallback, this);
  navigate_goal_sub=n_.subscribe("/move_base_simple/goal",1000,&CollectProcess::goalCallback,this);
}
void CollectProcess::statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    if (msg->status_list.empty()) {
        return;
    }
     actionlib_msgs::GoalStatus last_status = msg->status_list.back();
    // 如果导航成功并且机械臂处于IDLE状态，切换到WORKING状态
    if (last_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && arm_state == NAVIGATING) {
        ROS_INFO("reached start collect");
        arm_state = WORKING;
      }
  }

void  CollectProcess::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)//check if there is new navigation goal
{if( msg == nullptr){//no new goal.keep IDLE.
  return;
  }
  if (arm_state == IDLE)
  {
    arm_state =NAVIGATING;
  }
}
void CollectProcess::updateTransform() {
    try {
        this->tf_camera_to_robot.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(50.0));
    } catch (tf::TransformException &ex) {
        ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
        ros::Duration(1.0).sleep();
    }

    try {
        this->tf_camera_to_robot.lookupTransform("/base_link", "/camera_link", ros::Time(0), (this->camera_to_robot_));
    } catch (tf::TransformException &ex) {
        ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
    }
}

void CollectProcess::target_pointCb(const sensor_msgs::PointCloudConstPtr &msg)
{
  if (!grasp_running )
    {
      if (!(msg->points.empty())) 
        {
          empty_frame_start_time = ros::Time();//refresh empty frame
          ROS_INFO_STREAM("Processing the points to locate the Object...");
          float obj_x = msg->points[0].x;
          float obj_y = msg->points[0].y;
          float obj_z = msg->points[0].z;

          obj_camera_frame.setZ(obj_z);
          obj_camera_frame.setY(obj_y);
          obj_camera_frame.setX(obj_x);
          updateTransform();
          obj_robot_frame = camera_to_robot_ * obj_camera_frame;
          grasp_running = true;
          geometry_msgs::PointStamped target_msg;
          target_msg.header.stamp = ros::Time::now();
          target_msg.header.frame_id = "base_link";  // 坐标系名称
          target_msg.point.x = obj_robot_frame.getX();  // 目标点 x 坐标
          target_msg.point.y = obj_robot_frame.getY();  // 目标点 y 坐标
          target_msg.point.z = obj_robot_frame.getZ();  // 目标点 z 坐标
          targetpos_pub.publish(target_msg); 

        }
   else
    {
       if (empty_frame_start_time.isZero()) //record empty frame start time
       {
        empty_frame_start_time = ros::Time::now();
       }
       else if(ros::Time::now() - empty_frame_start_time > ros::Duration(2))
       {
            obj_robot_frame.setZ(0);
            obj_robot_frame.setY(0);
            obj_robot_frame.setX(0);
          if(search_num==search_area.size()-1){arm_state=IDLE;}//exit search continue navigate
        search_num=(search_num<search_area.size()-1)?(search_num+1):(0);//防止越界
        empty_frame_start_time = ros::Time();//update empty frame start time
       }
    }
   
  }
 
}

void CollectProcess::attainPosition(float x,float y, float z)
{//plann the arm move to a postion
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();
  geometry_msgs::Pose target_pose1;
  // Starting Postion before picking
  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;

  armgroup.setPoseTarget(target_pose1);
  armgroup.setMaxVelocityScalingFactor(0.3);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::MoveItErrorCode success = armgroup.plan(plan);
  ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");
  if(success)
      armgroup.execute(plan);
      sleep(1);
}

moveit::planning_interface::MoveItErrorCode CollectProcess::attainObject()
{//plan the arm move to grab eggs
  // ROS_INFO("The attain Object function called");

  attainPosition(obj_robot_frame.getX(), obj_robot_frame.getY(), obj_robot_frame.getZ()+0.08);

  // Open claw
  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("claw_release");
  grippergroup.move();

  // Slide down the Object
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();
  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = obj_robot_frame.getX();
  target_pose1.position.y = obj_robot_frame.getY();
  target_pose1.position.z = obj_robot_frame.getZ()+0.02;
  armgroup.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::MoveItErrorCode success = armgroup.plan(plan);
  ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");
  if(success)
      {
      armgroup.execute(plan);
      box_num=(box_num<eggboxs.size()-1)?(box_num+1):(0);
      }
    else
    {
      grab_attempt++;
      if(grab_attempt==2)//失败次数过多，不再尝试抓取。
      {
        grab_attempt=0;
        if(search_num==search_area.size()-1){
          arm_state=IDLE;
          }
        search_num=(search_num<search_area.size()-1)?(search_num+1):(0);//切换到下一搜索区  
      }
    }
    sleep(1);
  return success;
}

void CollectProcess::grasp()
{//close the claw
  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("claw_grap");
  grippergroup.move();
}

void CollectProcess::lift()
{ //put egg in the egg shell
  // For getting the pose
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();
  target_pose1.orientation = currPose.pose.orientation;
  armgroup.setNamedTarget(eggboxs[box_num]);//把鸡蛋移动到蛋盒内
  armgroup.move();

  // Put the eggs into the box, ready for next search
  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("claw_release");//松开鸡蛋放进去
   armgroup.setMaxVelocityScalingFactor(0.3);
  grippergroup.move();

  armgroup.setNamedTarget(search_area[search_num]);//机械臂回到搜索鸡蛋的状态
   armgroup.setMaxVelocityScalingFactor(0.3);
  armgroup.move();
  
  grippergroup.setNamedTarget("claw_grap");//关上机械爪
  grippergroup.move();
}

void CollectProcess::goHome()
{
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();
}

void CollectProcess::initiateGrasping()
{ 
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::WallDuration(3.0).sleep();
  if (arm_state== WORKING)
    {
    armgroup.setNamedTarget(search_area[search_num]);//机械臂进入到搜索鸡蛋的状态
    armgroup.setMaxVelocityScalingFactor(0.3);
    armgroup.move();
    homePose = armgroup.getCurrentPose();//搜索鸡蛋为默认状态

    
      //attainObject();
      if(obj_robot_frame.getX()!=0||obj_robot_frame.getY()!=0||obj_robot_frame.getZ()!=0)//all 0 means no valid target
      {
        if(attainObject())
        {//ROS_INFO_STREAM("Attempting to Grasp the Object now..");
          grasp();
          //ROS_INFO_STREAM("Lifting the Object....");
          lift();
        }

        
      }
      grasp_running = false;
    }
  else
    {
        armgroup.setNamedTarget("arm_retract");//retract arm for slam and navigate
        armgroup.move();
    }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pic_put_node");
  float length, breadth, pregrasp_x, pregrasp_y, pregrasp_z;
  ros::NodeHandle n;
  ros::Publisher joint_states_pub=n.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states",10); //synic state of arm to reality
  
  CollectProcess simGrasp(n);
  ROS_INFO_STREAM("Waiting for five seconds..");
  ros::WallDuration(5.0).sleep();
  while (ros::ok()) 
  {
    // Process image callback
    ros::spinOnce();

    simGrasp.initiateGrasping();
  }
  return 0;
}


