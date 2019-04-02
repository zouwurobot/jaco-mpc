#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <joint_trajectory_generator/trajectory_generation.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Vector3.h>
#include <kdl/velocityprofile_trap.hpp>
#include "kinova_driver/kinova_fake_joint_trajectory_controller.h"

using namespace trajectory;
using namespace std;
using namespace Eigen;


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

KDL::JntArray current_state(6);
geometry_msgs::Vector3 target_pos;
ros::Time simulationTime;

void printTraj(const trajectory_msgs::JointTrajectory& traj)
{
  cout << "Trajectory:" << endl;
  cout << " - Joints: " << endl;
  for (unsigned int i=0; i<traj.joint_names.size(); i++)
    cout << "    - " << traj.joint_names[i] << endl;
  cout << " - Points: " << endl;
  for (unsigned int i=0; i<traj.points.size(); i++){
    cout << "     - Time from start: " << traj.points[i].time_from_start.toSec()<< endl;
    cout << "     - Positions: " << endl;
    for (unsigned int j=0; j<traj.points[i].positions.size(); j++)
      cout << "      - " << traj.points[i].positions[j] << endl;
    cout << "     - Velocities: " << endl;
    for (unsigned int j=0; j<traj.points[i].velocities.size(); j++)
      cout << "      - " << traj.points[i].velocities[j] << endl;
    cout << "     - Accelerations: " << endl;
    for (unsigned int j=0; j<traj.points[i].accelerations.size(); j++)
      cout << "      - " << traj.points[i].accelerations[j] << endl;
  }
}

void JointStateCB(const kinova_msgs::JointAnglesConstPtr &angle_msg){

  current_state(0) = angle_msg->joint1;
  current_state(1) = angle_msg->joint2;
  current_state(2) = angle_msg->joint3;
  current_state(3) = angle_msg->joint4;
  current_state(4) = angle_msg->joint5;
  current_state(5) = angle_msg->joint6;

}

void SimulationTimeCB(const std_msgs::Float32ConstPtr &time){

  simulationTime = ros::Time(time->data);
}

void TargetPosCB(const geometry_msgs::Vector3ConstPtr &pos_msg){

    target_pos.x = pos_msg->x;
    target_pos.y = pos_msg->y;
    target_pos.z = pos_msg->z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_planning");
  ros::NodeHandle nh("~");

  int flag = 0;  
  float kinova_max_vel;
  float kinova_max_a;

  string chain_start, chain_end, urdf_param;
  double timeout;
  double eps = 1e-5;

  nh.param("chain_start", chain_start, string(""));
  nh.param("chain_end", chain_end, string(""));
  nh.param("kinova_max_vel", kinova_max_vel, 0.3f);
  nh.param("kinova_max_a", kinova_max_a, 1e9f);
  
  if (chain_start=="" || chain_end=="") {
    ROS_FATAL("Missing chain info in launch file");
    exit (-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, string("/robot_description"));

  ros::Subscriber joint_state_sub = nh.subscribe("/vrep/joint_angles", 1, &JointStateCB);
  ros::Subscriber simulation_time_sub = nh.subscribe("/vrep/simulationTime",1, &SimulationTimeCB);
  ros::Subscriber target_pos_sub = nh.subscribe("/vrep/target_pos", 1, &TargetPosCB);

  ros::Publisher catch_pub = nh.advertise<std_msgs::Bool>("/vrep/catch", 1);
  
  
  Client client("/j2n6s300/follow_joint_trajectory", true); // true -> don't need ros::spin()
  ros::Duration(2).sleep();
  // client.waitForServer();

  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);
  KDL::Chain chain;
  
  
  bool valid = tracik_solver.getKDLChain(chain);
  
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return 0;
  }

  // Set up KDL IK
  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  KDL::JntArray result;
  KDL::Frame end_effector_pose;  
  trajectory_msgs::JointTrajectory traj_in, traj_out;

  trajectory::TrajectoryGenerator generator(kinova_max_vel, kinova_max_a, 6);
  control_msgs::FollowJointTrajectoryGoal traj_goal;
  vector<trajectory_msgs::JointTrajectoryPoint> traj_points;
  ros::Duration traj_dur;
  ros::Duration delay;


  while(!flag){

    ros::spinOnce();
    KDL::Vector v(target_pos.x, target_pos.y, target_pos.z);
    KDL::Rotation R(0,-1,0,0,0,-1,1,0,0);
  
    end_effector_pose = KDL::Frame(R,v);
    tracik_solver.CartToJnt(current_state,end_effector_pose,result);

    cout << "current_state" << endl;
    
    for(int i = 0; i<6;i++)
      cout << current_state(i) <<endl;

    cout << "result" << endl;

    for(int i = 0; i<6;i++)
      cout << result(i) <<endl;
    
    traj_in.header.stamp = ros::Time::now();
    traj_in.joint_names = {"j2n6s300_joint_1", "j2n6s300_joint_2", "j2n6s300_joint_3", "j2n6s300_joint_4", "j2n6s300_joint_5","j2n6s300_joint_6"};

    trajectory_msgs::JointTrajectoryPoint start_point;
    trajectory_msgs::JointTrajectoryPoint end_point;
    start_point.positions = {current_state(0),current_state(1),current_state(2),current_state(3),current_state(4),current_state(5)};
    start_point.velocities = {0.0,0.0,0.0,0.0,0.0,0.0};
    end_point.positions = {result(0),(result(1)-3.14159*2),result(2),result(3),result(4),result(5)};
    end_point.velocities = {0.0,0.0,0.0,0.0,0.0,0.0};

    vector<trajectory_msgs::JointTrajectoryPoint>temp_traj(2);
    temp_traj[0] = start_point;
    temp_traj[1] = end_point;

    traj_in.points = temp_traj;
    // printTraj(traj_in);

    generator.generate(traj_in, traj_out);
    // printTraj(traj_out);

    traj_points = traj_out.points;
    traj_dur = traj_points[traj_points.size()-1].time_from_start - traj_points[0].time_from_start;

    traj_goal.trajectory = traj_out;
    client.sendGoal(traj_goal);

    if(traj_dur.toSec() < 0.5)
      delay = traj_dur;
    else if(traj_dur.toSec() > 2.0)
      delay = ros::Duration(1.0);
    else
      delay = ros::Duration(traj_dur.toSec()/2);

    delay.sleep();

    ros::spinOnce();
    flag = 1;
    for(int i = 0; i < 6; i++){
      
      if(i == 1){
        if(abs(current_state(i) - result(i) + 2*3.14159) > 0.01){
          cout<<"joint"<<endl;
          flag = 0;
        }
      }
      else{
        if(abs(current_state(i) - result(i)) > 0.01){
          cout<<"joint"<<endl;
          flag = 0;
        }
      }
    }

    if(flag == 1){
      KDL::Frame end_pose;
      fk_solver.JntToCart(current_state, end_pose);
      if(abs(end_pose(0,3) - target_pos.x) > 0.001){
        flag = 0;
        cout<<"pos1"<<endl;
      }
      if(abs(end_pose(1,3) - target_pos.y) > 0.001){
        flag = 0;
        cout<<"pos2"<<endl;
        cout<<end_pose(1,3)<<endl;
        cout<<target_pos.y<<endl;
      }
      if(abs(end_pose(2,3) - target_pos.z) > 0.001){
        flag = 0;
        cout<<"pos3"<<endl;
        cout<<end_pose(2,3)<<endl;
        cout<<target_pos.z<<endl;
      }

    }
    if(flag == 1){
      std_msgs::Bool catch_msg;
      catch_msg.data = true;
      catch_pub.publish(catch_msg);
      
    }

  }
  
  client.waitForResult(ros::Duration(20.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     printf("Yay! The dishes are now clean");


  return 0;
}

