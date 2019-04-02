#ifndef KINOVA_FAKE_JOINT_TRAJECTORY_CONTROLLER_H
#define KINOVA_FAKE_JOINT_TRAJECTORY_CONTROLLER_H


#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <kinova_msgs/JointVelocity.h>
#include <kinova_msgs/JointAngles.h>
#include <kinova_driver/SolveQP.h>
#include <boost/thread.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>


#include "kinova_ros_types.h"
#include "kinova_api.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>




namespace kinova
{

class FakeJointTrajectoryController
{
public:
    FakeJointTrajectoryController(ros::NodeHandle &n);
    ~FakeJointTrajectoryController();


private:
    ros::NodeHandle nh_;

    ros::Subscriber sub_command_;
    ros::Subscriber sub_velocity_;
    ros::Subscriber sub_joint_angles_;
    ros::Subscriber sub_obs_pos_;
    ros::Subscriber sub_obs_vel_;
    ros::Subscriber sub_dist_min_;
    ros::Subscriber sub_simulation_time_;

    ros::Publisher pub_joint_feedback_;
    ros::Publisher pub_joint_velocity_;

    ros::ServiceClient client_qp_solver_;

    ros::Time previous_pub_;
    ros::Time time_pub_joint_vel_;
    ros::Time simulationTime;
    ros::Time lastsimulationTime;


    ros::Timer timer_pub_joint_vel_;
    boost::mutex terminate_thread_mutex_;
    boost::thread* thread_update_state_;
    bool terminate_thread_;

    std::string chain_start, urdf_param;
    double timeout;
    double eps;

    std_msgs::Duration time_from_start_;
    sensor_msgs::JointState current_joint_state_;

    kinova_msgs::JointAngles current_joint_angles;
    kinova_msgs::JointVelocity current_joint_velocity;
    kinova_msgs::JointVelocity current_joint_command;

    geometry_msgs::Vector3 obs_pos;
    geometry_msgs::Vector3 obs_vel;

    kinova_msgs::JointAngles dist_min;
    float dis_current;
    float fai;

    float bound_a;
    float safe_bar;
    float kinova_max_vel;
    float kinova_max_a;
    float res_distance;
    float dmin;
    float balance_index;

    
    TrajectoryPoint kinova_traj_point_;

//    trajectory_msgs::JointTrajectory joint_traj_;
//    trajectory_msgs::JointTrajectoryPoint joint_traj_point_;
    std::string traj_frame_id_;
    std::vector<trajectory_msgs::JointTrajectoryPoint> traj_command_points_;
    control_msgs::FollowJointTrajectoryFeedback traj_feedback_msg_;

    // stores the command to send to robot, in Kinova type (KinovaAngles)
    std::vector<KinovaAngles> kinova_angle_command_;

    uint number_joint_;
    int traj_command_points_index_;
    std::vector<std::string> joint_names_;
    std::string prefix_;

    struct Segment
    {
        double start_time;
        double duration;
        std::vector<double> positions;
        std::vector<double> velocities;
    };


    // call back function when receive a trajectory command
    void commandCB(const trajectory_msgs::JointTrajectoryConstPtr &traj_msg);
    void velocityCB(const kinova_msgs::JointVelocityConstPtr &velocity_msg);
    void joint_anglesCB(const kinova_msgs::JointAnglesConstPtr &angle_msg);
    void obs_posCB(const geometry_msgs::Vector3ConstPtr &pos_msg);
    void obs_velCB(const geometry_msgs::Vector3ConstPtr &vel_msg);
    void dist_minCB(const kinova_msgs::JointAnglesConstPtr &dist_msg);
    void SimulationTimeCB(const std_msgs::Float32ConstPtr &time);

    // void function_grad(const real_1d_array &x, double &func, real_1d_array &grad, void *ptr);
    // reflash the robot state and publish the joint state: either by timer or thread
    void update_state(); // by thread

    void pub_joint_vel(const ros::TimerEvent&); // by timer
    int test;

};






}


#endif // KINOVA_JOINT_TRAJECTORY_CONTROLLER_H
