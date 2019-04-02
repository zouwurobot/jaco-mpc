
#include <kinova_driver/kinova_fake_joint_trajectory_controller.h>
#include <angles/angles.h>
#include <ros/console.h>


using namespace kinova;
using namespace Eigen;



float min_distance_cal(Vector3f M, Vector3f N, Vector3f A){
    float distance;
    float a;
    Vector3f MN = N - M;
    Vector3f AM = M - A;
    Vector3f AN = N - A;
    if (AM.dot(MN) >= 0)
        distance = sqrt(AM.dot(AM));
    else if (AN.dot(MN) <= 0)
        distance = sqrt(AN.dot(AN));
    else{
        a = AN.dot(MN)/sqrt(MN.dot(MN));
        distance = sqrt(AN.dot(AN) - a*a);
    }
    return distance;
    
}




FakeJointTrajectoryController::FakeJointTrajectoryController(ros::NodeHandle& n):
    // kinova_comm_(kinova_comm),
    nh_(n)
{
    //ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);

    ros::NodeHandle pn("~");
    // std::string robot_type;
    prefix_ = "j2n6s300";
    
    number_joint_ = 6;

    // Display debug information in teminal
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    chain_start = "j2n6s300_link_base";
    timeout = 0.005;
    eps = 1e-5;
    nh_.param("robot_description", urdf_param, std::string());
    nh_.param("bound_a", bound_a, 1.0f);
    nh_.param("safe_bar", safe_bar, 0.2f);
    nh_.param("kinova_max_vel", kinova_max_vel, 0.3f);
    nh_.param("kinova_max_a", kinova_max_a, 0.5f);
    nh_.param("res_distance", res_distance, 0.2f);
    nh_.param("dmin", dmin, 0.3f);
    nh_.param("balance_index", balance_index, 0.7f);
    sub_velocity_ = nh_.subscribe("/vrep/velocity", 1, &FakeJointTrajectoryController::velocityCB, this);
    sub_joint_angles_ = nh_.subscribe("/vrep/joint_angles", 1, &FakeJointTrajectoryController::joint_anglesCB, this);
    sub_command_ = nh_.subscribe("/trajectory_controller/command", 1, &FakeJointTrajectoryController::commandCB, this);
    sub_obs_pos_ = nh_.subscribe("/vrep/obstacle_pos", 1, &FakeJointTrajectoryController::obs_posCB, this);
    sub_obs_vel_ = nh_.subscribe("/vrep/obstacle_vel", 1, &FakeJointTrajectoryController::obs_velCB, this);
    sub_dist_min_ = nh_.subscribe("/vrep/distance_min", 1, &FakeJointTrajectoryController::dist_minCB, this);
    sub_simulation_time_ = nh_.subscribe("/vrep/simulationTime", 1, &FakeJointTrajectoryController::SimulationTimeCB, this);

    pub_joint_feedback_ = nh_.advertise<control_msgs::FollowJointTrajectoryFeedback>("/trajectory_controller/state", 1);
    pub_joint_velocity_ = nh_.advertise<kinova_msgs::JointVelocity>("/in/joint_velocity", 2);

    client_qp_solver_ = nh_.serviceClient<kinova_driver::SolveQP>("solve_qp");

    traj_frame_id_ = "root";   
    joint_names_.resize(number_joint_);
    //std::cout << "joint names in feedback of trajectory state are: " << std::endl;
    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = prefix_ + "_joint_" + boost::lexical_cast<std::string>(i+1);
        std::cout << joint_names_[i] << " ";
    }
    std::cout << std::endl;

    timer_pub_joint_vel_ = nh_.createTimer(ros::Duration(0.01), &FakeJointTrajectoryController::pub_joint_vel, this, false, false);
    terminate_thread_ = false;

    thread_update_state_ = new boost::thread(boost::bind(&FakeJointTrajectoryController::update_state, this));

    traj_feedback_msg_.joint_names.resize(joint_names_.size());
    traj_feedback_msg_.desired.positions.resize(joint_names_.size());
    traj_feedback_msg_.desired.velocities.resize(joint_names_.size());
    traj_feedback_msg_.actual.positions.resize(joint_names_.size());
    traj_feedback_msg_.actual.velocities.resize(joint_names_.size());
    traj_feedback_msg_.error.positions.resize(joint_names_.size());
    traj_feedback_msg_.error.velocities.resize(joint_names_.size());
    traj_feedback_msg_.joint_names = joint_names_;

    // counter in the timer to publish joint velocity command: pub_joint_vel()
    traj_command_points_index_ = 0;

    //ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);


  //   Program qp (CGAL::SMALLER, true, 0, false, 0); 
  
  // // now set the non-default entries: 
  // const double X = 0.5; 
  // const double Y = 1.5;
  // qp.set_a(X, 0,  1.1); qp.set_a(Y, 0, 1.2); qp.set_b(0, 9.3);  //  x + y  <= 7
  // qp.set_a(X, 1, -1); qp.set_a(Y, 1, 2.1); qp.set_b(1, 7.2);  // -x + 2y <= 4
  // qp.set_u(Y, true, 4);                                   //       y <= 4
  // qp.set_d(X, X, 4.2); qp.set_d (Y, Y, 8); // !!specify 2D!!    x^2 + 4 y^2
  // qp.set_c(Y, -50.5);                                       // -32y
  // qp.set_c0(32);                                          // +64
  // // solve the program, using ET as the exact type
  // Solution s = CGAL::solve_quadratic_program(qp, ET());
  // // assert (s.solves_quadratic_program(qp));
  // for(Solution::Variable_value_iterator it = s.variable_values_begin(); it != s.variable_values_end(); ++it){

  //   std::cout<<*it<<std::endl;
  // }


}

FakeJointTrajectoryController::~FakeJointTrajectoryController()
{
    //ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);
    ROS_WARN("destruction entered!");
    {
        boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
        terminate_thread_ = true;
    }

    sub_command_.shutdown();
    pub_joint_feedback_.shutdown();
    pub_joint_velocity_.shutdown();

    timer_pub_joint_vel_.stop();
    thread_update_state_->join();
    delete thread_update_state_;

    //ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}


void FakeJointTrajectoryController::velocityCB(const kinova_msgs::JointVelocityConstPtr &velocity_msg)
{
    current_joint_velocity.joint1 = velocity_msg->joint1;
    current_joint_velocity.joint2 = velocity_msg->joint2;
    current_joint_velocity.joint3 = velocity_msg->joint3;
    current_joint_velocity.joint4 = velocity_msg->joint4;
    current_joint_velocity.joint5 = velocity_msg->joint5;
    current_joint_velocity.joint6 = velocity_msg->joint6;

}


void FakeJointTrajectoryController::joint_anglesCB(const kinova_msgs::JointAnglesConstPtr &angle_msg)
{
    current_joint_angles.joint1 = angle_msg->joint1;
    current_joint_angles.joint2 = angle_msg->joint2;
    current_joint_angles.joint3 = angle_msg->joint3;
    current_joint_angles.joint4 = angle_msg->joint4;
    current_joint_angles.joint5 = angle_msg->joint5;
    current_joint_angles.joint6 = angle_msg->joint6;

}

void FakeJointTrajectoryController::obs_posCB(const geometry_msgs::Vector3ConstPtr &pos_msg)
{
    obs_pos.x = pos_msg->x;
    obs_pos.y = pos_msg->y;
    obs_pos.z = pos_msg->z;

}

void FakeJointTrajectoryController::obs_velCB(const geometry_msgs::Vector3ConstPtr &vel_msg)
{
    obs_vel.x = vel_msg->x;
    obs_vel.y = vel_msg->y;
    obs_vel.z = vel_msg->z;

}

void FakeJointTrajectoryController::dist_minCB(const kinova_msgs::JointAnglesConstPtr &dist_msg)
{
    dist_min.joint1 = dist_msg->joint1;
    dist_min.joint2 = dist_msg->joint2;
    dist_min.joint3 = dist_msg->joint3;
    dist_min.joint4 = dist_msg->joint4;

}

void FakeJointTrajectoryController::SimulationTimeCB(const std_msgs::Float32ConstPtr &time){

    simulationTime = ros::Time(time->data);
}


void FakeJointTrajectoryController::commandCB(const trajectory_msgs::JointTrajectoryConstPtr &traj_msg)
{
    //ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);

    bool command_abort = false;
    traj_command_points_index_ = 0;

//    // if receive new command, clear all trajectory and stop api
//    kinova_comm_.stopAPI();
//    if(!kinova_comm_.isStopped())
//    {
//        ros::Duration(0.01).sleep();
//    }
//    kinova_comm_.eraseAllTrajectories();

//    kinova_comm_.startAPI();
//    if(kinova_comm_.isStopped())
//    {
//        ros::Duration(0.01).sleep();
//    }

    traj_command_points_ = traj_msg->points;
    ROS_INFO_STREAM("Trajectory controller Receive trajectory with points number: " << traj_command_points_.size());

    // Map the index in joint_names and the msg
    std::vector<int> lookup(number_joint_, -1);

    for (size_t j = 0; j<number_joint_; j++)
    {
        for (size_t k = 0; k<traj_msg->joint_names.size(); k++)
            if(traj_msg->joint_names[k] == joint_names_[j]) // find joint_j in msg;
            {
                lookup[j] = k;
                break;
            }

        if (lookup[j] == -1) // if joint_j not found in msg;
        {
            std::string error_msg = "Joint name : " + joint_names_[j] + " not found in the msg.";
            ROS_ERROR("%s", error_msg.c_str());
            command_abort = true;
            return;
        }
    }

    // check msg validation
    for (size_t j = 0; j<traj_command_points_.size(); j++)
    {
        // position should not be empty
        if (traj_command_points_[j].positions.empty()) // find joint_j in msg;
        {
            ROS_ERROR_STREAM("Positions in trajectory command cannot be empty at point: " << j);
            command_abort = true;
            break;
        }
        // position size match
        if (traj_command_points_[j].positions.size() != number_joint_)
        {
            ROS_ERROR_STREAM("Positions at point " << j << " has size " << traj_command_points_[j].positions.size() << " in trajectory command, which does not match joint number! ");
            command_abort = true;
            break;
        }

        // if velocity provided, size match
        if (!traj_command_points_[j].velocities.empty() && traj_command_points_[j].velocities.size() != number_joint_)
        {
            ROS_ERROR_STREAM("Velocities at point " << j << " has size " << traj_command_points_[j].velocities.size() << " in trajectory command, which does not match joint number! ");
            command_abort = true;
            break;
        }
    }

    if(command_abort)
        return;

    // store angle velocity command sent to robot
//    std::vector<KinovaAngles> kinova_angle_command;
    kinova_angle_command_.resize(traj_command_points_.size());
    for (size_t i = 0; i<traj_command_points_.size(); i++)
    {
        kinova_angle_command_[i].InitStruct(); // initial joint velocity to zeros.

        kinova_angle_command_[i].Actuator1 = traj_command_points_[i].velocities[0];
        kinova_angle_command_[i].Actuator2 = traj_command_points_[i].velocities[1];
        kinova_angle_command_[i].Actuator3 = traj_command_points_[i].velocities[2];
        kinova_angle_command_[i].Actuator4 = traj_command_points_[i].velocities[3];
        if (number_joint_>=6)
        {
            kinova_angle_command_[i].Actuator5 = traj_command_points_[i].velocities[4];
            kinova_angle_command_[i].Actuator6 = traj_command_points_[i].velocities[5];
            if (number_joint_==7)
                kinova_angle_command_[i].Actuator7 = traj_command_points_[i].velocities[6];
        }
    }
    // replace last velocity command (which is zero) to previous non-zero value, trying to drive robot moving a forward to get closer to the goal.
    kinova_angle_command_[traj_command_points_.size()-1] = kinova_angle_command_[traj_command_points_.size()-2];

    std::vector<double> durations(traj_command_points_.size(), 0.0); // computed by time_from_start
    double trajectory_duration = traj_command_points_[0].time_from_start.toSec();

    durations[0] = trajectory_duration;
//    ROS_DEBUG_STREAM("durationsn 0 is: " << durations[0]);

    for (int i = 1; i<traj_command_points_.size(); i++)
    {
        durations[i] = (traj_command_points_[i].time_from_start - traj_command_points_[i-1].time_from_start).toSec();
        trajectory_duration += durations[i];
//        ROS_DEBUG_STREAM("durations " << i << " is: " << durations[i]);
    }

    // start timer thread to publish joint velocity command
    time_pub_joint_vel_ = ros::Time::now();
    // time_pub_joint_vel_ = simulationTime;
    timer_pub_joint_vel_.start();

    //ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}


void FakeJointTrajectoryController::pub_joint_vel(const ros::TimerEvent&)
{
    // send out each velocity command with corresponding duration delay.

    kinova_msgs::JointVelocity joint_velocity_msg;

    if (traj_command_points_index_ <  kinova_angle_command_.size() && ros::ok())
    {
        joint_velocity_msg.joint1 = kinova_angle_command_[traj_command_points_index_].Actuator1;
        joint_velocity_msg.joint2 = kinova_angle_command_[traj_command_points_index_].Actuator2;
        joint_velocity_msg.joint3 = kinova_angle_command_[traj_command_points_index_].Actuator3;
        joint_velocity_msg.joint4 = kinova_angle_command_[traj_command_points_index_].Actuator4;
        joint_velocity_msg.joint5 = kinova_angle_command_[traj_command_points_index_].Actuator5;
        joint_velocity_msg.joint6 = kinova_angle_command_[traj_command_points_index_].Actuator6;
        joint_velocity_msg.joint7 = kinova_angle_command_[traj_command_points_index_].Actuator7;

        // In debug: compare values with topic: follow_joint_trajectory/goal, command
//        ROS_DEBUG_STREAM_ONCE( std::endl <<" joint_velocity_msg.joint1: " << joint_velocity_msg.joint1 * M_PI/180 <<
//                          std::endl <<" joint_velocity_msg.joint2: " << joint_velocity_msg.joint2 * M_PI/180 <<
//                          std::endl <<" joint_velocity_msg.joint3: " << joint_velocity_msg.joint3 * M_PI/180 <<
//                          std::endl <<" joint_velocity_msg.joint4: " << joint_velocity_msg.joint4 * M_PI/180 <<
//                          std::endl <<" joint_velocity_msg.joint5: " << joint_velocity_msg.joint5 * M_PI/180 <<
//                          std::endl <<" joint_velocity_msg.joint6: " << joint_velocity_msg.joint6 * M_PI/180 );

        pub_joint_velocity_.publish(joint_velocity_msg);

        current_joint_command = joint_velocity_msg;

        // if( (simulationTime - time_pub_joint_vel_) >= traj_command_points_[traj_command_points_index_].time_from_start)
        // {
        //     std::cout << (traj_command_points_[traj_command_points_index_].positions[1]) << std::endl;
        //     std::cout << (current_joint_angles.joint2) << std::endl;
        //     std::cout << (traj_command_points_[traj_command_points_index_].velocities[1]) << std::endl;
        //     std::cout << (joint_velocity_msg.joint2) << std::endl;
        //     ROS_INFO_STREAM("Moved to point " << traj_command_points_index_++);

        // }
        if( (ros::Time::now() - time_pub_joint_vel_) >= traj_command_points_[traj_command_points_index_].time_from_start)
        {
            
            // std::cout << (traj_command_points_[traj_command_points_index_].positions[1]) << std::endl;
            // std::cout << (current_joint_angles.joint2) << std::endl;
            // std::cout << (traj_command_points_[traj_command_points_index_].positions[1]-current_joint_angles.joint2) << std::endl;
            // std::cout << (current_joint_command.joint2) << std::endl;
            // std::cout << (current_joint_velocity.joint2) << std::endl;
            ROS_INFO_STREAM("Moved to point " << traj_command_points_index_++);
            ros::Time init = ros::Time::now();
            float distance_min[] = {dist_min.joint1,dist_min.joint2,dist_min.joint3,dist_min.joint4};
            float min_distance = distance_min[0];
            int min_id = 0;
            
            for(int i = 1; i < 4; i++){
                if(distance_min[i] < min_distance){
                    min_distance = distance_min[i];
                    min_id = i;
                }
            }

            if(min_distance <= res_distance){
                
                std::cout<<min_distance<<std::endl;
                std::cout<<res_distance<<std::endl;
                std::string chain_end1, chain_end2;
                
                switch(min_id){
                    case 0:
                        chain_end1 = "j2n6s300_link_2";
                        chain_end2 = "j2n6s300_link_3";
        
                        break;
                    case 1:
                        chain_end1 = "j2n6s300_link_3";
                        chain_end2 = "j2n6s300_link_4";
                      
                        break;
                    case 2:
                        chain_end1 = "j2n6s300_link_4";
                        chain_end2 = "j2n6s300_link_5";
                
                        break;
                    case 3:
                        chain_end1 = "j2n6s300_link_5";
                        chain_end2 = "j2n6s300_link_6";
                        
                        break;
                }
                // TRAC_IK::TRAC_IK tracik_solver1(chain_start, chain_end1, urdf_param, timeout, eps);
                // TRAC_IK::TRAC_IK tracik_solver2(chain_start, chain_end1, urdf_param, timeout, eps);

                KDL::Tree tree;                             
                kdl_parser::treeFromString(urdf_param, tree);   
                KDL::Chain chain1;
                KDL::Chain chain2; 
                tree.getChain(chain_start, chain_end1, chain1);
                tree.getChain(chain_start, chain_end2, chain2);
                // KDL::Chain chain1;
                // KDL::Chain chain2;
                // tracik_solver1.getKDLChain(chain1);
                // tracik_solver2.getKDLChain(chain2);

                KDL::ChainFkSolverPos_recursive fk_solver1(chain1);
                KDL::ChainFkSolverPos_recursive fk_solver2(chain2);
                unsigned int n1 = chain1.getNrOfJoints();
                unsigned int n2 = chain2.getNrOfJoints();
                KDL::JntArray goal_joint1(n1);
                KDL::JntArray goal_joint2(n2);
                KDL::Frame end_effector_pose1;
                KDL::Frame end_effector_pose2;
                
                goal_joint1(0) = current_joint_angles.joint1;
                goal_joint2(0) = current_joint_angles.joint1;
                if(n1 > 1)
                    goal_joint1(1) = current_joint_angles.joint2;
                if(n1 > 2)
                    goal_joint1(2) = current_joint_angles.joint3;
                if(n1 > 3)
                    goal_joint1(3) = current_joint_angles.joint4;
                if(n1 > 4)
                    goal_joint1(4) = current_joint_angles.joint5;

                if(n2 > 1)
                    goal_joint2(1) = current_joint_angles.joint2;
                if(n2 > 2)
                    goal_joint2(2) = current_joint_angles.joint3;
                if(n2 > 3)
                    goal_joint2(3) = current_joint_angles.joint4;
                if(n2 > 4)
                    goal_joint2(4) = current_joint_angles.joint5;

                fk_solver1.JntToCart(goal_joint1, end_effector_pose1);
                fk_solver2.JntToCart(goal_joint2, end_effector_pose2);

                Vector3f end_point1(end_effector_pose1(0,3), end_effector_pose1(1,3), end_effector_pose1(2,3));
                Vector3f end_point2(end_effector_pose2(0,3), end_effector_pose2(1,3), end_effector_pose2(2,3));
                Vector3f obs_point(obs_pos.x, obs_pos.y, obs_pos.z);

                dis_current = min_distance_cal(end_point1, end_point2, obs_point);
                std::cout<<"dis_current"<<std::endl;
                std::cout<<dis_current<<std::endl;

                //Calculate the derivatives of dis_current to joints
                float d_angle = 0.01;
                VectorXf deri_joint(6);
                float dis_djoint;

                KDL::JntArray goal_joint10(n1);
                KDL::JntArray goal_joint20(n2);

                KDL::Frame end_effector_pose10;
                KDL::Frame end_effector_pose20;
                Vector3f end_point10;
                Vector3f end_point20;


                for(int i = 0; i < 6; i++){
                    if(n1 < (i+1) && n2 < (i+1)){
                        deri_joint[i] = 0;
                    }
                    else{
                        for(int i = 0; i < n1; i++)
                            goal_joint10(i) = goal_joint1(i);
                        for(int i = 0; i < n2; i++)
                            goal_joint20(i) = goal_joint2(i);

                        if(n1 >= (i+1)){
                            goal_joint10(i) = goal_joint10(i) + d_angle;
                            fk_solver1.JntToCart(goal_joint10, end_effector_pose10);
                            end_point10 << end_effector_pose10(0,3), end_effector_pose10(1,3), end_effector_pose10(2,3);
                        }
                        else{
                            end_point10 << end_point1(0), end_point1(1), end_point1(2);
                        }

                        if(n2 >= (i+1)){
                            goal_joint20(i) = goal_joint20(i) + d_angle;
                            fk_solver2.JntToCart(goal_joint20, end_effector_pose20);
                            end_point20 << end_effector_pose20(0,3), end_effector_pose20(1,3), end_effector_pose20(2,3);
                        }
                        else{
                            end_point20 = end_point2;
                        }
                        dis_djoint = min_distance_cal(end_point10, end_point20, obs_point);
                        deri_joint[i] = (dis_djoint - dis_current)/d_angle;

                    }
                    // std::cout<<i<<std::endl;
                    // std::cout<<deri_joint[i]<<std::endl;
                }

                //Calculate the derivatives of dis_current to obs_pos
                float d_pos = 0.001;
                float dis_dpos;
                Vector3f deri_obs_pos;
                Vector3f obs_point0;

                for(int i = 0; i < 3; i++){
                    for(int j = 0; j < 3; j++)
                        obs_point0[j] = obs_point[j];
                    obs_point0[i] = obs_point0[i] + d_pos;
                    dis_dpos = min_distance_cal(end_point1, end_point2, obs_point0);
                    deri_obs_pos[i] = (dis_dpos - dis_current)/d_pos;
                    // std::cout<<deri_obs_pos[i]<<std::endl;

                }

                //Calculate the derivatives of dis_current to time
                float deri_t;
                VectorXf joint_vel(6);
                joint_vel[0] = current_joint_velocity.joint1;
                joint_vel[1] = current_joint_velocity.joint2;
                joint_vel[2] = current_joint_velocity.joint3;
                joint_vel[3] = current_joint_velocity.joint4;
                joint_vel[4] = current_joint_velocity.joint5;
                joint_vel[5] = current_joint_velocity.joint6;

                Vector3f obstacle_vel(obs_vel.x, obs_vel.y, obs_vel.z);

                deri_t = deri_obs_pos.dot(obstacle_vel) + deri_joint.dot(joint_vel);

                //Calculate the safety index
                fai = dmin*dmin - dis_current*dis_current - deri_t;

                if(fai >= 0){

                    //Prepare for optimization
                    Vector3f obstacle_vel_max;
                    Vector3f obstacle_vel_min;
                    
                    Vector3f obs_a(bound_a, bound_a, bound_a);
                    obstacle_vel_max = obstacle_vel + obs_a * 0.1;
                    obstacle_vel_min = obstacle_vel - obs_a * 0.1;

                    VectorXf G(6);
                    float h1, h2, c;
                    G = -2*dis_current*deri_joint;

                    h1 = -safe_bar - deri_obs_pos.dot(obstacle_vel_max);
                    h2 = -safe_bar - deri_obs_pos.dot(obstacle_vel_min);

                    //Optimization set up
                    kinova_driver::SolveQP srv;
                    srv.request.G1 = G[0];
                    srv.request.G2 = G[1];
                    srv.request.G3 = G[2];
                    srv.request.G4 = G[3];
                    srv.request.G5 = G[4];
                    srv.request.G6 = G[5];
                    srv.request.h1 = h1;
                    srv.request.h2 = h2;
                    srv.request.v_max = kinova_max_vel;
                    srv.request.U01 = kinova_angle_command_[traj_command_points_index_].Actuator1;
                    srv.request.U02 = kinova_angle_command_[traj_command_points_index_].Actuator2;
                    srv.request.U03 = kinova_angle_command_[traj_command_points_index_].Actuator3;
                    srv.request.U04 = kinova_angle_command_[traj_command_points_index_].Actuator4;
                    srv.request.U05 = kinova_angle_command_[traj_command_points_index_].Actuator5;
                    srv.request.U06 = kinova_angle_command_[traj_command_points_index_].Actuator6;

                    if(client_qp_solver_.call(srv)){
                        ROS_INFO("Service request called");

                        kinova_angle_command_[traj_command_points_index_].Actuator1 = srv.response.U1*(1-balance_index) + kinova_angle_command_[traj_command_points_index_].Actuator1*balance_index;
                        kinova_angle_command_[traj_command_points_index_].Actuator2 = srv.response.U2*(1-balance_index) + kinova_angle_command_[traj_command_points_index_].Actuator2*balance_index;
                        kinova_angle_command_[traj_command_points_index_].Actuator3 = srv.response.U3*(1-balance_index) + kinova_angle_command_[traj_command_points_index_].Actuator3*balance_index;
                        kinova_angle_command_[traj_command_points_index_].Actuator4 = srv.response.U4*(1-balance_index) + kinova_angle_command_[traj_command_points_index_].Actuator4*balance_index;
                        kinova_angle_command_[traj_command_points_index_].Actuator5 = srv.response.U5*(1-balance_index) + kinova_angle_command_[traj_command_points_index_].Actuator5*balance_index;
                        kinova_angle_command_[traj_command_points_index_].Actuator6 = srv.response.U6*(1-balance_index) + kinova_angle_command_[traj_command_points_index_].Actuator6*balance_index;


                    }
                    else{
                        ROS_ERROR("Service request called failed");
                    }
                    ros::Time mid = ros::Time::now();
                    std::cout<<(mid-init)<<std::endl;
                }
  
            }  

        }
  
    }
    else // if come accross all the points, then stop timer.
    {
        ROS_INFO("Get across all the points");
        joint_velocity_msg.joint1 = 0;
        joint_velocity_msg.joint2 = 0;
        joint_velocity_msg.joint3 = 0;
        joint_velocity_msg.joint4 = 0;
        joint_velocity_msg.joint5 = 0;
        joint_velocity_msg.joint6 = 0;
        joint_velocity_msg.joint7 = 0;

        pub_joint_velocity_.publish(joint_velocity_msg);

        traj_command_points_.clear();

        traj_command_points_index_ = 0;
        timer_pub_joint_vel_.stop();
    }
}

void FakeJointTrajectoryController::update_state()
{
    //ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);

    ros::Rate update_rate(10);
    previous_pub_ = ros::Time::now();
    while (nh_.ok())
    {
        // check if terminate command is sent from main thread
        {
            boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
            if (terminate_thread_)
            {
                break;
            }
        }

        traj_feedback_msg_.header.frame_id = traj_frame_id_;
        traj_feedback_msg_.header.stamp = ros::Time::now();
        // KinovaAngles current_joint_angles;
        // KinovaAngles current_joint_velocity;
        // AngularPosition current_joint_command;

        // kinova_comm_.getAngularCommand(current_joint_command);
        // kinova_comm_.getJointAngles(current_joint_angles);
        // kinova_comm_.getJointVelocities(current_joint_velocity);


        traj_feedback_msg_.desired.velocities[0] = current_joint_command.joint1;
        traj_feedback_msg_.desired.velocities[1] = current_joint_command.joint2;
        traj_feedback_msg_.desired.velocities[2] = current_joint_command.joint3;
        traj_feedback_msg_.desired.velocities[3] = current_joint_command.joint4;
        if (number_joint_>=6)
        {
            traj_feedback_msg_.desired.velocities[4] = current_joint_command.joint5;
            traj_feedback_msg_.desired.velocities[5] = current_joint_command.joint6;

        }

        traj_feedback_msg_.actual.positions[0] = current_joint_angles.joint1;
        traj_feedback_msg_.actual.positions[1] = current_joint_angles.joint2;
        traj_feedback_msg_.actual.positions[2] = current_joint_angles.joint3;
        traj_feedback_msg_.actual.positions[3] = current_joint_angles.joint4;
        if (number_joint_>=6)
        {
            traj_feedback_msg_.actual.positions[4] = current_joint_angles.joint5;
            traj_feedback_msg_.actual.positions[5] = current_joint_angles.joint6;
        }

        traj_feedback_msg_.actual.velocities[0] = current_joint_velocity.joint1;
        traj_feedback_msg_.actual.velocities[1] = current_joint_velocity.joint2;
        traj_feedback_msg_.actual.velocities[2] = current_joint_velocity.joint3;
        traj_feedback_msg_.actual.velocities[3] = current_joint_velocity.joint4;
        if (number_joint_>=6)
        {
            traj_feedback_msg_.actual.velocities[4] = current_joint_velocity.joint5;
            traj_feedback_msg_.actual.velocities[5] = current_joint_velocity.joint6;
        }

        for (size_t j = 0; j<joint_names_.size(); j++)
        {
            traj_feedback_msg_.error.velocities[j] = traj_feedback_msg_.actual.velocities[j] - traj_feedback_msg_.desired.velocities[j];
        }

        //        ROS_WARN_STREAM("I'm publishing after second: " << (ros::Time::now() - previous_pub_).toSec());
        pub_joint_feedback_.publish(traj_feedback_msg_);
        previous_pub_ = ros::Time::now();
        update_rate.sleep();
    }
    //ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_joint_trajecotry_controller");
    ros::NodeHandle node;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    kinova::FakeJointTrajectoryController fake_joint_trajectory_controller(node);


    ros::spin();
    return 0;
}
