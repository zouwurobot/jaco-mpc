#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <kinova_driver/kinova_fake_joint_trajectory_controller.h>
#include <angles/angles.h>
#include <ros/console.h>

// example: construct a quadratic program from data
// the QP below is the first quadratic program example in the user manual
#include <iostream>
#include <cassert>
#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

// choose exact integral type

#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
// program and solution types
typedef CGAL::Quadratic_program<double> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;



int main(int argc, char **argv)
{
 

  ros::init(argc, argv, "talker");
  
  ros::NodeHandle n;
  ros::Time init = ros::Time::now();
  
    // by default, we have a nonnegative QP with Ax <= b
  Program qp (CGAL::SMALLER, true, 0, false, 0); 
  
  // now set the non-default entries: 
  const double X = 0.5; 
  const double Y = 1.5;
  qp.set_a(X, 0,  1.1); qp.set_a(Y, 0, 1.2); qp.set_b(0, 9.3);  //  x + y  <= 7
  qp.set_a(X, 1, -1); qp.set_a(Y, 1, 2.1); qp.set_b(1, 7.2);  // -x + 2y <= 4
  qp.set_u(Y, true, 4);                                   //       y <= 4
  qp.set_d(X, X, 4.2); qp.set_d (Y, Y, 8); // !!specify 2D!!    x^2 + 4 y^2
  qp.set_c(Y, -50.5);                                       // -32y
  qp.set_c0(32);                                          // +64
  // solve the program, using ET as the exact type
  Solution s = CGAL::solve_quadratic_program(qp, ET());
  // assert (s.solves_quadratic_program(qp));
  float a;
  for(Solution::Variable_value_iterator it = s.variable_values_begin(); it != s.variable_values_end(); ++it){
    // a = float(*it);
    std::cout<<*it<<std::endl;
  }
      

  // ros::Time end = ros::Time::now();
  // std::cout<<(end-init)<<std::endl;
  // output solution
  std::cout << s; 


  
  return 0;
 }