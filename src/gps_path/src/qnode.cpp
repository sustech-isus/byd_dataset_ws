/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/



/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() 
{
    if(ros::isStarted()) 
    {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

bool QNode::init(TableEditor *ptr_table) 
{
    ros::init(init_argc,init_argv,"gps_path");
    if ( ! ros::master::check() ) 
    {
	return false;
    }

    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle node;
    // Add your ros communications here.
    m_gpsfix_sub = node.subscribe("fix", 10, &TableEditor::callback,ptr_table);
    start();
    return true;
}

void QNode::run() 
{
    ros::spin();
}



