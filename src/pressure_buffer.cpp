// FILE			: pressure_buffer.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 28 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <iostream>

#include    <ros/ros.h>

#include    <zeabus_utility/ServiceDepth.h>

#include    <zeabus_utility/HeaderFloat64.h>

#include    <zeabus/ros/node.hpp>

int main( int argv , char** argc )
{

    zeabus_ros::Node node( argv , argc , "sensor_pressure" );

    ros::NodeHandle ph("~");
    ros::NodeHandle nh("");

    int frequency;
    ph.param< int >( "frequency" , frequency , 50 );

    std::string topic_service;
    ph.param< std::string >( "topic_input" ,
            topic_service,
            "sensor/pressure");

    std::string topic_output;
    ph.param< std::string >( "topic_output",
            topic_output,
            "sensor/pressure");

    ros::ServiceClient client_pressure = nh.serviceClient< zeabus_utility::ServiceDepth >( 
            topic_service );
    zeabus_utility::ServiceDepth service_pressure;

    zeabus_utility::HeaderFloat64 message_output;
    ros::Publisher publisher_pressure = nh.advertise< zeabus_utility::HeaderFloat64 >( 
            topic_output , 1 );

    ros::Rate rate( frequency );

    while( ros::ok() )
    {
        rate.sleep();
        if( client_pressure.call( service_pressure ) )
        {
            message_output.header = service_pressure.response.header;
            message_output.data = -1.0 * service_pressure.response.depth;
            publisher_pressure.publish( message_output );
        }
        else
        {
            ROS_ERROR( "Failure client to call topic for pressure sensor data");
        }
        
    }

}
