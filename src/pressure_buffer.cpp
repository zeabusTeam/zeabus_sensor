// FILE			: pressure_buffer.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 28 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define SIZE_DATA 5
#define LIMIT_VARIANCE 10.0
//#define LOG_ORIGIN_PRESSURE 

// MACRO CONDITION

#include    <iostream>

#include    <ros/ros.h>

#include    <zeabus_utility/ServiceDepth.h>

#include    <zeabus_utility/HeaderFloat64.h>

#include    <zeabus/filter/outliner.hpp>

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

    std::string topic_original;
    ph.param< std::string >( "topic_original",
            topic_original,
            "sensor/original_pressure" );

    std::string topic_output;
    ph.param< std::string >( "topic_output",
            topic_output,
            "sensor/pressure");

    zeabus::filter::Outliner< double , 11 > filter_pressure( 0 );
    filter_pressure.set_cut_size( 2 );

    ros::ServiceClient client_pressure = nh.serviceClient< zeabus_utility::ServiceDepth >( 
            topic_service );
    zeabus_utility::ServiceDepth service_pressure;

    zeabus_utility::HeaderFloat64 message_output;
    ros::Publisher publisher_pressure = nh.advertise< zeabus_utility::HeaderFloat64 >( 
            topic_output , 1 );

#ifdef LOG_ORIGIN_PRESSURE
    zeabus_utility::HeaderFloat64 message_original;
    ros::Publisher publisher_original = nh.advertise< zeabus_utility::HeaderFloat64 >( 
            topic_original , 1 );
#endif // LOG_ORIGIN_PRESSURE
    
    ros::Rate rate( frequency );

fill_data: // This will fill data to full array of filter
    for( unsigned int count = 0 ; count < 7 && ros::ok() ; )
    {
        rate.sleep();
        if( client_pressure.call( service_pressure ) )
        {
            if( message_output.header.stamp != service_pressure.response.header.stamp )
            {
                // publish original data for collect data
#ifdef LOG_ORIGIN_PRESSURE
                message_original.header = service_pressure.response.header;
                message_original.data = service_pressure.response.depth ;
                publisher_original.publish( message_original );
#endif // LOG_ORIGIN_PRESSURE
                filter_pressure.push_data( service_pressure.response.depth );
                count++;
            } // condition defferent time stamp
            else
            {
                ROS_INFO( "Failure get same data of pressure sensor" );
            } // condition same time stamp
        } // condition success call service
        else
        {
            ROS_ERROR( "Failure client to call topic for pressure sensor data");
        } // condition failure call service
    } // loop while of fill data

active_main:
    while( ros::ok() )
    {
        rate.sleep();
        if( client_pressure.call( service_pressure ) )
        {
            if( message_output.header.stamp != service_pressure.response.header.stamp )
            {
                // publish original data for collect data
#ifdef LOG_ORIGIN_PRESSURE
                message_original.header = service_pressure.response.header;
                message_original.data = service_pressure.response.depth ;
                publisher_original.publish( message_original );
#endif // LOG_ORIGIN_PRESSURE
                filter_pressure.push_data( service_pressure.response.depth );
                // publish output data for collect data
                message_output.header = service_pressure.response.header;
//                message_output.data = filter_pressure.get_result();
                message_output.data = filter_pressure.median();
                publisher_pressure.publish( message_output );
            } // condition defferent time stamp
            else
            {
                ROS_INFO( "Failure get same data of pressure sensor" );
            } // condition same time stamp
        } // condition success call service
        else
        {
            ROS_ERROR( "Failure client to call topic for pressure sensor data");
        } // condition failure call service
    } // loop while of active main

}
