// FILE			: pressure_buffer.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, October 28 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define SIZE_DATA 5
#define LIMIT_VARIANCE 1.0

// MACRO CONDITION

#include    <iostream>

#include    <ros/ros.h>

#include    <zeabus_utility/ServiceDepth.h>

#include    <zeabus_utility/HeaderFloat64.h>

#include    <zeabus/ros/node.hpp>

float variance_rule( float data , bool setup = false);
float calculate_variance( float* list_data , float mean );

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

full_buffer:
    while( ros::ok() )
    {
        rate.sleep();
        if( client_pressure.call( service_pressure ) )
        {
            variance_rule( -1.0 * service_pressure.response.depth , true );
        }
        else
        {
            ROS_ERROR( "Failure client to call topic for pressure sensor data : Full Buffer mode");
        }
        
    }

active_main:
    while( ros::ok() )
    {
        rate.sleep();
        if( client_pressure.call( service_pressure ) )
        {
            message_output.header = service_pressure.response.header;
            message_output.data = variance_rule( -1.0 * service_pressure.response.depth );
            publisher_pressure.publish( message_output );
        }
        else
        {
            ROS_ERROR( "Failure client to call topic for pressure sensor data");
        }
        
    }

}

float variance_rule( float data , bool setup )
{
    float answer = data;
    static float list_data[ SIZE_DATA ];
    static unsigned int run = 0;
    static float summation = 0;
    if( setup )
    {
        for( unsigned int count = 0 ; count < SIZE_DATA ; count++ )
        {
            list_data[ count ] = data;
            summation += list_data[ count ];
        }
    }
    else if( calculate_variance( list_data , summation / SIZE_DATA ) > LIMIT_VARIANCE )
    {
            std::cout   << "Disagree data " << data << " because make over limit variance\n";
            answer = summation / SIZE_DATA;
    }
    else
    {
        summation -= list_data[ run ];
        list_data[ run ] = data;
        summation += list_data[ run ];
        answer = summation / SIZE_DATA;
    }
    return answer;
} // function variance_rule

float calculate_variance( float* list_data , float mean )
{
    float answer = 0;
    for( unsigned int count = 0 ; count < SIZE_DATA ; count++ )
    {
        float temp = list_data[ count ] - mean;
        answer += temp * temp;
    }
    return answer / SIZE_DATA;
}
