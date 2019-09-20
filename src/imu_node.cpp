// FILE			: imu_node.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, September 19 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  ref01 : https://en.cppreference.com/w/cpp/language/goto

// MACRO SET

// MACRO CONDITION

#include    <vector>

#include    <iostream>

#include    <ros/ros.h>

#include    <sensor_msgs/Imu.h>

#include    <zeabus/ros/node.hpp>

#include    <zeabus/sensor/imu/connector.hpp>

#include    <zeabus/sensor/imu/LORD_IMU_COMMUNICATION.hpp>


int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "imu" );

    ros::NodeHandle ph("~"); // param node

    // Parameter of path of IMU device
    std::string device_path;
    param_handle.param< std::string >( "device_path" ,
            device_path ,
            "/dev/microstrain/3dm_gx5_45_0000__6251.65903");

    // Parameter of frequency
    std::string frequency;
    param_handle.param< int >( "frequency" , frequency , 50 );

    zeabus::sensor::imu::Connector imu( device_path );

    if( ! imu.open_port() )
    {
        ROS_FATAL_NAMED( "SENSOR_IMU" , "Failure to open port %s" , device_path.c_str() );
        goto exit_main;
    }

    // Part setup option or characteristic of port
    if( ros::ok() )
    {
        (void)imu.set_option_port( boost::asio::serial_port_base::flow_control( 
                boost::asio::serial_port_base::flow_control::none ) );
        (void)imu.set_option_port( boost::asio::serial_port_base::parity( 
                boost::asio::serial_port_base::parity::none ) );
        (void)imu.set_option_port( boost::asio::serial_port_base::stop_bits( 
                boost::asio::serial_port_base::stop_bits::one ) );
        (void)imu.set_option_port( 
                boost::asio::serial_port_base::character_size( (unsigned char) 8 ) );
        imu.set_imu_rate( frequency );
    }

    // Part of command connect to IMU
    if( ! imu.set_idle() ) goto exit_main;
    if( ! imu.set_imu_message_format( 
                _imu_protocol::DATA::IMU_DATA_SET::SCALED_ACCELEROMETER_VECTOR,
                _imu_protocol::DATA::IMU_DATA_SET::SCALED_GYRO_VECTOR,
                _imu_protocol::DATA::IMU_DATA_SET::CF_QUATERNION ) ) goto exit_main;
    if( ! imu.enable_imu_data_stream() ) goto exit_main;
    if( ! imu.resume() ) goto exit_main;


    ros::Rate loop_rate( frequency );
    while( ros::ok() )
    {
        
    }



exit_main:    
    ros::shutdown();
    if( imu.is_open() )
    {
        imu.set_idle();
        imu.close_port();
    }
    node.join();
}
