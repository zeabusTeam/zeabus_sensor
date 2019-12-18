// FILE			: imu_cf.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, September 19 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE
//  ref01 : https://en.cppreference.com/w/cpp/language/goto

// MACRO SET
#define COLLECT_RAW_DATA
//#define PRINT_GRAVITY_BIAS

// MACRO CONDITION

#include    <vector>

#include    <iostream>

#include    <ros/ros.h>

#include    <tf/LinearMath/Quaternion.h>

#include    <sensor_msgs/Imu.h>

#include    <zeabus/ros/node.hpp>

#include    <zeabus/sensor/imu/connector.hpp>

#include    <zeabus/ros/convert/geometry_vector3.hpp>

#include    <zeabus/ros/convert/geometry_quaternion.hpp>

#include    <zeabus/sensor/imu/LORD_IMU_COMMUNICATION.hpp>


int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "imu" );

    ros::NodeHandle ph("~"); // param node handle
    ros::NodeHandle nh(""); // general node handle

    // Path to reduce bias of acceleration
    const static double unit_acceleration_imu = 9.80665;
    const static double gravity_acceleration = 9.71945; 
    const static tf::Quaternion quaternion_gravity( 0 , 0 , gravity_acceleration , 0 );
    tf::Quaternion quaternion_orientation( 0 , 0 , 0 , 1 );
    tf::Quaternion quaternion_gravity_bias( 0 , 0 , gravity_acceleration , 0 );

    node.spin();
    // Parameter of path of IMU device
    std::string device_path;
    ph.param< std::string >( "device_path" ,
            device_path ,
            "/dev/microstrain/3dm_gx5_45_0000__6251.65901");

    // Parameter of frequency
    int frequency;
    ph.param< int >( "frequency" , frequency , 50 );

    // Parameter of topic output
    std::string topic_output;
    ph.param< std::string >( "topic_output" ,
            topic_output ,
            "sensor/imu" );

    zeabus::sensor::imu::Connector imu( device_path );
    
    ros::Rate loop_rate( frequency );
    std::vector< unsigned char >::iterator start_point;
    std::vector< unsigned char >::iterator last_point;
    sensor_msgs::Imu message;
    ros::Publisher imu_publisher = nh.advertise< sensor_msgs::Imu >( topic_output , 1 );
#ifdef COLLECT_RAW_DATA
    sensor_msgs::Imu message_original;
    ros::Publisher imu_original_publisher = nh.advertise< sensor_msgs::Imu >( 
            topic_output + "_original" , 1 );
#endif // COLLECT_RAW_DATA

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
        imu.set_imu_rate( 500 / frequency );
    }

    // Part of command connect to IMU
    if( ! imu.set_idle() ) goto exit_main;
    if( ! imu.set_imu_message_format( 
                _imu_protocol::DATA::IMU_DATA_SET::SCALED_ACCELEROMETER_VECTOR,
                _imu_protocol::DATA::IMU_DATA_SET::SCALED_GYRO_VECTOR,
                _imu_protocol::DATA::IMU_DATA_SET::CF_QUATERNION ) ) goto exit_main;
    if( ! imu.enable_imu_data_stream() ) goto exit_main;
    if( ! imu.resume() ) goto exit_main;

    while( ros::ok() )
    {
        if( imu.read_stream( &start_point , &last_point ) )
        {

convert: // This you to loop convert data
            if( start_point < last_point )
            {
                switch( *start_point )
                {
                case _imu_protocol::DATA::IMU_DATA_SET::SCALED_ACCELEROMETER_VECTOR :
                    (void)zeabus_ros::convert::geometry_vector3::bytes( start_point , 
                            &message.linear_acceleration ,
                            1 );
                    start_point += 14 ; // skip point self field descriptor byte <1 byte>
                                        // skip point self length data 3 float < 3*4 bytes>
                                        // skip point next field length data <1 byte>
                                        // Now point to next field descriptor byte **if have**
                    goto convert;
                case _imu_protocol::DATA::IMU_DATA_SET::SCALED_GYRO_VECTOR :
                    (void)zeabus_ros::convert::geometry_vector3::bytes( start_point ,
                            &message.angular_velocity ,
                            1 );
                    start_point += 14 ; // skip point self field descriptor byte <1 byte>
                                        // skip point self length data 3 float < 3*4 bytes>
                                        // skip point next field length data <1 byte>
                                        // Now point to next field descriptor byte **if have**
                    goto convert;
                case _imu_protocol::DATA::IMU_DATA_SET::CF_QUATERNION :
                    (void)zeabus_ros::convert::geometry_quaternion::bytes( start_point ,
                            &message.orientation ,
                            1 );
                    start_point += 18 ; // skip point self field descriptor byte <1 byte>
                                        // skip point self length data 3 float < 4*4 bytes>
                                        // skip point next field length data <1 byte>
                                        // Now point to next field descriptor byte **if have**
                    goto convert;
                }
            }
            else
            {
                message.header.stamp = ros::Time::now();
#ifdef COLLECT_RAW_DATA
                message_original = message;
                imu_original_publisher.publish( message_original );
#endif // COLLECT_RAW_DATA
                zeabus_ros::convert::geometry_quaternion::tf( &message.orientation, 
                        &quaternion_orientation );
                // gravity is inertia frame and we will convert to imu frame
                quaternion_gravity_bias = quaternion_orientation.inverse() *
                        quaternion_gravity *
                        quaternion_orientation;
#ifdef PRINT_GRAVITY_BIAS
                std::cout   << quaternion_gravity_bias.x() << " "
                            << quaternion_gravity_bias.y() << " "
                            << quaternion_gravity_bias.z() << "\n";
#endif // PRINT_GRAVITY_BIAS
                message.linear_acceleration.x *= unit_acceleration_imu * -1;
                message.linear_acceleration.y *= unit_acceleration_imu * -1;
                message.linear_acceleration.z *= unit_acceleration_imu * -1;
                message.linear_acceleration.x -= quaternion_gravity_bias.x();
                message.linear_acceleration.y -= quaternion_gravity_bias.y();
                message.linear_acceleration.z -= quaternion_gravity_bias.z();
                imu_publisher.publish( message );
            }

        } // read stream finish
        loop_rate.sleep();
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
