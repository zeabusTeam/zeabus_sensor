// FILE			: imu_ahrs.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, September 26 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL
//  _RAW_DATA_  :   use this macro if you want to see raw data

// README

// REFERENCE

// MACRO SET
//#define _RAW_DATA_ // (NO AVAILABLE NOW)

// MACRO CONDITION

#include    <vector>

#include    <iostream>

#include    <ros/ros.h>

#include    <sensor_msgs/Imu.h>

#include    <zeabus/ros/node.hpp>

#include    <zeabus_utility/Ahrs.h>

#include    <zeabus/sensor/imu/connector.hpp>

#include    <zeabus/ros/convert/geometry_vector3.hpp>

#include    <zeabus/sensor/imu/LORD_IMU_COMMUNICATION.hpp>


int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "imu" );

    ros::NodeHandle ph("~"); // param node handle
    ros::NodeHandle nh(""); // general node handle
    node.spin();
    // Parameter of path of IMU device
    std::string device_path;
    ph.param< std::string >( "device_path" ,
            device_path ,
            "/dev/microstrain/3dm_gx5_45_0000__6251.65903");

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
    zeabus_utility::Ahrs message;
    ros::Publisher imu_publisher = nh.advertise< zeabus_utility::Ahrs >( topic_output , 1 );

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
                _imu_protocol::DATA::IMU_DATA_SET::SCALED_MAGNETOMETER_VECTOR ) ) goto exit_main;
    if( ! imu.enable_imu_data_stream() ) goto exit_main;
    if( ! imu.resume() ) goto exit_main;

    while( ros::ok() )
    {
        if( imu.read_stream( &start_point , &last_point ) )
        {

convert: // This you to loop convert data
#ifdef _RAW_DATA_
            imu.reader.print_data_hex( "RAW message from IMU : " );
#endif // _RAW_DATA_
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
                case _imu_protocol::DATA::IMU_DATA_SET::SCALED_MAGNETOMETER_VECTOR :
                    (void)zeabus_ros::convert::geometry_vector3::bytes( start_point ,
                            &message.magnetic_field ,
                            1 );
                    start_point += 14 ; // skip point self field descriptor byte <1 byte>
                                        // skip point self length data 3 float < 4*4 bytes>
                                        // skip point next field length data <1 byte>
                                        // Now point to next field descriptor byte **if have**
                    goto convert;
                }
            }
            else
            {
                message.header.stamp = ros::Time::now();
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

    return 0;
}
