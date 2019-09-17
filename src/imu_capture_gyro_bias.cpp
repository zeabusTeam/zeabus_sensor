// FILE			: imu_capture_gyro_bias.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, September 17 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <iostream>

#include    <zeabus/sensor/imu/connector.hpp>

int main( int argv , char** argc )
{
    ros::init( argv , argc , "capture_gyro_bias" );

    // get path of imu device for start capture gyro bias
    std::string full_path;
    ros::param::param< std::string >("imu_path" , 
        full_path , 
        "/dev/microstrain/3dm_gx5_45_0000__6251.65903" );

    zeabus::sensor::imu::Connector imu( full_path );

    if( imu.open_port() )
    {
        (void)imu.set_option_port( boost::asio::serial_port_base::flow_control( 
                boost::asio::serial_port_base::flow_control::none ) );
        (void)imu.set_option_port( boost::asio::serial_port_base::parity( 
                boost::asio::serial_port_base::parity::none ) );
        (void)imu.set_option_port( boost::asio::serial_port_base::stop_bits( 
                boost::asio::serial_port_base::stop_bits::one ) );
        (void)imu.set_option_port( 
                boost::asio::serial_port_base::character_size( (unsigned char) 8 ) );
        (void)imu.capture_gyro_bias();
        (void)imu.set_idle();
        imu.close_port();
    }
    else
    {
        std::cout   << "Failue to open port\n";
    }

    return 0;
}
