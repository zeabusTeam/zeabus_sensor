// FILE			: pressure_generate.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, December 18 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <mutex>

#include    <iostream>

#include    <ros/ros.h>

#include    <zeabus_utility/ServiceDepth.h>

#include    <zeabus_utility/HeaderFloat64.h>

#include    <zeabus/ros/node.hpp>

#include    <zeabus/ros/subscriber/base_class.hpp>

class ServerDepth
{
    public:
        ServerDepth( ros::NodeHandle* ptr_node_handle )
        {
            this->ptr_node_handle = ptr_node_handle;
        }

        void setup_all_variable( zeabus_utility::HeaderFloat64* ptr_data, 
                std::mutex* ptr_lock )
        {
            this->ptr_lock = ptr_lock;
            this->ptr_data = ptr_data;
        }

        void setup_all_service()
        {
            this->service_depth = this->ptr_node_handle->advertiseService( "/sensor/pressure",
                    &ServerDepth::callback_service_depth , this );
        }

        ros::ServiceServer service_depth;
        bool callback_service_depth( zeabus_utility::ServiceDepth::Request& request,
                zeabus_utility::ServiceDepth::Response& response )
        {
            this->ptr_lock->lock();
            response.header = this->ptr_data->header;
            response.depth = this->ptr_data->data;
            this->ptr_lock->unlock();
            return true;
        }

        protected:
            std::mutex* ptr_lock;
            zeabus_utility::HeaderFloat64* ptr_data;
            ros::NodeHandle* ptr_node_handle;
};

int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "sensor_pressure_generate" );

    ros::NodeHandle nh("");

    std::mutex lock_sensor_pressure_origin;
    zeabus_utility::HeaderFloat64 message_pressure_origin;
    zeabus_ros::subscriber::BaseClass< zeabus_utility::HeaderFloat64 > listener_pressure_origin( &nh,
            &message_pressure_origin );
    listener_pressure_origin.setup_mutex_data( &lock_sensor_pressure_origin );
    listener_pressure_origin.setup_subscriber_timestamp( "/sensor/pressure_original" , 1 );

    ServerDepth service_depth( &nh );
    service_depth.setup_all_variable( &message_pressure_origin , &lock_sensor_pressure_origin );
    service_depth.setup_all_service();

    ros::spin();
}
