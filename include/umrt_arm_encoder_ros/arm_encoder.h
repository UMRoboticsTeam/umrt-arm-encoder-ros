#ifndef ARM_ENCODER
#define ARM_ENCODER
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <fstream>
#include <streambuf> 
#include <boost/smart_ptr/shared_ptr.hpp> 
#include <boost/core/null_deleter.hpp> 
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/async_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include "../arm-encoder-driver/cpp/src/encoder_interface.h"
#include <boost/signals2/slot.hpp>


namespace ArmEncoder{
    class EncoderDiagnosticsBuffer: public std::streambuf{
        public:
            EncoderDiagnosticsBuffer(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_publisher); 
        protected:
            int overflow (int c) override; 
        private:  
            void publisher_callback(); 
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_diagnostics_publisher; 
            std::string diagnostics_buffer; 

    };  

    class Ros2ostream: public std::ostream{
        public:
            explicit Ros2ostream(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_publisher);

        private:
            EncoderDiagnosticsBuffer buffer; 
    }; 


    class ArmEncoderNode: public rclcpp::Node{
        private:
            Interface can_interface; 
            void temperature_handler(uint32_t can_id, double temp); 
            void angle_handler(uint32_t can_id,double angle, double angular_vel, uint16_t n_rotations); 
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr angle_publisher; 
            rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_publisher; 
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_publisher; 

        public:
            ArmEncoderNode(); 
            void setup_logging(); 

    }; 
}






#endif 