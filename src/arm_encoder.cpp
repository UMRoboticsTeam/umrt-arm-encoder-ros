#include "../include/umrt_arm_encoder_ros/arm_encoder.h"

namespace boost_log = boost::log; 
namespace sources = boost::log::sources; 
namespace expr = boost::log::expressions; 
namespace sinks = boost::log::sinks; 
namespace keywords = boost::log::keywords; 

typedef sinks::asynchronous_sink<sinks::text_ostream_backend> sink_t; 


ArmEncoder::EncoderDiagnosticsBuffer::EncoderDiagnosticsBuffer(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_publisher):m_diagnostics_publisher(diagnostics_publisher){}; 

int ArmEncoder::EncoderDiagnosticsBuffer::overflow(int c){
    if(c != EOF){
         
        if(c == '\n'){
            publisher_callback(); 

        }
        else{
            diagnostics_buffer+= static_cast<char>(c); 
        }
    }
    return c; 
}; 

void ArmEncoder::EncoderDiagnosticsBuffer::publisher_callback(){
    std_msgs::msg::String msg; 
    msg.data = diagnostics_buffer; 
    m_diagnostics_publisher->publish(msg); 
    diagnostics_buffer = ""; 
}; 

ArmEncoder::Ros2ostream::Ros2ostream(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_publisher): std::ostream(&buffer), buffer(diagnostics_publisher){}; 

ArmEncoder::ArmEncoderNode::ArmEncoderNode():Node("arm_encoder_node"){
    angle_publisher = this->create_publisher<sensor_msgs::msg::JointState>("encoder_angle",10); 
    temp_publisher = this->create_publisher<sensor_msgs::msg::Temperature>("encoder_temperature",10); 
    diagnostics_publisher = this->create_publisher<std_msgs::msg::String>("encoder_diagnostics",10); 
    can_interface = Interface(); 
    setup_logging(); 
    //setup handlers
    can_interface.angle_signal.connect([&](uint32_t can_id, double angle, double angular_vel, uint16_t n_rotations){angle_handler(can_id,angle,angular_vel,n_rotations);}); 
    can_interface.temp_signal.connect([&](uint32_t can_id, double temp){temperature_handler(can_id,temp);}); 
    
    can_interface.initialize_channel(); 
    can_interface.begin_read_loop(); 
   
}; 

void ArmEncoder::ArmEncoderNode::temperature_handler(uint32_t can_id, double temp){
    sensor_msgs::msg::Temperature msg; 
    msg.header.frame_id = std::to_string(can_id);
    msg.temperature = temp; 
    temp_publisher->publish(msg); 
}; 
void ArmEncoder::ArmEncoderNode::angle_handler(uint32_t can_id,double angle, double angular_vel, uint16_t n_rotations ){
    sensor_msgs::msg::JointState msg; 
    msg.name[0] = std::to_string(can_id); 
    msg.position[0] = angle; 
    msg.velocity[0] = angular_vel; 
    angle_publisher->publish(msg); 
}; 

void ArmEncoder::ArmEncoderNode::setup_logging(){
    boost::shared_ptr<boost_log::core> core = boost_log::core::get(); 
    auto backend = boost::make_shared<sinks::text_ostream_backend>(); 
    auto ros_stream = boost::make_shared<Ros2ostream>(diagnostics_publisher); 
    backend->add_stream(ros_stream); 
    boost::shared_ptr<sink_t> sink(new sink_t(backend)); 
    core->add_sink(sink); 
}; 
