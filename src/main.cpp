#include "../include/umrt_arm_encoder_ros/arm_encoder.h"



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmEncoder::ArmEncoderNode>());
  rclcpp::shutdown();
  
  return 0;
}