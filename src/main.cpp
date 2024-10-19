#include "rclcpp/rclcpp.hpp"
#include "udp_receiver.hpp"
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
    std::cout<<"init"<<std::endl;
      auto node = std::make_shared<UdpReceiver>();
  node->init_image_transport();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
