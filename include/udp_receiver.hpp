#ifndef UDP_RECEIVER_HPP
#define UDP_RECEIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include <image_transport/image_transport.hpp>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>

#define BUFFER_SIZE 65507 // 최대 UDP 패킷 크기

class UdpReceiver : public rclcpp::Node {
public:
  UdpReceiver();
  ~UdpReceiver();
  bool receiveImage(cv::Mat &img); // 이미지를 수신하는 함수
  void process(); // 이미지를 수신하고 퍼블리시하는 함수
  void init_image_transport();

private:
  int server_socket;
  struct sockaddr_in serverAddress, clientAddress;
  socklen_t client_addr_size;
  unsigned char buffer[BUFFER_SIZE];
  rclcpp::TimerBase::SharedPtr timer;

  image_transport::Publisher image_publisher; // 이미지 퍼블리셔
};

#endif // UDP_RECEIVER_HPP
