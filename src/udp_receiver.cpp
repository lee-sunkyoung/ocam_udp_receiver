#include "udp_receiver.hpp"
#include <arpa/inet.h>
#include <cv_bridge/cv_bridge.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

UdpReceiver::UdpReceiver() : Node("udp_receiver_node") {
  // 소켓 생성
  if ((server_socket = socket(PF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("socket 생성 실패");
    exit(0);
  }

  // 서버 주소 설정
  memset(&serverAddress, 0, sizeof(serverAddress));
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_addr.s_addr = inet_addr("20.20.20.48"); // 서버 IP 주소 설정
  serverAddress.sin_port = 8888;

  // 소켓에 주소 바인딩
  if (bind(server_socket, (struct sockaddr *)&serverAddress,
           sizeof(serverAddress)) == -1) {
    perror("bind 실패");
    close(server_socket);
    exit(0);
  }

  // 클라이언트 주소 크기 초기화
  client_addr_size = sizeof(clientAddress);

  RCLCPP_INFO(this->get_logger(), "UDP 서버가 포트 %d에서 대기 중입니다...",
              8888);

  timer = this->create_wall_timer(std::chrono::milliseconds(100),
                                  std::bind(&UdpReceiver::process, this));
}

void UdpReceiver::init_image_transport() {
  image_transport::ImageTransport it(shared_from_this());
  image_publisher = it.advertise("camera/image_raw", 10);
}

UdpReceiver::~UdpReceiver() { close(server_socket); }

bool UdpReceiver::receiveImage(cv::Mat &img) {
  // 데이터 수신
  ssize_t receivedBytes =
      recvfrom(server_socket, buffer, BUFFER_SIZE, 0,
               (struct sockaddr *)&clientAddress, &client_addr_size);

  if (receivedBytes == -1) {
    perror("recvfrom 에러");
    return false;
  }

  // 수신된 데이터를 JPEG로 디코딩
  std::vector<uchar> encodedImage(buffer, buffer + receivedBytes);
  img = cv::imdecode(encodedImage, cv::IMREAD_COLOR);

  // 디코딩 성공 여부 확인
  if (img.empty()) {
    RCLCPP_WARN(this->get_logger(), "이미지 디코딩 실패");
    return false;
  }

  return true;
}

void UdpReceiver::process() {
  if (!image_publisher) {
    RCLCPP_ERROR(this->get_logger(), "Publisher is not initialized correctly.");
    return;
  }
  rclcpp::Rate rate(100);

  cv::Mat img;
  while (rclcpp::ok()) {
    if (receiveImage(img)) {

      cv_bridge::CvImage cv_image;
      rclcpp::Time t = this->now();
      cv_image.image = img;
      cv_image.encoding = sensor_msgs::image_encodings::BGR8;
      cv_image.header.frame_id = "camera_frame";
      cv_image.header.stamp = t;
      image_publisher.publish(cv_image.toImageMsg());

      cv::imshow("Received Image", img);
      rate.sleep();

      // if (cv::waitKey(1) == 'q') {
      //   break;
      // }
    }
  }

  cv::destroyWindow("Received Image");
}
