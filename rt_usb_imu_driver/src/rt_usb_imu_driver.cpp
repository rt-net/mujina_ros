/*
* The MIT License (MIT)
*
* Copyright (c) 2026 RT Corporation
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of
* this software and associated documentation files (the "Software"), to deal in
* the Software without restriction, including without limitation the rights to
* use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
* the Software, and to permit persons to whom the Software is furnished to do so,
* subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
* FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
* COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
* IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include "rclcpp/rclcpp.hpp"
#include "rt_usb_imu_driver/parser.hpp"
#include "sensor_msgs/msg/imu.hpp"
//
#include <errno.h> // Error integer and strerror() function
#include <fcntl.h> // Contains file controls like O_RDWR
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include <chrono>
#include <functional>
#include <iomanip> // std::setw(int), std::setfill(char)
#include <iostream>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class RtUsbImuDriverNode : public rclcpp::Node {
public:
  RtUsbImuDriverNode() : Node("rt_usb_imu_driver") {
    this->declare_parameter("port_name", "/dev/ttyUSB0");
    std::string port_name = this->get_parameter("port_name").as_string();
    // open port
    bool res = openPort(port_name);
    if (!res) {
      // open失敗時処理
    }
    clearPort();
    // Normally you wouldn't do this memset() call, but since we will just
    // receive ASCII data for this example, we'll set everything to 0 so we can
    // call printf() easily.
    memset(&read_buf_, '\0', sizeof(read_buf_));

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    data_parse_timer_ = this->create_wall_timer(
        1ms, std::bind(&RtUsbImuDriverNode::data_parse_callback, this));
  }
  ~RtUsbImuDriverNode() { close(port_fd_); }

private:
  void data_parse_callback() {
    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    int num_bytes = readPort(read_buf_);

    // n is the number of bytes read. n may be 0 if no bytes were received, and
    // can also be -1 to signal an error.
    if (num_bytes < 0) {
      RCLCPP_WARN(this->get_logger(), "Error reading: %s", strerror(errno));
      return;
    } else if (num_bytes == 0) {
      // Do nothing.
    } else {
      p_.append_data(read_buf_, num_bytes);
      bool is_update = p_.parse();
      if (is_update) {
        auto latest_data = p_.get_latest_data();
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "imu_link";

        msg.orientation.x = latest_data[1];
        msg.orientation.y = latest_data[2];
        msg.orientation.z = latest_data[3];
        msg.orientation.w = latest_data[4];
        msg.angular_velocity.x = latest_data[5];
        msg.angular_velocity.y = latest_data[6];
        msg.angular_velocity.z = latest_data[7];

        publisher_->publish(msg);
      }
    }
  }

  void clearPort(void) {
    size_t buff_num;
    ssize_t rx_res;
    uint32_t failsafe;
    uint8_t dummy[MAX_PACKET_LEN];

    if (port_fd_) {
      for (failsafe = 0; failsafe < 0x7FFFFFFF; ++failsafe) {
        buff_num = 0;
        ioctl(port_fd_, TIOCINQ, &buff_num);
        if (buff_num > 0) {
          if (buff_num > MAX_PACKET_LEN) {
            // 受信上限の制限
            buff_num = MAX_PACKET_LEN;
          }
          rx_res = read(port_fd_, dummy, buff_num);
          if (!(rx_res > 0)) {
            // エラー処理
            break;
          }
        } else {
          // バッファクリアできた
          break;
        }
      }
    }
    return;
  }

  bool openPort(std::string &port_name) {
    port_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (port_fd_ < 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to open port %s !",
                  port_name.c_str());
      return false;
    } else {
      struct termios term;
      term.c_cflag &= ~PARENB; // Clear parity bit, disabling parity
      term.c_cflag &=
          ~CSTOPB; // Clear stop field, only one stop bit used in communication
      term.c_cflag &= ~CSIZE;   // Clear all bits that set the data size
      term.c_cflag |= CS8;      // 8 bits per byte
      term.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
      term.c_cflag |=
          CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

      term.c_iflag = IGNPAR;

      term.c_oflag = 0;
      term.c_lflag = 0;
      term.c_cc[VTIME] = 0; //
      term.c_cc[VMIN] = 0;

      cfsetispeed(&term, B38400);
      cfsetospeed(&term, B38400);

      if (tcsetattr(port_fd_, TCSANOW, &term) != 0) {
        RCLCPP_WARN(this->get_logger(), "Error %i from tcsetattr: %s\n", errno,
                    strerror(errno));
        return false;
      }
      tcflush(port_fd_, TCIFLUSH);
      tcsetattr(port_fd_, TCSANOW, &term);
      return true;
    }
  }

  /**
   * @brief 通信ポートread処理
   * @param data：読み込みデータ格納先アドレス
   * @return 読み込みデータ数[byte]
   */
  int32_t readPort(char *data) {
    size_t buff_num;
    int32_t rx_res = 0;

    if (port_fd_) {
      buff_num = 0;
      ioctl(port_fd_, TIOCINQ, &buff_num);
      if (buff_num > 0) {
        if (buff_num > MAX_PACKET_LEN) {
          // 受信上限の制限
          buff_num = MAX_PACKET_LEN;
        }
        rx_res = (int32_t)read(port_fd_, data, buff_num);
      }
    }
    return rx_res;
  }

  rclcpp::TimerBase::SharedPtr data_parse_timer_;
  rclcpp::TimerBase::SharedPtr status_pub_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

  rt_usb_imu_data_parser::Parser p_;

  static const int MAX_PACKET_LEN = 256;
  int port_fd_;
  // Allocate memory for read buffer, set size according to your needs
  char read_buf_[MAX_PACKET_LEN];
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RtUsbImuDriverNode>());
  rclcpp::shutdown();
  return 0;
}