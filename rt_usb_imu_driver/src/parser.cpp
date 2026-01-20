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

#include "rt_usb_imu_driver/parser.hpp"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

namespace rt_usb_imu_data_parser {
Parser::Parser() { status_.resize(SCALE_NUM_, ScaleStatus::CANNOT_USE); }

void Parser::append_data(const char *s, std::size_t n) {
  serial_data_.append(s, n);
}

std::vector<double> Parser::get_latest_data() { return latest_data_; }

bool Parser::parse() {
  size_t start = 0;
  bool update_flag = false;
  while (true) {
    size_t end = serial_data_.find('\n', start);
    if (end == std::string::npos)
      break;
    std::string line = serial_data_.substr(start, end - start);
    std::cout << "Parse line: " << line << std::endl;
    std::vector<double> values;
    size_t pos = 0;
    while (pos < line.size()) {
      size_t comma = line.find(',', pos);
      std::string token = (comma == std::string::npos)
                              ? line.substr(pos)
                              : line.substr(pos, comma - pos);
      if (!token.empty()) {
        try {
          values.push_back(std::stod(token));
        } catch (...) {
          // ignore parse errors
        }
      }
      if (comma == std::string::npos)
        break;
      pos = comma + 1;
    }
    if (values.size() == 8) {
      latest_data_ = values;
      update_flag = true;
    } else {
      std::cout << "Warning: Expected 8 values, got " << values.size()
                << std::endl;
    }
    start = end + 1;
  }
  // Remove processed data
  serial_data_ = serial_data_.substr(start);
  return update_flag;
}

} // namespace rt_usb_imu_data_parser
