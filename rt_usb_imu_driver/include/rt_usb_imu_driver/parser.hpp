#ifndef TDW_205_PARSER_H
#define TDW_205_PARSER_H

#include <cstring>
#include <string>
#include <vector>

namespace rt_usb_imu_data_parser {

class Parser {
public:
  Parser();
  void append_data(const char *__s, std::size_t __n);
  bool parse();
  std::vector<double> get_latest_data();

private:
  std::vector<double> latest_data_;

  std::string serial_data_;
};
} // namespace rt_usb_imu_data_parser

#endif // PARSER_H
