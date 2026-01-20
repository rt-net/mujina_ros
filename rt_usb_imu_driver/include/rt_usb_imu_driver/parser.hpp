#ifndef TDW_205_PARSER_H
#define TDW_205_PARSER_H

#include <cstring>
#include <string>
#include <vector>

namespace rt_usb_imu_data_parser {
enum class ScaleStatus {
  EMPTY = 0,
  MEASURED,
  COMBINED,
  WRONG_PICKUP_ERROR,
  REPLACE_REQUEST,
  CANNOT_USE = 9,
};

class Parser {
public:
  Parser();
  void append_data(const char *__s, std::size_t __n);
  bool parse();
  std::vector<double> get_latest_data();

private:
  const int SCALE_NUM_ = 12;
  const int MSG_DATA_TEXT_LENGTH = 15;
  const char MSG_DATA_HEADER[10] = {
      0x01, 0x01, '0', '1',
      ' ',  ' ',  0x02}; // char array should be bigger than header.
  const int MSG_DATA_HEADER_LENGTH = strlen(MSG_DATA_HEADER);
  const int MSG_DATA_FOOTER_LENGTH = 2;
  const int MSG_BCC_BYTE = MSG_DATA_HEADER_LENGTH + MSG_DATA_TEXT_LENGTH + 1;

  std::vector<double> latest_data_;

  std::vector<ScaleStatus> status_;
  std::string serial_data_;
};
} // namespace rt_usb_imu_data_parser

#endif // PARSER_H
