// https://stackoverflow.com/a/868894/996852

// input_parser.cpp
#include "input_parser.h"
#include <algorithm>
#include <vector>

InputParser::InputParser(int &argc, char **argv) {
  for (int i = 1; i < argc; ++i)
    this->tokens.push_back(std::string(argv[i]));
}

const std::string &InputParser::get_cmd_option(const std::string &opt) const {
  std::vector<std::string>::const_iterator itr;
  itr = std::find(this->tokens.begin(), this->tokens.end(), opt);
  if (itr != this->tokens.end() && ++itr != this->tokens.end()) {
    return *itr;
  }
  static const std::string empty_string("");
  return empty_string;
}

bool InputParser::cmd_option_exists(const std::string &opt) const {
  bool ok = std::find(this->tokens.begin(), this->tokens.end(), opt) !=
            this->tokens.end();
  return ok;
}

std::vector<std::string> tokens;