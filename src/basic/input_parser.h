// input_parser.h
#ifndef INPUT_PARSER_H
#define INPUT_PARSER_H

#include "basic_types.h"

class InputParser {

public:
  InputParser(int &argc, char **argv);
  const std::string &get_cmd_option(const std::string &opt) const;
  bool cmd_option_exists(const std::string &opt) const;

private:
  std::vector<std::string> tokens;
};

#endif
