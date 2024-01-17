#pragma once


#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>
inline int try_parse_int(std::string str)
{
  try {
    return boost::lexical_cast<int>(str);
  } catch (...) {
    return -1;
  }
}
inline bool starts_with(std::string str, std::string prefix) {
        return str.rfind(prefix, 0) == 0;
}

