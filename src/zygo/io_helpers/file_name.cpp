#include "file_name.h"
#include <sstream>
#include <ctime>


namespace zygo {

std::string makeTimestampedFileName( std::string prefix, std::string postfix )
{
  time_t t = time( 0 );   // get time now
  tm now;
  errno_t err = localtime_s( &now, &t );

  std::stringstream ss;
  ss << prefix
    << (now.tm_year + 1900) << '_'
    << (now.tm_mon + 1) << '_'
    << now.tm_mday << '_'
    << now.tm_hour << '_'
    << now.tm_min << '_'
    << now.tm_sec
    << postfix;

  return ss.str();
}

} // namespace zygo
