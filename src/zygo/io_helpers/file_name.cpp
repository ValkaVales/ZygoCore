#include "file_name.h"
#include <sstream>
#include <ctime>


namespace zygo {

namespace
{
  bool getLocalTime( std::time_t time, std::tm& result ) noexcept
  {
#if defined(_MSC_VER)
    return ::localtime_s( &result, &time ) == 0;
#elif defined(__unix__) || defined(__APPLE__)
    return ::localtime_r( &time, &result ) != nullptr;
#else
  #error Unsupported platform: thread-safe local time conversion is not implemented.
#endif
  }
}

std::string makeTimestampedFileName( std::string prefix, std::string postfix )
{
  std::time_t const current_time = std::time( nullptr );
  if ( current_time == static_cast<std::time_t>( -1 ) )
    return {};

  std::tm now{}; // local time
  if ( !getLocalTime( current_time, now ) )
    return {};

#if 1
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
#else
  char buffer[64];

  if ( 0 == std::strftime(
    buffer,
    sizeof(buffer),
    "%Y_%m_%d_%H_%M_%S",
    &now
  ) )
  {
    return {};
  }

  return buffer;
#endif
}

} // namespace zygo
