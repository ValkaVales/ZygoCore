#include "vectors_to_string.h"
#include <sstream>


namespace zygo {

std::string toStringVector2Short( Vector2Short const& v )
{
  std::stringstream res;
  res << "(" << v.x << "," << v.y << ")";
  return res.str();
}

std::string toStringVector3Short( Vector3Short const& v )
{
  std::stringstream res;
  res << "(" << v.x << "," << v.y << "," << v.z << ")";
  return res.str();
}

} // namespace zygo
