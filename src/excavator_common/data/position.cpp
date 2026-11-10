#include "position.h"

namespace excavator {
namespace planning {
namespace data {
Point::Point(const double _x, const double _y, const double _z)
    : x(_x), y(_y), z(_z) {}

std::string Point::ToString(const int precision) const {
  std::stringstream ss;
  if (precision == 0) {
    ss << " (x:" << x << ", y:" << y << ", z:" << z << ")";
  } else {
    ss << std::setprecision(precision) << "(x:" << x << ", y:" << y
       << ", z:" << z << ")";
  }
  return ss.str();
}

std::string Point::XYToString(const int precision) const {
  std::stringstream ss;
  if (precision == 0) {
    ss << " (x:" << x << ", y:" << y << ")";
  } else {
    ss << std::setprecision(precision) << "(x:" << x << ", y:" << y << ")";
  }
  return ss.str();
}

}  // namespace data
}  // namespace planning
}  // namespace excavator