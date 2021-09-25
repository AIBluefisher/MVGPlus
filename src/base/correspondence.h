#ifndef SRC_BASE_CORRESPONDENCE_H_
#define SRC_BASE_CORRESPONDENCE_H_

#include <vector>

#include "base/point2d.h"

namespace mvgplus {

struct Correspondence2D {
  Point2D point1;
  Point2D point2;

  Correspondence2D(const Point2D& p1, const Point2D p2)
    : point1(p1), point2(p2) {}
};
typedef std::vector<Correspondence2D> Correspondences2D;

}  // namespace mvgplus

#endif  // SRC_BASE_CORRESPONDENCE_H_
