#ifndef SRC_BASE_LINE2D_H_
#define SRC_BASE_LINE2D_H_

namespace mvgplus {

// Ax + By + C = 0.
struct Line2D {
  double A;
  double B;
  double C;

  Line2D() {}
  Line2D(const double a, const double b, const double c)
    : A(a), B(b), C(c) {}
};

}  // namespace mvgplus

#endif  // SRC_BASE_LINE2D_H_
