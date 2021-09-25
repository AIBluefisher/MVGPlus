#ifndef SRC_MATH_CONSTRAINED_LINEAR_LEAST_SQUARE_H_
#define SRC_MATH_CONSTRAINED_LINEAR_LEAST_SQUARE_H_

#include <Eigen/Core>
#include <Eigen/Dense>

namespace mvgplus {

// min. ||Ax||, s.t. ||x|| = 1.
void SolveConstrainedLeastSquares(const Eigen::MatrixXd& A,
                                  Eigen::VectorXd* x) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);

  const Eigen::MatrixXd V = svd.matrixV();
  (*x) = V.col(V.cols() - 1); // .hnormalized().homogeneous();
}

}  // namespace mvgplus

#endif  // SRC_MATH_CONSTRAINED_LINEAR_LEAST_SQUARE_H_
