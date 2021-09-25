#include "geometry/homography2d_estimator.h"

#include "math/constrained_linear_least_square.h"

namespace mvgplus {

namespace {
double SingleImageError(const Correspondence2D& data,
                        const Homography2D& model) {
  const Eigen::Vector3d point1 = data.point1.ToHomogeneous();
  const Eigen::Vector3d point2 = data.point2.ToHomogeneous();

  const Eigen::Vector3d transformed_point2 = model * point1;
  return (point2.head<2>() - transformed_point2.hnormalized()).squaredNorm();
}

double SymmetricTransferError(const Correspondence2D& data,
                              const Homography2D& model) {
  Correspondence2D symmetry_data(data.point2, data.point1);
  const double error = SingleImageError(data, model) +
                       SingleImageError(symmetry_data, model.inverse());
}

void CenterAndNormalizeImagePoints(const std::vector<Eigen::Vector2d>& points,
                                   std::vector<Eigen::Vector2d>* normed_points,
                                   Eigen::Matrix3d* matrix) {
  // Calculate centroid
  Eigen::Vector2d centroid(0, 0);
  for (const auto point : points) {
    centroid += point;
  }
  centroid /= points.size();

  // Root mean square error to centroid of all points
  double rms_mean_dist = 0;
  for (const auto point : points) {
    rms_mean_dist += (point - centroid).squaredNorm();
  }
  rms_mean_dist = std::sqrt(rms_mean_dist / points.size());

  // Compose normalization matrix
  const double norm_factor = std::sqrt(2.0) / rms_mean_dist;
  *matrix << norm_factor, 0, -norm_factor * centroid(0), 0, norm_factor,
      -norm_factor * centroid(1), 0, 0, 1;

  // Apply normalization matrix
  normed_points->resize(points.size());

  const double M_00 = (*matrix)(0, 0);
  const double M_01 = (*matrix)(0, 1);
  const double M_02 = (*matrix)(0, 2);
  const double M_10 = (*matrix)(1, 0);
  const double M_11 = (*matrix)(1, 1);
  const double M_12 = (*matrix)(1, 2);
  const double M_20 = (*matrix)(2, 0);
  const double M_21 = (*matrix)(2, 1);
  const double M_22 = (*matrix)(2, 2);

  for (size_t i = 0; i < points.size(); ++i) {
    const double p_0 = points[i](0);
    const double p_1 = points[i](1);

    const double np_0 = M_00 * p_0 + M_01 * p_1 + M_02;
    const double np_1 = M_10 * p_0 + M_11 * p_1 + M_12;
    const double np_2 = M_20 * p_0 + M_21 * p_1 + M_22;

    const double inv_np_2 = 1.0 / np_2;
    (*normed_points)[i](0) = np_0 * inv_np_2;
    (*normed_points)[i](1) = np_1 * inv_np_2;
  }
}
}  // namespace

Homography2DEstimator::Homography2DEstimator() {}

Homography2DEstimator::~Homography2DEstimator() {}

bool Homography2DEstimator::ComputeModel(
    const std::vector<Correspondence2D>& data,
    std::vector<Homography2D>* models) {
  if (data.size() < SampleSize()) {
    return false;
  }

  std::vector<Eigen::Vector2d> points1, points2;
  std::vector<Eigen::Vector2d> normed_points1, normed_points2;
  Eigen::Matrix3d points1_norm_matrix, points2_norm_matrix;

  for (size_t i = 0; i < SampleSize(); i++) {
    points1.emplace_back(data[i].point1.XY());
    points2.emplace_back(data[i].point2.XY());
  }
  CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
  CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(8, 9);
  Eigen::VectorXd h = Eigen::VectorXd::Zero(9);

  for (size_t i = 0, j = SampleSize(); i < SampleSize(); i++, j++) {
    const Eigen::Vector3d point1 = normed_points1[i].homogeneous();
    const Eigen::Vector3d point2 = normed_points2[i].homogeneous();
    const Eigen::Matrix<double, 1, 3> point1_row_vec = point1.transpose();

    A.block<1, 3>(2 * i + 0, 3) = -point2[2] * point1_row_vec;
    A.block<1, 3>(2 * i + 0, 6) =  point2[1] * point1_row_vec;
    A.block<1, 3>(2 * i + 1, 0) =  point2[2] * point1_row_vec;
    A.block<1, 3>(2 * i + 1, 6) = -point2[0] * point1_row_vec;
  }

  SolveConstrainedLeastSquares(A, &h);
  
  Homography2D H;
  H(0, 0) = h[0]; H(0, 1) = h[1]; H(0, 2) = h[2];
  H(1, 0) = h[3]; H(1, 1) = h[4]; H(1, 2) = h[5];
  H(2, 0) = h[6]; H(2, 1) = h[7]; H(2, 2) = h[8];

  *models = {points2_norm_matrix.inverse() *
                                   H * points1_norm_matrix};

  return true;
}

size_t Homography2DEstimator::SampleSize() const {
  return 4;
}

double Homography2DEstimator::ComputeResidual(
    const Correspondence2D& data,
    const Homography2D& model) const {
  return SingleImageError(data, model);
}

}  // namespace mvgplus
