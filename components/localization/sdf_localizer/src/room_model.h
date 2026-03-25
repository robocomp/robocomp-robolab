#pragma once

#include <memory>
#include <vector>
#include <limits>
#include <optional>

// ---- PyTorch vs Qt macros (slots/signals/emit) ----
#ifdef slots
  #define RC_QT_SLOTS_WAS_DEFINED
  #undef slots
#endif
#ifdef signals
  #define RC_QT_SIGNALS_WAS_DEFINED
  #undef signals
#endif
#ifdef emit
  #define RC_QT_EMIT_WAS_DEFINED
  #undef emit
#endif

#include <torch/torch.h>

#ifdef RC_QT_SLOTS_WAS_DEFINED
  #define slots Q_SLOTS
  #undef RC_QT_SLOTS_WAS_DEFINED
#endif
#ifdef RC_QT_SIGNALS_WAS_DEFINED
  #define signals Q_SIGNALS
  #undef RC_QT_SIGNALS_WAS_DEFINED
#endif
#ifdef RC_QT_EMIT_WAS_DEFINED
  #define emit Q_EMIT
  #undef RC_QT_EMIT_WAS_DEFINED
#endif

#include <Eigen/Dense>

namespace rc
{

/**
 * Model - Room model for Active Inference SLAM
 * Supports both axis-aligned box and polygon rooms
 *
 * State (5): [width, length, x, y, phi]
 *  - width/length: room dimensions (m)
 *  - x,y,phi: robot pose wrt room center
 */
class Model : public torch::nn::Module
{
public:
    // Device for tensor operations (CPU or CUDA)
    torch::Device device_ = torch::kCPU;

    // Room parameters (fixed)
    torch::Tensor half_extents;      // [half_width, half_length] - for box mode
    torch::Tensor polygon_vertices;  // [N, 2] polygon vertices in room frame
    bool use_polygon = false;

    // Robot pose wrt room center (optimized)
    torch::Tensor robot_pos;     // [x, y]
    torch::Tensor robot_theta;   // [phi]

    float half_height = 1.2f;    // half of wall height (fixed)

    // Prediction fields for prior loss (Active Inference)
    torch::Tensor predicted_pos;     // [x, y] from motion model
    torch::Tensor predicted_theta;   // [phi] from motion model
    torch::Tensor prediction_precision_matrix; // Inverse covariance [3x3]
    bool has_prediction = false;

    // State for EKF-style prediction
    torch::Tensor prev_cov;
    std::optional<Eigen::Affine2f> robot_prev_pose;

    // Pre-computed segment data for polygon SDF
    torch::Tensor seg_a_;      // [num_segs, 2] segment start points
    torch::Tensor seg_ab_;     // [num_segs, 2] segment vectors (end - start)
    torch::Tensor seg_ab_sq_;  // [num_segs] squared length of each segment

    // Set the device for all tensors
    void set_device(torch::Device device);

    // Initialize from rectangular room
    void init_from_state(float width, float length, float x, float y, float phi, float wall_height);

    // Initialize from polygon room
    void init_from_polygon(const std::vector<Eigen::Vector2f>& vertices,
                           float x, float y, float phi, float wall_height);

    // Set prediction for Active Inference prior
    void set_prediction(const Eigen::Vector2f& pred_pos, float pred_theta,
                        const Eigen::Matrix3f& precision);

    // Compute prior loss (Mahalanobis distance from prediction)
    torch::Tensor prior_loss() const;

    // Compute SDF for points in robot frame
    torch::Tensor sdf(const torch::Tensor& points_robot) const;

    // Get current state as Eigen vector [width, length, x, y, phi]
    Eigen::Matrix<float, 5, 1> get_state() const;

    // Get optimizable parameters
    std::vector<torch::Tensor> optim_parameters() const;

private:
    void init_common();

    // SDF for box room
    torch::Tensor sdf_box(const torch::Tensor& points_robot,
                          const torch::Tensor& points_room_xy) const;

    // SDF for polygon room
    torch::Tensor sdf_polygon(const torch::Tensor& points_room_xy) const;
};

} // namespace rc

