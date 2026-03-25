#include "room_model.h"

namespace rc
{

void Model::set_device(torch::Device device)
{
    device_ = device;
}

void Model::init_from_state(float width, float length, float x, float y, float phi, float wall_height)
{
    use_polygon = false;
    const float half_w = width * 0.5f;
    const float half_l = length * 0.5f;
    half_height = wall_height * 0.5f;

    // fixed room size
    half_extents = torch::tensor({half_w, half_l},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    // optimized robot pose
    robot_pos = torch::tensor({x, y},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));
    robot_theta = torch::tensor({phi},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));

    register_parameter("robot_pos", robot_pos);
    register_parameter("robot_theta", robot_theta);

    init_common();
}

void Model::init_from_polygon(const std::vector<Eigen::Vector2f>& vertices,
                               float x, float y, float phi, float wall_height)
{
    use_polygon = true;
    half_height = wall_height * 0.5f;

    // Store polygon vertices as tensor [N, 2]
    std::vector<float> verts_flat;
    verts_flat.reserve(vertices.size() * 2);
    for (const auto& v : vertices)
    {
        verts_flat.push_back(v.x());
        verts_flat.push_back(v.y());
    }
    polygon_vertices = torch::from_blob(verts_flat.data(),
        {static_cast<long>(vertices.size()), 2}, torch::kFloat32).clone().to(device_);

    // Pre-compute segment data for faster SDF
    const int64_t num_verts = polygon_vertices.size(0);
    auto indices_a = torch::arange(num_verts,
        torch::TensorOptions().dtype(torch::kLong).device(device_));
    auto indices_b = (indices_a + 1) % num_verts;
    seg_a_ = polygon_vertices.index_select(0, indices_a).contiguous();
    const auto seg_b = polygon_vertices.index_select(0, indices_b).contiguous();
    seg_ab_ = (seg_b - seg_a_).contiguous();
    seg_ab_sq_ = torch::sum(seg_ab_ * seg_ab_, /*dim=*/1).contiguous();

    // Compute bounding box for half_extents (for compatibility)
    float min_x = vertices[0].x(), max_x = vertices[0].x();
    float min_y = vertices[0].y(), max_y = vertices[0].y();
    for (const auto& v : vertices)
    {
        min_x = std::min(min_x, v.x()); max_x = std::max(max_x, v.x());
        min_y = std::min(min_y, v.y()); max_y = std::max(max_y, v.y());
    }
    half_extents = torch::tensor({(max_x - min_x) / 2.f, (max_y - min_y) / 2.f},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    // optimized robot pose
    robot_pos = torch::tensor({x, y},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));
    robot_theta = torch::tensor({phi},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_).requires_grad(true));

    register_parameter("robot_pos", robot_pos);
    register_parameter("robot_theta", robot_theta);

    init_common();
}

void Model::init_common()
{
    // Initialize prediction tensors (detached from graph)
    predicted_pos = torch::zeros({2},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    predicted_theta = torch::zeros({1},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    prediction_precision_matrix = torch::eye(3,
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    has_prediction = false;

    // Initialize covariance for EKF prediction (moderate initial uncertainty)
    prev_cov = 0.1f * torch::eye(3,
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    // Initialize previous pose
    robot_prev_pose = std::nullopt;
}

void Model::set_prediction(const Eigen::Vector2f& pred_pos, float pred_theta,
                           const Eigen::Matrix3f& precision)
{
    predicted_pos = torch::tensor({pred_pos.x(), pred_pos.y()},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    predicted_theta = torch::tensor({pred_theta},
        torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    // Convert Eigen Matrix to Tensor
    prediction_precision_matrix = torch::from_blob(
        const_cast<float*>(precision.data()),
        {3, 3},
        torch::kFloat32).clone().to(device_);

    has_prediction = true;
}

torch::Tensor Model::prior_loss() const
{
    if (!has_prediction)
        return torch::tensor(0.0f, torch::TensorOptions().device(device_));

    // Current state vector [x, y, theta]
    auto current_state = torch::cat({robot_pos, robot_theta}, 0); // [3]
    auto pred_state = torch::cat({predicted_pos, predicted_theta}, 0); // [3]

    auto diff = (current_state - pred_state).unsqueeze(1); // [3, 1]

    // (s - mu)^T * Sigma^-1 * (s - mu)
    // [1, 3] * [3, 3] * [3, 1] -> [1, 1]
    auto loss = torch::matmul(diff.t(), torch::matmul(prediction_precision_matrix, diff));

    return 0.5f * loss.squeeze();
}

torch::Tensor Model::sdf(const torch::Tensor& points_robot) const
{
    // Transform points from robot frame to room frame
    // Use the device of input points as the reference
    const auto device = points_robot.device();
    const auto pxy = points_robot.index({torch::indexing::Slice(), torch::indexing::Slice(0,2)});

    // Move model parameters to the same device as points (no-op if already there)
    const auto theta = robot_theta.to(device);
    const auto pos = robot_pos.to(device);

    const auto c = torch::cos(theta);
    const auto s = torch::sin(theta);

    // Build rotation matrix on the same device as points
    auto rot = torch::zeros({2, 2}, pxy.options());
    rot[0][0] = c.squeeze();
    rot[0][1] = -s.squeeze();
    rot[1][0] = s.squeeze();
    rot[1][1] = c.squeeze();

    // Map points to room frame: p_room = R(phi) * p_robot + t
    const auto points_room_xy = torch::matmul(pxy, rot.transpose(0,1)) + pos;

    if (use_polygon)
    {
        return sdf_polygon(points_room_xy);
    }
    else
    {
        return sdf_box(points_robot, points_room_xy);
    }
}

torch::Tensor Model::sdf_box(const torch::Tensor& points_robot,
                              const torch::Tensor& points_room_xy) const
{
    const auto device = points_room_xy.device();
    const auto pz = points_robot.index({torch::indexing::Slice(), 2}).reshape({-1,1});
    const auto points_room = torch::cat({points_room_xy, pz.to(device)}, 1);

    const auto hz = torch::full({1}, half_height, points_room.options());
    const auto half_ext = half_extents.to(device);
    const auto half_sizes = torch::cat({half_ext, hz}, 0);

    const auto abs_points = torch::abs(points_room);
    const auto d = abs_points - half_sizes;

    const auto outside = torch::norm(torch::max(d, torch::zeros_like(d)), 2, 1);
    const auto inside = torch::clamp_max(
        torch::max(torch::max(d.select(1,0), d.select(1,1)), d.select(1,2)), 0.0);
    return outside + inside;
}

torch::Tensor Model::sdf_polygon(const torch::Tensor& points_room_xy) const
{
    // Fully vectorized: broadcast [N,1,2] vs [1,S,2] for all segments at once
    // Segment tensors (seg_a_, seg_ab_, seg_ab_sq_) are already on device_ from init

    // points: [N, 2] → [N, 1, 2];  segments: [S, 2] → [1, S, 2]
    const auto pts = points_room_xy.unsqueeze(1);  // [N, 1, 2]
    const auto a_  = seg_a_.unsqueeze(0);           // [1, S, 2]
    const auto ab_ = seg_ab_.unsqueeze(0);          // [1, S, 2]

    // ap = pts - a  →  [N, S, 2]
    const auto ap = pts - a_;

    // t = clamp(dot(ap, ab) / |ab|², 0, 1)  →  [N, S]
    auto t = torch::sum(ap * ab_, /*dim=*/2) / (seg_ab_sq_.unsqueeze(0) + 1e-8f);
    t = torch::clamp(t, 0.0f, 1.0f);

    // closest = a + t * ab  →  [N, S, 2]
    const auto closest = a_ + t.unsqueeze(2) * ab_;

    // dist² = |pts - closest|²  →  [N, S]
    const auto diff = pts - closest;
    const auto dist_sq = torch::sum(diff * diff, /*dim=*/2);

    // Min over segments → [N]
    const auto min_dist_sq = std::get<0>(torch::min(dist_sq, /*dim=*/1));

    return torch::sqrt(min_dist_sq + 1e-8f);
}

Eigen::Matrix<float, 5, 1> Model::get_state() const
{
    // Must convert to CPU for accessor
    auto ext_cpu = half_extents.to(torch::kCPU);
    auto pos_cpu = robot_pos.to(torch::kCPU);
    auto th_cpu  = robot_theta.to(torch::kCPU);
    const auto ext = ext_cpu.accessor<float,1>();
    const auto pos = pos_cpu.accessor<float,1>();
    const auto th  = th_cpu.accessor<float,1>();
    Eigen::Matrix<float,5,1> s;
    s << 2.f*ext[0], 2.f*ext[1], pos[0], pos[1], th[0];
    return s;
}

std::vector<torch::Tensor> Model::optim_parameters() const
{
    return {robot_pos, robot_theta};
}


} // namespace rc

