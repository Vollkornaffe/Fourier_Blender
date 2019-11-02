#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <functional>

#define PI 3.14159265

using V2 = Eigen::Vector2d;
using V3 = Eigen::Vector3d;
using V4 = Eigen::Vector4d;
using Q = Eigen::Quaternion<double>;

struct Barycentric {

  V3 get_point(const Q & q) {
    return q.w() * a + q.x() * b + q.y() * c + q.z() * d;
  }

  V3 a = V3(1,1,1);
  V3 b = V3(-1,-1,1);
  V3 c = V3(-1,1,-1);
  V3 d = V3(1,-1,-1);

};

struct QuaternionFourier {

  QuaternionFourier(size_t max_spin = 5) {

    num_terms = (max_spin*2 + 1) * (max_spin*2 + 1);

    u.reserve(num_terms);
    u.push_back(V2::Zero());
    for (int spin_u = 1; spin_u < max_spin + 1; spin_u ++) {
      for (int spin_v = 1; spin_v < max_spin + 1; spin_v ++) {
        u.push_back(V2(-spin_u, -spin_v));
        u.push_back(V2(-spin_u,  spin_v));
        u.push_back(V2( spin_u, -spin_v));
        u.push_back(V2( spin_u,  spin_v));
      }
    }

    f_hat.resize(num_terms, Q(1.0,0.0,0.0,0.0));

  }

  Q get_term(size_t n, V2 x) {

    V2 phase = 2.0 * PI * x.cwiseProduct(u[n]);

    Q left(cos(phase[0]),
           sin(phase[0]),
           0.0,
           0.0);

    Q right(cos(phase[1]),
            0.0,
            sin(phase[1]),
            0.0);

    return left * f_hat[n] * right;

  }

  size_t num_terms;

  std::vector<V2> u;
  std::vector<Q> f_hat;

};

struct BlenderData {

  BlenderData(size_t num_objects, size_t num_frames) : num_objects(num_objects), num_frames(num_frames) {

    scales_s.resize(num_objects);
    locations_s.resize(num_objects);
    rotations_s.resize(num_objects);

    Barycentric bc;
    QuaternionFourier qf(sqrt(num_objects)/2 + 1);

    for (size_t o = 0; o < num_objects; o++) {
      scales_s[o].resize(num_frames);
      locations_s[o].resize(num_frames);
      rotations_s[o].resize(num_frames);
    }

    for (size_t f = 0; f < num_frames; f++) {

      V3 offset = V3::Zero();
      double fd = static_cast<double>(f) / static_cast<double>(num_frames);
      V2 x(sin(fd), cos(fd));

      for (size_t o = 0; o < num_objects; o++) {

        locations_s[o][f] = offset;

        Q q_term = qf.get_term(o, x);

        V3 add_offset = bc.get_point(q_term);

        scales_s[o][f][0] = add_offset.norm();
        scales_s[o][f][1] = 1.0;
        scales_s[o][f][2] = 1.0;

        Q to_offset = Q::FromTwoVectors(V3::UnitX(), add_offset);

        rotations_s[o][f][0] = to_offset.w();
        rotations_s[o][f][1] = to_offset.x();
        rotations_s[o][f][2] = to_offset.y();
        rotations_s[o][f][3] = to_offset.z();

        offset += add_offset;

      }
    }

  }

  size_t num_objects;
  size_t num_frames;

  std::vector<std::vector<V3>> scales_s;
  std::vector<std::vector<V3>> locations_s;
  std::vector<std::vector<V4>> rotations_s;

};


extern "C" {
  BlenderData* construct(size_t resolution, size_t num_frames) {
    return new BlenderData(resolution, num_frames);
  }
  double * get_scales(BlenderData* data, size_t i) { return data->scales_s[i][0].data(); }
  double * get_locations(BlenderData* data, size_t i) { return data->locations_s[i][0].data(); }
  double * get_rotations(BlenderData* data, size_t i) { return data->rotations_s[i][0].data(); }
}
