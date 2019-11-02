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

inline V4 sane_conv(const Q & q) {
  return V4(
    q.w(),
    q.x(),
    q.y(),
    q.z()
  );
}
inline Q sane_conv(const V4 & v) {
  return Q(
    v[0],
    v[1],
    v[2],
    v[3]
  );
}
inline void q_add(Q & a, const Q & b, double s) {
  a.w() += s * b.w();
  a.x() += s * b.x();
  a.y() += s * b.y();
  a.z() += s * b.z();
}

struct Barycentric {

  Barycentric(
    V3 a = V3(1,1,1),
    V3 b = V3(-1,-1,1),
    V3 c = V3(-1,1,-1),
    V3 d = V3(1,-1,-1)
  ) : a(a), b(b), c(c), d(d) {

    Eigen::Matrix4d tmp;
    tmp <<
      a[0], b[0], c[0], d[0],
      a[1], b[1], c[1], d[1],
      a[2], b[2], c[2], d[2],
      1.0 , 1.0 , 1.0 , 1.0;

    inv = tmp.inverse();

  }

  inline V3 get_point(const Q & q) {
    return q.w() * a + q.x() * b + q.y() * c + q.z() * d;
  }

  inline V4 get_coords(const V3 & p) {
    return inv * V4(p[0],p[1],p[2],1.0);
  }

  inline Q get_quaternion(const V3 & p) {
    return sane_conv(get_coords(p));
  }

  V3 a,b,c,d;

  Eigen::Matrix4d inv;

};

struct QuaternionFourier {

  QuaternionFourier(size_t max_spin = 5) {

    num_terms = (max_spin*2 + 1) * (max_spin*2 + 1);

    u.reserve(num_terms);
    u.push_back(V2::Zero());
    int bound = max_spin;
    for (int spin_u = -bound; spin_u < bound + 1; spin_u ++) {
      for (int spin_v = -bound; spin_v < bound + 1; spin_v ++) {
        u.push_back(V2(spin_u, spin_v));
      }
    }

    f_hat.resize(num_terms, Q(0.0,0.0,0.0,0.0));

    interpolate([](const V2 & x) {
      V2 phase = 2.0*PI*x;
      return V3(
        (1.0 + 0.5*cos(phase[0])) * cos(phase[1]),
        (1.0 + 0.5*cos(phase[0])) * sin(phase[1]),
        0.5*sin(phase[0])
      );
    });

  }

  void interpolate(std::function<V3(const V2 &)> f, size_t interpolation_res = 100) {

    double du = 1.0 / static_cast<double>(interpolation_res * interpolation_res);

    V2 x;
    for (size_t n = 0; n < interpolation_res; n++) {
      x[0] = static_cast<double>(n) / static_cast<double>(interpolation_res);

      for (size_t m = 0; m < interpolation_res; m++) {
        x[1] = static_cast<double>(m) / static_cast<double>(interpolation_res);

        V3 tmp = f(x);
        Q q = barycentric.get_quaternion(tmp);

        for (size_t t = 0; t < num_terms; t ++) {

          V2 phase = - 2.0 * PI * x.cwiseProduct(u[t]);

          Q left(cos(phase[0]), sin(phase[0]), 0.0, 0.0);
          Q right(cos(phase[1]), 0.0, sin(phase[1]), 0.0);

          q_add(f_hat[t], left * q * right, du);
        }
      }
    }
  }

  inline Q get_term(size_t n, V2 x) {

    V2 phase = 2.0 * PI * x.cwiseProduct(u[n]);

    Q left(cos(phase[0]), sin(phase[0]), 0.0, 0.0);
    Q right(cos(phase[1]), 0.0, sin(phase[1]), 0.0);

    return left * f_hat[n] * right;

  }

  inline V3 get_term_as_point(size_t n, V2 x) {

    Q tmp = get_term(n, x);

    return barycentric.get_point(tmp);

  }

  size_t num_terms;

  std::vector<V2> u;
  std::vector<Q> f_hat;

  Barycentric barycentric;

};

struct BlenderData {

  BlenderData(size_t num_objects, size_t num_frames, double v) : num_objects(num_objects), num_frames(num_frames) {

    scales_s.resize(num_objects);
    locations_s.resize(num_objects);
    rotations_s.resize(num_objects);

    QuaternionFourier qf(sqrt(num_objects)/2 + 1);

    for (size_t o = 0; o < num_objects; o++) {
      scales_s[o].resize(num_frames);
      locations_s[o].resize(num_frames);
      rotations_s[o].resize(num_frames);
    }

    for (size_t f = 0; f < num_frames; f++) {

      V3 offset = V3::Zero();
      double fd = static_cast<double>(f) / static_cast<double>(num_frames);


      V2 x(v,fd);


      for (size_t o = 0; o < num_objects; o++) {

        locations_s[o][f] = offset;

        V3 add_offset = qf.get_term_as_point(o, x);

        scales_s[o][f][0] = add_offset.norm();
        scales_s[o][f][1] = add_offset.norm();
        scales_s[o][f][2] = add_offset.norm();

        Q to_offset = Q::FromTwoVectors(V3::UnitX(), add_offset);

        rotations_s[o][f] = sane_conv(to_offset);

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
  BlenderData* construct(size_t resolution, size_t num_frames, double v) {
    return new BlenderData(resolution, num_frames, v);
  }
  double * get_scales(BlenderData* data, size_t i) { return data->scales_s[i][0].data(); }
  double * get_locations(BlenderData* data, size_t i) { return data->locations_s[i][0].data(); }
  double * get_rotations(BlenderData* data, size_t i) { return data->rotations_s[i][0].data(); }
}
