#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <functional>

#define PI 3.14159265

using V2 = Eigen::Vector2d;
using V3 = Eigen::Vector3d;
using V4 = Eigen::Vector4d;

struct Data {

  Data(size_t resolution, size_t num_frames) : resolution(resolution), num_frames(num_frames) {

    scales_s.resize(resolution);
    locations_s.resize(resolution);
    rotations_s.resize(resolution);
    for (size_t r = 0; r < resolution; r++) {
      double rd = static_cast<double>(r)/static_cast<double>(resolution);
      double ra = rd * PI * 2.0;
      auto & scales = scales_s[r];
      auto & locations = locations_s[r];
      auto & rotations = rotations_s[r];


      scales.resize(num_frames);
      locations.resize(num_frames);
      rotations.resize(num_frames);
      for (size_t f = 0; f < num_frames; f++) {
        double fd = static_cast<double>(f)/static_cast<double>(num_frames);
        double fa = fd * PI * 2.0;
        auto & scale = scales[f];
        auto & location = locations[f];
        auto & rotation = rotations[f];

        scale = V3::Ones();

        if (r > 0) {
          location = Eigen::Quaternion<double>(
              rotations_s[r-1][f][0],
              rotations_s[r-1][f][1],
              rotations_s[r-1][f][2],
              rotations_s[r-1][f][3]
          ) * V3::UnitX();
          location += locations_s[r-1][f];
        } else {
          location = V3::Zero();
        }

        V2 rotA(sin(rd*5.0), cos(rd*5.0));
        V2 rotB(-sin(fd+PI), cos(fd+PI));

        rotA *= sin(fd*rd*rd*50.0);
        rotB *= cos(fd*rd*rd*50.0);

        rotation = V4(rotA.x(), rotA.y(), rotB.x(), rotB.y());
      }
    }
  }

  size_t resolution;
  size_t num_frames;

  std::vector<std::vector<Eigen::Vector3d>> scales_s;
  std::vector<std::vector<Eigen::Vector3d>> locations_s;
  std::vector<std::vector<Eigen::Vector4d>> rotations_s;

};

extern "C" {
  Data* construct(size_t resolution, size_t num_frames) {
    return new Data(resolution, num_frames);
  }
  double * get_scales(Data* data, size_t i) { return data->scales_s[i][0].data(); }
  double * get_locations(Data* data, size_t i) { return data->locations_s[i][0].data(); }
  double * get_rotations(Data* data, size_t i) { return data->rotations_s[i][0].data(); }
}
