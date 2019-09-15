#include <vector>
#include <Eigen/Dense>

char* hello() {
  return "Hello, World!";
}

using Vector = Eigen::Vector3d;

std::vector<Vector> surface_points() {
  return {
    Vector(-1,-1,-1),
    Vector(-1, 1,-1),
    Vector( 1,-1,-1),
    Vector( 1, 1,-1),
    Vector(-1,-1, 1),
    Vector(-1, 1, 1),
    Vector( 1,-1, 1),
    Vector( 1, 1, 1),
  };
}
