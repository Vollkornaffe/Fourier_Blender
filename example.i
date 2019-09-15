%module example
%{
extern char* hello();
#include <Eigen/Dense>
#include <vector>
extern std::vector<Eigen::Vector3d> surface_points();
%}

extern std::vector<Eigen::Vector3d> surface_points();
