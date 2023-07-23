#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "/home/ym/workspace/ppp-cpp/include/fem_pos_deviation_smoother.h"

using namespace std;

struct PathPoint {
  double x = 0.0;
  double y = 0.0;
  double vel = 0.0;
  double heading_change = 0.0;
  bool is_stop = false;
  int index = -1;
};

class PathPointsProcessor {
 public:
  static void Process(
      const std::vector<std::pair<double, double>>& raw_path_point2d);

 private:
  static bool SmoothPathPoints(
      const std::vector<std::pair<double, double>>& raw_path_point2d,
      std::vector<std::pair<double, double>>* smoothed_path_point2d);

  static void EvaluatePathPoints(std::vector<PathPoint*> path_points);

  static void FindTurningPathPoints(std::vector<PathPoint*> path_points,
                                    std::vector<int>* turning_points_index);

  static void FindStoppingPathPoints(
      std::vector<PathPoint*> path_points,
      std::vector<vector<int>>* stop_points_index);
};
