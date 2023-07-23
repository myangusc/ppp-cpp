#include "/home/ym/workspace/ppp-cpp/include/ppp.h"

#include <cmath>

namespace {
constexpr double dt = 0.12;                      // time: s.
constexpr double turn_heading_threshold = 0.14;  // About 80 degree.
constexpr double static_point_threshold = 5.0;   //
}  // namespace

void PathPointsProcessor::Process(
    const std::vector<std::pair<double, double>>& raw_path_point2d) {
  // Smooth path points with APOLLO DiscretePoints Smoother.
  // For more information, visit:
  // https://zhuanlan.zhihu.com/p/371585754
  std::vector<std::pair<double, double>> smoothed_path_point2d;
  if (!SmoothPathPoints(raw_path_point2d, &smoothed_path_point2d)) {
    std::cout << "Failed to smooth path points." << std::endl;
    return;
  }

  // Initialize path points.
  std::vector<PathPoint*> path_points;
  for (const auto smoothed_path_point : smoothed_path_point2d) {
    PathPoint* path_point = new PathPoint;
    path_point->x = smoothed_path_point.first;
    path_point->y = smoothed_path_point.second;
    path_points.push_back(path_point);
  }

  // Evaluate path points, calculate path point's velocity and heading.
  EvaluatePathPoints(path_points);

  // Find turning points.
  std::vector<int> turning_points_index;
  FindTurningPathPoints(path_points, &turning_points_index);
  for (auto i : turning_points_index) {
    std::cout << "turning_id: " << i << std::endl;
  }

  // Find stop points.
  std::vector<vector<int>> stop_points_index;
  FindStoppingPathPoints(path_points, &stop_points_index);
  for (int i = 0; i < stop_points_index.size(); ++i) {
    std::cout << "stop_point: " << stop_points_index[i].front() << ": "
              << stop_points_index[i].back() << std::endl;
  }
  return;
}

bool PathPointsProcessor::SmoothPathPoints(
    const std::vector<std::pair<double, double>>& raw_path_point2d,
    std::vector<std::pair<double, double>>* smoothed_path_point2d) {
  // Path_points' size should bigger than 2.
  if (raw_path_point2d.size() <= 2) {
    std::cout << "Path points size less than 2." << std::endl;
    return false;
  }

  constexpr double point_bound = 0.5;
  std::vector<double> box_bounds(raw_path_point2d.size(), point_bound);

  // fix front and back points to avoid end states deviate from the original
  // position.
  box_bounds.front() = 0.0;
  box_bounds.back() = 0.0;

  // box contraints on pos are used in fem pos smoother, thus shrink the
  // bounds by 1.0 / sqrt(2.0)
  const double box_ratio = 1.0 / std::sqrt(2.0);
  for (auto& bound : box_bounds) {
    bound *= box_ratio;
  }

  std::vector<double> opt_x;
  std::vector<double> opt_y;
  FemPosDeviationSmoother smoother;
  bool status = smoother.Solve(raw_path_point2d, box_bounds, &opt_x, &opt_y);

  if (!status) {
    std::cout << "Fem Pos reference line smoothing failed" << std::endl;
    return false;
  }

  if (opt_x.size() < 2 || opt_y.size() < 2) {
    std::cout << "Return by fem pos smoother is wrong. Size smaller than 2 "
              << std::endl;
    return false;
  }

  for (int i = 0; i < opt_x.size(); ++i) {
    smoothed_path_point2d->emplace_back(opt_x[i], opt_y[i]);
  }

  return true;
}

void PathPointsProcessor::EvaluatePathPoints(
    std::vector<PathPoint*> path_points) {
  for (int i = 0; i < path_points.size(); ++i) {
    // Set path point's index.
    path_points[i]->index = i + 1;

    // Skip evaluate the first and the last path point.
    if (i == 0 || i == (path_points.size() - 1)) {
      continue;
    }

    // Calculate path point's heading.
    const double theta = atan2((path_points[i + 1]->y - path_points[i]->y),
                               (path_points[i + 1]->x - path_points[i]->x)) -
                         atan2((path_points[i]->y - path_points[i - 1]->y),
                               (path_points[i]->x - path_points[i - 1]->x));
    path_points[i]->heading_change = theta;

    // Calculate path point's vel.
    path_points[i]->vel =
        std::sqrt((path_points[i + 1]->x - path_points[i - 1]->x) *
                      (path_points[i + 1]->x - path_points[i - 1]->x) +
                  (path_points[i + 1]->y - path_points[i - 1]->y) *
                      (path_points[i + 1]->y - path_points[i - 1]->y)) /
        (2 * dt);
  }
}

void PathPointsProcessor::FindTurningPathPoints(
    std::vector<PathPoint*> path_points,
    std::vector<int>* turning_points_index) {
  for (const auto* path_point : path_points) {
    if (path_point->heading_change > turn_heading_threshold) {
      turning_points_index->emplace_back(path_point->index);
    }
  }
}

void PathPointsProcessor::FindStoppingPathPoints(
    std::vector<PathPoint*> path_points,
    std::vector<vector<int>>* stop_points_index) {
  int stop_points_unit_index = 0;
  std::vector<int> stop_points_unit;
  for (int i = 1; i < path_points.size() - 1; ++i) {
    if (path_points[i]->vel <= static_point_threshold &&
        path_points[i - 1]->vel > static_point_threshold) {
      stop_points_unit.push_back(i);
    }

    if (path_points[i]->vel <= static_point_threshold &&
        path_points[i + 1]->vel > static_point_threshold) {
      stop_points_unit.push_back(i);
      stop_points_unit_index++;
    }

    if (stop_points_unit.size() == 2) {
      stop_points_index->push_back(stop_points_unit);
      stop_points_unit.clear();
    }
  }
}

int main() {
  const std::vector<std::pair<double, double>> raw_path_point2d = {
      {59.0, 318.0},   {61.0, 317.5},   {62.5, 317.5},   {63.0, 319.5},
      {64.0, 321.5},   {66.0, 324.5},   {67.5, 328.0},   {69.0, 330.5},
      {70.5, 332.0},   {71.0, 332.5},   {71.5, 333.0},   {72.5, 335.0},
      {72.5, 338.5},   {73.5, 342.0},   {73.5, 345.5},   {73.5, 349.5},
      {74.0, 353.5},   {74.5, 355.0},   {74.5, 355.5},   {75.0, 356.0},
      {78.5, 356.0},   {82.0, 356.5},   {87.5, 358.0},   {92.0, 360.0},
      {96.5, 363.0},   {99.0, 366.5},   {101.5, 368.0},  {103.0, 368.5},
      {104.5, 368.5},  {106.0, 369.5},  {106.5, 373.0},  {108.5, 378.0},
      {113.5, 381.5},  {117.5, 384.0},  {121.5, 387.5},  {124.0, 390.5},
      {126.0, 391.5},  {127.5, 392.0},  {128.5, 392.0},  {133.0, 392.0},
      {139.5, 392.5},  {147.0, 394.0},  {154.0, 396.5},  {159.5, 399.5},
      {164.0, 402.5},  {166.5, 404.0},  {168.0, 405.0},  {169.5, 404.5},
      {170.5, 405.5},  {171.5, 409.0},  {176.5, 413.5},  {182.0, 415.5},
      {187.5, 418.5},  {192.0, 421.0},  {194.5, 423.5},  {197.0, 424.0},
      {199.0, 425.0},  {201.0, 425.0},  {201.5, 425.0},  {204.5, 425.0},
      {208.0, 425.0},  {211.5, 424.5},  {214.0, 424.5},  {214.0, 424.5},
      {215.0, 424.0},  {215.5, 424.0},  {216.0, 423.0},  {217.0, 423.0},
      {218.0, 423.0},  {219.5, 424.0},  {219.5, 424.5},  {220.0, 424.5},
      {220.0, 425.0},  {219.5, 425.0},  {218.5, 425.0},  {218.5, 425.5},
      {218.0, 425.0},  {217.5, 424.5},  {217.0, 424.5},  {216.5, 424.0},
      {216.5, 424.0},  {216.0, 424.0},  {217.0, 424.0},  {217.0, 424.0},
      {217.0, 424.0},  {217.0, 424.0},  {217.0, 424.0},  {216.5, 424.0},
      {216.5, 424.0},  {216.5, 424.0},  {217.0, 424.5},  {217.5, 424.0},
      {217.5, 424.0},  {218.0, 424.0},  {218.5, 424.0},  {219.5, 424.0},
      {219.0, 424.0},  {219.0, 424.0},  {219.0, 424.5},  {219.0, 424.5},
      {218.5, 424.5},  {218.5, 424.5},  {218.0, 424.5},  {218.5, 424.5},
      {218.0, 424.5},  {218.5, 425.0},  {218.5, 425.5},  {218.5, 425.5},
      {219.5, 425.5},  {219.5, 425.5},  {220.0, 425.5},  {220.5, 425.5},
      {220.5, 425.0},  {221.0, 425.0},  {221.0, 424.5},  {221.0, 424.0},
      {221.0, 423.5},  {221.0, 423.5},  {221.0, 423.0},  {221.0, 423.0},
      {220.5, 423.0},  {220.0, 423.0},  {220.0, 423.0},  {219.5, 423.5},
      {219.5, 423.5},  {219.0, 424.0},  {219.5, 424.0},  {219.0, 424.0},
      {219.0, 424.5},  {218.5, 425.0},  {218.5, 425.5},  {217.5, 426.0},
      {217.0, 426.5},  {217.0, 427.0},  {217.5, 427.5},  {220.5, 428.0},
      {225.0, 428.0},  {231.0, 429.5},  {237.0, 430.0},  {243.5, 430.5},
      {250.0, 431.0},  {255.0, 432.0},  {259.5, 433.0},  {264.0, 433.5},
      {268.5, 433.0},  {272.0, 432.5},  {275.0, 433.5},  {278.0, 434.5},
      {286.0, 438.0},  {299.0, 440.5},  {311.0, 442.5},  {320.5, 444.5},
      {328.0, 447.0},  {334.0, 449.5},  {340.0, 450.0},  {344.5, 449.5},
      {348.5, 449.5},  {352.5, 449.0},  {363.5, 448.5},  {376.0, 449.5},
      {395.5, 451.0},  {409.5, 452.0},  {420.5, 454.5},  {428.0, 455.5},
      {435.0, 456.0},  {441.5, 456.0},  {446.0, 455.5},  {450.5, 458.0},
      {463.0, 461.5},  {481.0, 463.0},  {497.5, 466.0},  {511.5, 469.0},
      {522.5, 472.0},  {532.0, 474.0},  {541.0, 475.0},  {549.0, 474.5},
      {557.0, 474.0},  {567.0, 473.5},  {585.0, 474.5},  {610.5, 475.0},
      {626.5, 477.0},  {635.5, 479.5},  {642.5, 480.5},  {651.0, 481.0},
      {664.5, 481.0},  {676.0, 481.5},  {687.0, 485.5},  {709.5, 489.5},
      {730.5, 493.0},  {749.5, 496.5},  {760.0, 499.0},  {768.0, 502.5},
      {776.5, 504.5},  {787.5, 505.0},  {804.0, 505.0},  {820.5, 504.5},
      {839.0, 504.5},  {852.5, 505.5},  {873.5, 508.0},  {888.0, 509.5},
      {895.5, 514.0},  {903.0, 515.0},  {915.5, 517.0},  {930.0, 517.0},
      {952.5, 520.5},  {973.5, 526.0},  {989.0, 531.0},  {1007.0, 533.0},
      {1024.0, 538.5}, {1035.0, 544.0}, {1057.5, 547.5}, {1075.5, 549.0},
      {1066.0, 550.0}, {1084.0, 550.0}, {1102.5, 550.0}, {1115.5, 549.5},
      {1133.5, 551.0}, {1149.0, 554.0}, {1158.5, 557.5}, {1161.5, 558.5},
      {1177.5, 561.5}, {1211.5, 562.0}, {1230.0, 563.0}, {1244.0, 567.0},
      {1255.0, 572.5}, {1272.5, 578.5}, {1286.0, 583.0}, {1296.5, 586.5},
      {1323.0, 591.5}, {1327.0, 595.0}, {1352.0, 597.0}, {1368.0, 599.0},
      {1371.0, 598.5}, {1384.0, 599.5}, {1393.0, 600.5}, {1402.5, 603.0},
      {1409.5, 605.5}, {1417.0, 608.0}, {1426.5, 614.0}, {1442.5, 617.0},
      {1480.0, 618.5}, {1500.0, 619.0}, {1512.5, 620.0}, {1519.5, 625.0},
      {1533.5, 637.5}, {1538.5, 638.5}, {1563.0, 640.0}, {1579.0, 643.5},
      {1614.5, 647.5}, {1629.5, 648.5}, {1638.5, 649.0}, {1644.0, 649.0},
      {1649.0, 649.0}, {1652.0, 649.5}, {1656.5, 649.5}, {1659.5, 648.5},
      {1664.5, 645.5}, {1684.5, 642.0}, {1701.0, 638.0}, {1706.5, 640.0},
      {1710.5, 638.0}, {1710.5, 638.0}, {1712.0, 635.5}, {1712.5, 634.5},
      {1712.5, 633.5}, {1712.5, 633.0}, {1712.0, 632.0}, {1711.5, 629.0},
      {1712.5, 628.5}, {1711.5, 624.0}, {1711.5, 622.0}, {1711.0, 621.0},
      {1710.0, 621.0}};
  PathPointsProcessor::Process(raw_path_point2d);
  std::cout << "======Path_Point_Process_Finish======" << std::endl;
  return 0;
}
