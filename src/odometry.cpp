#include "odometry.h"
#include <cmath>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm) {
  // Linear velocity (m/s) = (wheel circumference * revolutions per second)
  double rps = rpm / 60.0;
  linear_vel = 2 * M_PI * radius * rps;
}

double Odometry::distance(int x1, int y1, int x2, int y2) {
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double Odometry::angle(int x1, int y1, int x2, int y2) {
  // atan2 returns radians, convert to degrees for the final output.
  return atan2(y2 - y1, x2 - x1) * 180.0 / M_PI;
}

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {
  MotionCommand res = {0.0, 0.0}; // store total angle and time

  if (path.size() < 2) {
    return res;
  }

  double total_distance = 0.0;
  double total_angle_turned = 0.0;
  // Assume the rover starts facing along the positive Y-axis (angle = 90 deg)
  // based on how grid coordinates (row, col) map to (x, y).
  // Let's assume initial angle is 0 for simplicity, facing positive X (column direction).
  double current_angle_deg = 0.0; 

  for (size_t i = 0; i < path.size() - 1; ++i) {
    pair<int, int> p1 = path[i];
    pair<int, int> p2 = path[i+1];

    // --- Calculate distance for this segment ---
    total_distance += distance(p1.first, p1.second, p2.first, p2.second);

    // --- Calculate angle for this segment ---
    double target_angle_deg = angle(p1.first, p1.second, p2.first, p2.second);
    
    // Calculate the turn required
    double turn_angle = target_angle_deg - current_angle_deg;

    // Normalize the angle to the shortest turn (-180 to 180)
    while (turn_angle > 180.0) turn_angle -= 360.0;
    while (turn_angle <= -180.0) turn_angle += 360.0;

    // Add the absolute value of the turn to the total
    total_angle_turned += abs(turn_angle);

    // Update the rover's current angle
    current_angle_deg = target_angle_deg;
  }

  // Calculate total time for linear traversal
  if (linear_vel > 0) {
    res.time_sec = total_distance / linear_vel;
  }

  res.angle_deg = total_angle_turned;

  return res;
}
