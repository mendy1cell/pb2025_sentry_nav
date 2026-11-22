#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

#include "gz_livox_simulation/scan_pattern.hpp"

int main()
{
  const std::vector<gz_livox_simulation::RotateInfo> pattern{{0., 0., 0.}, {1., 1., 2.}, {2., 3., 4.}};
  const auto scan = gz_livox_simulation::BuildScan(pattern, 2, 10, 3);
  assert(scan.size() == 4U);                    // 2,5,8,11
  assert(scan[0].csv_index == 2U && scan[1].csv_index == 2U);
  assert(scan[2].csv_index == 2U && scan[3].csv_index == 2U);
  assert(scan[0].ray_index == 0U && scan[3].ray_index == 3U);
  assert(scan[0].offset_time_ns == 0U);
  assert(scan[1].offset_time_ns == 25000000U);
  assert(scan[3].offset_time_ns == 75000000U);
  double x{}, y{}, z{};
  gz_livox_simulation::AxisFromRotate({0., 0., 0.}, x, y, z);
  assert(x == 1.0 && y == 0.0 && z == 0.0);
  std::cout << "scan-pattern regression passed\n";
}
