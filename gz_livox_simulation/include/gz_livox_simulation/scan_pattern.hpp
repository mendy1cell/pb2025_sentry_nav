// Gazebo Sim-independent Livox scan pattern logic. Keep this aligned with
// ros2_livox_simulation's Classic LivoxPointsPlugin.
#ifndef GZ_LIVOX_SIMULATION__SCAN_PATTERN_HPP_
#define GZ_LIVOX_SIMULATION__SCAN_PATTERN_HPP_

#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace gz_livox_simulation
{
struct RotateInfo
{
  double time;
  double azimuth;  // radians
  double zenith;   // radians, CSV zenith - pi/2
};

struct RaySample
{
  std::size_t ray_index;
  std::size_t csv_index;
  RotateInfo rotate;
  std::uint32_t offset_time_ns;
};

inline std::vector<RotateInfo> ReadPatternCsv(const std::string & file_name)
{
  std::ifstream stream(file_name);
  if (!stream.is_open()) {
    throw std::runtime_error("cannot read CSV: " + file_name);
  }
  std::string line;
  std::getline(stream, line);  // Classic reader unconditionally consumes header.
  std::vector<RotateInfo> result;
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream fields(line);
    std::string value;
    std::vector<double> row;
    while (std::getline(fields, value, ',')) {
      row.push_back(std::stod(value));
    }
    if (row.size() != 3U) {
      throw std::runtime_error("CSV row must contain time,azimuth,zenith: " + line);
    }
    result.push_back({row[0], row[1] * kDegToRad, row[2] * kDegToRad - 1.57079632679489661923});
  }
  if (result.empty()) {
    throw std::runtime_error("CSV contains no scan rows: " + file_name);
  }
  return result;
}

inline std::vector<RaySample> BuildScan(
  const std::vector<RotateInfo> & pattern, std::int64_t current_start_index,
  std::int64_t samples, std::int64_t downsample)
{
  if (pattern.empty() || samples <= 0 || downsample <= 0) {
    throw std::invalid_argument("pattern must be non-empty; samples/downsample must be positive");
  }
  std::vector<RaySample> result;
  result.reserve(static_cast<std::size_t>((samples + downsample - 1) / downsample));
  constexpr std::uint64_t kScanPeriodNs = 100000000ULL;  // Classic 10 Hz period.
  const std::uint64_t count = static_cast<std::uint64_t>((samples + downsample - 1) / downsample);
  const std::uint64_t point_period_ns = kScanPeriodNs / count;
  std::size_t ray_index = 0;
  for (std::int64_t k = current_start_index; k < current_start_index + samples; k += downsample) {
    const auto csv_index = static_cast<std::size_t>(k % static_cast<std::int64_t>(pattern.size()));
    result.push_back({ray_index, csv_index, pattern[csv_index],
      static_cast<std::uint32_t>(ray_index * point_period_ns)});
    ++ray_index;
  }
  return result;
}

// Matches Classic: Euler(0, zenith, azimuth) * unit X. The sensor pose is
// applied only to ray start/end; output point coordinates use this local axis.
inline void AxisFromRotate(const RotateInfo & rotate, double & x, double & y, double & z)
{
  x = std::cos(rotate.azimuth) * std::cos(rotate.zenith);
  y = std::sin(rotate.azimuth) * std::cos(rotate.zenith);
  z = -std::sin(rotate.zenith);
}
}  // namespace gz_livox_simulation
#endif
