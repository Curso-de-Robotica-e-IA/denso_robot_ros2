#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace denso_collab
{

inline bool valid_target_batch(const std::vector<double> & data, const std::string & mode)
{
  const bool captcha = mode == "captcha";
  if ((!captcha && mode != "rgb") || data.empty() || data.size() % 6 != 0 ||
    (captcha ? data.size() > 9 * 6 : data.size() % 18 != 0))
  {
    return false;
  }
  std::array<int, 3> counts{};
  for (std::size_t index = 0; index < data.size(); index += 6) {
    const auto row = data.begin() + index;
    if (!std::all_of(row, row + 6, [](double value) {return std::isfinite(value);}) ||
      (captcha ? row[0] != 3.0 : row[0] < 0.0 || row[0] > 2.0) ||
      row[0] != std::floor(row[0]) || row[1] < 0.0 || row[2] < 0.0 || row[3] <= 0.0)
    {
      return false;
    }
    if (!captcha) {
      ++counts[static_cast<std::size_t>(row[0])];
    }
  }
  return captcha || (counts[0] == counts[1] && counts[1] == counts[2]);
}

}  // namespace denso_collab
