#undef NDEBUG
#include <cassert>
#include <limits>
#include "denso_collab/target_batch.hpp"

int main()
{
  using denso_collab::valid_target_batch;
  const std::vector<double> square{3, 100, 200, 12, -0.02, 0.01};
  const std::vector<double> rgb{0, 100, 200, 12, -0.02, 0.01,
    1, 150, 200, 12, 0, 0.01, 2, 200, 200, 12, 0.02, 0.01};
  assert(valid_target_batch(square, "captcha"));
  assert(valid_target_batch(rgb, "rgb"));
  assert(!valid_target_batch(square, "rgb"));
  assert(!valid_target_batch(rgb, "captcha"));
  assert(!valid_target_batch(square, "unknown"));
  assert(!valid_target_batch({}, "captcha"));
  auto batch = square;
  for (int count = 2; count <= 9; ++count) {
    batch.insert(batch.end(), square.begin(), square.end());
    assert(valid_target_batch(batch, "captcha"));
  }
  batch.insert(batch.end(), square.begin(), square.end());
  assert(!valid_target_batch(batch, "captcha"));
  batch = rgb;
  batch[6] = 0;
  assert(!valid_target_batch(batch, "rgb"));
  for (std::size_t index = 0; index < square.size(); ++index) {
    batch = square;
    batch[index] = std::numeric_limits<double>::quiet_NaN();
    assert(!valid_target_batch(batch, "captcha"));
  }
  for (const double kind : {-1.0, 0.0, 2.5, 4.0}) {
    batch = square;
    batch[0] = kind;
    assert(!valid_target_batch(batch, "captcha"));
  }
  batch = square;
  batch[3] = 0;
  assert(!valid_target_batch(batch, "captcha"));
  batch = square;
  batch.pop_back();
  assert(!valid_target_batch(batch, "captcha"));
}
