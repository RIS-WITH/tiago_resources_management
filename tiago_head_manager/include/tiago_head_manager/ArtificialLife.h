#ifndef tiago_head_manager_ARTIFICIALLIFE_H
#define tiago_head_manager_ARTIFICIALLIFE_H

#include "resource_management/artificial_life/ArtificialLife.h"

#include <random>
#include <chrono>

namespace tiago_head_manager {

class ArtificialLife : public resource_management::ArtificialLife
{
public:
  explicit ArtificialLife(std::shared_ptr<resource_management::ReactiveBuffer> buffer);

private:
  virtual void inLoop();
  virtual void init();

  std::default_random_engine generator;
  std::poisson_distribution<int> poisson_distribution;
  std::normal_distribution<double> normal_distribution;
  std::chrono::time_point<std::chrono::system_clock> last_moved_time;
  int next_duration;

  double pitch;
  double yaw;

  double getNearValue(double previous_value, double max_delta);
};

} // namespace tiago_head_manager

#endif // tiago_head_manager_ARTIFICIALLIFE_H
