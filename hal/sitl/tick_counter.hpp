#pragma once

#include <stdint.h>

namespace sixsim::hal {

class SitlTickCounter {
 public:
  void advance(uint64_t ticks) { tick_count_ += ticks; }

  uint64_t read() const { return tick_count_; }

 private:
  uint64_t tick_count_{};
};

}  // namespace sixsim::hal
