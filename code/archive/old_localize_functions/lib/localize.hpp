
#ifndef INCLUDED_localize_HPP
#define INCLUDED_localize_HPP

//#include "apps/object_visualizer/window.hpp"
//#include "util/config.hpp"
#include <memory>

namespace localize
{
  static constexpr const char *k_enabled = "PerspectiveViewWindow/Enabled";
  static constexpr const char *k_resolution = "PerspectiveViewWindow/Resolution";
  static constexpr const char *k_solver = "PerspectiveViewWindow/PnPSolverAlgorithm";
  static constexpr const char *k_ransac = "PerspectiveViewWindow/UseRANSAC";

  //std::shared_ptr<i_window> create(const util::config::Node &config);
}

#endif  // INCLUDED_localize_HPP
