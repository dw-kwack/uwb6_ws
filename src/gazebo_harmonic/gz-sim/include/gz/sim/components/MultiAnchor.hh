////////// 2025.08.12 by Noh begin
#ifndef GZ_SIM_COMPONENTS_MULTIANCHOR_HH_
#define GZ_SIM_COMPONENTS_MULTIANCHOR_HH_

#include <sdf/Sensor.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Serialization.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
  /// \brief A component type that contains a MultiAnchor sensor,
  /// sdf::MultiAnchor, information.
  using MultiAnchor = Component<sdf::Sensor, class MultiAnchorTag,
      serializers::SensorSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MultiAnchor", MultiAnchor)
}
}
}
}
#endif
////////// end
