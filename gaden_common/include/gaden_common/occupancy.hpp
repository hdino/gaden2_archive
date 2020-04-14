#ifndef OCCUPANCY_HPP
#define OCCUPANCY_HPP

#include <cstdint>

namespace gaden {

enum class Occupancy : int32_t { Free = 0, Occupied = 1, Outlet = 2 };

} // namespace gaden

#endif // OCCUPANCY_HPP
