#ifndef OCCUPANCY_HPP
#define OCCUPANCY_HPP

#include <cstdint>
#include <string>

namespace gaden {

enum class Occupancy : int32_t { Free = 0, Occupied = 1, Outlet = 2, OutOfWorld = 3 };

std::string toString(Occupancy occupancy);

} // namespace gaden

#endif // OCCUPANCY_HPP
