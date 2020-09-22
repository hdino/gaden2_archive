#include <gaden_common/occupancy.hpp>

namespace gaden {

std::string toString(Occupancy occupancy)
{
    switch (occupancy)
    {
    case Occupancy::Free: return "Free";
    case Occupancy::Occupied: return "Occupied";
    case Occupancy::Outlet: return "Outlet";
    case Occupancy::OutOfWorld: return "OutOfWorld";
    default: return "Invalid";
    }
}

} // namespace gaden
