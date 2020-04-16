#include <gaden_common/openvdb_helper.h>

#include <openvdb/openvdb.h>

namespace gaden {

std::string toString(const openvdb::Coord &coord, size_t indention)
{
    (void)indention;
    return "[" + std::to_string(coord.x()) + ", "
               + std::to_string(coord.y()) + ", "
               + std::to_string(coord.z()) + "]";
}

void initialiseOpenVdb()
{
    static bool openvdb_initialised = false;

    if (!openvdb_initialised)
    {
        openvdb::initialize();
        openvdb_initialised = true;
    }
}

} // namespace gaden
