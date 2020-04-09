#include <gaden_common/openvdb_helper.h>

#include <openvdb/openvdb.h>

namespace gaden {

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
