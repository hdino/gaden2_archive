#find_package(Boost REQUIRED COMPONENTS system)
#list(APPEND gaden_common_LIBRARIES ${Boost_LIBRARIES})

get_filename_component(OPENVDB_CMAKE_DIR ${CMAKE_INSTALL_PREFIX}/../OpenVDB/lib/cmake/OpenVDB ABSOLUTE)
LIST(APPEND CMAKE_MODULE_PATH "${OPENVDB_CMAKE_DIR}")
