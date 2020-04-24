INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD

SOURCES += \
    $$PWD/src/cad_model.cpp \
    $$PWD/src/eigen_helper.cpp \
    $$PWD/src/file_read_helper.cpp \
    $$PWD/src/filesystem.cpp \
    $$PWD/src/gaden1_occupancy_grid_importer.cpp \
    $$PWD/src/gas_source.cpp \
    $$PWD/src/grid_to_marker.cpp \
    $$PWD/src/inline_environment.cpp \
    $$PWD/src/interpolation.cpp \
    $$PWD/src/occupancy_grid.cpp \
    $$PWD/src/openvdb_box.cpp \
    $$PWD/src/openvdb_helper.cpp \
    $$PWD/src/playground.cpp \
    $$PWD/src/ros_parameters.cpp \
    $$PWD/src/ros_type_helper.cpp

HEADERS += \
    $$PWD/include/gaden_common/cache_grid.hpp \
    $$PWD/include/gaden_common/cad_model.h \
    $$PWD/include/gaden_common/chemical_substance.hpp \
    $$PWD/include/gaden_common/eigen_helper.hpp \
    $$PWD/include/gaden_common/file_read_helper.h \
    $$PWD/include/gaden_common/filesystem.h \
    $$PWD/include/gaden_common/gaden1_occupancy_grid_importer.h \
    $$PWD/include/gaden_common/gas_source.hpp \
    $$PWD/include/gaden_common/grid_helper.hpp \
    $$PWD/include/gaden_common/grid_to_marker.hpp \
    $$PWD/include/gaden_common/inline_environment.hpp \
    $$PWD/include/gaden_common/interpolation.hpp \
    $$PWD/include/gaden_common/math_helper.h \
    $$PWD/include/gaden_common/multidim_vector.h \
    $$PWD/include/gaden_common/occupancy.hpp \
    $$PWD/include/gaden_common/occupancy_grid.h \
    $$PWD/include/gaden_common/occupancy_grid_type.h \
    $$PWD/include/gaden_common/openvdb_box.hpp \
    $$PWD/include/gaden_common/openvdb_coordinates.hpp \
    $$PWD/include/gaden_common/openvdb_helper.h \
    $$PWD/include/gaden_common/ros_parameters.h \
    $$PWD/include/gaden_common/ros_type_helper.h \
    $$PWD/include/gaden_common/to_string.hpp
