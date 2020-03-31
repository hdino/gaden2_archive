INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD

SOURCES += \
    $$PWD/src/cad_model.cpp \
    $$PWD/src/filesystem.cpp \
    $$PWD/src/gaden1_occupancy_grid_importer.cpp \
    $$PWD/src/occupancy_grid.cpp \
    $$PWD/src/ros_parameters.cpp \
    $$PWD/src/ros_type_helper.cpp

HEADERS += \
    $$PWD/include/gaden_common/cad_model.h \
    $$PWD/include/gaden_common/filesystem.h \
    $$PWD/include/gaden_common/gaden1_occupancy_grid_importer.h \
    $$PWD/include/gaden_common/math_helper.h \
    $$PWD/include/gaden_common/multidim_vector.h \
    $$PWD/include/gaden_common/occupancy_grid.h \
    $$PWD/include/gaden_common/occupancy_grid_type.h \
    $$PWD/include/gaden_common/ros_parameters.h \
    $$PWD/include/gaden_common/ros_type_helper.h
