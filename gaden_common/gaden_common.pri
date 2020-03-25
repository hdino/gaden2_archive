INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD

SOURCES += \
    $$PWD/src/cad_model.cpp \
    $$PWD/src/ros_parameters.cpp \
    $$PWD/src/ros_type_helper.cpp

HEADERS += \
    $$PWD/include/gaden_common/cad_model.h \
    $$PWD/include/gaden_common/filesystem.h \
    $$PWD/include/gaden_common/ros_parameters.h \
    $$PWD/include/gaden_common/ros_type_helper.h
