TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

DEFINES += "__cplusplus=201703L"

CONFIG += link_pkgconfig
PKGCONFIG += eigen3

INCLUDEPATH += \
    /opt/ros/eloquent/include \
    $$PWD/../install/octree/include \
    $$PWD/../install/OpenVDB/include \
    $$PWD/../install/rl_logging/include

include(gaden_common/gaden_common.pri)
include(gaden_environment/gaden_environment.pri)
include(gaden_filament_simulator/gaden_filament_simulator.pri)
include(gaden_preprocessing/gaden_preprocessing.pri)
include(olfaction_msgs/olfaction_msgs.pri)
