TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

CONFIG += link_pkgconfig
PKGCONFIG += eigen3

INCLUDEPATH += \
    /opt/ros/eloquent/include \
    $$PWD/../install/OpenVDB/include

include(gaden_common/gaden_common.pri)
include(gaden_environment/gaden_environment.pri)
include(gaden_preprocessing/gaden_preprocessing.pri)
