INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD

SOURCES += \
    $$PWD/src/environment_visualisation.cpp \
    $$PWD/src/filament.cpp \
    $$PWD/src/filament_simulator.cpp \
    $$PWD/src/main.cpp \
    $$PWD/src/occupancy.cpp \
    $$PWD/src/simulator_config.cpp

HEADERS += \
    $$PWD/include/gaden_filament_simulator/environment_visualisation.hpp \
    $$PWD/include/gaden_filament_simulator/filament.h \
    $$PWD/include/gaden_filament_simulator/filament_simulator.h \
    $$PWD/include/gaden_filament_simulator/occupancy.hpp \
    $$PWD/include/gaden_filament_simulator/simulator_config.hpp
