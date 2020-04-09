INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD

SOURCES += \
    $$PWD/src/environment_model.cpp \
    $$PWD/src/environment_visualisation.cpp \
    $$PWD/src/filament.cpp \
    $$PWD/src/filament_model.cpp \
    $$PWD/src/filament_simulator.cpp \
    $$PWD/src/gas_dispersion_model.cpp \
    $$PWD/src/inline_wind_model.cpp \
    $$PWD/src/main.cpp \
    $$PWD/src/occupancy.cpp \
    $$PWD/src/openvdb_environment_model.cpp \
    $$PWD/src/simulator.cpp \
    $$PWD/src/simulator_config.cpp \
    $$PWD/src/wind_model.cpp

HEADERS += \
    $$PWD/include/gaden_filament_simulator/environment_model.hpp \
    $$PWD/include/gaden_filament_simulator/environment_visualisation.hpp \
    $$PWD/include/gaden_filament_simulator/filament.h \
    $$PWD/include/gaden_filament_simulator/filament.hpp \
    $$PWD/include/gaden_filament_simulator/filament_model.hpp \
    $$PWD/include/gaden_filament_simulator/filament_simulator.h \
    $$PWD/include/gaden_filament_simulator/gas_dispersion_model.hpp \
    $$PWD/include/gaden_filament_simulator/inline_wind_model.hpp \
    $$PWD/include/gaden_filament_simulator/occupancy.hpp \
    $$PWD/include/gaden_filament_simulator/openvdb_environment_model.hpp \
    $$PWD/include/gaden_filament_simulator/simulator.hpp \
    $$PWD/include/gaden_filament_simulator/simulator_config.hpp \
    $$PWD/include/gaden_filament_simulator/wind_model.hpp
