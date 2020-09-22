INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD

SOURCES += \
    $$PWD/src/environment_model.cpp \
    $$PWD/src/environment_visualisation.cpp \
    $$PWD/src/farrells_wind_model.cpp \
    $$PWD/src/farrells_wind_model_noise.cpp \
    $$PWD/src/filament_grid.cpp \
    $$PWD/src/filament_model.cpp \
    $$PWD/src/filament_model_rviz.cpp \
    $$PWD/src/gas_dispersion_model.cpp \
    $$PWD/src/inline_wind_model.cpp \
    $$PWD/src/main.cpp \
    $$PWD/src/occupancy.cpp \
    $$PWD/src/openvdb_environment_model.cpp \
    $$PWD/src/sensor/insitu.cpp \
    $$PWD/src/sensor/open_path.cpp \
    $$PWD/src/sensor/sensor_base.cpp \
    $$PWD/src/simulator.cpp \
    $$PWD/src/simulator_config.cpp \
    $$PWD/src/wind_model.cpp

HEADERS += \
    $$PWD/include/gaden_filament_simulator/environment_model.hpp \
    $$PWD/include/gaden_filament_simulator/environment_visualisation.hpp \
    $$PWD/include/gaden_filament_simulator/farrells_wind_model.hpp \
    $$PWD/include/gaden_filament_simulator/farrells_wind_model_noise.hpp \
    $$PWD/include/gaden_filament_simulator/filament.hpp \
    $$PWD/include/gaden_filament_simulator/filament_grid.hpp \
    $$PWD/include/gaden_filament_simulator/filament_model.hpp \
    $$PWD/include/gaden_filament_simulator/filament_model_rviz.hpp \
    $$PWD/include/gaden_filament_simulator/gas_dispersion_model.hpp \
    $$PWD/include/gaden_filament_simulator/inline_wind_model.hpp \
    $$PWD/include/gaden_filament_simulator/occupancy.hpp \
    $$PWD/include/gaden_filament_simulator/openvdb_environment_model.hpp \
    $$PWD/include/gaden_filament_simulator/physical_constants.hpp \
    $$PWD/include/gaden_filament_simulator/sensor/insitu.hpp \
    $$PWD/include/gaden_filament_simulator/sensor/open_path.hpp \
    $$PWD/include/gaden_filament_simulator/sensor/sensor_base.hpp \
    $$PWD/include/gaden_filament_simulator/simulator.hpp \
    $$PWD/include/gaden_filament_simulator/simulator_config.hpp \
    $$PWD/include/gaden_filament_simulator/wind_model.hpp
