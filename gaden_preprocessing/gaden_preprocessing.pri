INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD

SOURCES += \
    $$PWD/src/main.cpp \
    $$PWD/src/occupancy_grid.cpp \
    $$PWD/src/preprocessing.cpp \
    $$PWD/src/stl_data.cpp \
    $$PWD/src/stl_format.cpp

HEADERS += \
    $$PWD/include/gaden_preprocessing/occupancy_grid.h \
    $$PWD/include/gaden_preprocessing/occupancy_grid_type.h \
    $$PWD/include/gaden_preprocessing/preprocessing.h \
    $$PWD/include/gaden_preprocessing/stl_data.h \
    $$PWD/include/gaden_preprocessing/stl_format.h
