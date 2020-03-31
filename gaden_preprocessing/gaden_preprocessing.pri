INCLUDEPATH += $$PWD/include
DEPENDPATH += $$PWD

SOURCES += \
    $$PWD/src/main.cpp \
    $$PWD/src/main_occupancy_converter.cpp \
    $$PWD/src/png_exporter.cpp \
    $$PWD/src/preprocessing.cpp \
    $$PWD/src/stl_data.cpp \
    $$PWD/src/stl_format.cpp \
    $$PWD/src/stl_to_grid.cpp \
    $$PWD/src/wind_converter.cpp

HEADERS += \
    $$PWD/include/gaden_preprocessing/png_exporter.h \
    $$PWD/include/gaden_preprocessing/preprocessing.h \
    $$PWD/include/gaden_preprocessing/stl_data.h \
    $$PWD/include/gaden_preprocessing/stl_format.h \
    $$PWD/include/gaden_preprocessing/stl_to_grid.h \
    $$PWD/include/gaden_preprocessing/vector_helper.h \
    $$PWD/include/gaden_preprocessing/wind_converter.h
