TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += main.cpp

OTHER_FILES += \
    CMakeLists.txt

INCLUDEPATH += \
    /usr/include/pcl-1.7

INCLUDEPATH += \
    /opt/ros/hydro/include

HEADERS += \
    include/edge_detection.hpp \
    include/select_bin.hpp
