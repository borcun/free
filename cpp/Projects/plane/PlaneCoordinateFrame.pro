TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    ABVisualModel.cpp \
    ABCalibration.cpp

HEADERS += \
    ABVisualModel.h \
    ABRobotScannerUtil.h \
    ABCalibration.h

LIBS += -lopencv_core \
        -lopencv_highgui \
        -lopencv_imgproc \
        -lopencv_calib3d \
        -losg \
        -losgViewer \
        -losgGA \
        -losgDB \


INCLUDEPATH += /usr/local/include
