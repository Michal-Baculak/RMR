#-------------------------------------------------
#
# Project created by QtCreator 2018-02-11T14:35:38
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
greaterThan(QT_MAJOR_VERSION, 5) {
    message(Using Qt6)
    DEFINES += USING_QT6
} else {
    message(Using Qt5)
    DEFINES += USING_QT5
}


include($$PWD/../compile_defines.pri)
!contains(DEFINES, DISABLE_JOYSTICK)
{
include ($$PWD/../QJoysticks-master/QJoysticks.pri)
}
TARGET = demoRMR
TEMPLATE = app
win32 {
LIBS += -lws2_32
LIBS += -lWinmm
}
INCLUDEPATH += ../librobot
LIBS += -L../bin -llibrobot

!contains(DEFINES, DISABLE_OPENCV) {
    INCLUDEPATH += C:/opencv/include/

win32 {
contains(DEFINES, USING_QT6) {
message(LINKING OPENCV in QT6)  # Debugging message
INCLUDEPATH += C:/opencv_vc17/include/

LIBS +=-LC:/opencv_vc17/x64/vc17/bin
LIBS +=-LC:/opencv_vc17/x64/vc17/lib

CONFIG(debug, debug|release) {
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_core4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_highgui4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_imgcodecs4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_imgproc4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_features2d4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_calib3d4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_videoio4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_ml4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_dnn4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_flann4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_objdetect4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_photo4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_shape4100d
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_video4100d
}
else {
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_core4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_highgui4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_imgcodecs4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_imgproc4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_features2d4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_calib3d4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_videoio4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_ml4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_dnn4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_flann4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_objdetect4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_photo4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_shape4100
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib/ -lopencv_video4100
}
}
else
{
message(LINKING OPENCV in QT5)  # Debugging message
    INCLUDEPATH += C:/opencv_vc16/include/

    LIBS +=-LC:/opencv_vc16/bin
    LIBS +=-LC:/opencv_vc16/lib

    CONFIG(debug, debug|release) {
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_core440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_highgui440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_imgcodecs440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_imgproc440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_features2d440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_calib3d440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_videoio440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_ml440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_dnn440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_flann440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_objdetect440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_photo440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_shape440d
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_video440d
    }
    else {
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_core440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_highgui440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_imgcodecs440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_imgproc440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_features2d440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_calib3d440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_videoio440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_ml440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_dnn440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_flann440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_objdetect440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_photo440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_shape440
        LIBS +=-LC:/opencv_vc16/lib/ -lopencv_video440
    }
    }
}
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
unix {
    PKGCONFIG += opencv4

    INCLUDEPATH += /usr/include/opencv4/
    INCLUDEPATH += /usr/local/include/opencv4/

    LIBS += -L/usr/local/lib/        \
        -l:libopencv_core.so       \
        -l:libopencv_highgui.so    \
        -l:libopencv_imgcodecs.so  \
        -l:libopencv_imgproc.so    \
        -l:libopencv_features2d.so\
        -l:libopencv_calib3d.so    \
        -l:libopencv_videoio.so    \
        -l:libopencv_ml.so         \
        -l:libopencv_dnn.so        \
        -l:libopencv_flann.so      \
        -l:libopencv_objdetect.so \
        -l:libopencv_photo.so      \
        -l:libopencv_video.so
}

}
SOURCES += main.cpp\
        mainwindow.cpp \
        robot.cpp

HEADERS  += mainwindow.h \
    robot.h

FORMS    += mainwindow.ui
message(DEFINES in demoRMR: $$DEFINES)  # Debugging message
