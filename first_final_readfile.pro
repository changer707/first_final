QT += core gui
QT += multimediawidgets
QT += multimedia
QT += network
QT += sql
QT += charts

HEADERS += \
    adjpara.h \
    connection.h \
    dec_match.h \
    dection.h \
    defthreshold.h \
    distance.h \
    getline.h \
    header.h \
    improcess.h \
    kalman_filter.h \
    matrix.h \
    missingbox.h \
    mulplayer.h \
    munkres.h \
    my_pthread.h \
    mybox.h \
    paint.h \
    popvideo.h \
    tracker.h \
    utils.h

SOURCES += \
    adjpara.cpp \
    dec_match.cpp \
    distance.cpp \
    getline.cpp \
    impress.cpp \
    judge.cpp \
    kalman_filter.cpp \
    main.cpp \
    match.cpp \
    matrix.cpp \
    mulplayer.cpp \
    munkres.cpp \
    my_pthread.cpp \
    mytry.cpp \
    paint.cpp \
    popvideo.cpp \
    read_or_output.cpp \
    tracker.cpp
INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv \
               /usr/local/include/opencv2\
#               /media/zykzyh/study/forth/darknet-master\
#              /media/zykzyh/study/forth/darknet-master/include
              /home/ps/zk/darknet-master-1\
              /home/ps/zk/darknet-master-1/include
#         /home/idriver/darknet\
#        /home/idriver/darknet/include

LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so    \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_imgcodecs.so\
      # /media/zykzyh/study/forth/darknet-master/libdarknet.so
     /home/ps/zk/darknet-master-1/libdarknet.so
    # /home/idriver/darknet/libdarknet.so
unix|win32: LIBS += -lopencv_video

unix|win32: LIBS += -lopencv_videoio



#unix|win32: LIBS += -ldarknet

#unix|win32: LIBS += -ldarknet

FORMS +=\
    adjpara.ui \
    mulplayer.ui \
    paint.ui \
    popvideo.ui

qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

