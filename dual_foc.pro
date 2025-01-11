#-------------------------------------------------
#
# Project created by QtCreator 2021-04-07T18:26:54
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = dual_foc
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

DEFINES += QT_DLL QWT_DLL

#如果使用release下面这行要注释掉，文件夹下的所有.dll也不能是d.dll（d代表debug）
#LIBS += -L"D:\software\Qt\5.8\mingw53_32\lib" -lqwtd
LIBS += -L"D:\software\Qt\5.8\mingw53_32\lib" -lqwt
INCLUDEPATH += D:\software\Qt\5.8\mingw53_32\include\Qwt
INCLUDEPATH += D:\software\Eigen
LIBS += -lopengl32
LIBS += -lglu32
LIBS += -lglut
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        dual_foc.cpp \
    zeng_data_receive.cpp \
    Zeng_3d_display.cpp

HEADERS  += dual_foc.h \
    zeng_data_receive.h \
    Zeng_3d_display.h

FORMS    += dual_foc.ui
