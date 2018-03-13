TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    fuselage.cpp \
    parafoil.cpp \
    aileron.cpp \
    model.cpp

HEADERS += \
    algorithm.h \
    fuselage.h \
    def.math.h \
    def.physic.h \
    def.h \
    def.model.h \
    parafoil.h \
    aileron.h \
    model.h

LIBS += -llapack -lblas -larmadillo
