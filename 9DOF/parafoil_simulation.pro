TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    fuselage.cpp \
    parafoil.cpp \
    model.cpp

HEADERS += \
    algorithm.h \
    fuselage.h \
    def/def.math.h \
    def/def.physic.h \
    def/def.h \
    def/def.model.h \
    parafoil.h \
    model.h \
    consumption.h

LIBS += -llapack -lblas -larmadillo
