TRICK_CFLAGS += -I${TRICK_HOME}/trick_models -Imodels/includes -g
TRICK_CXXFLAGS += -I${TRICK_HOME}/trick_models -Imodels/includes -g
TRICK_USER_LINK_LIBS += -larmadillo
MAKEFLAGS += -j16
