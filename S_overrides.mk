TRICK_CFLAGS += --std=c++1z -I${TRICK_HOME}/trick_models -Imodels/includes -g
TRICK_CXXFLAGS += --std=c++1z -I${TRICK_HOME}/trick_models -Imodels/includes -g
TRICK_USER_LINK_LIBS += -larmadillo
MAKEFLAGS += -j8
