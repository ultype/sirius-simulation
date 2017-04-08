TRICK_CFLAGS += --std=c++1z -I${TRICK_HOME}/trick_models -I../../../models/includes -g
TRICK_CXXFLAGS += --std=c++1z -I${TRICK_HOME}/trick_models -I../../../models/includes -g
TRICK_USER_LINK_LIBS += -larmadillo -lboost_serialization -lbcm2835
MAKEFLAGS += -j16
