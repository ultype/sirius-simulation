INCLUDES = -I${TRICK_HOME}/trick_models \
		   -I../../../models/gnc/include \
		   -I../../../models/dm/include \
		   -I../../../models/cad/include \
		   -I../../../models/math/include \
		   -I../../../models/aux/include \
		   -I../../../models/sensor/include \
		   -I../../../models/driver/include \
		   -I../../../models/icf/include

TRICK_CFLAGS += --std=c++11 ${INCLUDES} -g
TRICK_CXXFLAGS += --std=c++11 ${INCLUDES} -g
TRICK_USER_LINK_LIBS += -larmadillo -lboost_serialization
MAKEFLAGS += -j16
