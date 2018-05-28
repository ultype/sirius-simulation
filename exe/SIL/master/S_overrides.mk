MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
SIRIUS_HOME = $(patsubst %/exe/SIL/master/S_overrides.mk, %, $(MKFILE_PATH))

$(info MKFILE_PATH = $(MKFILE_PATH))
$(info SIRIUS_HOME = $(SIRIUS_HOME))

INCLUDES = -I${TRICK_HOME}/trick_models \
		   -I$(SIRIUS_HOME)/models/gnc/include \
		   -I$(SIRIUS_HOME)/models/dm/include \
		   -I$(SIRIUS_HOME)/models/cad/include \
		   -I$(SIRIUS_HOME)/models/math/include \
		   -I$(SIRIUS_HOME)/models/aux/include \
		   -I$(SIRIUS_HOME)/models/sensor/include \
		   -I$(SIRIUS_HOME)/models/driver/include \
		   -I$(SIRIUS_HOME)/models/icf/include \
		   -I$(SIRIUS_HOME)/models/equipment_protocol/include \
		   -I$(SIRIUS_HOME)/models/mission/include

TRICK_CFLAGS += --std=c++11 ${INCLUDES} -g -D_GNU_SOURCE -DCONFIG_SIL_ENABLE
TRICK_CFLAGS += -Wall -Wmissing-prototypes -Wextra -Wshadow
TRICK_CXXFLAGS += --std=c++11 ${INCLUDES} -g -DCONFIG_SIL_ENABLE
TRICK_CXXFLAGS += -Wall -Wextra -Wshadow
TRICK_USER_LINK_LIBS += -larmadillo -lboost_serialization
MAKEFLAGS += -j16
