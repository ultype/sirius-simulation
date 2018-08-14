
MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
CANNON_HOME = $(patsubst %/exe/HIL/fc_can_test/S_overrides.mk, %, $(MKFILE_PATH))

$(info MKFILE_PATH = $(MKFILE_PATH))
$(info CANNON_HOME = $(CANNON_HOME))

INCLUDES = -I${CANNON_HOME}/models/clock_source_mgmt \
		   -I$(CANNON_HOME)/models/gnc/include \
		   -I$(CANNON_HOME)/models/icf/include \
		   -I$(CANNON_HOME)/models/equipment_protocol/include

TRICK_CFLAGS += ${INCLUDES} -g -D_GNU_SOURCE -DCONFIG_FC_CAN_TEST_ENABLE
TRICK_CFLAGS += -Wall -Wmissing-prototypes -Wextra -Wshadow
TRICK_CXXFLAGS += --std=c++11 ${INCLUDES} -g
TRICK_CXXFLAGS += -Wall -Wextra -Wshadow -Wno-narrowing
TRICK_USER_LINK_LIBS += -larmadillo -lboost_serialization -lbiodaq
MAKEFLAGS += -j16
