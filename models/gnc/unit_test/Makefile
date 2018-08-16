MKFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
GNC_DIR := $(patsubst %/unit_test/Makefile, %, $(MKFILE_PATH))
SIRIUS_HOME = $(patsubst %/models/gnc, %, $(GNC_DIR))
$(info MKFILE_PATH = $(MKFILE_PATH))
$(info GNC_PATH = $(GNC_DIR))
$(info SIRIUS_HOME = $(SIRIUS_HOME))
###### CXX flags #####
CXX = g++
CXXFLAGS = -Wall  --std=c++11 -g
CXXFLAGS += -I$(SIRIUS_HOME)/models/math/include\
		  -I$(SIRIUS_HOME)/models/cad/include\
		  -I$(GNC_DIR)/include\
		  -I$(SIRIUS_HOME)/models/aux/include
CXXLDLIB = -lgsl -lgslcblas -lm -larmadillo -lstdc++
###### C flags #####
CC = gcc
CFLAGS = -Wall -g -std=c11
CFLAGS += -I$(SIRIUS_HOME)/models/math/include\
		  -I$(GNC_DIR)/include\
		  -I$(SIRIUS_HOME)/models/cad/include
CLDLIB = -lm
##### CPP Source #####
MATH_CPP_SOURCES += $(SIRIUS_HOME)/models/math/src/matrix/utility.cpp
MATH_CPP_SOURCES += $(SIRIUS_HOME)/models/math/src/math_utility.cpp
MATH_CPP_SOURCES += $(SIRIUS_HOME)/models/cad/src/cad_utility.cpp
MATH_CPP_SOURCES += $(SIRIUS_HOME)/models/aux/src/Time_management.cpp
MATH_CPP_SOURCES += $(SIRIUS_HOME)/models/math/src/time_utility.cpp
MATH_CPP_SOURCES += $(SIRIUS_HOME)/models/math/src/integrate.cpp
GNC_TEST_CPP_SOURCES += $(GNC_DIR)/unit_test/unit_test.cpp
GNC_TEST_CPP_SOURCES += $(GNC_DIR)/src/Ins.cpp
##### C Source #####
MATH_C_SOURCE = $(SIRIUS_HOME)/models/math/src/math_utility_c.c
MATH_C_SOURCE += $(SIRIUS_HOME)/models/math/src/time_utility_c.c
MATH_C_SOURCE += $(SIRIUS_HOME)/models/cad/src/global_constants.c
MATH_C_SOURCE += $(SIRIUS_HOME)/models/cad/src/cad_utility_c.c
GNC_TEST_C_SOURCES = $(GNC_DIR)/src/Ins_c.c
GNC_TEST_C_SOURCES += $(GNC_DIR)/src/gnc_var.c
GNC_TEST_C_SOURCES += $(GNC_DIR)/src/dm_delta_ut.c
##### OBJECTS #####
MATH_OBJECTS += $(patsubst %.cpp, %.o, $(MATH_TEST_CPP_SOURCES))
MATH_OBJECTS += $(patsubst %.cpp, %.o, $(MATH_CPP_SOURCES))
MATH_C_OBJECTS = $(patsubst %.c, %.o, $(MATH_C_SOURCE))
GNC_OBJECTS = $(patsubst %.cpp, %.o, $(GNC_TEST_CPP_SOURCES))
GNC_OBJECTS += $(patsubst %.c, %.o, $(GNC_TEST_C_SOURCES))

all: gnctest

deps := $(MATH_OBJECTS:%.o=%.o.d) $(MATH_C_OBJECTS:%.o=%.o.d) $(GNC_OBJECTS:%.o=%.o.d)

%.o: %.cpp
	$(CXX) -c $< -o $@ $(CXXFLAGS)

%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

gnctest: $(MATH_OBJECTS) $(MATH_C_OBJECTS) $(GNC_OBJECTS)
	$(CXX) $(CXXFLAGS) $(MATH_OBJECTS) $(MATH_C_OBJECTS) $(GNC_OBJECTS) -o $@ $(CLDLIB) $(CXXLDLIB)

run: all
	./gnctest
.PHONY : clean
clean:
	rm -f  *.o gnctest
	find $(GNC_DIR)/src -name *.o -type f -delete
	find $(SIRIUS_HOME)/models/math/src/matrix -name *.o -type f -delete
	find $(SIRIUS_HOME)/models/cad/src -name *.o -type f -delete
	find $(SIRIUS_HOME)/models/aux/src -name *.o -type f -delete
	find ../../ -name *.d -type f -delete
