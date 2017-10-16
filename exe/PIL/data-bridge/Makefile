
CXX := g++
CPPFLAGS := -O2 -std=c++11
LDFLAGS :=

INCLUDES = -I../../../models/aux/include -I../../../models/sensor/include -I../../../models/driver/include

CPPFLAGS += $(INCLUDES)

all: databridge.cpp
	$(CXX) $(CPPFLAGS) $(LDFLAGS) -o databridge databridge.cpp
