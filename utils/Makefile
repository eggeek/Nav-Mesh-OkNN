MAKEFLAGS += -r
PU_FOLDERS = fadeutils
PU_SRC = $(foreach folder,$(PU_FOLDERS),$(wildcard $(folder)/*.cpp))
PU_OBJ = $(PU_SRC:.cpp=.o)
PU_INCLUDES = $(addprefix -I,$(PU_FOLDERS))

CXX = g++
CXXFLAGS = -std=c++11 -pedantic -Wall -Wno-strict-aliasing -Wno-long-long -Wno-deprecated -Wno-deprecated-declarations -Werror
FAST_CXXFLAGS = -O3 -DNDEBUG
DEV_CXXFLAGS = -g -ggdb -O0
FADE2DFLAGS = -Ifade2d -Llib/ubuntu16.10_x86_64 -lfade2d -Wl,-rpath=lib/ubuntu16.10_x86_64

TARGETS = visualiser poly2mesh
BIN_TARGETS = $(addprefix bin/,$(TARGETS))

all: $(TARGETS) gridmap2poly
fast: CXXFLAGS += $(FAST_CXXFLAGS)
dev: CXXFLAGS += $(DEV_CXXFLAGS)
fast dev: all

clean:
    rm -rf ./bin/*
    rm -f $(PU_OBJ)

.PHONY: $(TARGETS) gridmap2poly
$(TARGETS) gridmap2poly: % : bin/%

$(BIN_TARGETS): bin/%: %.cpp $(PU_OBJ)
    @mkdir -p ./bin
    $(CXX) $(CXXFLAGS) $(FADE2DFLAGS) $(PU_INCLUDES) $(PU_OBJ) $(@:bin/%=%).cpp -o $(@)

bin/gridmap2poly:
    @mkdir -p ./bin
    $(CXX) $(CXXFLAGS) gridmap2poly.cpp -o ./bin/gridmap2poly

%.o: %.cpp
    $(CXX) $(CXXFLAGS) $(FADE2DFLAGS) $(INCLUDES) $< -c -o $@
