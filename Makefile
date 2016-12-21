PU_FOLDERS = fadeutils
PU_SRC = $(foreach folder,$(PU_FOLDERS),$(wildcard $(folder)/*.cpp))
PU_OBJ = $(PU_SRC:.cpp=.o)
PU_INCLUDES = $(addprefix -I,$(PU_FOLDERS))

CXX = g++
CXXFLAGS = -std=c++11 -pedantic -Wall -Wno-strict-aliasing -Wno-long-long -Wno-deprecated -Wno-deprecated-declarations -Werror
FAST_CXXFLAGS = -O3 -DNDEBUG
DEV_CXXFLAGS = -g -ggdb -O0
FADE2DFLAGS = -Ifade2d -Llib/ubuntu16.10_x86_64 -lfade2d -Wl,-rpath=lib/ubuntu16.10_x86_64

TARGETS = visualiser gridmap2poly

clean:
	rm -rf ./bin/*
	rm -f $(PU_OBJ)

$(TARGETS): $(PU_OBJ)
	$(CXX) $(CXXFLAGS) $(FADE2DFLAGS) $(PU_INCLUDES) $< $(@).cpp -o ./bin/$(@)

$(PU_OBJ):
	$(CXX) $(CXXFLAGS) $(FADE2DFLAGS) $(INCLUDES) $(@:.o=.cpp) -c -o $@
