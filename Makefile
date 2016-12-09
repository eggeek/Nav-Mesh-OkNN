CXX = g++
CXXFLAGS = -std=c++11 -pedantic -Wall -Wno-strict-aliasing -Wno-long-long -Wno-deprecated -Wno-deprecated-declarations -Werror
FAST_CXXFLAGS = -O3 -DNDEBUG
DEV_CXXFLAGS = -g -ggdb -O0
FADE2DFLAGS = -Iinclude/fade2d -Llib/ubuntu16.10_x86_64 -lfade2d -Wl,-rpath=lib/ubuntu16.10_x86_64

# Uses Fade2D.
FADE2D = visualiser
# Does not use Fade2D.
NOFADE2D = gridmap2poly

$(FADE2D): CXXFLAGS += $(FADE2DFLAGS)
$(NOFADE2D):

.cpp:
	mkdir -p ./bin
	$(CXX) $(CXXFLAGS) $< -o ./bin/$@
