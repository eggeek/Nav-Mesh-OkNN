MAKEFLAGS += -r
PA_FOLDERS = structs
PA_SRC = $(foreach folder,$(PA_FOLDERS),$(wildcard $(folder)/*.cpp))
PA_OBJ = $(PA_SRC:.cpp=.o)
PA_INCLUDES = $(addprefix -I,$(PA_FOLDERS))

CXX = g++
CXXFLAGS = -std=c++11 -pedantic -Wall -Wno-strict-aliasing -Wno-long-long -Wno-deprecated -Wno-deprecated-declarations -Werror
FAST_CXXFLAGS = -O3 -DNDEBUG
DEV_CXXFLAGS = -g -ggdb -O0

TARGETS = test
BIN_TARGETS = $(addprefix bin/,$(TARGETS))

all: $(TARGETS)

clean:
	rm -rf ./bin/*
	rm -f $(PA_OBJ)

.PHONY: $(TARGETS)
$(TARGETS): % : bin/%

$(BIN_TARGETS): bin/%: %.cpp $(PA_OBJ)
	@mkdir -p ./bin
	$(CXX) $(CXXFLAGS) $(PA_INCLUDES) $(PA_OBJ) $(@:bin/%=%).cpp -o $(@)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(PA_INCLUDES) $< -c -o $@
