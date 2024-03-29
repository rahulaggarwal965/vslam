# Compiler
CC = g++
CPPFLAGS := -Iinclude -MMD -MP
CFLAGS := $(shell pkg-config --cflags opencv4 eigen3 pangolin) -Wall -std=c++11

# Linker
LDFLAGS := $(shell pkg-config --libs opencv4 glew pangolin)
LDLIBS := -pthread -lm

# Project Directories
SRC_DIR := src

# Project Files
TARGET := vslam
SRC := $(wildcard $(SRC_DIR)/*.cpp)
OBJ := $(SRC:$(SRC_DIR)/%.cpp=%.o)

# Debug Build
DBG_DIR := debug
DBG_TARGET := $(DBG_DIR)/$(TARGET)
DBG_OBJS := $(addprefix $(DBG_DIR)/, $(OBJ))
DBG_CFLAGS := -g -O0 -DDEBUG

# Release Build
REL_DIR := release
REL_TARGET := $(REL_DIR)/$(TARGET)
REL_OBJS := $(addprefix $(REL_DIR)/, $(OBJ))
REL_CFLAGS := -O3 -DNDEBUG

# Tests
TEST_DIR := tests
TEST_BIN_DIR := tests/bin
TESTS := $(wildcard $(TEST_DIR)/*.cpp)
TEST_TARGETS := $(addprefix $(TEST_BIN_DIR)/, $(TESTS:$(TEST_DIR)/%.cpp=%))

.PHONY: all clean debug release test remake

all: prepare release

# Debug Rules
debug: $(DBG_TARGET)

$(DBG_TARGET): $(DBG_OBJS)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(DBG_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CC) $(CPPFLAGS) $(CFLAGS) $(DBG_CFLAGS) -c $< -o $@

# Release Rules
release: $(REL_TARGET)

$(REL_TARGET): $(REL_OBJS)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(REL_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CC) $(CPPFLAGS) $(CFLAGS) $(REL_CFLAGS) -c $< -o $@

# Test Rules
test: $(TEST_TARGETS)

$(TEST_BIN_DIR)/%: $(TEST_DIR)/%.cpp
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $< $(LDFLAGS) $(LDLIBS)

# Other rules
prepare:
	@mkdir -p $(DBG_DIR) $(REL_DIR) $(TEST_DIR) $(TEST_BIN_DIR)

remake: clean all

clean:
	@$(RM) -rv $(REL_DIR) $(DBG_DIR)

-include $(OBJ:.o=.d)
