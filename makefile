# https://stackoverflow.com/questions/30573481/path-include-and-src-directory-makefile/30602701

EXE = main

CC = g++
SRC_DIR = src
OBJ_DIR = obj

SRC = $(wildcard $(SRC_DIR)/*.cpp)

OBJ = $(SRC:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
#DEPENDS = $(SRC:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
#$(info $(SRC))

CPPFLAGS += -Iinclude -std=c++11 -Wno-narrowing
CFLAGS += -Wall #-MMD -MP
LDFLAGS += -Llib
LDLIBS += -pthread -lm -lwiringPi     

.PHONY: all clean

all: $(EXE)

$(EXE): $(OBJ)
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

#-include $(DEPENDS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

clean:
	$(RM) $(OBJ)
