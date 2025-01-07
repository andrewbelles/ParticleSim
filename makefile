CC = gcc
CXX = g++
CFLAGS = -g -Wall -Iinclude/ -I/mingw64/include
CXXFLAGS = $(CFLAGS)
LDFLAGS = -L/mingw64/lib -lSDL2 -lglew32 -lopengl32

EXEC = simulator
C_SRC = $(wildcard src/*.c)
CXX_SRC = $(wildcard src/*.cpp)
OBJ = $(C_SRC:src/%.c=bin/%.o) $(CXX_SRC:src/%.cpp=bin/%.o)

all: $(EXEC)

$(EXEC): $(OBJ)
	$(CXX) $^ -o $@ $(LDFLAGS)

bin/%.o: src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

bin/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	del /Q bin\*.o $(EXEC).exe

.PHONY: all clean