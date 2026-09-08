# Compiler settings
CC = gcc
CFLAGS = -Ix86_64-w64-mingw32/include
LDFLAGS = -Lx86_64-w64-mingw32/lib -lSDL3

# Default source and output (can be overridden via command line)
SRC ?= main.c
OBJ = $(SRC:.c=.o)
EXE ?= main.exe

# Default build target
all: $(EXE)

# Link the executable
$(EXE): $(OBJ)
	$(CC) $(OBJ) -o $(EXE) $(LDFLAGS)

# Compile source files into object files
%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

# Clean up build files
clean:
	del /Q *.o *.exe

