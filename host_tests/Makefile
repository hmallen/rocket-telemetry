CXX = g++
CXXFLAGS = -Wall -Wextra -I. -I../src
SRC = ../src/crc32.cpp test_crc32.cpp
TARGET = run_tests

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

clean:
	rm -f $(TARGET) *.o

run: $(TARGET)
	./$(TARGET)
