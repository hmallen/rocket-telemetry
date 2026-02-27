CXX = g++
CXXFLAGS = -I. -I../src -std=c++14 -DDEBUG_MODE=1

# Mock objects needed for linking
MOCKS = Arduino.o RadioLib.o

# Source file to test
SRC = ../src/lora_link.cpp

all: run_tests

run_tests: test_lora_link
	./test_lora_link

test_lora_link: test_lora_link.o $(MOCKS) $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f *.o test_lora_link

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Dummy rules for mocks if they were separate files, but here headers suffice
Arduino.o: Arduino.h
	touch Arduino.o
RadioLib.o: RadioLib.h
	touch RadioLib.o
