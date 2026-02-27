CXX = g++
CXXFLAGS = -I. -I../src -std=c++11 -Wall -Wextra -g

# Add any additional source files here if needed
SRCS = ../src/gnss_ubx.cpp
OBJS = $(SRCS:.cpp=.o)

# The test runner
TEST_SRC = test_ubx.cpp
TEST_EXE = test_runner

all: $(TEST_EXE)

$(TEST_EXE): $(TEST_SRC) $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(TEST_EXE) $(OBJS)

run: $(TEST_EXE)
	./$(TEST_EXE)
