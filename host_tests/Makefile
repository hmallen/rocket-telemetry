CXX = g++
CXXFLAGS = -Wall -I. -I../src -g -std=c++11
LDFLAGS =

# Sources
SRCS = test_gnss_ubx.cpp ../src/gnss_ubx.cpp

# Objects
OBJS = $(SRCS:.cpp=.o)

# Target
TARGET = run_tests

# Build rules
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
