CXX = g++
CXXFLAGS = -I. -Wall -Wextra -g

# Output binary name
TARGET = test_psram_spool

# Source files
SRCS = test_psram_spool.cpp

# Default target
all: $(TARGET)

# Link
$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Clean
clean:
	rm -f $(TARGET)

# Run
run: $(TARGET)
	./$(TARGET)
