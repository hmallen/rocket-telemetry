CC = g++
CFLAGS = -I. -I../src
TARGET = test_byte_ring

all: $(TARGET)

$(TARGET): test_byte_ring.cpp
	$(CC) $(CFLAGS) -o $(TARGET) test_byte_ring.cpp

clean:
	rm -f $(TARGET)
