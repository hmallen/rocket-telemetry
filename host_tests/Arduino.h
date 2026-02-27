#pragma once
#include <cstdint>
#include <cstring>
#include <iostream>
#include <algorithm>

using namespace std;

// PsramSpool uses min(). Standard Arduino uses macros which break STL containers.
// We use std::min/std::max via 'using namespace std' in the test file,
// but psram_spool.h might need them to be available without std:: qualification.
// Since 'using namespace std' is active, unqualified min() works.

class Print {
public:
    virtual size_t write(uint8_t) = 0;
};
class Stream : public Print {
public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
};
