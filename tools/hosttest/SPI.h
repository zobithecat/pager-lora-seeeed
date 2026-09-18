#pragma once
struct SPIMock { void begin(int, int, int, int) {} };
static SPIMock SPI;
