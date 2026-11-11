CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pthread

all: navigator

navigator: backend/main.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

run: navigator
	./navigator

clean:
	rm -f navigator

.PHONY: all run clean
