CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -pthread
# Link the C++ runtime statically so the binary runs on any Linux image,
# even one with an older libstdc++ than the compiler used to build it.
LDFLAGS  ?= -static-libstdc++ -static-libgcc
 
all: navigator
 
navigator: backend/main.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)
 
run: navigator
	./navigator
 
clean:
	rm -f navigator
 
.PHONY: all run clean
 