CXX = g++
CXXFLAGS = -std=c++17 -O3 -Wall
TARGET = fp
SRCDIR = src
SRCS = $(SRCDIR)/main.cpp $(SRCDIR)/floorplanner.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(SRCDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
