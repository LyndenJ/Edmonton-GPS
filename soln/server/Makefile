#--------------------------------------------
#   Name: Lynden Jones
#   ID: 1661689
#   CCID: lgjones
#   CMPUT 275, Winter 2022
#   Assignment 1 Navigation System: Part 2
#--------------------------------------------
CC = g++
CC_FLAGS = -c -O2 -Wall -std=c++11 -g

BINARIES = server
OBJS = dijkstra.o server.o digraph.o

server: dijkstra.o server.o digraph.o
	$(CC) $(OBJS) -o server

dijkstra.o: dijkstra.cpp dijkstra.h
	$(CC) $(CC_FLAGS) dijkstra.cpp

server.o: server.cpp
	$(CC) $(CC_FLAGS) server.cpp

digraph.o: digraph.cpp digraph.h
	$(CC) $(CC_FLAGS) digraph.cpp

clean:
	rm -f $(BINARIES) $(OBJS)