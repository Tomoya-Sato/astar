main: main.o astar.o
	g++ -o $@ $^

astar.o: astar.cpp
	g++ -c -std=c++11 -o $@ $^

clean:
	rm -f *.o main
