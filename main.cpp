#include <iostream>
#include "astar.h"

void astar_callback() {
	char filename[256];
	std::cout << "input file name." << std::endl;
	std::cin >> filename;

	Astar hoge(filename);

	Coord std_node;

	hoge.InputMap();
	// hoge.debugPrintmap();
	// hoge.debugPrintStatus();
	hoge.OpenStart();
	// hoge.debugPrintStatus();	
	std_node = hoge.SearchStdNode();
	hoge.OpenAround(std_node);
	// hoge.debugPrintStatus();

	while (hoge.CheckGoal()) {
		std_node = hoge.SearchStdNode();
		hoge.OpenAround(std_node);
		// hoge.debugPrintStatus();
	}

	hoge.FindPath();
	hoge.DeleteMap();

}

int main()
{
	astar_callback();
	
	return 0;
}
