#include <iostream>
#include "astar.h"

void astar_callback() {
	char filename[256];
	std::cout << "input file name." << std::endl;
	std::cin >> filename;

	int mode;
	std::cout << "select mode." << std::endl;
	std::cout << "1: non-stop." << std::endl;
	std::cout << "2: step run." << std::endl;

	std::cin >> mode;

	while (mode != 1 && mode != 2) {
		std::cout << "please input 1 or 2." << std::endl;
		std::cin >> mode;
	}

	Astar hoge(filename);

	Coord std_node;

	hoge.InputMap();
	hoge.OpenStart();
	std_node = hoge.SearchStdNode();
	hoge.OpenAround(std_node);

	if (mode == 2) hoge.debugPrintStatus();

	while (hoge.CheckGoal()) {
		if (mode == 2) {
			std::cout << "1: finish stepping." << std::endl;
		    std::cout << "2: continue stepping." << std::endl;													 std::cin >> mode;
		}	
		std_node = hoge.SearchStdNode();
		hoge.OpenAround(std_node);
		if (mode == 2)hoge.debugPrintStatus();
	}

	hoge.FindPath();
	hoge.DeleteMap();

}

int main()
{
	astar_callback();
	
	return 0;
}
