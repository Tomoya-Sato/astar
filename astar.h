#include <string.h>
#include <stdlib.h>
#include <errno.h>

#define MAX_OPEN 1024

enum class Status
{
	NONE = 0,
	OPEN = 1,
	CLOSED = 2,
};

typedef struct _Coord {
	int x;
	int y;
}Coord;

typedef struct _Node {
	Status status;
	int map_cost;
	int r_cost;
	int e_cost;
	int score;
	Coord parent;
}Node;

class Astar {
 public:

	Astar(char input[256]) {
		input_fp_ = fopen(input, "r");
		if (input_fp_ == NULL) {
			perror("input_fp_ fopen");
			exit(1);
		}
	}

	~Astar() {
		fclose(input_fp_);
	}

	int GetScore(Coord nd) {
		return map_[nd.y][nd.x].score;
	}

	void InputMap();
	void OpenStart();
	Coord SearchStdNode();
	void OpenAround(Coord nd);
	bool CheckGoal();
	void DeleteMap();
	void debugPrintmap();
	void debugPrintStatus();
	void FindPath();
	
 private:
	
	void OpenNode(Coord nd, Coord p);
	void CloseNode(Coord nd);
	/* source file */
	FILE *input_fp_;

	/* map size */
	int x_range_;
	int y_range_;
	
	/* map info */
	Node **map_;
	
	/* open list */
	Coord openlist_[MAX_OPEN];
	int list_count_;

	/* start & end node */
	Coord start_;
	Coord goal_;

};
