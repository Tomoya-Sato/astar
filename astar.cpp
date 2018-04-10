#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <errno.h>
#include "astar.h"

#define LINE 1025

void Astar::InputMap() {
	char ss[LINE];

	if (fgets(ss, LINE-1, input_fp_) == NULL) {
		perror("can't read first line");
		exit(1);
	}

	sscanf(ss, "%d %d", &x_range_, &y_range_);

	if (fgets(ss, LINE-1, input_fp_) == NULL) {
		perror("can't read start node");
		exit(1);
	}

	sscanf(ss, "%d %d", &start_.x, &start_.y);

	if (fgets(ss, LINE-1, input_fp_) == NULL) {
		perror("can't read goal node");
		exit(1);
	}

	sscanf(ss, "%d %d", &goal_.x, &goal_.y);

	map_ = new Node*[y_range_];
	for (int i = 0; i < y_range_; i++) {
		map_[i] = new Node[x_range_];
	}

	for (int i = 0; i < y_range_; i++) {
		if (fgets(ss, LINE-1, input_fp_) == NULL) {
			perror("reading map score.");
			exit(1);
		}

		map_[i][0].map_cost = atoi(strtok(ss, ","));

		for (int j = 1; j < x_range_; j++) {
			map_[i][j].map_cost = atoi(strtok(NULL, ","));
			if (map_[i][j].map_cost > 0) map_[i][j].status = Status::NONE;
			else map_[i][j].status = Status::DIST;
		}
	}
}

void Astar::OpenNode(Coord nd, Coord p) {
	if (map_[nd.y][nd.x].status == Status::NONE) {
		map_[nd.y][nd.x].status = Status::OPEN;
		map_[nd.y][nd.x].r_cost = map_[p.y][p.x].r_cost + map_[nd.y][nd.x].map_cost;
		map_[nd.y][nd.x].e_cost = std::abs(goal_.x - nd.x) + std::abs(goal_.y - nd.y);
		map_[nd.y][nd.x].score = map_[nd.y][nd.x].r_cost + map_[nd.y][nd.x].e_cost;
		map_[nd.y][nd.x].parent.x = p.x;
		map_[nd.y][nd.x].parent.y = p.y;
		openlist_[list_count_] = nd;
		list_count_++;
	}
	else if (map_[nd.y][nd.x].status == Status::OPEN) {
		int tmp_r_cost, tmp_e_cost, tmp_score;
		tmp_r_cost = map_[p.y][p.x].r_cost + map_[nd.y][nd.x].map_cost;
		tmp_e_cost = map_[nd.y][nd.x].e_cost;
		tmp_score = tmp_r_cost + tmp_e_cost;
		if (tmp_score < map_[nd.y][nd.x].score) {
			map_[nd.y][nd.x].score = tmp_score;
			map_[nd.y][nd.x].r_cost = tmp_r_cost;
			map_[nd.y][nd.x].parent.x = p.x;
			map_[nd.y][nd.x].parent.y = p.y;
		}
	}
}

void Astar::CloseNode(Coord nd) {
	map_[nd.y][nd.x].status = Status::CLOSED;

	int flag = 1;
	for (int i = 0; i < list_count_; i++) {
		if (flag && (openlist_[i].x == nd.x) && (openlist_[i].y == nd.y)) {
			flag = 0;
			i++;
		}
		if (flag == 0) openlist_[i-1] = openlist_[i];
	}
	list_count_--;
}
			
void Astar::OpenStart() {
	map_[start_.y][start_.x].status = Status::OPEN;
	map_[start_.y][start_.x].r_cost = 0;
	map_[start_.y][start_.x].e_cost = std::abs(goal_.x - start_.x) + std::abs(goal_.y - start_.y);
	map_[start_.y][start_.x].score = map_[start_.y][start_.x].r_cost + map_[start_.y][start_.x].e_cost;
	map_[start_.y][start_.x].parent.x = -1;
	map_[start_.y][start_.y].parent.y = -1;
	openlist_[list_count_] = start_;
	list_count_++;
}

Coord Astar::SearchStdNode() {
	Coord tmp, ret;

	ret = openlist_[0];

	for (int i = 1; i < list_count_; i++) {
		tmp = openlist_[i];
		if (GetScore(ret) > GetScore(tmp)) ret = tmp;
	}

	return ret;
}

void Astar::OpenAround(Coord nd) {
	Coord tmp = nd;
	if (nd.x > 0) {
		tmp.x = nd.x - 1;
		if (map_[tmp.y][tmp.x].map_cost > 0)
			OpenNode(tmp, nd);
	}
	if (nd.x < x_range_ - 1) {
		tmp.x = nd.x + 1;
		if (map_[tmp.y][tmp.x].map_cost > 0)
			OpenNode(tmp, nd);
	}
	tmp.x = nd.x;
	if (nd.y > 0) {
		tmp.y = nd.y - 1;
		if (map_[tmp.y][tmp.x].map_cost > 0)		
			OpenNode(tmp, nd);
	}
	if (nd.y < y_range_ - 1) {
		tmp.y = nd.y + 1;
		if (map_[tmp.y][tmp.x].map_cost > 0)		
			OpenNode(tmp, nd);
	}

	CloseNode(nd);
	
}

bool Astar::CheckGoal() {
	if (map_[goal_.y][goal_.x].status == Status::NONE)
		return true;
	else
		return false;
}

void print_arrow(int x) {
	switch (x) {
	case 0:
		std::cout << "0";
		break;
	case 1:
		std::cout << "<";
		break;
	case -1:
		std::cout << ">";
		break;
	case 2:
		std::cout << "^";
		break;
	case -2:
		std::cout << "v";
		break;
	case 4:
		std::cout << "S";
		break;
	case 5:
		std::cout << "G";
		break;
	case 6:
		std::cout << "X";
		break;
	default:
		std::cout << x;
		break;
	}
}
			

void Astar::FindPath() {
	Coord path[x_range_*y_range_];
	int count = 1;
	Coord tmp = goal_;

	path[0] = tmp;
	
	while (map_[tmp.y][tmp.x].parent.x >= 0) {
		tmp = map_[tmp.y][tmp.x].parent;
		path[count] = tmp;
		count++;
	}

	int path_map[y_range_][x_range_];

	for (int i = 0; i < y_range_; i++) {
		for (int j = 0; j < x_range_; j++) {
			if (map_[i][j].map_cost == 0) path_map[i][j] = 6;
			else path_map[i][j] = 0;
		}
	}

	int total_cost = 0;

	path_map[path[0].y][path[0].x] = 5;
	for (int i = 1; i < count-1; i++) {
		total_cost += map_[path[i].x][path[i].y].map_cost;
		path_map[path[i].y][path[i].x] = (path[i].x - path[i-1].x) + (path[i].y - path[i-1].y) * 2;
	}

	total_cost += map_[path[count-1].x][path[count-1].y].map_cost;
	path_map[path[count-1].y][path[count-1].x] = 4;

	std::cout << std::endl << "the shortest path." << std::endl;

	for (int i = 0; i < y_range_; i++) {
		for (int j = 0; j < x_range_; j++) {
			print_arrow(path_map[i][j]);
			std::cout << " ";
		}
		std::cout << std::endl;
	}
	std::cout << "total cost: " << total_cost << std::endl;
}					

void Astar::DeleteMap() {
	for (int i = 0; i < y_range_; i++) {
		delete[] map_[i];
	}
	delete[] map_;
	map_ = 0;
}

void Astar::debugPrintmap() {
	for (int i = 0; i < y_range_; i++) {
		for (int j = 0; j < x_range_; j++) {
			std::cout << map_[i][j].map_cost << " ";
		}
		std::cout << std::endl;
	}
}

void Astar::debugPrintStatus() {
	std::cout << " ------------------------------------- " << std::endl;
	for (int i = 0; i < y_range_; i++) {
		for (int j = 0; j < x_range_; j++) {
			if (map_[i][j].status == Status::NONE)
				std::cout << "NONE\t";
			else if (map_[i][j].status == Status::OPEN)
				std::cout << "OPEN\t";
			else if (map_[i][j].status == Status::CLOSED)
				std::cout << "CLOSED\t";
			else
				std::cout << "DIST\t";
		}
		std::cout << std::endl;				
		for (int j = 0; j < x_range_; j++) {
			std::cout << map_[i][j].map_cost << "\t";
		}
		std::cout << std::endl;
		for (int j = 0; j < x_range_; j++) {
			std::cout << map_[i][j].r_cost << "\t";
		}
		std::cout << std::endl;
		for (int j = 0; j < x_range_; j++) {
			std::cout << map_[i][j].e_cost << "\t";
		}
		std::cout << std::endl;
		for (int j = 0; j < x_range_; j++) {
			std::cout << map_[i][j].score << "\t";
		}
		std::cout << std::endl << std::endl;
	}
	std::cout << " ------------------------------------- " << std::endl;	
}


