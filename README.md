# 2D Astar Seach

## Description

The shortest path on 2D map can be found by Astar Search. In this map, the move is limited to vertical and horizontal direction.

## How to Use

1. Clone this repository
2. "make" in the directory
3. Create mapfile or use "sample_map" (detail regulation of map is mentioned below)
4. Run "./main"
5. Type your map name

## How to Create Mapfile

Map is limited to rectangle.  
The first line is the x size and the y size of map.  
The second line is the coordinate of the start.  
The third line is the coordinate of the goal.  
The following is the move cost separated by ",". The block which has the move cost "0" can't be entered.  

For detail, please look at "sample_map.txt".
