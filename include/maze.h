#ifndef maze_h
#define maze_h

#include <iostream>
#include <list>
#include <stack>
#include <vector>
#include <queue>
#include "cell.h"

using namespace std;

class maze
{
public:
	maze(size_t width, size_t height);	// Constructor
	cell& getCell(size_t x, size_t y);
	size_t getWidth() const;
	size_t getHeight() const;
	void drawMaze() const;

	list<pair<size_t, size_t>> solveMaze(pair<size_t, size_t> startPos, pair<size_t, size_t> endPos);

private:
	vector<vector<cell>> cells;
	size_t width, height;
};

#endif // !maze_h
