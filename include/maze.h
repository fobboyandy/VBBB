#ifndef maze_h
#define maze_h
#include "cell.h"
#include <vector>
#include <list>
#include <queue>
using namespace std;

class maze
{
public:
	maze(size_t width, size_t height) :width(width), height(height) //N x N maze
	{
		cells.resize(width);//adjust width
		for (size_t i = 0; i < width; i++)
		{
			cells[i].resize(height);//adjust height
			for (size_t j = 0; j < height; j++)//initialize maze to be all accessible
				cells[i][j] = cell(i, j, cell::cell_t::blank);
		}
	}
	cell& getCell(size_t x, size_t y) { return cells[x][y]; }
	size_t getWidth() const
	{
		return maze::width;
	}
	size_t getHeight() const
	{
		return maze::height;
	}
	//returns a list of x y cell (not pixel!) coordinates
	//function assumes valid start and end positions, else will return garbage solution
	
	list<pair<size_t, size_t>> solveMaze(pair<size_t, size_t> startPos, pair<size_t, size_t> endPos) //cell coordinates of starting and ending positions

private:
	vector<vector<cell>> cells;
	size_t width, height;
};
#endif // !maze_h
