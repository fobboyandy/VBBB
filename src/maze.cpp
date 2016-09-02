#include "maze.h"

maze::maze(size_t width, size_t height) :width(width), height(height) //N x N maze
{
	cells.resize(width);//adjust width
	for (size_t i = 0; i < width; i++)
	{
		cells[i].resize(height);	//adjust height

									//initialize maze to be all accessible
		for (size_t j = 0; j < height; j++)
			cells[i][j] = cell(i, j, cell::cell_t::blank);
	}
}

cell & maze::getCell(size_t x, size_t y)
{
	return cells[x][y];
}

size_t maze::getWidth() const
{
	return maze::width;
}

size_t maze::getHeight() const
{
	return maze::height;
}

void maze::drawMaze() const
{
	for (size_t row = 0; row < height; row++)
	{
		for (size_t col = 0; col < width; col++)
		{
			switch (cells[col][row].type)
			{
			case cell::cell_t::wall:
				cout << 'X';
				break;
			case cell::cell_t::path:
				cout << 'O';
				break;
			default:
				cout << ' ';
			}
		}
		cout << endl;
	}
}

list<pair<size_t, size_t>> maze::solveMaze(pair<size_t, size_t> startPos, pair<size_t, size_t> endPos) // Cell coordinates of starting and ending positions
{
	queue<pair<size_t, size_t>> bfsQueue;
	queue<list<pair<size_t, size_t>>> solutions;

	vector<vector<bool>> visited;
	visited.resize(width);

	for (size_t i = 0; i < width; i++) // Initialize visited to false
	{
		visited[i].resize(height);
		for (size_t j = 0; j < height; j++)
			visited[i][j] = false;
	}

	list<pair<size_t, size_t>> currSol;
	bfsQueue.push(startPos);
	visited[startPos.first][startPos.second] = true;
	
	while(!bfsQueue.empty())
	{
		pair<int32_t, int32_t> currXY = bfsQueue.front();
		bfsQueue.pop();
		if(!solutions.empty())
		{
			currSol = solutions.front();
			solutions.pop();
		}
		
		list<pair<size_t, size_t>> newSol;
		if (currXY.first == endPos.first && currXY.second == endPos.second)
		{
			currSol.push_back(currXY);
			return currSol;
		}
		if((currXY.second - 1) >= 0 && getCell(currXY.first, currXY.second - 1).type != cell::cell_t::wall && !visited[currXY.first][currXY.second - 1]) //check for north cell
		{
			bfsQueue.push(pair<size_t, size_t>(currXY.first, currXY.second - 1));
			newSol = currSol;
			newSol.push_back(currXY);
			newSol.push_back(pair<size_t, size_t>(currXY.first, currXY.second - 1));
			solutions.push(newSol);
			visited[currXY.first][currXY.second - 1] = true;
		}
		if ((currXY.first + 1) <= width - 1 && getCell(currXY.first + 1, currXY.second).type != cell::cell_t::wall && !visited[currXY.first + 1][currXY.second]) //check for east cell
		{
			bfsQueue.push(pair<size_t, size_t>(currXY.first + 1, currXY.second));
			newSol = currSol;
			newSol.push_back(currXY);
			newSol.push_back(pair<size_t, size_t>(currXY.first + 1, currXY.second));
			solutions.push(newSol);
			visited[currXY.first + 1][currXY.second] = true;
		}
		if ((currXY.second) + 1 <= height - 1 && getCell(currXY.first, currXY.second + 1).type != cell::cell_t::wall && !visited[currXY.first][currXY.second + 1]) //check for south cell
		{
			bfsQueue.push(pair<size_t, size_t>(currXY.first, currXY.second + 1));
			newSol = currSol;
			newSol.push_back(currXY);
			newSol.push_back(pair<size_t, size_t>(currXY.first, currXY.second + 1));
			solutions.push(newSol);
			visited[currXY.first][currXY.second + 1] = true;
		}
		if ((currXY.first) - 1 >= 0 && getCell(currXY.first - 1, currXY.second).type != cell::cell_t::wall && !visited[currXY.first - 1][currXY.second]) //check for west cell
		{
			bfsQueue.push(pair<size_t, size_t>(currXY.first - 1, currXY.second));
			newSol = currSol;
			newSol.push_back(currXY);
			newSol.push_back(pair<size_t, size_t>(currXY.first - 1, currXY.second));
			solutions.push(newSol);
			visited[currXY.first - 1][currXY.second] = true;
		}
	}
	return currSol;
}

