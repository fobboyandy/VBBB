#include "maze.h"

list<pair<size_t, size_t>> maze::solveMaze(pair<size_t, size_t> startPos, pair<size_t, size_t> endPos) //cell coordinates of starting and ending positions
{
	queue<pair<size_t, size_t>> bfsQueue;
	queue<list<pair<size_t, size_t>>> solutions;

	vector<vector<bool>> visited;
	visited.resize(width);
	for (size_t i = 0; i < width; i++) //initialize visited to false
	{
		visited[i].resize(height);
		for (size_t j = 0; j < height; j++)
			visited[i][j] = false;
	}
	list<pair<size_t, size_t>> currSol;
	bfsQueue.push(startPos);
	visited[startPos.first][startPos.second] = true;
	while (!bfsQueue.empty())
	{
		pair<size_t, size_t> currXY = bfsQueue.front();
		bfsQueue.pop();
		if (!solutions.empty())
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
		if ((currXY.second) - 1 >= 0 && getCell(currXY.first, currXY.second - 1).type != cell::cell_t::wall && !visited[currXY.first][currXY.second - 1]) //check for north cell
		{
			bfsQueue.push(pair<size_t, size_t>(currXY.first, currXY.second - 1));
			newSol = currSol;
			newSol.push_back(currXY);
			newSol.push_back(pair<size_t, size_t>(currXY.first, currXY.second - 1));
			solutions.push(newSol);
			visited[currXY.first][currXY.second - 1] = true;
		}
		if ((currXY.first) + 1 <= width - 1 && getCell(currXY.first + 1, currXY.second).type != cell::cell_t::wall && !visited[currXY.first + 1][currXY.second]) //check for east cell
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