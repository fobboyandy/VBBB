#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <list>
#include <stack>

using namespace cv;
using namespace std;

Mat src; Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

struct cell
{
	enum cell_t
	{
		blank,
		wall,
		path
	};
	cell() {};
	cell(size_t xC, size_t yC, cell_t cT) :x(xC), y(yC), type(cT) {};
	size_t x, y;
	cell_t type;
};

class maze 
{
public:
	maze(size_t width, size_t height):width(width), height(height) //N x N maze
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
	//assumes that start and end are valid
	//using iterative solution because rapsberry pi has less memory so a recursive solution may overflow stack
	list<pair<size_t, size_t>> solveMaze(size_t startX, size_t startY, size_t endX, size_t endY)
	{
		vector<vector<bool>> visited;
		stack<pair<size_t, size_t>> dfsPath;
		stack<pair<pair<size_t, size_t>, size_t>> currPath; //this is the solution from start to end the 
		list<pair<size_t, size_t>> solution;

		visited.resize(width);
		for (size_t i = 0; i < width; i++) //initialize all to false
		{
			visited[i].resize(height);
			for (size_t j = 0; j < height; j++)
				visited[i][j] = false;
		}

		//start dfs
		dfsPath.push(pair<size_t, size_t>(startX, startY));
		visited[startX][startY] = true;
		while (!dfsPath.empty())
		{
			size_t branches = 0; //tests for branches
			pair<size_t, size_t> currXY = dfsPath.top();
			if (currXY.first == endX && currXY.second == endY)
			{
				solution.push_front(pair<size_t, size_t>(currXY.first, currXY.second));//add the current location
				while (!currPath.empty())//add the rest of the solution
				{
					solution.push_front(pair<size_t, size_t>(currPath.top().first.first, currPath.top().first.second));//push_front because stack is reversed
					currPath.pop();
				}
				return solution;
			}
			dfsPath.pop();
			
			//push order is n e s w 
			//valid cell conditions: valid coordinate, not a wall, and not visited yet 
			//currXY.first = x value
			//currXY.second = y value
			if ((currXY.second) - 1 >= 0 && getCell(currXY.first, currXY.second - 1).type != cell::cell_t::wall && !visited[currXY.first][currXY.second - 1]) //check for north cell
			{
				dfsPath.push(pair<size_t, size_t>(currXY.first, currXY.second - 1));
				visited[currXY.first][currXY.second - 1] = true;
				branches++;
			}
			if ((currXY.first) + 1 <= width - 1 && getCell(currXY.first + 1, currXY.second).type != cell::cell_t::wall && !visited[currXY.first + 1][currXY.second]) //check for east cell
			{
				dfsPath.push(pair<size_t, size_t>(currXY.first + 1, currXY.second));
				visited[currXY.first + 1][currXY.second] = true;
				branches++;
			}
			if ((currXY.second) + 1 <= height - 1 && getCell(currXY.first, currXY.second + 1).type != cell::cell_t::wall && !visited[currXY.first][currXY.second + 1]) //check for south cell
			{
				dfsPath.push(pair<size_t, size_t>(currXY.first, currXY.second + 1));
				visited[currXY.first][currXY.second + 1] = true;
				branches++;
			}
			if ((currXY.first) - 1 >= 0 && getCell(currXY.first - 1, currXY.second).type != cell::cell_t::wall && !visited[currXY.first - 1][currXY.second]) //check for west cell
			{
				dfsPath.push(pair<size_t, size_t>(currXY.first - 1, currXY.second));
				visited[currXY.first - 1][currXY.second] = true;
				branches++;
			}
			currPath.push(pair<pair<size_t, size_t>, size_t>(currXY, branches));
			if (branches == 0) //deadend
			{
				while (currPath.top().first.second < 2) //pop back until previous branch
					currPath.pop();
				(currPath.top().second)--;
			}
		}

		return solution;
	}
private:
	vector<vector<cell>> cells;
	size_t width, height;
};



/** @function main */
int main(int argc, char** argv)
{
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		return -1;

	while(1)
	{
		Mat frame;
		//frame = imread("../x64/Debug/maze.png", 1);
		cap >> frame;

		// Load frame image and convert it to gray
		cvtColor(frame, src_gray, CV_BGR2GRAY);
		//detect edges
		Mat canny;
		Canny(src_gray, canny, thresh, thresh * 2, 3);

		//construct maze
		size_t ballSize = 30;
		size_t mazeWidth = canny.cols % ballSize > 0 ? mazeWidth = canny.cols / ballSize + 1 : mazeWidth = canny.cols / ballSize;
		size_t mazeHeight = canny.rows % ballSize > 0 ? mazeHeight = canny.rows / ballSize + 1 : mazeHeight = canny.cols / ballSize;
		maze m(mazeWidth, mazeHeight);

		//iterate through the pixels pX and pY
		for (size_t pixelY = 0; pixelY < canny.rows; pixelY++)
		{
			for (size_t pixelX = 0; pixelX < canny.cols; pixelX++)
			{ 
				size_t cellX = pixelX / ballSize; //x coordinate of cell is truncated result of division
				size_t cellY = pixelY / ballSize; //y ''
				//cout << pixelX << ' ' << pixelY << endl;
				//cout << (size_t)canny.at<uchar>(pixelY, pixelX) << endl;
				if ((size_t)canny.at<uchar>(pixelY, pixelX) == 255) //if pixel is a wall; 255 is black; 0 white
				{
					m.getCell(cellX, cellY) = cell(cellX, cellY, cell::cell_t::wall);//toggle on wall
					pixelX += ballSize - (pixelX % ballSize); //skip to the next cell on the same row
				}
			}
		}
		//list<pair<size_t, size_t>> solution = m.solveMaze(3, 3, 3, 16);
		/*if(solution.size() > 0)
			for (auto it = solution.begin(); it != solution.end(); it++)
			{
				cout << it->first << ' ' << it->second << endl;
				m.getCell(it->first, it->second).type = cell::cell_t::path;
			}*/
		for (size_t row = 0; row < m.getHeight(); row++)
		{
			for (size_t col = 0; col < m.getWidth(); col++)
			{
				if (m.getCell(col, row).type == cell::cell_t::wall)
					cout << 'X';
				else if (m.getCell(col, row).type == cell::cell_t::path)
					cout << 'O';
				else
					cout << ' ';
			}
			cout << endl;
		}
		
	}

	return 0;
}
