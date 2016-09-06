#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <windows.h>
#include "maze.h"

#define THRESH	100
#define MAX_THRESH	255

using namespace cv;
using namespace std;

Mat src, src_gray;

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
		Canny(src_gray, canny, THRESH, THRESH * 2, 3);

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

		// Solve Maze
		list<pair<size_t, size_t>> solution = m.solveMaze(pair<size_t, size_t>(3,3), pair<size_t, size_t>(9,25));

		// Add solution path
		if (solution.size() > 0)
		{
			for (auto it = solution.begin(); it != solution.end(); it++)
			{
				// cout << it->first << ' ' << it->second << endl;
				m.getCell(it->first, it->second).type = cell::cell_t::path;
			}
		}

		m.drawMaze();

		Sleep(100);
		system("pause");



	}

	return 0;
}
