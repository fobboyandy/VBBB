#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <list>
#include <time.h>
#include <queue>
#include <stack>
#include "Object.h"
#include "SerialClass.h"
#include <string>
#include <conio.h>

#define CW_INDEX 1
#define INIT_WORD 0xF0
#define END_WORD '\n'

#define THRESH	100
#define MAX_THRESH	255

using namespace cv;
using namespace std;

//default capture width and height
const int FRAME_WIDTH = 640 >> 1;
const int FRAME_HEIGHT = 480 >> 1;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;

void morphOps(Mat &thresh) {

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}

//returns true when obj is valid
bool trackObject(Object& obj, const Mat& src)
{

	Mat HSV, threshold;
	Object o;

	cvtColor(src, HSV, COLOR_BGR2HSV);
	inRange(HSV, Scalar(0, 16, 0), Scalar(7, 256, 256), threshold);
	morphOps(threshold);

	vector <Object> objects;
	Object object;
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(threshold, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	bool objectFound = false;
	if (hierarchy.size() > 0) 
	{
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects < MAX_NUM_OBJECTS)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area > MIN_OBJECT_AREA)
				{

					object.setXPos(moment.m10 / area);
					object.setYPos(moment.m01 / area);
					object.setArea(area);
					objects.push_back(object);
					objectFound = true;
					obj = object;
				}
				else 
					objectFound = false;
			}
		}
	}
	return objectFound;
}

//enter Mat x y position 
void moveTo(Mat& src, int x, int y)
{
	Object o;
	/// Show the image
	if (trackObject(o, src))
	{
		cv::circle(src, cv::Point(o.getXPos(), o.getYPos()), 10, cv::Scalar(255, 0, 255));
		cout << o.getArea() << "\t" << sqrt((o.getArea() / CV_PI))*2 << endl;
	}
}

class ball
{
public:
	ball(const Mat &src)
	{
		Mat src_out;
		cvtColor(src, src_out, COLOR_BGR2HSV);
		inRange(src_out, Scalar(97, 72, 16), Scalar(130, 255, 255), src_out); //get only blue component
		blur(src_out, src_out, Size(3, 3));

		Mat canny_output;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		/// Detect edges using canny
		Canny(src_out, canny_output, THRESH, THRESH * 2, 3);
		/// Find contours
		findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		/// Get the moments
		if (contours.size() > 0)
		{
			vector<Moments> mu(contours.size());
			Moments max = moments(contours[0], false); //initialize maximum
			for (int i = 0; i < contours.size(); i++)
			{
				mu[i] = moments(contours[i], false);
				if (mu[i].m00 > max.m00)
					max = mu[i];						//finds the largest orange blob
			}

			x = (int)(max.m10 / max.m00);			//truncate to size_t
			y = (int)(max.m01 / max.m00);

			//determine ball diameter
			diameter = (int)(max.m00);
		}
		else
		{
			x = -1;
			y = -1;
			diameter = -1;
		}
		cout << "x " << x << endl;
		cout << "y " << y << endl;
		cout << "diameter " << diameter << endl;

	}
	void updatePosition()
	{

	}
	pair<size_t, size_t> getPosition()
	{
		return pair<size_t, size_t>(x, y);
	}

	size_t getDiameter()
	{
		return diameter;
	}
private:
	int x, y;	//position of the ball
	int diameter;
};


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
	maze(const Mat& src)
	{
		//filter out only get black color
		Object o;
		Mat srcWalls;

		cvtColor(src, srcWalls, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		inRange(srcWalls, Scalar(0, 0, 0), Scalar(179, 255, 71), srcWalls); //Threshold the image
		size_t ballSize;
		if (!trackObject(o, src))
		{
			valid = false;
			return;
		}

		valid = true;
		ballSize = sqrt((o.getArea() / CV_PI)) * 2;	// b.getDiameter(); //assign a temp value for now
														//determine maze dimensions based on ball size
	
		width = srcWalls.cols % ballSize > 0 ? width = srcWalls.cols / ballSize + 1 : width = srcWalls.cols / ballSize;
		height = srcWalls.rows % ballSize > 0 ? height = srcWalls.rows / ballSize + 1 : height = srcWalls.cols / ballSize;
		//construct maze graph structure
		cells.resize(width);//adjust width
		for (size_t i = 0; i < width; i++)
		{
			cells[i].resize(height);//adjust height
			for (size_t j = 0; j < height; j++)//initialize maze to be all accessible
				cells[i][j] = cell(i, j, cell::cell_t::blank);
		}

		//iterate through the pixels and look for walls
		for (size_t pixelY = 0; pixelY < srcWalls.rows; pixelY++)
		{
			for (size_t pixelX = 0; pixelX < srcWalls.cols; pixelX++)
			{
				//determine logical coordinate
				size_t cellX = pixelX / ballSize;
				size_t cellY = pixelY / ballSize;
				if ((size_t)srcWalls.at<uchar>(pixelY, pixelX) > 0) //if pixel is a wall; 255 is black; 0 white
				{
					cells[cellX][cellY] = cell(cellX, cellY, cell::cell_t::wall);//mark inaccessible
					pixelX += ballSize - (pixelX % ballSize); //skip to the next cell on the same row
				}
			}
		}

		//get starting and ending location based on ball location
		//solve the maze and store result in private member

	}
	size_t getWidth() const
	{
		return maze::width;
	}
	size_t getHeight() const
	{
		return maze::height;
	}

	//function assumes valid start and end positions, else will return garbage solution
	list<pair<size_t, size_t>> solveMaze(pair<size_t, size_t> startPos, pair<size_t, size_t> endPos) //cell coordinates of starting and ending positions
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
			if ((currXY.second) - 1 >= 0 && cells[currXY.first][currXY.second - 1].type != cell::cell_t::wall && !visited[currXY.first][currXY.second - 1]) //check for north cell
			{
				bfsQueue.push(pair<size_t, size_t>(currXY.first, currXY.second - 1));
				newSol = currSol;
				newSol.push_back(currXY);
				newSol.push_back(pair<size_t, size_t>(currXY.first, currXY.second - 1));
				solutions.push(newSol);
				visited[currXY.first][currXY.second - 1] = true;
			}
			if ((currXY.first) + 1 <= width - 1 && cells[currXY.first + 1][currXY.second].type != cell::cell_t::wall && !visited[currXY.first + 1][currXY.second]) //check for east cell
			{
				bfsQueue.push(pair<size_t, size_t>(currXY.first + 1, currXY.second));
				newSol = currSol;
				newSol.push_back(currXY);
				newSol.push_back(pair<size_t, size_t>(currXY.first + 1, currXY.second));
				solutions.push(newSol);
				visited[currXY.first + 1][currXY.second] = true;
			}
			if ((currXY.second) + 1 <= height - 1 && cells[currXY.first][currXY.second + 1].type != cell::cell_t::wall && !visited[currXY.first][currXY.second + 1]) //check for south cell
			{
				bfsQueue.push(pair<size_t, size_t>(currXY.first, currXY.second + 1));
				newSol = currSol;
				newSol.push_back(currXY);
				newSol.push_back(pair<size_t, size_t>(currXY.first, currXY.second + 1));
				solutions.push(newSol);
				visited[currXY.first][currXY.second + 1] = true;
			}
			if ((currXY.first) - 1 >= 0 && cells[currXY.first - 1][currXY.second].type != cell::cell_t::wall && !visited[currXY.first - 1][currXY.second]) //check for west cell
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

	cell getCell(size_t x, size_t y)
	{
		return cells[x][y];
	}

	bool isValid()
	{
		return valid;
	}
private:
	vector<vector<cell>> cells;
	size_t width, height;
	bool valid;
};


pair<float, float> getSpeed(pair<int, int> dxdy, double timeElapsed)
{
	pair<double, double> speed(0.0, 0.0);

	//update x speed
	if (abs(dxdy.first) > 2)
		speed.first = dxdy.first / timeElapsed;

	if (abs(dxdy.second) > 2)
		speed.second = dxdy.second / timeElapsed;

	return speed;
}




//	Init Word
//	 ________________
//	|1|1|1|1|0|0|0|0|
//
//	Control Word
//	NSWE used to control direction and bits 0 to 3 controls angular granularity of servo
//	________________
//	|N|S|W|E|3|2|1|0|
//

//	End Word - New Line Character
//	


class labyrinth
{
private:
	typedef enum
	{
		UP = 0x80,
		DOWN = 0x40,
		LEFT = 0x20,
		RIGHT = 0x10
	}dir_t;
	Serial& serialConnection;
public:
	labyrinth(Serial& serialConnection) : serialConnection(serialConnection)
	{

	}

	bool moveBall(dir_t dir, uint8_t speed)
	{
		char control_packet[3] = { INIT_WORD, 0, END_WORD };
		control_packet[1] = dir | speed;
		return serialConnection.WriteData(control_packet, 3);
	}
	
	bool moveUp(uint8_t speed)
	{
		return moveBall(UP, speed);
	}
	bool moveDown(uint8_t speed)
	{
		return moveBall(DOWN, speed);
	}
	bool moveLeft(uint8_t speed)
	{
		return moveBall(LEFT, speed);
	}
	bool moveRight(uint8_t speed)
	{
		return moveBall(RIGHT, speed);
	}
	bool isConnected()
	{
		return serialConnection.IsConnected();
	}
};



//
/////** @function main */
int main(int argc, char** argv)
{
	//establish a serial connection
	Serial s("COM4");
	//create a labyrinth with the serial connection
	labyrinth l(s);


	//Matrix to store each frame of the webcam feed
	Mat cameraFeed;
	int prevX, prevY;
	int currX, currY;
	int dX, dY;
	float seconds;
	clock_t begin_time;

	Object o;
	//video capture object to acquire webcam feed
	VideoCapture capture;
	//open capture object at location zero (default location for webcam)
	capture.open(0);
	//set height and width of capture frame
	capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop
	
	waitKey(1000);

	capture >> cameraFeed;
	trackObject(o, cameraFeed);
	prevX = o.getXPos();
	prevY = o.getYPos();

	currX = prevX;
	currY = prevY;
	
	while (1)
	{
		if (_kbhit())
		{
			switch (_getch())
			{
			case('w'):
				l.moveUp(5);
				break;
			case('s'):
				l.moveDown(5);
				break;
			case('a'):
				l.moveLeft(5);
				break;
			case('d'):
				l.moveRight(5);
				break;
			default:
				break;
			}
		}
		begin_time = clock();
		capture >> cameraFeed;
		trackObject(o, cameraFeed);
		currX = o.getXPos();
		currY = o.getYPos();

	
		dX = currX - prevX;
		//update prevX
		prevX = currX;
		

		dY = currY - prevY;
		//update prevY
		prevY = currY;
		
		imshow("cam", cameraFeed);
		waitKey(30);


		seconds = float(clock() - begin_time) / CLOCKS_PER_SEC;
		pair <double, double> speed = getSpeed(pair<int, int>(dX, dY), seconds);

		printf("X Speed =\t%.2lf\t Y Speed =\t%.2lf\t\t\n", speed.first, speed.second);

	}


	system("pause");
	return 0;
}
