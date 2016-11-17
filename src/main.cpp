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

#define HALF_SIZE 1

#define ROI_X (45 >> HALF_SIZE)
#define ROI_Y (20 >> HALF_SIZE)

#define ROI_WIDTH (540 >> HALF_SIZE)
#define ROI_HEIGHT (435 >> HALF_SIZE)

#define SIZE_TOLERANCE (10 >> HALF_SIZE)


//default capture width and height
#define FRAME_WIDTH  (640 >> HALF_SIZE)
#define FRAME_HEIGHT  (480 >> HALF_SIZE)


using namespace cv;
using namespace std;

//minimum and maximum object area
const int MIN_OBJECT_AREA ((20*20) >> HALF_SIZE);


//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;

void morphOps(Mat &thresh) {

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	//erode(thresh, thresh, erodeElement);
	//erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}

//returns true when obj is valid
bool trackObject(Object& obj, const Mat& src)
{

	Mat HSV, threshold;
	static double largestArea = 0;
	cvtColor(src, HSV, COLOR_BGR2HSV);
	inRange(HSV, Scalar(175, 30, 60), Scalar(256, 256, 256), threshold);
	morphOps(threshold);
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
					largestArea = area;

					obj.setXPos(moment.m10 / area);
					obj.setYPos(moment.m01 / area);
					obj.setArea(area);
					objectFound = true;
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


pair<float, float> getSpeed(pair<int, int> dxdy, double dt)
{
	pair<double, double> speed(0.0, 0.0);

	//update x speed
	if (abs(dxdy.first) > 1)
		speed.first = dxdy.first / dt;

	if (abs(dxdy.second) > 1)
		speed.second = dxdy.second / dt;

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
		DEFAULT,
		UP = 0x8,
		DOWN = 0x4,
		LEFT = 0x2,
		RIGHT = 0x1
	}dir_t;
	Serial& serialConnection;
public:
	labyrinth(Serial& serialConnection) : serialConnection(serialConnection)
	{

	}

	bool moveBall(dir_t dir, uint8_t speed)
	{
		char control_packet[4] = { INIT_WORD, 0, 0, END_WORD };
		control_packet[1] = (uint8_t)(dir);
		control_packet[2] = speed;
		return serialConnection.WriteData(control_packet, 4);
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
	bool moveReset()
	{
		return moveBall(DEFAULT, 0x0);
	}

	bool turnNorthServo(int8_t angle)
	{
		char control_packet[4] = { INIT_WORD, 0, 0, END_WORD };
		control_packet[1] = (uint8_t)(0xF0);
		control_packet[2] = angle;
		return serialConnection.WriteData(control_packet, 4);
	}
	bool turnWestServo(int8_t angle)
	{
		char control_packet[4] = { INIT_WORD, 0, 0, END_WORD };
		control_packet[1] = (uint8_t)(0x0F);
		control_packet[2] = angle;
		return serialConnection.WriteData(control_packet, 4);
	}


	bool isConnected()
	{
		return serialConnection.IsConnected();
	}
};



int kPNorth = 0;
int kINorth = 0;
int kDNorth = 0;

int kPMax = 100;
int kIMax = 100;
int kDMax = 100;

void createTrackbarsNorth() {


	char trackbarWindowName[50] = "PID Coefficient Picker North";
	//create window for trackbars
	namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "kP", kPMax);
	sprintf(TrackbarName, "kI", kIMax);
	sprintf(TrackbarName, "kD", kDMax);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH),
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->
	createTrackbar("kP", trackbarWindowName, &kPNorth, kPMax, NULL);
	createTrackbar("kI", trackbarWindowName, &kINorth, kIMax, NULL);
	createTrackbar("kD", trackbarWindowName, &kDNorth, kDMax, NULL);
}


int kPWest = 0;
int kIWest = 0;
int kDWest = 0;

void createTrackbarsWest() {


	char trackbarWindowName[50] = "PID Coefficient Picker West";
	//create window for trackbars
	namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf(TrackbarName, "kP", kPMax);
	sprintf(TrackbarName, "kI", kIMax);
	sprintf(TrackbarName, "kD", kDMax);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH),
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->
	createTrackbar("kP", trackbarWindowName, &kPWest, kPMax, NULL);
	createTrackbar("kI", trackbarWindowName, &kIWest, kIMax, NULL);
	createTrackbar("kD", trackbarWindowName, &kDWest, kDMax, NULL);
}



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
	int proportionalSpeed;
	double dt;
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
	
	double prevErrorX = 0;
	double integralX = 0;
	double derivativeX;
	double errorX;
	double outputX;
	int setpointX = 70;



	double prevErrorY = 0;
	double integralY = 0;
	double derivativeY;
	double errorY;
	double outputY;
	int setpointY = 35;



	begin_time = clock();
	createTrackbarsNorth();
	createTrackbarsWest();
	while (1)
	{


		if (_kbhit())
		{
			switch (_getch())
			{
			case('w'):
				l.turnWestServo(90);
				break;
			case('s'):
				l.turnWestServo(-90);
				break;
			case('a'):
				l.turnNorthServo(-90);
				break;
			case('d'):
				l.turnNorthServo(90);
				break;
			default:
				l.turnWestServo(0);
				l.turnNorthServo(0);
				break;
			}
		}

		capture >> cameraFeed;
		cameraFeed = cameraFeed(Rect(ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT));
		if (trackObject(o, cameraFeed))
		{

			currX = o.getXPos();
			currY = o.getYPos();


			///pid stuff
			dt = float(clock() - begin_time) / CLOCKS_PER_SEC;

			errorX = setpointX - currX;
			integralX = integralX + errorX*dt;
			derivativeX = (errorX - prevErrorX) / dt;
			prevErrorX = errorX;
			outputX = ((double)kPNorth / 100 * errorX) + ((double)kINorth / 10000 * integralX) + ((double)kPNorth / 100 * derivativeX);


			errorY = setpointY - currY;
			integralY = integralY + errorY*dt;
			derivativeY = (errorY - prevErrorY) / dt;
			prevErrorY = errorY;
			outputY = ((double)kPWest / 100 * errorY) + ((double)kIWest / 10000 * integralY) + ((double)kDWest / 100 * derivativeY);




			printf("outputX =\t%.2lf\t outputY =\t%.2lf\t\t\n", outputX, outputY);
			waitKey(5);
			l.turnNorthServo((int8_t)outputX);
			l.turnWestServo((int8_t)-outputY);

			//record starting time
			begin_time = clock();

		}
		else
			cout << "NOT FOUND!" << endl;
		//printf("X Speed =\t%.2lf\t Y Speed =\t%.2lf\t\t\n", speed.first, speed.second);

	}


	//system("pause");
	return 0;
}


//Written by  Kyle Hounslow 2013

// modified by: Ahmad Kaifi, Hassan Althobaiti

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software")
//, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
//and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//IN THE SOFTWARE.
//
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include <iostream>
//#include <string>
//#include "Object.h"
//
//#define HALF_SIZE 1
//
//#define ROI_X (45 >> HALF_SIZE)
//#define ROI_Y (20 >> HALF_SIZE)
//
//#define ROI_WIDTH (540 >> HALF_SIZE)
//#define ROI_HEIGHT (435 >> HALF_SIZE)
//
//
//using namespace cv;
//using namespace std;
//
////initial min and max HSV filter values.
////these will be changed using trackbars
//int H_MIN = 0;
//int H_MAX = 256;
//int S_MIN = 0;
//int S_MAX = 256;
//int V_MIN = 0;
//int V_MAX = 256;
////default capture width and height
//const int FRAME_WIDTH = 640 >> HALF_SIZE;
//const int FRAME_HEIGHT = 480 >> HALF_SIZE;
////max number of objects to be detected in frame
//const int MAX_NUM_OBJECTS = 50;
////minimum and maximum object area
//const int MIN_OBJECT_AREA = (20 * 20) >> HALF_SIZE;
//const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 1.5;
////names that will appear at the top of each window
//const string windowName = "Original Image";
//const string windowName1 = "HSV Image";
//const string windowName2 = "Thresholded Image";
//const string windowName3 = "After Morphological Operations";
//const string trackbarWindowName = "Trackbars";
//
////The following for canny edge detec
//Mat dst, detected_edges;
//Mat src, src_gray;
//int edgeThresh = 1;
//int lowThreshold;
//int const max_lowThreshold = 100;
//int ratio = 3;
//int kernel_size = 3;
//const char* window_name = "Edge Map";
//
//void on_trackbar(int, void*)
//{//This function gets called whenever a
// // trackbar position is changed
//
//}
//
//string intToString(int number) {
//
//	std::stringstream ss;
//	ss << number;
//	return ss.str();
//}
//
//void createTrackbars() {
//	//create window for trackbars
//	namedWindow(trackbarWindowName, 0);
//	//create memory to store trackbar name on window
//	char TrackbarName[50];
//	sprintf(TrackbarName, "H_MIN", H_MIN);
//	sprintf(TrackbarName, "H_MAX", H_MAX);
//	sprintf(TrackbarName, "S_MIN", S_MIN);
//	sprintf(TrackbarName, "S_MAX", S_MAX);
//	sprintf(TrackbarName, "V_MIN", V_MIN);
//	sprintf(TrackbarName, "V_MAX", V_MAX);
//	//create trackbars and insert them into window
//	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
//	//the max value the trackbar can move (eg. H_HIGH),
//	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
//	//                                  ---->    ---->     ---->
//	createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
//	createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
//	createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
//	createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
//	createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
//	createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);
//}
//
//void drawObject(vector<Object> theObjects, Mat &frame, Mat &temp, vector< vector<Point> > contours, vector<Vec4i> hierarchy) {
//
//	for (int i = 0; i<theObjects.size(); i++) {
//		cv::drawContours(frame, contours, i, theObjects.at(i).getColor(), 3, 8, hierarchy);
//		cv::circle(frame, cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos()), 5, theObjects.at(i).getColor());
//		cv::putText(frame, intToString(theObjects.at(i).getXPos()) + " , " + intToString(theObjects.at(i).getYPos()), cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos() + 20), 1, 1, theObjects.at(i).getColor());
//		cv::putText(frame, theObjects.at(i).getType(), cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos() - 20), 1, 2, theObjects.at(i).getColor());
//	}
//}
//
//void drawObject(vector<Object> theObjects, Mat &frame) {
//
//	for (int i = 0; i<theObjects.size(); i++) {
//
//		cv::circle(frame, cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos()), 10, cv::Scalar(0, 0, 255));
//		cv::putText(frame, intToString(theObjects.at(i).getXPos()) + " , " + intToString(theObjects.at(i).getYPos()), cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos() + 20), 1, 1, Scalar(0, 255, 0));
//		cv::putText(frame, theObjects.at(i).getType(), cv::Point(theObjects.at(i).getXPos(), theObjects.at(i).getYPos() - 30), 1, 2, theObjects.at(i).getColor());
//	}
//}
//
//void morphOps(Mat &thresh) {
//
//	//create structuring element that will be used to "dilate" and "erode" image.
//	//the element chosen here is a 3px by 3px rectangle
//	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
//	//dilate with larger element so make sure object is nicely visible
//	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));
//
//	//erode(thresh, thresh, erodeElement);
//	//erode(thresh, thresh, erodeElement);
//
//	dilate(thresh, thresh, dilateElement);
//	dilate(thresh, thresh, dilateElement);
//}
//void trackFilteredObject(Mat threshold, Mat HSV, Mat &cameraFeed)
//{
//	vector <Object> objects;
//	Mat temp;
//	threshold.copyTo(temp);
//	//these two vectors needed for output of findContours
//	vector< vector<Point> > contours;
//	vector<Vec4i> hierarchy;
//	//find contours of filtered image using openCV findContours function
//	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
//	//use moments method to find our filtered object
//	double refArea = 0;
//	bool objectFound = false;
//	if (hierarchy.size() > 0) {
//		int numObjects = hierarchy.size();
//		Object largestObject;
//		double largestArea = 0;
//		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
//		if (numObjects<MAX_NUM_OBJECTS)
//		{
//			for (int index = 0; index >= 0; index = hierarchy[index][0])
//			{
//				Moments moment = moments((cv::Mat)contours[index]);
//				double area = moment.m00;
//
//				//if the area is less than 20 px by 20px then it is probably just noise
//				//if the area is the same as the 3/2 of the image size, probably just a bad filter
//				//we only want the object with the largest area so we safe a reference area each
//				//iteration and compare it to the area in the next iteration.
//				if (area > MIN_OBJECT_AREA && area > largestArea)
//				{
//					largestArea = area;
//					largestObject.setXPos(moment.m10 / area);
//					largestObject.setYPos(moment.m01 / area);
//					objectFound = true;
//
//				}
//				else objectFound = false;
//			}
//			//let user know you found an object
//			if (objectFound == true)
//			{
//				objects.clear();
//				objects.push_back(largestObject);
//				//draw object location on screen
//				drawObject(objects, cameraFeed);
//			}
//		}
//		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
//	}
//}
//
//void trackFilteredObject(Object theObject, Mat threshold, Mat HSV, Mat &cameraFeed) {
//
//	vector <Object> objects;
//	Mat temp;
//	threshold.copyTo(temp);
//	//these two vectors needed for output of findContours
//	vector< vector<Point> > contours;
//	vector<Vec4i> hierarchy;
//	//find contours of filtered image using openCV findContours function
//	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
//	//use moments method to find our filtered object
//	double refArea = 0;
//	bool objectFound = false;
//	if (hierarchy.size() > 0) {
//		double maxArea = 0;
//		int numObjects = hierarchy.size();
//		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
//		if (numObjects<MAX_NUM_OBJECTS) 
//		{
//			for (int index = 0; index >= 0; index = hierarchy[index][0]) 
//			{
//
//				Moments moment = moments((cv::Mat)contours[index]);
//				double area = moment.m00;
//
//
//				//if the area is less than 20 px by 20px then it is probably just noise
//				//if the area is the same as the 3/2 of the image size, probably just a bad filter
//				//we only want the object with the largest area so we safe a reference area each
//				//iteration and compare it to the area in the next iteration.
//				if (area>MIN_OBJECT_AREA && area > maxArea) 
//				{
//
//					Object object;
//					maxArea = area;
//					object.setXPos(moment.m10 / area);
//					object.setYPos(moment.m01 / area);
//					object.setType(theObject.getType());
//					object.setColor(theObject.getColor());
//
//					objects.push_back(object);
//
//					objectFound = true;
//
//				}
//				else objectFound = false;
//			}
//			//let user know you found an object
//			if (objectFound == true) {
//				//draw object location on screen
//				drawObject(objects, cameraFeed, temp, contours, hierarchy);
//			}
//
//		}
//		else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
//	}
//}
//
//int main(int argc, char* argv[])
//{
//	//if we would like to calibrate our filter values, set to true.
//	bool calibrationMode = true;
//
//	//Matrix to store each frame of the webcam feed
//	Mat cameraFeed;
//	Mat threshold;
//	Mat HSV;
//
//	if (calibrationMode) {
//		//create slider bars for HSV filtering
//		createTrackbars();
//	}
//	//video capture object to acquire webcam feed
//	VideoCapture capture;
//	//open capture object at location zero (default location for webcam)
//	capture.open(0);
//	//set height and width of capture frame
//	capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
//	capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
//	//start an infinite loop where webcam feed is copied to cameraFeed matrix
//	//all of our operations will be performed within this loop
//	waitKey(1000);
//	Mat roi;
//	while (1) {
//		//store image to matrix
//		capture.read(cameraFeed);
//
//		src = cameraFeed;
//		
//		roi = src(Rect(ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT));
//
//		if (!src.data)
//		{
//			return -1;
//		}
//
//
//
//		//convert frame from BGR to HSV colorspace
//		cvtColor(roi, HSV, COLOR_BGR2HSV);
//
//		if (calibrationMode == true) {
//
//			//need to find the appropriate color range values
//			// calibrationMode must be false
//
//			//if in calibration mode, we track objects based on the HSV slider values.
//			cvtColor(roi, HSV, COLOR_BGR2HSV);
//			inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
//			morphOps(threshold);
//			imshow(windowName2, threshold);
//
//			//the folowing for canny edge detec
//			/// Create a matrix of the same type and size as src (for dst)
//			dst.create(roi.size(), roi.type());
//			/// Convert the image to grayscale
//			cvtColor(roi, src_gray, CV_BGR2GRAY);
//			/// Create a window
//			namedWindow(window_name, CV_WINDOW_AUTOSIZE);
//			/// Create a Trackbar for user to enter threshold
//			createTrackbar("Min Threshold:", window_name, &lowThreshold, max_lowThreshold);
//			/// Show the image
//			trackFilteredObject(threshold, HSV, roi);
//		}
//		else {
//			//create some temp fruit objects so that
//			//we can use their member functions/information
//			Object blue("blue"), yellow("yellow"), red("red"), green("green");
//
//			//first find blue objects
//			cvtColor(roi, HSV, COLOR_BGR2HSV);
//			inRange(HSV, blue.getHSVmin(), blue.getHSVmax(), threshold);
//			morphOps(threshold);
//			trackFilteredObject(blue, threshold, HSV, roi);
//			//then yellows
//			cvtColor(roi, HSV, COLOR_BGR2HSV);
//			inRange(HSV, yellow.getHSVmin(), yellow.getHSVmax(), threshold);
//			morphOps(threshold);
//			trackFilteredObject(yellow, threshold, HSV, roi);
//			//then reds
//			cvtColor(roi, HSV, COLOR_BGR2HSV);
//			inRange(HSV, red.getHSVmin(), red.getHSVmax(), threshold);
//			morphOps(threshold);
//			trackFilteredObject(red, threshold, HSV, roi);
//			//then greens
//			cvtColor(roi, HSV, COLOR_BGR2HSV);
//			inRange(HSV, green.getHSVmin(), green.getHSVmax(), threshold);
//			morphOps(threshold);
//			trackFilteredObject(green, threshold, HSV, roi);
//
//		}
//		//show frames
//		//imshow(windowName2,threshold);
//
//		imshow(windowName, roi);
//		//imshow(windowName1,HSV);
//
//		//delay 30ms so that screen can refresh.
//		//image will not appear without this waitKey() command
//		waitKey(30);
//	}
//	return 0;
//}
