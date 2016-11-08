#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <list>
#include <queue>
#include <stack>
#include "Object.h"

#define THRESH	100
#define MAX_THRESH	255

using namespace cv;
using namespace std;

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

		Mat srcWalls;
		
		cvtColor(src, srcWalls, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		inRange(srcWalls, Scalar(0, 0, 0), Scalar(179, 255, 71), srcWalls); //Threshold the image


		////get ball diameter
		ball b(src);

		size_t ballSize = 5;// b.getDiameter(); //assign a temp value for now
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
private:
	vector<vector<cell>> cells;
	size_t width, height;

};

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

//The following for canny edge detec
Mat dst, detected_edges;
Mat src, src_gray;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";

void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed

}

string intToString(int number){

	std::stringstream ss;
	ss << number;
	return ss.str();
}

void createTrackbars(){
	//create window for trackbars
	namedWindow(trackbarWindowName,0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH),
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->
	createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
	createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
	createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
	createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
	createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
	createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
}

void drawObject(vector<Object> theObjects,Mat &frame, Mat &temp, vector< vector<Point> > contours, vector<Vec4i> hierarchy){

	for(int i =0; i<theObjects.size(); i++){
	cv::drawContours(frame,contours,i,theObjects.at(i).getColor(),3,8,hierarchy);
	cv::circle(frame,cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()),5,theObjects.at(i).getColor());
	cv::putText(frame,intToString(theObjects.at(i).getXPos())+ " , " + intToString(theObjects.at(i).getYPos()),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()+20),1,1,theObjects.at(i).getColor());
	cv::putText(frame,theObjects.at(i).getType(),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()-20),1,2,theObjects.at(i).getColor());
	}
}

void drawObject(vector<Object> theObjects,Mat &frame){

	for(int i =0; i<theObjects.size(); i++){

	cv::circle(frame,cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()),10,cv::Scalar(0,0,255));
	cv::putText(frame,intToString(theObjects.at(i).getXPos())+ " , " + intToString(theObjects.at(i).getYPos()),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()+20),1,1,Scalar(0,255,0));
	cv::putText(frame,theObjects.at(i).getType(),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()-30),1,2,theObjects.at(i).getColor());
	}
}

void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
}
Object trackFilteredObject(Mat threshold,Mat HSV, Mat &cameraFeed)
{
	vector <Object> objects;
	Mat temp;
	Object object;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS)
		{
			for (int index = 0; index >= 0; index = hierarchy[index][0])
			{
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA)
				{

					object.setXPos(moment.m10/area);
					object.setYPos(moment.m01/area);

					objects.push_back(object);

					objectFound = true;

				}
				else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true)
			{
				//draw object location on screen
				drawObject(objects,cameraFeed);
			}
		}
		else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
	return object;
}

void trackFilteredObject(Object theObject,Mat threshold,Mat HSV, Mat &cameraFeed)
{
	vector <Object> objects;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				cout << "area = " << area << endl;
		//if the area is less than 20 px by 20px then it is probably just noise
		//if the area is the same as the 3/2 of the image size, probably just a bad filter
		//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA){

					Object object;

					object.setXPos(moment.m10/area);
					object.setYPos(moment.m01/area);
					object.setType(theObject.getType());
					object.setColor(theObject.getColor());

					objects.push_back(object);

					objectFound = true;

				}else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true){
				//draw object location on screen
				drawObject(objects,cameraFeed,temp,contours,hierarchy);}
		}else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

//
/////** @function main */
int main(int argc, char** argv)
{
		//Matrix to store each frame of the webcam feed
		Mat cameraFeed;
		Mat threshold;
		Mat HSV;

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
		while (1) {
			//store image to matrix
			capture.read(cameraFeed);

			src = cameraFeed;

			if (!src.data)
			{
				return -1;
			}


			//need to find the appropriate color range values
			// calibrationMode must be false

			//if in calibration mode, we track objects based on the HSV slider values.
			cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
			inRange(HSV, Scalar(0, 16, 0), Scalar(7, 256, 256), threshold);
			morphOps(threshold);
			imshow(windowName2, threshold);


			/// Convert the image to grayscale
			cvtColor(src, src_gray, CV_BGR2GRAY);
			/// Create a window
			namedWindow(window_name, CV_WINDOW_AUTOSIZE);
			/// Show the image
			trackFilteredObject(o, threshold, HSV, cameraFeed);
		
			imshow(windowName, cameraFeed);

			waitKey(30);
		}
		return 0;
}



