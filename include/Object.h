#pragma once
#include <string>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;

class Object
{
public:
	Object();
	~Object(void);

	Object(string name);

	int getXPos();
	void setXPos(int x);

	int getYPos();
	void setYPos(int y);

	double getArea();
	void setArea(double area);

	Scalar getHSVmin();
	Scalar getHSVmax();

	void setHSVmin(Scalar min);
	void setHSVmax(Scalar max);

	string getType(){return type;}
	void setType(string t){type = t;}

	Scalar getColor(){
		return Color;
	}
	void setColor(Scalar c){

		Color = c;
	}

private:

	int xPos, yPos;
	double area;
	string type;
	Scalar HSVmin, HSVmax;
	Scalar Color;
};
