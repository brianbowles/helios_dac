//Example program scanning a line from top to bottom on the Helios
#include <iostream>
#include <unistd.h>
#include <vector>
#include "HeliosDac.h"


#define MAX_WAIT_AMOUNT 50
#define BLANKING_AMOUNT 50

class Point {

public: // this doesnt need accessors and that sort of thing, make these pub and access directly.
  int x,y,r,g,b,i;
public:
  Point() {
    x = 0;
    y = 0;
    r = 0;
    g = 0;
    b = 0;
    i = 0;
  }
  Point(int X,int Y,int R, int G, int B, int I) {


    x = X;
    y = Y;
    g = G;
    b = B;
    r = R;
    i = I;
	       
  }
  
};
  
class Wait {

 private:
  Point point;
  std::vector<Point> v;
 public:
  Wait(Point &p) {

    point.x = p.x;
    point.x = p.y;
    point.x = p.r;
    point.x = p.g;
    point.x = p.b;
    point.x = p.i;
    
  }
  // TODO make destructor clean up memory !
  
  // return a Vector of Points ? Do we need path?
  std::vector<Point> draw(int count) {
    int i;
    for (i = 0; i < count; i++) {
      v.push_back(Point(point.x, point.y, point.r, point.g, point.b, point.i) );
    }
    return v;
  }
};

//class Line {
  /* import { Shape } from './Shape';
import { Point, Color } from './Point';
import { Wait } from './Wait';
import { BLANKING_AMOUNT, MAX_WAIT_AMOUNT } from './constants';

interface Coordinates {
  x: number;
  y: number;
}
  */
  /*
interface LineOptions {
  from: Coordinates;
  to: Coordinates;
  color: Color;
  blankBefore?: boolean;
  blankAfter?: boolean;
  waitAmount?: number;
  blankingAmount?: number;
}
  */
#include<math.h>
  
class Line { // extends Shape {
private:
  Point from;//: Coordinates;
  Point to;//: Coordinates;
  int r,g,b,i;
  int blankBefore;//: boolean;
  int blankAfter;//: boolean;
  int waitAmount;
  int blankingAmount;
  
  Line(Point &to_, Point &from_, int r_, int g_, int b_, int i_, int blankBefore_ = 0, int blankAfter_ = 0, int waitAmount_ = MAX_WAIT_AMOUNT, int blankingAmount_ = BLANKING_AMOUNT) {
    //super();
    from = from_;// So as of now, the rgb of these coords is ignored/undefined.
    to = to_;
    r = r_;
    g = g_;
    b = b_;
    i = i_;
    blankBefore = blankBefore_;
    blankAfter = blankAfter_;
    waitAmount = waitAmount_;
    blankingAmount = blankingAmount_;
    
  }

  std::vector<Point> draw(int resolution) {
    const float distanceX = from.x - to.x;
    const float distanceY = from.y - to.y;
    
    // Calculate distance using the Pythagorean theorem.
    const float distance = sqrt( pow(distanceX, 2) + pow(distanceY, 2));
    const float steps = round(distance * resolution);

    //let points: Point[] = [];

    std::vector<Point> points ;

    Point p = Point(from.x,from.y, from.r, from.g, from.b, from.i);
    if (blankBefore) {
      // Add blanking points.
      points = Wait(p).draw(blankingAmount);
    }

    int stepNumber;
    for (stepNumber = 1; stepNumber <= steps; stepNumber++) {
      points.push_back(
        Point(
          from.x - (distanceX / steps) * stepNumber,
          from.y - (distanceY / steps) * stepNumber,
          r,
	  g,
	  b,
	  i
        )
      );
    }
    //appending elements of vector v2 to vector v1
    //points.insert( points.end(), linePoints.begin(), linePoints.end() ); // No, thats what push_back is doing. 


    if (blankAfter) {
      Point p = Point(to.x,to.y, from.r, from.g, from.b, from.i);
	std::vector<Point> postBlank = Wait(p).draw(blankingAmount);
      points.insert( points.end(), postBlank.begin(), postBlank.end() );
    }

    /*
      // Add blanking points.
      points = [
        ...points,
        ...new Wait({
          x: this.to.x,
          y: this.to.y,
          color: this.color,
          amount: this.waitAmount / 2
        }).draw()
      ];
      } */

    return points;
  }
};

double returnClippedDouble(double d)
{
  if (d < 0.0) {
    return 0.0;
  }
  if (d > 1.0) {
    return 1.0;
  }
  return d;
}


int main(int argc,char ** argv)
{
	//make frames
	HeliosPoint frameR[30][1000];
	HeliosPoint frameG[30][1000];
	HeliosPoint frameB[30][1000];
	int x = 0;
	int y = 0;
	int r = 0, g = 0, b = 0;
	int RED = 0;//0xd0;
	int BLUE = 0xd0;
	int GREEN = 0;//0xff;
	int const POINTSINFRAME = 1000;
	int BLANKINGPOINTS = 30;
	int devNum = 0;
	double const height = 0.1; // approx width and height of crosshair
	double const width = 0.1;

	double chX, chY; // the X and Y requested by user
	int pps;
	int framesDisplayed;
	std::cout << "X Y pps devNum r g b blanking_points total_frames_displayed" << std::endl;

	
	sscanf(argv[1],"%lf",&chX);
	sscanf(argv[2],"%lf",&chY);
	if (chX < 0.0) {
	  std::cout << "x is negative" << std::endl;
	  exit(2);
	}
	if (chX > 1.0) {
	  std::cout << "x is above 1" << std::endl;
	  exit(2);
	}
	
	if (chY < 0.0) {
	  std::cout << "y is negative" << std::endl;
	  exit(2);
	}
	if (chY > 1.0) {
	  std::cout << "y is above 1" << std::endl;
	  exit(2);
	}
	chX = 1.0 - chX;
	
	sscanf(argv[3],"%u",&pps);
	sscanf(argv[4],"%u",&devNum);
	if (argc > 4) {
	  sscanf(argv[5],"%u",&r);
	  sscanf(argv[6],"%u",&g);
	  sscanf(argv[7],"%u",&b);
	  RED = r;
	  BLUE = b;
	  GREEN = g;
	}
	if (argc > 7) {
	  sscanf(argv[8],"%u",&BLANKINGPOINTS);
	} else {
	  BLANKINGPOINTS = 30;
	  
	}
	
	if (argc > 8) { // this is total frames displayed...
	  sscanf(argv[9],"%u",&framesDisplayed);
	} else {
	  framesDisplayed = 150;
	}
	
	std::cout << "BLANKING POINTS = " << BLANKINGPOINTS << std::endl;
	std::cout << "frames displayed = " << framesDisplayed << std::endl;

	double startX = (chX - width/ 2);
	double endX = (chX + width/ 2);
	double startY = (chY - height/ 2);
	double endY =  (chY + height/ 2);

	for (int i = 0; i < 30; i++)
	{
	  //y = i * 0xFFF / 30;
		for (int j = 0; j < POINTSINFRAME; j++)
		{
		  if (j < 500) { // horz slash
		    x = returnClippedDouble(startX + (width / (POINTSINFRAME / 2) * j) ) * 0xFFF;
		    // chX * 0xFFF - 
		    //y = 0xFFF / 2;
		    y = 0xFFF * chY;
		    /* if (j <  250) {
		      frameR[i][j].r = 0xff;
		      frameR[i][j].g = 0;
		      frameR[i][j].b = 0;
		    } else {
		      frameR[i][j].r = 0;
		      frameR[i][j].g = 0;
		      frameR[i][j].b = 0xff;
		    }
		    */
		    frameR[i][j].r = 0xff;
		    frameR[i][j].g = 0xff;
		    frameR[i][j].b = 0xff;
		  } else { // vertical slash drawn secondly
		    y = returnClippedDouble(startY + (height / (POINTSINFRAME / 2) * (j - 500))) * 0xFFF ;
		    //y = j * 0xFFF / 500;
		    x = 0xFFF * chX;
		    //x = 0xFFF - ((j - 500) * 0xFFF / 500);
		    frameR[i][j].r = 0xff;
		    frameR[i][j].g = 0xff;
		    frameR[i][j].b = 0xff;
		  }
		  if ((j >= (POINTSINFRAME / 2))&& (j <= ((POINTSINFRAME / 2) + BLANKINGPOINTS))) { // halfway through frame move it.
		    frameR[i][j].r = 0;
		    frameR[i][j].g = 0;
		    frameR[i][j].b = 0;
		    frameR[i][j].i = 0; // I doesn't seem to matter with helios / cclaser
				    
		  } else if (j < BLANKINGPOINTS) {
		    frameR[i][j].r = 0;
		    frameR[i][j].g = 0;
		    frameR[i][j].b = 0;
		    frameR[i][j].i = 0; // I doesn't seem to matter with helios / cclaser
				    
		  }
		  /*else {
		    //std::cout << x << "," << y << std::endl;
		    frameR[i][j].r = RED;
		    frameR[i][j].g = GREEN;
		    frameR[i][j].b = BLUE;
		    frameR[i][j].i = 0xff; // I doesn't seem to matter with helios / cclaser	
			    
		    }*/
		  frameR[i][j].x = x;
		  frameR[i][j].y = y;


		}
	}

	for (int i = 0; i < 30; i++)
	{
	  //y = i * 0xFFF / 30;
		for (int j = 0; j < POINTSINFRAME; j++)
		{
		  if (j < 500) { // horz slash
		    x = returnClippedDouble(startX + (width / (POINTSINFRAME / 2) * j) ) * 0xFFF;
		    // chX * 0xFFF - 
		    //y = 0xFFF / 2;
		    y = 0xFFF * chY;
		    /* if (j <  250) {
		      frameG[i][j].r = 0xff;
		      frameG[i][j].g = 0;
		      frameG[i][j].b = 0;
		    } else {
		      frameG[i][j].r = 0;
		      frameG[i][j].g = 0;
		      frameG[i][j].b = 0xff;
		    }
		    */
		    frameG[i][j].r = 0xff;
		    frameG[i][j].g = 0xff;
		    frameG[i][j].b = 0xff;
		  } else { // vertical slash drawn secondly
		    y = returnClippedDouble(startY + (height / (POINTSINFRAME / 2) * (j - 500))) * 0xFFF ;
		    //y = j * 0xFFF / 500;
		    x = 0xFFF * chX;
		    //x = 0xFFF - ((j - 500) * 0xFFF / 500);
		    frameG[i][j].r = 0xff;
		    frameG[i][j].g = 0xff;
		    frameG[i][j].b = 0xff;
		  }
		  if ((j >= (POINTSINFRAME / 2))&& (j <= ((POINTSINFRAME / 2) + BLANKINGPOINTS))) { // halfway through frame move it.
		    frameG[i][j].r = 0;
		    frameG[i][j].g = 0;
		    frameG[i][j].b = 0;
		    frameG[i][j].i = 0; // I doesn't seem to matter with helios / cclaser
				    
		  } else if (j < BLANKINGPOINTS) {
		    frameG[i][j].r = 0;
		    frameG[i][j].g = 0;
		    frameG[i][j].b = 0;
		    frameG[i][j].i = 0; // I doesn't seem to matter with helios / cclaser
				    
		  }
		  /*else {
		    //std::cout << x << "," << y << std::endl;
		    frameG[i][j].r = RED;
		    frameG[i][j].g = GREEN;
		    frameG[i][j].b = BLUE;
		    frameG[i][j].i = 0xff; // I doesn't seem to matter with helios / cclaser
				    
		    }*/
		  frameG[i][j].x = x;
		  frameG[i][j].y = y;


		}
	}

	for (int i = 0; i < 30; i++)
	{
	  //y = i * 0xFFF / 30;
		for (int j = 0; j < POINTSINFRAME; j++)
		{
		  if (j < 500) { // horz slash
		    x = returnClippedDouble(startX + (width / (POINTSINFRAME / 2) * j) ) * 0xFFF;
		    // chX * 0xFFF - 
		    //y = 0xFFF / 2;
		    y = 0xFFF * chY;
		    /* if (j <  250) {
		      frame[i][j].r = 0xff;
		      frame[i][j].g = 0;
		      frame[i][j].b = 0;
		    } else {
		      frame[i][j].r = 0;
		      frame[i][j].g = 0;
		      frame[i][j].b = 0xff;
		    }
		    */
		    frameB[i][j].r = 0xff;
		    frameB[i][j].g = 0xff;
		    frameB[i][j].b = 0xff;
		  } else { // vertical slash drawn secondly
		    y = returnClippedDouble(startY + (height / (POINTSINFRAME / 2) * (j - 500))) * 0xFFF ;
		    //y = j * 0xFFF / 500;
		    x = 0xFFF * chX;
		    //x = 0xFFF - ((j - 500) * 0xFFF / 500);
		    frameB[i][j].r = 0xff;
		    frameB[i][j].g = 0xff;
		    frameB[i][j].b = 0xff;
		  }
		  if ((j >= (POINTSINFRAME / 2))&& (j <= ((POINTSINFRAME / 2) + BLANKINGPOINTS))) { // halfway through frame move it.
		    frameB[i][j].r = 0;
		    frameB[i][j].g = 0;
		    frameB[i][j].b = 0;
		    frameB[i][j].i = 0; // I doesn't seem to matter with helios / cclaser
				    
		  } else if (j < BLANKINGPOINTS) {
		    frameB[i][j].r = 0;
		    frameB[i][j].g = 0;
		    frameB[i][j].b = 0;
		    frameB[i][j].i = 0; // I doesn't seem to matter with helios / cclaser
				    
		  }
		  /*else {
		    //std::cout << x << "," << y << std::endl;
		    frameB[i][j].r = RED;
		    frameB[i][j].g = GREEN;
		    frameB[i][j].b = BLUE;
		    frameB[i][j].i = 0xff; // I doesn't seem to matter with helios / cclaser
				    
		    }*/
		  frameB[i][j].x = x;
		  frameB[i][j].y = y;


		}
	}



	//connect to DACs and output frames
	HeliosDac helios;
	helios.CloseDevices(); // try closing them to fix state?
	sleep(1);
	int numDevs = helios.OpenDevices();
	std::cout << "numDevs = " << numDevs << std::endl;

	int i = 0;
	int status = 0;
	int getStatus;
	int framed[8];
	int f;
	int flags;
	while (1)
	{
	  for(f=0; f < 8; f++) {
	    framed[f] = 0;
	  }
	  
	  i++;
	  if (i > framesDisplayed) //cancel after 5 cycles, 30 frames each .. no .. switched to cmdline arg
	    break;
	  //for (int j = 0; j < numDevs; j++)
	  //{
	  //wait for ready status
	  devNum = 0;
	  for (unsigned int k = 0; k < 512; k++)
	    {
	      getStatus = helios.GetStatus(devNum);
	      if (getStatus  == 1) {
		if ((i == 1) || (i == 150)) {
		  std::cout << "GetStatus == 1 devNum = "<< devNum << std::endl;
		  status = 1;
		}
		break;
	      }
	    }
	  if (! status) {
	    std::cout << "GetStatus = " << getStatus <<" devNum = "<< devNum << std::endl;
	    }
	  if ((i == 1) || (i == 150)) {
	    std::cout << "Writing Frame " << i << " devNum=" << devNum << std::endl;
	  }
	  flags = HELIOS_FLAGS_DEFAULT;
	  flags = 2; // repeat frame
	  helios.WriteFrame(devNum, pps, flags, &frameR[i % 30][0], 1000); //send the next frame
	  
	  devNum = 1;
	  for (unsigned int k = 0; k < 512; k++)
	    {
	      getStatus = helios.GetStatus(devNum);
	      if (getStatus  == 1) {
		if ((i == 1) || (i == 150)) {
		  std::cout << "GetStatus == 1 devNum = "<< devNum << std::endl;
		  status = 1;
		}
		break;
	      }
	    }
	  if (! status) {
	    std::cout << "GetStatus = " << getStatus <<" devNum = "<< devNum << std::endl;
	    }
	  if ((i == 1) || (i == 150)) {
	    std::cout << "Writing Frame " << i << " devNum=" << devNum << std::endl;
	  }
	  flags = HELIOS_FLAGS_DEFAULT;
	  flags = 2; // repeat frame
	  helios.WriteFrame(devNum, pps, flags, &frameG[i % 30][0], 1000); //send the next frame

	  devNum = 2;		  
	  for (unsigned int k = 0; k < 512; k++)
	    {
	      getStatus = helios.GetStatus(devNum);
	      if (getStatus  == 1) {
		if ((i == 1) || (i == 150)) {
		  std::cout << "GetStatus == 1 devNum = "<< devNum << std::endl;
		  status = 1;
		}
		break;
	      }
	    }
	  if (! status) {
	    std::cout << "GetStatus = " << getStatus <<" devNum = "<< devNum << std::endl;
	    }
	  if ((i == 1) || (i == 150)) {
	    std::cout << "Writing Frame " << i << " devNum=" << devNum << std::endl;
	  }
	  flags = HELIOS_FLAGS_DEFAULT;
	  flags = 2; // repeat frame
	  helios.WriteFrame(devNum, pps, flags, &frameB[i % 30][0], 1000); //send the next frame
	  //}
	}

	//freeing connection
	helios.CloseDevices();
}
