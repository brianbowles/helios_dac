//Example program scanning a line from top to bottom on the Helios

#include <stdio.h>      // for sprintf()
#include <iostream>
#include <unistd.h>
#include <vector>
#include<string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "HeliosDac.h"

#define MAX_LINE_SEGMENTS 500
#define MAX_WAIT_AMOUNT 50 // this isn't actually used .. trash
#define BLANKING_AMOUNT 100
// lol fix
#define TRUE 1
#define FALSE 0

//-----------------------------------------------------------------------------
// Format current time (calculated as an offset in current day) in this form:
//
//     "hh:mm:ss.SSS" (where "SSS" are milliseconds)
//-----------------------------------------------------------------------------
std::string now_str()
{
    // Get current time from the clock, using microseconds resolution
    const boost::posix_time::ptime now = 
        boost::posix_time::microsec_clock::local_time();

    // Get the time offset in current day
    const boost::posix_time::time_duration td = now.time_of_day();

    //
    // Extract hours, minutes, seconds and milliseconds.
    //
    // Since there is no direct accessor ".milliseconds()",
    // milliseconds are computed _by difference_ between total milliseconds
    // (for which there is an accessor), and the hours/minutes/seconds
    // values previously fetched.
    //
    const long hours        = td.hours();
    const long minutes      = td.minutes();
    const long seconds      = td.seconds();
    const long milliseconds = td.total_milliseconds() -
                              ((hours * 3600 + minutes * 60 + seconds) * 1000);

    //
    // Format like this:
    //
    //      hh:mm:ss.SSS
    //
    // e.g. 02:15:40:321
    //
    //      ^          ^
    //      |          |
    //      123456789*12
    //      ---------10-     --> 12 chars + \0 --> 13 chars should suffice
    //  
    // 
    char buf[40];
    sprintf(buf, "%02ld:%02ld:%02ld.%03ld", 
        hours, minutes, seconds, milliseconds);

    return buf;
}

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

//point data structure
/*
typedef struct
{

} HeliosPoint;
*/
  /*
After compiled, we took this code and substitued it over what was HeliosPoint struct
class Point {

public: // this doesnt need accessors and that sort of thing, make these pub and access directly.
  std::uint16_t x; //12 bit (from 0 to 0xFFF)
  std::uint16_t y; //12 bit (from 0 to 0xFFF)
  std::uint8_t r;	//8 bit	(from 0 to 0xFF)
  std::uint8_t g;	//8 bit (from 0 to 0xFF)
  std::uint8_t b;	//8 bit (from 0 to 0xFF)
  std::uint8_t i;	//8 bit (from 0 to 0xFF)
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
  
  */
class Wait {

 private:
  HeliosPoint point;
  std::vector<HeliosPoint> v;
 public:
  Wait(HeliosPoint &p) {

    point.x = p.x;
    point.y = p.y;
    point.r = p.r;
    point.g = p.g;
    point.b = p.b;
    point.i = p.i;
    
  }
  // TODO make destructor clean up memory !
  
  // return a Vector of HeliosPoints ? Do we need path?
  // useColor means does it use the color on the trailing blanking points
  std::vector<HeliosPoint> draw(int count, std::vector<HeliosPoint> v, const int useColor) {

    //std::cout << now_str() << " wait.draw(existingsize=" << v.size() << " count= " << count << " x= " << point.x << " y=" << point.y << " useColor=" << useColor << std::endl;
    int i;
    HeliosPoint hp;
    if (useColor) {
      hp = HeliosPoint(point.x, point.y, point.r, point.g, point.b, point.i);  
    } else {
      hp = HeliosPoint(point.x, point.y, 0, 0, 0, 0);
    }
    
    for (i = 0; i < count; i++) {
      v.push_back(hp);
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
  HeliosPoint from;//: Coordinates;
  HeliosPoint to;//: Coordinates;
  int r,g,b,i;
  int blankBefore;//: boolean;
  int blankAfter;//: boolean;
  int waitAmount;
  int blankingAmount;
public:
  Line(HeliosPoint &from_, HeliosPoint &to_, int r_, int g_, int b_, int i_, int blankBefore_ = 0, int blankAfter_ = 0, int blankingAmount_ = BLANKING_AMOUNT) {
    //super();
    from = from_;// So as of now, the rgb of these coords is ignored/undefined.
    to = to_;
    r = r_;
    g = g_;
    b = b_;
    i = i_;
    blankBefore = blankBefore_;
    blankAfter = blankAfter_;
    //waitAmount = waitAmount_; // remove
    blankingAmount = blankingAmount_;
    
  }
  // points is the existing vector of points
  // drawPostColor lets you toggle the laser on trailing blanking points.
  // resolution operates such that distance x resolution = points in line. 
  std::vector<HeliosPoint> draw(float resolution, std::vector<HeliosPoint> &points,const int drawPostColor) {
    
    const float distanceX = from.x - to.x; // ints
    const float distanceY = from.y - to.y;
    
    // Calculate distance using the Pythagorean theorem.
    const float distance = sqrt( pow(distanceX, 2) + pow(distanceY, 2));
    //std::cout << now_str() << " line.draw(points size= " << points.size()  << ")distance = " << distance << " drawPostColor=" << drawPostColor << std::endl;
    float steps = round(distance * resolution);
    if (steps < 1) {
      std::cout << now_str() << " Too few steps. Setting to 1 " << std::endl;
      steps = 1;
    }
    if (steps > MAX_LINE_SEGMENTS) {
      std::cout << now_str() << " Too many steps. (" << steps << ") Setting to 50 " << std::endl;
      steps = MAX_LINE_SEGMENTS;
    } 

    //let points: Point[] = [];

    //std::vector<HeliosPoint> points ;
    //std::cout << now_str() << " intial size =" << points.size() << std::endl;
    HeliosPoint p = HeliosPoint(from.x,from.y, 0 , 0 ,0 ,0 );
    if (blankBefore) {
      // Add blanking points.
      //std::cout << now_str() << " pre-blanking " << std::endl;
      points = Wait(p).draw(blankingAmount,points,FALSE);
    }
    //std::cout << now_str() << " blankBefore wait.draw(...) =" << points.size() << std::endl;
    std::cout << now_str() << " ( " << from.x << "," << from.y << ") -> (" << to.x << "," << to.y << ") " << points.size() << std::endl;

    int stepNumber;
    int pushbackCnt = 0;
    for (stepNumber = 1; stepNumber <= steps; stepNumber++) {
      pushbackCnt ++;

      int fromX = from.x - (distanceX / steps) * stepNumber;
      int fromY = from.y - (distanceY / steps) * stepNumber;

      //std::cout << "x= " << fromX << " y= "<< fromY << " distanceX= " << distanceX << " steps= " << steps << " stepNumber= " << stepNumber << std::endl;
      points.push_back(
        HeliosPoint(
		    fromX,
		    fromY,
		    r,
		    g,
		    b,
		    i
        )
      );
      //std::cout << "pushback cnt " << pushbackCnt << std::endl;
    }
    //appending elements of vector v2 to vector v1
    //points.insert( points.end(), linePoints.begin(), linePoints.end() ); // No, thats what push_back is doing. 

    //std::cout << now_str() << " draw size after blanks + line =" << points.size() << std::endl;
    if (blankAfter) {
      //std::cout << now_str() << " post-blank x = " << to.x << " ,y= " << to.y << std::endl;
      HeliosPoint p = HeliosPoint(to.x, to.y, r, g, b, i);
      points = Wait(p).draw(blankingAmount, points, drawPostColor);
      //points.insert( points.end(), postBlank.begin(), postBlank.end() );
      //std::cout << now_str() << " size after wait.draw(...) =" << points.size() << std::endl;
    } else {
      std::cout << now_str() << " blankAfter = FALSE" << std::endl;
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




int main(int argc,char ** argv)
{
  //make frames
  HeliosPoint frameR[30][1000]; // only using one of these now
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

  // THis was the very first code that doesnt rely on any libraries to draw a crude cross-hair
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
  // THis was the very first code that doesnt rely on any libraries to draw a crude cross-hair
  for (int i = 0; i < 30; i++)
    {
      //y = i * 0xFFF / 30;
      for (int j = 0; j < POINTSINFRAME; j++)
	{
	}
    }


  //connect to DACs and output frames
  HeliosDac helios;
  helios.CloseDevices(); // try closing them to fix state?
  sleep(1);
  int numDevs = helios.OpenDevices();
  std::cout << "numDevs = " << numDevs << std::endl;
  if (numDevs != 3) {
    std::cout << "Error. Found " << numDevs <<" DACs" << std::endl;
    exit(2);
  }
  int frameCnt = 0;
  int status = 0;
  int getStatus;
  int framed[8];
  int f;
  int flags;
  int blankingPoints = 20;

  //for(blankingPoints = 100; blankingPoints > 0; blankingPoints--) {
  for(int xDiff = 0; xDiff < 200; xDiff++) {
    frameCnt = 0;
    while (frameCnt < framesDisplayed)
      {
	for(f=0; f < 8; f++) {
	  framed[f] = 0;
	}
	
	frameCnt++;
	if (frameCnt > framesDisplayed) //cancel after 5 cycles, 30 frames each .. no .. switched to cmdline arg
	  break; // this never has us go to xDiff
	//for (int j = 0; j < numDevs; j++)
	//{
	//wait for ready status
	devNum = 1;
	//for(devNum = 0; devNum < numDevs; devNum++) {
	{
	  for (unsigned int k = 0; k < 512; k++)
	    {
	      getStatus = helios.GetStatus(devNum);
	      if (getStatus  == 1) {
		if ((frameCnt == 1) || (frameCnt == framesDisplayed)) {
		  //std::cout << "GetStatus == 1 devNum = "<< devNum << std::endl;
		  status = 1;
		}
		break;
	      }
	    }
	  if (! status) {
	    std::cout << "GetStatus = " << getStatus <<" devNum = "<< devNum << std::endl;
	  }
	  if ((frameCnt == 1) || (frameCnt == framesDisplayed)) {// only print first and last
	    //std::cout << "Writing Frame " << frameCnt << " devNum=" << devNum << " " << now_str() << std::endl;
	  }
	  flags = HELIOS_FLAGS_DEFAULT;
	  flags = 2; // repeat frame if buffer is starved 
	  
	  // helios.WriteFrame(devNum, pps, flags, &frameB[i % 30][0], 1000); //send the next frame, the 1000 is hardcoded in the loop 
	  //helios.WriteFrame(devNum, pps, flags, &frameB[i % 30][0], 1000); //send the next frame, the 1000 is hardcoded in the loop
	  //HeliosPoint start = HeliosPoint(4095, (i % 30) * 4096/150, 0xff, 0xff, 0xff, 0xff); // colors are ignored.
	  //HeliosPoint end = HeliosPoint(0, (i % 30) * 4096/150, 0xff, 0xff, 0xff, 0xff);

	  /* This code will draw a RGB test pattern frame
	  HeliosPoint p1 = HeliosPoint(0, 0,       0,0,0,0); // colors are ignored for Wait points
	  HeliosPoint p2 = HeliosPoint(4095, 0,    0,0,0,0); // colors are ignored for Wait points
	  HeliosPoint p3 = HeliosPoint(4095, 4095, 0,0,0,0); // colors are ignored for Wait points
	  HeliosPoint p4 = HeliosPoint(0, 4095,    0,0,0,0); // colors are ignored for Wait points
	
	  // int blankBefore_ = 0, int blankAfter_ = 0, int waitAmount_ = MAX_WAIT_AMOUNT, int blankingAmount_ = BLANKING_AMOUNT) {
	  
	  int tempR = (devNum == 0) * 0xff;
	  int tempG = (devNum == 1) * 0xff;
	  int tempB = (devNum == 2) * 0xff;
	  std::vector<HeliosPoint> buf;
	  
	  //blankingPoints = 40;
	  int BLANKAFTER = 1; // there is whether you have blank points and also whether they are colored - this doesnt speak to coloredness
	  //tempR = 0xff; tempG = 0; tempB = 0;	
	  Line l1 = Line(p1, p2, tempR, tempG, tempB, 0xff, TRUE, BLANKAFTER, blankingPoints);
	  //tempR = 0; tempG = 0xff; tempB = 0;
	  Line l2 = Line(p2, p3, tempR, tempG, tempB, 0xff, TRUE, BLANKAFTER, blankingPoints);
	  //tempR = 0; tempG = 0; tempB = 0xff;
	  Line l3 = Line(p3, p4, tempR, tempG, tempB, 0xff, TRUE, BLANKAFTER, blankingPoints);
	  //tempR = 0xff; tempG = 0xff; tempB = 0;
	  Line l4 = Line(p4, p1, tempR, tempG, tempB, 0xff, TRUE, BLANKAFTER, blankingPoints);
	  
	  //std::vector<HeliosPoint> v = l.draw(0.1,buf);
	  float resolution = 0.08;
	  resolution = 1/2000;
	  const int drawPostWait = 1;
	  buf = l1.draw(resolution, buf, drawPostWait);
	  //buf = l2.draw(0.005,buf) this resolution was too small.. the minimum is too low // so this is the left line .. need to invert X
	  buf = l2.draw(resolution, buf, drawPostWait); // so this is the left line .. need to invert X
	  buf = l3.draw(resolution, buf, drawPostWait);
	  buf = l4.draw(resolution, buf, drawPostWait);
	  */

	  framesDisplayed = 4;

	  startX = 2000;
	  HeliosPoint p1 = HeliosPoint(startX, 0,       0,0,0,0); // colors are ignored for Wait points
       	  HeliosPoint p2 = HeliosPoint(startX - xDiff * 5, 4095,    0,0,0,0); // colors are ignored for Wait points

	  std::cout << " xDiff= " << xDiff << " frameCnt= " << frameCnt <<std::endl;
	  HeliosPoint p3 = HeliosPoint(startX, 0, 0,0,0,0); // colors are ignored for Wait points
	  HeliosPoint p4 = HeliosPoint(startX + xDiff*5, 4095,    0,0,0,0); // colors are ignored for Wait points
	
	  // int blankBefore_ = 0, int blankAfter_ = 0, int waitAmount_ = MAX_WAIT_AMOUNT, int blankingAmount_ = BLANKING_AMOUNT) {
	  
	  std::vector<HeliosPoint> buf;
	  int tempR,tempG,tempB;
	  blankingPoints = 20;
	  int BLANKAFTER = 1; // there is whether you have blank points and also whether they are colored - this doesnt speak to coloredness
	  tempR = 0x0; tempG = 0xff; tempB = 0;	
	  Line l1 = Line(p1, p2, tempR, tempG, tempB, 0xff, TRUE, BLANKAFTER, blankingPoints);
	  tempR = 0xff; tempG = 0x0; tempB = 0;
	  Line l3 = Line(p3, p4, tempR, tempG, tempB, 0xff, TRUE, BLANKAFTER, blankingPoints);
	  //tempR = 0xff; tempG = 0xff; tempB = 0;
	  
	  //std::vector<HeliosPoint> v = l.draw(0.1,buf);
	  float resolution = 0.01;
	  //resolution = 20/2000;
	  const int drawPostWait = 1;
	  buf = l1.draw(resolution, buf, drawPostWait);
	  //buf = l2.draw(0.005,buf) this resolution was too small.. the minimum is too low // so this is the left line .. need to invert X
	  //buf = l2.draw(resolution, buf, drawPostWait); // so this is the left line .. need to invert X
	  buf = l3.draw(resolution, buf, drawPostWait);
	  //buf = l4.draw(resolution, buf, drawPostWait);
	  



	  

	  if ((frameCnt == 1) || (frameCnt == framesDisplayed)) {// only print first and last
	    //std::cout << now_str() << " line.draw returned  " << buf.size() << " points " << std::endl;
	  }
	  
	
	  // writeFrame has to be rewritten ..  it uses HeliosPoint and not Point..
	  helios.WriteFrame(devNum, pps, flags,&buf[0], buf.size()); //send the next frame, the 1000 is hardcoded in the loop
	}
	
      }
  }
  //freeing connection
  helios.CloseDevices();
}
