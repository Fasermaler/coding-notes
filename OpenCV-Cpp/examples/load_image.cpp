//Uncomment the following line if you are compiling this code in Visual Studio
//#include "stdafx.h"

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Put file name here
String file_name("test.jpg");

int main(int argc, char** argv)
{
 // Read the image file
 Mat image = imread(file_name, IMREAD_COLOR);

 // Check for failure
 if (image.empty()) 
 {
  cout << "Could not open or find the image" << endl;
  cin.get(); //wait for any key press
  return -1;
 }

 String windowName = "The Guitar"; //Name of the window

 namedWindow(windowName); // Create a window

 imshow(windowName, image); // Show our image inside the created window.

 waitKey(0); // Wait for any keystroke in the window

 destroyWindow(windowName); //destroy the created window

 return 0;
}
