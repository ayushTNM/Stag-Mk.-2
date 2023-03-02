
#include "Stag2.h"
#include "opencv2/opencv.hpp"

int main() {

   cv::Mat image;
   cv::VideoCapture cap(2);//Declaring an object to capture stream of frames from default camera

   if (!cap.isOpened()){ //This section prompt an error message if no video stream is found
      std::cout << "No video stream detected" << std::endl;
      system("pause");
      return-1;
   }

  while (true){ //Taking an everlasting loop to show the video
      Stag2 stag2(11, 7, true);
      cap >> image;

      if (image.empty()){ //Breaking the loop if no video frame is detected
         break;
      }

      stag2.detectMarkers(image);
      stag2.logResults(image,"markers", false);

      char c = (char)cv::waitKey(1);//Allowing 1 milliseconds frame processing time and initiating break condition
      if (c == 27){ //If 'Esc' is entered break the loop
         break;
      }
   }
   cap.release();//Releasing the buffer memory
   return 0;
}