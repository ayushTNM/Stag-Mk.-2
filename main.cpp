
#include "Stag.h"
#include "opencv2/opencv.hpp"

int main() {
  // cv::Mat image = cv::imread("1.png", cv::IMREAD_GRAYSCALE);
  cv::Mat image,image1;
  cv::VideoCapture cap(0);//Declaring an object to capture stream of frames from default camera//
   if (!cap.isOpened()){ //This section prompt an error message if no video stream is found//
      std::cout << "No video stream detected" << std::endl;
      system("pause");
      return-1;
   }
   Stag stag(11, 6, true);
  // return 0;
   // int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);//Getting the frame height//
   // int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);//Getting the frame width//
   // cv::VideoWriter video("outcpp.avi",cv::VideoWriter::fourcc('M','J','P','G'),10, cv::Size(frame_width,frame_height));
  while (true){ //Taking an everlasting loop to show the video//
      cap >> image;
      cv::cvtColor(image,image1,cv::COLOR_RGB2GRAY);
      if (image1.empty()){ //Breaking the loop if no video frame is detected//
         break;
      }
      // imshow("Video Player", image);//Showing the video//
      std::cout << "in" << std::endl;
      // Stag stag(11, 7, true);
      stag.detectMarkers(image1);
      std::cout << "out" << std::endl;
      stag.logResults(image,"");
      char c = (char)cv::waitKey(1);//Allowing 25 milliseconds frame processing time and initiating break condition//
      if (c == 27){ //If 'Esc' is entered break the loop//
         break;
      }
   }
   cap.release();//Releasing the buffer memory//
   return 0;
}