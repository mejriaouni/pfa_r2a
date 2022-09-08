#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// comparison function object
bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i < j );
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 
  // Pointer used for the conversion from a ROS message to 
  // an OpenCV-compatible image
  cv_bridge::CvImagePtr cv_ptr;
   
  try
  { 
   
    // Convert the ROS message  
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
     
    // Store the values of the OpenCV-compatible image
    // into the current_frame variable
    cv::Mat img = cv_ptr->image;
     
    // Display the current frame
    cv::imshow("view", img); 
    //RGB to Gray scale conversion
    cv::Mat img_gray;
    cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
    //cv::imshow("gray_view", img_gray);
    
    //Noise removal with iterative bilateral filter(removes noise while preserving edges)
    cv::Mat noise_removal;
    bilateralFilter (img_gray,noise_removal,9,75,75);
    //cv::imshow("Noise Removed Image", noise_removal);
    
    //Thresholding the image
    cv::Mat thresh_image;
    threshold(noise_removal,thresh_image,0,255,cv::THRESH_OTSU);
    //cv::imshow("Image after Thresholding", thresh_image);
    

    // Applying Canny Edge detection
    cv::Mat canny_image;
    cv::Canny(thresh_image,canny_image,250,255);
    //cv::imshow("Image after applying Canny", canny_image);
    cv::convertScaleAbs(canny_image,canny_image);
    cv::imshow("Image after applying Canny", canny_image);

    // dilation to strengthen the edges
    cv::Mat kernel = cv::Mat(3, 3, CV_8UC1, cv::Scalar(1));
    cv::Mat dilated_image;
    cv::dilate(canny_image,dilated_image,kernel,cv::Point(-1,-1),1);
    cv::imshow("Dilation", dilated_image);

    //find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(dilated_image,contours,1,2);
    std::sort(contours.begin(), contours.end(),compareContourAreas);
    cv::Point pt = cv::Point(180, 3 * img.rows / 4);
    
    std::vector<std::vector<cv::Point>> approx;
    /*for (size_t i = 0; i < contours.size(); i++)
    {
      cv::Mat cnt = contours[i];
      approxPolyDP(cnt,approx,0.01*cv::arcLength(cnt,true),true);

    }*/
    
    // Display frame for 30 milliseconds
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
 
int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "frame_listener");
   
  // Default handler for nodes in ROS
  ros::NodeHandle nh;
   
  // Used to publish and subscribe to images
  image_transport::ImageTransport it(nh);
   
  // Subscribe to the /camera topic
  image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback);
   
  // Make sure we keep reading new video frames by calling the imageCallback function
  ros::spin();
   
  // Close down OpenCV
  cv::destroyWindow("view");
}


