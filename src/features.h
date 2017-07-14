extern double getAverageColor(cv::Mat& gray, cv::Point center);
extern std::pair<cv::Point, cv::Point> getTargetVector(cv::Mat& gray, std::vector<cv::Vec3f> circles);
extern void processImage(cv::Mat&);
