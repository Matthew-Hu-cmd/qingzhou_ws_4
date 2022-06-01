#include <opencv2/opencv.hpp>
#include <iostream>
 
using namespace cv;
using namespace std;
 
#define WINDOW_NAME "【效果图窗口】"        //为窗口标题定义的宏 
 
void pickHighLightFire(Mat& inputFrame, Mat& outputFrame);
void on_MouseHandle(int event, int x, int y, int flags, void* param);
int main()
{
	int multiple = 1;//图片的放大倍数
	Mat inputImage = imread("ld//53.jpg");//这里放置自己的文件路径。
	Mat outputImage;
	resize(inputImage, inputImage, Size(multiple  * inputImage.cols, multiple  * inputImage.rows));
	cvtColor(inputImage, outputImage, COLOR_BGR2HSV);
 
	//设置鼠标操作回调函数
	namedWindow(WINDOW_NAME);
	setMouseCallback(WINDOW_NAME, on_MouseHandle, (void*)&outputImage);
	imshow(WINDOW_NAME, inputImage);
	while (1)
	{
		if (waitKey(10) == 27) break;//按下ESC键，程序退出
	}
	waitKey();
	return 0;
}
 
void on_MouseHandle(int event, int x, int y, int flags, void* param)
{
 
	Mat& image = *(cv::Mat*) param;
	switch (event)
	{
	//左键按下消息
	case EVENT_LBUTTONDOWN:
	{
 
		cout << static_cast<int>( image.at<Vec3b>(y, x)[0]) << ",";
		cout << static_cast<int>(image.at<Vec3b>(y, x)[1]) << ",";
		cout << static_cast<int>(image.at<Vec3b>(y, x)[2]) << endl;
	}
	break;
	}
}
————————————————
版权声明：本文为CSDN博主「c1learning」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/c1learning/article/details/100130805