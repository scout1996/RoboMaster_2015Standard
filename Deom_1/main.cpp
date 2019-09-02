#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include <math.h>
#include <iostream>


using namespace std;
using namespace cv;


int main()
{

    double heights[16];
    int hi = 0;
    VideoCapture capture("123.avi");
    //VideoCapture capture(1);
    Mat frame, binary;
	//frame=imread("3");
    RotatedRect RA[16], R[16];       //旋转矩形


    for(;;)
    {
        capture>> frame;
        frame.copyTo(binary);

        cvtColor(frame,frame,COLOR_BGR2GRAY);
		blur(frame, frame, Size(5, 5));
		inRange(frame,Scalar(200,0,0),Scalar(255,255,255),frame);
        //threshold(frame, frame, 200, 255,cv::THRESH_BINARY);//调阈值就差不多了
		dilate(frame, frame, Mat(), Point(-1,-1), 3);   //图像膨胀
		erode(frame, frame, Mat(), Point(-1,-1), 1);
        
		Canny(frame,frame,3,9,3);
        vector<vector<Point>> contours;
        imshow("input",frame);
		// 找到阈值图像中的所有轮廓
        findContours(frame, contours, RETR_LIST, CHAIN_APPROX_NONE);


        for (size_t i = 0; i < contours.size(); i++)
		{

            vector<Point> points_1;
            double area = contourArea(contours[i]);   //获取轮廓的面积
            if (area < 20 || 1e3 < area) continue;    //限定面积范围
            drawContours(frame, contours, static_cast<int>(i), Scalar(0), 4);  //绘制轮廓

            double high_1;
            points_1 = contours[i];

            RotatedRect rrect = fitEllipse(points_1);
            Point2f* vertices = new Point2f[4];                              //得出每个灯条的顶点坐标
            rrect.points(vertices);


               for (int j = 0; j < 4; j++)
                {
                    //line(binary, vertices[j], vertices[(j + 1) % 4], Scalar(0, 255, 0),4);  //圈出所绘制的轮廓

                }

 
             high_1 = rrect.size.height;         //矩形模板的高度

			// cout<<high<<endl;
             for(int j = 1;j < contours.size();j++)
			 {

                 vector<Point> points_2;
                 double area = contourArea(contours[j]);
                 if (area < 20 || 1e3 < area) continue;


                 double high_2, distance;
                 double slop ;
                 points_2 = contours[j];

                 RotatedRect rrectA = fitEllipse(points_2);

                 slop = abs(rrect.angle - rrectA.angle);     //计算角度差绝对值
			
                 high_2 = rrectA.size.height;
                 distance  =sqrt((rrect.center.x-rrectA.center.x)*(rrect.center.x-rrectA.center.x)+
                                 (rrect.center.y-rrectA.center.y)*(rrect.center.y-rrectA.center.y));    //质心距离


                 double max_height, min_height;
                 if(rrect.size.height > rrectA.size.height){
                     max_height = rrect.size.height;
                     min_height = rrectA.size.height;
                 }
                 else{
                     max_height = rrectA.size.height;
                     min_height = rrect.size.height;
                 }

                 double line_x = abs(rrect.center.x-rrectA.center.x);                //相邻灯条质心的距离x（绝对值）            
                 double difference = max_height - min_height;                        //高的灯条的高度差
                 double aim =   distance/((high_1+high_2)/2);                        //相邻灯条的高度差
                 double difference3 = abs(rrect.size.width -rrectA.size.width);
                 double height = (rrect.size.height+rrectA.size.height)/200;
                 double slop_low = abs(rrect.angle + rrectA.angle)/2;


                 if( aim < 5.0-height && aim > 3.2 - height && slop <= 7 && difference <=15 && difference3 <= 8 &&(slop_low <= 30 || slop_low >=150) && line_x >0.7*distance) 

				 {
					 //符合条件则识别为装甲板
                     heights[hi] = (rrect.size.height+rrectA.size.height)/2;
                     R[hi] = rrect;
                     RA[hi] = rrectA;
                     hi++;
					cout<<"rrect="<<rrect.center.x<<endl;
					cout<<"rrectA="<<rrectA.center.x<<endl;

                 }
             }
        }


        double max = 0;
        int mark;
        for(int i = 0;i < hi;i++){    
            
                mark = i;
            //rectangle(binary,Point(R[mark].center.x,R[mark].center.y+25),Point(RA[mark].center.x,RA[mark].center.y-25),Scalar(0,255,0),2);//绘制矩形
			circle(binary,Point((R[mark].center.x+RA[mark].center.x)/2,(R[mark].center.y+RA[mark].center.y)/2),2,Scalar(0,0,255),2);		//绘制出中心坐标的点
        }
        if(hi != 0){
            //circle(binary,Point((R[mark].center.x+RA[mark].center.x)/2,(R[mark].center.y+RA[mark].center.y)/2),2,Scalar(0,0,255),2);
                                     
			rectangle(binary,Point(R[mark].center.x,R[mark].center.y+25),Point(RA[mark].center.x,RA[mark].center.y-25),Scalar(0,255,0),2);//绘制矩形
		}

        

        imshow("result",binary);
        waitKey(10);

        hi = 0;
		
    }
	system("pause");
   return 0;
}
