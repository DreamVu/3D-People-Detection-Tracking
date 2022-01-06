/*

CODE SAMPLE # 005: HQ Detection
This code sample allows users to run HQ Object Detection


>>>>>> Compile this code using the following command....

g++ 005_hq_detection.cpp /usr/src/tensorrt/bin/common/logger.o ../lib/libPAL.so ../lib/libPAL_DE.so ../lib/libPAL_SSD.so ../lib/libPAL_DEPTH.so ../lib/libPAL_HQDEC.so `pkg-config --libs --cflags opencv`   -O3  -o 005_hq_detection.out -I../include/ -lv4l2 -lpthread -lcudart -L/usr/local/cuda/lib64 -lnvinfer -lnvvpi -lnvparsers -lnvinfer_plugin -lnvonnxparser -lmyelin -lnvrtc -lcudart -lcublas -lcudnn -lrt -ldl

>>>>>> Execute the binary file by typing the following command...

./005_hq_detection.out


>>>>>> KEYBOARD CONTROLS:

	Press ESC to close the window

*/
# include <chrono>
# include <stdio.h>
# include <opencv2/opencv.hpp>
# include "PAL.h"
#include <time.h>
#include <unistd.h>
#include "TimeLogger.h"

using namespace cv;
using namespace std;
using namespace std::chrono;

bool InitHQDec(float threshold);
std::vector<int> ProcessHQDEC(cv::Mat left);
void CloseHQDec();

vector<string> CLASSES = {"person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light","fire hydrant",
    "stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe",
    "backpack","umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat",
    "baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon","bowl",
    "banana","apple","sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant",
    "bed","dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"};

vector<float> _COLORS = {0.000, 0.447, 0.741,0.850, 0.325, 0.098,0.929, 0.694, 0.125,0.494, 0.184, 0.556,0.466, 0.674, 0.188,0.301, 0.745, 0.933,
0.635, 0.078, 0.184,0.300, 0.300, 0.300,0.600, 0.600, 0.600,1.000, 0.000, 0.000,1.000, 0.500, 0.000,0.749, 0.749, 0.000,
0.000, 1.000, 0.000,0.000, 0.000, 1.000,0.667, 0.000, 1.000,0.333, 0.333, 0.000,0.333, 0.667, 0.000,0.333, 1.000, 0.000,
0.667, 0.333, 0.000,0.667, 0.667, 0.000,0.667, 1.000, 0.000,1.000, 0.333, 0.000,1.000, 0.667, 0.000,1.000, 1.000, 0.000,
0.000, 0.333, 0.500,0.000, 0.667, 0.500,0.000, 1.000, 0.500,0.333, 0.000, 0.500,0.333, 0.333, 0.500,0.333, 0.667, 0.500,
0.333, 1.000, 0.500,0.667, 0.000, 0.500,0.667, 0.333, 0.500,0.667, 0.667, 0.500,0.667, 1.000, 0.500,1.000, 0.000, 0.500,
1.000, 0.333, 0.500,1.000, 0.667, 0.500,1.000, 1.000, 0.500,0.000, 0.333, 1.000,0.000, 0.667, 1.000,0.000, 1.000, 1.000,
0.333, 0.000, 1.000,0.333, 0.333, 1.000,0.333, 0.667, 1.000,0.333, 1.000, 1.000,0.667, 0.000, 1.000,0.667, 0.333, 1.000,
0.667, 0.667, 1.000,0.667, 1.000, 1.000,1.000, 0.000, 1.000,1.000, 0.333, 1.000,1.000, 0.667, 1.000,0.333, 0.000, 0.000,
0.500, 0.000, 0.000,0.667, 0.000, 0.000,0.833, 0.000, 0.000,1.000, 0.000, 0.000,0.000, 0.167, 0.000,0.000, 0.333, 0.000,
0.000, 0.500, 0.000,0.000, 0.667, 0.000,0.000, 0.833, 0.000,0.000, 1.000, 0.000,0.000, 0.000, 0.167,0.000, 0.000, 0.333,
0.000, 0.000, 0.500,0.000, 0.000, 0.667,0.000, 0.000, 0.833,0.000, 0.000, 1.000,0.000, 0.000, 0.000,0.143, 0.143, 0.143,
0.286, 0.286, 0.286,0.429, 0.429, 0.429,0.571, 0.571, 0.571,0.714, 0.714, 0.714,0.857, 0.857, 0.857,0.000, 0.447, 0.741,
0.314, 0.717, 0.741,0.50, 0.5, 0};

void setBoxes(cv::Mat& img, std::vector<int> Boxes)
{
    int num_of_persons = Boxes.empty()? 0 : Boxes.size()/6;
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.8;
    int thickness = 2;
    int baseline = 1;

    for (int i = 0; i < num_of_persons; i++)
    {
    	int class_idx = (int)Boxes[i*6+5];
    	int x1 = Boxes[i*6];
    	int y1 = Boxes[i*6+1];
    	int x2 = Boxes[i*6+2];
    	int y2 = Boxes[i*6+3];
    	string label = CLASSES[class_idx];
	
    	int txt_color = (_COLORS[class_idx*3]+_COLORS[class_idx*3+1]+_COLORS[class_idx*3+2]) > 1.5 ? 0 : 255;
    	
	cv::Size txt_size = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    	
	cv::rectangle(img, cv::Point(x1, y1), cv::Point(x2, y2), 
    		cv::Scalar(_COLORS[class_idx*3]*255, _COLORS[class_idx*3+1]*255, _COLORS[class_idx*3+2]*255), 3);
        
	cv::rectangle(
            img,
            cv::Point(x1, y1 + 1),
            cv::Point(x1 + txt_size.width + 1, y1 + 1.5*txt_size.height),
            cv::Scalar(_COLORS[class_idx*3]*255*0.7, _COLORS[class_idx*3+1]*255*0.7, _COLORS[class_idx*3+2]*255*0.7),
            cv::FILLED);
        
	cv::putText(img, label, 
        	cv::Point(x1, y1 + txt_size.height), 
        	fontface, scale, 
        	cv::Scalar(txt_color, txt_color, txt_color), 
        	thickness, 4);
    }
}


int main(int argc, char *argv[])
{
	namedWindow( "PAL HQ Detection", WINDOW_NORMAL ); // Create a window for display.

	int width, height;
    PAL::Mode mode = PAL::Mode::STEREO;
    if(PAL::Init(width, height,-1, &mode) != PAL::SUCCESS) //Connect to the PAL camera
    {
        printf("Init failed\n");
        return 1;
    }
    
    usleep(10);
    
    PAL::CameraProperties data; 
    PAL::Acknowledgement ack = PAL::LoadProperties("../Explorer/SavedPalProperties.txt", &data);
	if(ack != PAL::SUCCESS)
	{
	    printf("Error Loading settings\n");
	}

	//If a command line argument is passed then that will be taken as threshold
	float threshold = (argc>1) ? atof(argv[1]) : 0.4;

	InitHQDec(threshold);

    for(int i=0; i<10; i++)
    {
        PAL::Data::Stereo data;
        data = PAL::GetStereoData();

        cv::Mat output = data.left;
		std::vector<int> Boxes = ProcessHQDEC(output); 
    
    }
    for(int i=0; i<10; i++)
    {
        PAL::Data::Stereo data;
        data = PAL::GetStereoData();

    }
	//width and height are the dimensions of each panorama.
	//Each of the panoramas are displayed at quarter their original resolution.
	//Since the left+right+disparity are vertically stacked, the window height should be thrice the quarter-height
	resizeWindow("PAL HQ Detection", width/2, (height/2));

	int key = ' ';

	printf("Press ESC to close the window\n");   

	//27 = esc key. Run the loop until the ESC key is pressed
	int k=0;
	while(key != 27)
   	 {	
		//Function to Query 
		//Image data: rgb

		PAL::Data::Stereo data;
        data = PAL::GetStereoData();

        cv::Mat output = cv::min(data.left,data.right);

		std::vector<int> Boxes = ProcessHQDEC(output);
		setBoxes(output, Boxes);

		imshow("PAL HQ Detection", output);

		//Wait for the keypress - with a timeout of 1 ms
		key = waitKey(1) & 255;
    }
    
    CloseHQDec();
    printf("exiting the application\n");
    PAL::Destroy();
    
    return 0;
}
