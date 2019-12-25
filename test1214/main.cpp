#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

#include "stdafx.h"  
#include "ImageRenderer.h" 
#include <fstream>
#include <time.h> 

using namespace cv;
using namespace std;





//申明全局函数
cv::Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);// 转换depth图像到cv::Mat  
cv::Mat  ConvertDepthMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth);// 转换16bit depth图像
cv::Mat ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight);// 转换color图像到cv::Mat 
string getTime();
std::vector<ColorSpacePoint> MappingMatrix(512 * 424);


int main()
{
	int depth_width = 512; //depth图像就是这么小  
	int depth_height = 424;
	int color_widht = 1920; //color图像就是辣么大  
	int color_height = 1080;

	cv::Mat depthImg_show = cv::Mat::zeros(depth_height, depth_width, CV_8UC3);//原始UINT16 深度图像不适合用来显示，所以需要砍成8位的就可以了，但是显示出来也不是非常好，最好能用原始16位图像颜色编码，凑合着看了  
	cv::Mat depthImg = cv::Mat::zeros(depth_height, depth_width, CV_16UC1);//the depth image  
	cv::Mat colorImg = cv::Mat::zeros(color_height, color_widht, CV_8UC3);//the color image  
	

	HRESULT hr;
	// Current Kinect  
	IKinectSensor* m_pKinectSensor = NULL;
	// Depth reader  
	IDepthFrameReader*  m_pDepthFrameReader = NULL;
	// Color reader  
	IColorFrameReader*  m_pColorFrameReader = NULL;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get the depth reader  
		IDepthFrameSource* pDepthFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		SafeRelease(pDepthFrameSource);

		// for color  
		// Initialize the Kinect and get the color reader  
		IColorFrameSource* pColorFrameSource = NULL;
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}

		SafeRelease(pColorFrameSource);
	}

	//valify the depth reader  
	if (!m_pDepthFrameReader)
	{
		cout << " Can not find the m_pDepthFrameReader!" << endl;
		cv::waitKey(0);
		exit(0);
	}
	//valify the color reader  
	if (!m_pDepthFrameReader)
	{
		cout << "Can not find the m_pColorFrameReader!" << endl;
		cv::waitKey(0);
		exit(0);
	}

	// get the data!  
	UINT nBufferSize_depth = 0;
	UINT16 *pBuffer_depth = NULL;
	UINT nBufferSize_coloar = 0;
	RGBQUAD *pBuffer_color = NULL;;
	char key = 0;
	int imageIndex = 0;//图片编号

	char* g_buffer = (char*)malloc(512 * 424 * 20);
	Sleep(3000);
	printf("摄像头每隔2秒自动拍摄一张图片，结束拍摄请按ESC键。\n开始采集图片\n");
	while (true)
	{
		IDepthFrame* pDepthFrame = NULL;
		HRESULT hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
		if (SUCCEEDED(hr))
		{
			USHORT nDepthMinReliableDistance = 0;
			USHORT nDepthMaxReliableDistance = 0;
			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
			}
			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize_depth, &pBuffer_depth);
				depthImg_show = ConvertMat(pBuffer_depth, depth_width, depth_height, nDepthMinReliableDistance, nDepthMaxReliableDistance);
				depthImg = ConvertDepthMat(pBuffer_depth, depth_width, depth_height, nDepthMinReliableDistance, nDepthMaxReliableDistance);
			}
		}
		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;
			IFrameDescription* pFrameDescription = NULL;
			int nWidth = 0;
			int nHeight = 0;
			USHORT nDepthMinReliableDistance = 0;
			USHORT nDepthMaxDistance = 0;
			UINT nBufferSize = 0;
			UINT16 *pBuffer = NULL;

			hr = pDepthFrame->get_RelativeTime(&nTime);

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
			}

			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Width(&nWidth);
			}

			if (SUCCEEDED(hr))
			{
				hr = pFrameDescription->get_Height(&nHeight);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			}

			if (SUCCEEDED(hr))
			{
				// In order to see the full range of depth (including the less reliable far field depth)
				// we are setting nDepthMaxDistance to the extreme potential depth threshold
				nDepthMaxDistance = USHRT_MAX;

				// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
				//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
			}

			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
			}

			//if (SUCCEEDED(hr))
			//{
			//	ProcessDepth(nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
			//}
			if (SUCCEEDED(hr))
			{
				ICoordinateMapper* pCoordinateMapper;
				m_pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
				pCoordinateMapper->MapDepthFrameToColorSpace(512 * 424, pBuffer, 512 * 424, &MappingMatrix[0]);
			}
			SafeRelease(pFrameDescription);
		}
		SafeRelease(pDepthFrame);

		//for color
		
		IColorFrame* pColorFrame = NULL;
		hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
		ColorImageFormat imageFormat = ColorImageFormat_None;
		if (SUCCEEDED(hr))
		{
			ColorImageFormat imageFormat = ColorImageFormat_None;
			if (SUCCEEDED(hr))
			{
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
			}
			RGBQUAD*  m_pColorRGBX = NULL;
			m_pColorRGBX = new RGBQUAD[color_widht * color_height];
			if (SUCCEEDED(hr))
			{
				if (imageFormat == ColorImageFormat_Bgra)//这里有两个format，不知道具体含义，大概一个预先分配内存，一个需要自己开空间吧  
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize_coloar, reinterpret_cast<BYTE**>(&pBuffer_color));
				}
				else if (m_pColorRGBX)
				{
					pBuffer_color = m_pColorRGBX;
					nBufferSize_coloar = color_widht * color_height * sizeof(RGBQUAD);
					hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize_coloar, reinterpret_cast<BYTE*>(pBuffer_color), ColorImageFormat_Bgra);
				}

				else
				{
					hr = E_FAIL;
				}
				colorImg = ConvertMat(pBuffer_color, color_widht, color_height);
			}

			SafeRelease(pColorFrame);
			delete[] m_pColorRGBX;


		}
		//获取时间戳
		string currtime =getTime();
		//定义存储路径
		string savepath = "D:\\kinect\\data\\";
		//存储3种图片
		cv::imwrite(savepath + "depth_show\\"+ currtime+"-"+ to_string(imageIndex)+ ".png", depthImg_show);
		cv::imwrite(savepath + "depth\\"+currtime + "-" + to_string(imageIndex) + ".png", depthImg);
		cv::imwrite(savepath + "color\\"+currtime + "-" + to_string(imageIndex) + ".png", colorImg);
		//存储映射关系	
		char* b_temp = g_buffer;
		
		for (size_t i = 0; i < 512 * 424; i++)
		{
			//std::cout<<sizeof(float)<<" "<<sizeof(" ")<< " " <<sizeof("\n") <<std::endl;
			float x = MappingMatrix[i].X;
			float y = MappingMatrix[i].Y;
			//if (x >-100000 && x <100000) {
			if (x > 0 && x < 10000) {
				//std::cout << x << " " << y << std::endl;
			}
			else {
				x = -10000;
				y = -10000;
			}
			
			sprintf(b_temp,"%09.1lf %09.1lf\n", x , y);
			
			//std::cout << MappingMatrix[i].X << " " << MappingMatrix[i].Y << std::endl;
			b_temp = b_temp + 20;
			
		}
		
		//std::ofstream out(savepath + "text\\" + to_string(currtime) + "_" + to_string(imageIndex) + ".txt");
		//out << g_buffer << endl;
		//out.close();
		FILE * stream;
		stream = fopen((savepath + "text\\" + currtime + "-" + to_string(imageIndex) + ".txt").c_str(), "w");
		fwrite(g_buffer, 20*512*424, 1, stream);
		imageIndex++;
		fclose(stream);
		
		
		namedWindow("depth", 0);
		cv::imshow("depth", depthImg_show);
		namedWindow("color", 0);
		cv::imshow("color", colorImg);
		
		key = cv::waitKey(1);
		
		if (key == 27)
		{
			break;
		}

		printf("已采集 %d 图片\n",imageIndex);
		Sleep(1500);
	}
	delete g_buffer;
	
	return 0;
}



//获取日期
string getTime() {
	time_t tt = time(NULL);
	struct tm *stm = localtime(&tt);

	char tmp[32];
	sprintf(tmp, "%04d-%02d-%2d-%2d-%2d-%02d", 1900 + stm->tm_year, 1 + stm->tm_mon, stm->tm_mday, stm->tm_hour,
		stm->tm_min, stm->tm_sec);

	return tmp;
}

// 转换depth图像到cv::Mat  
cv::Mat ConvertMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	cv::Mat img(nHeight, nWidth, CV_8UC3);
	uchar* p_mat = img.data;

	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		USHORT depth = *pBuffer;

		//BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth / 25) : 0);
		BYTE intensity;
		if(depth <= nMinDepth) {
			intensity = 0;
		}
		else if (depth >= nMaxDepth) {
			intensity = 255;
		}
		else {
			intensity = static_cast<BYTE>(255*(depth - nMinDepth)/(nMaxDepth - nMinDepth));
		}
		*p_mat = intensity;
		p_mat++;
		*p_mat = intensity;
		p_mat++;
		*p_mat = intensity;
		p_mat++;

		++pBuffer;
	}
	return img;
}
//转换成16位png的depth图
cv::Mat ConvertDepthMat(const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	cv::Mat img(nHeight, nWidth, CV_16UC1);
	UINT16* p_mat = (UINT16*)img.data;

	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		UINT16 depth = *pBuffer;

		//BYTE intensity = static_cast<BYTE>((depth >= nMinDepth) && (depth <= nMaxDepth) ? (depth / 25) : 0);
		UINT16 intensity = static_cast<USHORT> (depth);
		

		*p_mat = intensity;
		p_mat++;
		++pBuffer;
	}
	return img;
}
// 转换color图像到cv::Mat  
cv::Mat ConvertMat(const RGBQUAD* pBuffer, int nWidth, int nHeight)
{
	cv::Mat img(nHeight, nWidth, CV_8UC3);
	uchar* p_mat = img.data;

	const RGBQUAD* pBufferEnd = pBuffer + (nWidth * nHeight);

	while (pBuffer < pBufferEnd)
	{
		*p_mat = pBuffer->rgbBlue;
		p_mat++;
		*p_mat = pBuffer->rgbGreen;
		p_mat++;
		*p_mat = pBuffer->rgbRed;
		p_mat++;

		++pBuffer;
	}
	return img;
}

