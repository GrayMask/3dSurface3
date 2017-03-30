#include <opencv2/highgui.hpp>
#include <opencv2/structured_light.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <io.h>
#include <direct.h>
#include "GrayCodePattern.h"
#include "Path.h"
#include "Const.h"
#include "Tools.h"
#include "Utilities.h"

using namespace std;
using namespace cv;

void GrayCodePattern::getGrayCodeImages()
{
	structured_light::GrayCodePattern::Params params;
	params.width = proj_width;
	params.height = proj_height;
	Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create(params);
	// Storage for pattern
	vector<Mat> pattern;
	graycode->generate(pattern);
	cout << pattern.size() << " pattern images + 2 images for shadows mask computation to acquire with both cameras"
		<< endl;
	// Generate the all-white and all-black images needed for shadows mask computation
	Mat white;
	Mat black;
	graycode->getImagesForShadowMasks(black, white);
	pattern.push_back(white);
	pattern.push_back(black);
	// Setting pattern window on second monitor (the projector's one)
	namedWindow("Pattern Window", WINDOW_NORMAL);
	resizeWindow("Pattern Window", params.width, params.height);
	moveWindow("Pattern Window", params.width + 316, -20);
	setWindowProperty("Pattern Window", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
	// Open camera number 1, using libgphoto2
	VideoCapture cap1(0);
	if (!cap1.isOpened())
	{
		// check if cam1 opened
		cout << "cam1 not opened!" << endl;
		exit(-1);
	}
	if (_access((root_dir + expr_dir).c_str(), 6) == -1)
	{
		int result = _mkdir((root_dir + expr_dir).c_str());
	}
	// Turning off autofocus
	//cap1.set(CAP_PROP_SETTINGS, 1);
	cap1.set(CV_CAP_PROP_EXPOSURE, cam_exp);
	cap1.set(CV_CAP_PROP_FRAME_WIDTH, cam_width);
	cap1.set(CV_CAP_PROP_FRAME_HEIGHT, cam_height);
	int projectorGroupNum = 0;
	cout << "Press \"r\" to acquire data of new projector position" << endl;
	cout << "Press \"q\" to quit" << endl;
	while (true) {
		int key = waitKey(1);
		if (key == 113) { // q
			break;
		}
		else if ( key == 114) { // r
			int imagesGroupNum = 0;
			char* projectorGroupDirTemp = new char[projector_group_dir_length];
			sprintf(projectorGroupDirTemp, projector_group_dir, projectorGroupNum);
			String imagesDir1 = root_dir + expr_dir + String(projectorGroupDirTemp);
			if (_access(imagesDir1.c_str(), 6) == -1)
			{
				int result = _mkdir(imagesDir1.c_str());
			}
			cout << "Press \"Enter\" to acquire a new group of data" << endl;
			cout << "Press \"q\" to quit" << endl;
			while (true) {
				int key = waitKey(1);
				if (key == 113) { // q
					break;
				}
				else if (key == 13) { // enter
					char* imagesGroupDirTemp = new char[images_group_dir_length];
					sprintf(imagesGroupDirTemp, images_group_dir, imagesGroupNum);
					String imagesDir2 = imagesDir1 + String(imagesGroupDirTemp);
					if (_access(imagesDir2.c_str(), 6) == -1)
					{
						int result = _mkdir(imagesDir2.c_str());
					}
					FileStorage fs1(imagesDir2 + imagesName_file, FileStorage::WRITE);
					fs1 << "imagelist" << "[";
					int i = 0;
					while (i < (int)pattern.size())
					{
						cout << "Waiting to save image number " << i + 1 << endl << "Press \"s\" to acquire the photo" << endl;
						imshow("Pattern Window", pattern[i]);
						Mat frame1, showFrame1;
						while (1)
						{
							cap1 >> frame1;  // get a new frame from camera 1
							resize(frame1, showFrame1, Size(cam_width / 2, cam_height / 2), 0, 0, 3);
							imshow("cam1", showFrame1);
							int key = waitKey(1);
							if (key == 115) // s
							{
								bool save1 = false;
								ostringstream name;
								name << i + 1;
								String imagesFile1 = imagesDir2 + images_file + name.str() + imgType;
								cout << imagesFile1 << endl;
								save1 = imwrite(imagesFile1, frame1);
								if (save1) {
									cout << "pattern cam1 images number " << i + 1 << " saved" << endl << endl;
									fs1 << images_file + name.str() + imgType;
									i++;
									break;
								}
								cout << "pattern cam1 images number " << i + 1 << " NOT saved" << endl << endl << "Retry, check the path" << endl << endl;
							}
						}
					}
					fs1 << "]";
					//destroyWindow("cam1");
					imagesGroupNum++;
				}
			}
			Tools::writeGroupNumFile(imagesDir1 + imageGroupNum_file, imagesGroupNum);
			projectorGroupNum++;
		}
	}
	Tools::writeGroupNumFile(root_dir + projectorGroupNum_file, projectorGroupNum);
}


void GrayCodePattern::getGrayCodeImagesForIphone()
{
	structured_light::GrayCodePattern::Params params;
	params.width = proj_width;
	params.height = proj_height;
	Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create(params);
	// Storage for pattern
	vector<Mat> pattern;
	graycode->generate(pattern);
	cout << pattern.size() << " pattern images + 2 images for shadows mask computation to acquire with both cameras"
		<< endl;
	// Generate the all-white and all-black images needed for shadows mask computation
	Mat white;
	Mat black;
	graycode->getImagesForShadowMasks(black, white);
	pattern.push_back(white);
	pattern.push_back(black);
	// Setting pattern window on second monitor (the projector's one)
	namedWindow("Pattern Window", WINDOW_NORMAL);
	resizeWindow("Pattern Window", params.width, params.height);
	moveWindow("Pattern Window", params.width + 316, -20);
	setWindowProperty("Pattern Window", WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
	if (_access((root_dir + expr_dir).c_str(), 6) == -1)
	{
		int result = _mkdir((root_dir + expr_dir).c_str());
	}
	int projectorGroupNum = 0;
	cout << "Press \"r\" to acquire data of new projector position" << endl;
	cout << "Press \"q\" to quit" << endl;
	while (true) {
		int key = waitKey(1);
		if (key == 113) { // q
			break;
		}
		else if (key == 114) { // r
			int imagesGroupNum = 0;
			char* projectorGroupDirTemp = new char[projector_group_dir_length];
			sprintf(projectorGroupDirTemp, projector_group_dir, projectorGroupNum);
			String imagesDir1 = root_dir + expr_dir + String(projectorGroupDirTemp);
			if (_access(imagesDir1.c_str(), 6) == -1)
			{
				int result = _mkdir(imagesDir1.c_str());
			}
			cout << "Press \"Enter\" to acquire a new group of data" << endl;
			cout << "Press \"q\" to quit" << endl;
			while (true) {
				int key = waitKey(1);
				if (key == 113) { // q
					break;
				}
				else if (key == 13) { // enter
					char* imagesGroupDirTemp = new char[images_group_dir_length];
					sprintf(imagesGroupDirTemp, images_group_dir, imagesGroupNum);
					String imagesDir2 = imagesDir1 + String(imagesGroupDirTemp);
					if (_access(imagesDir2.c_str(), 6) == -1)
					{
						int result = _mkdir(imagesDir2.c_str());
					}
					FileStorage fs1(imagesDir2 + imagesName_file, FileStorage::WRITE);
					fs1 << "imagelist" << "[";
					int i = 0;
					while (i < (int)pattern.size())
					{
						cout << "Waiting to save image number " << i + 1 << endl << "Press \"s\" to acquire the photo" << endl;
						imshow("Pattern Window", pattern[i]);
						while (1)
						{
							int key = waitKey(1);
							if (key == 115) // s
							{
								bool save1 = false;
								ostringstream name;
								name << i + 1;
								String imagesFile1 = imagesDir2 + images_file + name.str() + imgType;
								cout << imagesFile1 << endl;
								cout << "pattern cam1 images number " << i + 1 << " saved" << endl << endl;
								fs1 << images_file + name.str() + imgType;
								i++;
								break;
							}
						}
					}
					fs1 << "]";
					//destroyWindow("cam1");
					imagesGroupNum++;
				}
			}
			Tools::writeGroupNumFile(imagesDir1 + imageGroupNum_file, imagesGroupNum);
			projectorGroupNum++;
		}
	}
	Tools::writeGroupNumFile(root_dir + projectorGroupNum_file, projectorGroupNum);
}

void GrayCodePattern::changeIphoneFileName() {

	int count = 856;
	int numOfProjectorGroup;
	Tools::readGroupNumFile(root_dir + projectorGroupNum_file, numOfProjectorGroup);
	for (int i = 0; i < numOfProjectorGroup; i++) {
		int numOfImageGroup;
		Utilities::readNumOfImageGroup(i, numOfImageGroup);
		for (int j = 0; j < numOfImageGroup; j++) {
			for (int k = 1; k <= 42;) {
				char* iphoneFileTemp = new char[iphone_file_length];
				sprintf(iphoneFileTemp, iphone_file, count);
				String imagesDir2 = imagesDir1 + String(iphoneFileTemp);
				fstream f;
				f.open(original_name.c_str());
				if (f)
				{
					rename(original_name.c_str(), new_name.c_str());
					MessageBox(NULL, TEXT("RENAME SUCCESS"), NULL, NULL);
					f.close();
				}
			}
		}
	}
}