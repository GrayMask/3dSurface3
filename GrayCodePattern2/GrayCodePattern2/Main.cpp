#include "GrayCodePattern.h"
#include "Sfm.h"
#include <iostream>
#include <opencv2/core.hpp>
using namespace std;

int main(int argh, char* argv[])
{
	cout << "Task List\n1. Scan with webcamera \n2. Scan with iphone \n3. Get frame from vedio \n4. Decode \n5. Match feature points \n6. Simplify match file \n\n Pleace select task! ";

	int select;
	cin >> select;

	//clear console
	system("cls");
	system("time");
	switch (select)
	{
	case 1:
		GrayCodePattern::getGrayCodeImages();
		break;
		// Scan with webcamera
	case 2:
		GrayCodePattern::getGrayCodeImagesForIphone();
		break;
		// Scan with iphone
	case 3:
		GrayCodePattern::getFrameFromVedio();
		break;
		// Get frame from vedio
	case 4:
		Sfm::executeDecoding();
		break;
		// Decode
	case 5:
		Sfm::executeMatching();
		break;
		// Match feature points
	case 6:
		Sfm::simplifyMatchFile();
		break;
		// Simplify match file
	}
	system("time");
	std::cout << "\nPress any key to exit.";
	cin >> select;
	return 1;
}