#include "GrayCodePattern.h"
#include "Sfm.h"
#include <iostream>
#include <opencv2/core.hpp>
using namespace std;

int main(int argh, char* argv[])
{
	cout << "Task List\n1. Scan with webcamera \n2. Scan with iphone \n3. Change iphone file name \n4. Decode \n5. Match feature points \n6. Simplify match file \n\n Pleace select task! ";

	int select;
	cin >> select;

	//clear console
	system("cls");

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
		GrayCodePattern::changeIphoneFileName();
		break;
		// Change iphone file name
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
	return 1;
}