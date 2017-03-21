#include "GrayCodePattern.h"
#include "Sfm.h"
#include <iostream>
#include <opencv2/core.hpp>
using namespace std;

int main(int argh, char* argv[])
{
	cout << "Task List\n1. Scan with webcamera \n2. Scan with iphone \n3. Decode \n4. Match feature points \n\n Pleace select task! ";

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
		Sfm::executeDecoding();
		break;
		// Decode
	case 4:
		Sfm::executeMatching();
		break;
		// Match feature points
	}
	return 1;
}