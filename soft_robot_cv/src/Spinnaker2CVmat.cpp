//=============================================================================
// Copyright ? 2015 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of 
// Point Grey Research, Inc. ("Confidential Information"). You shall not
// disclose such Confidential Information and shall use it only in 
// accordance with the terms of the "License Agreement" that you 
// entered into with PGR in connection with this software.
//
// UNLESS OTHERWISE SET OUT IN THE LICENSE AGREEMENT, THIS SOFTWARE IS 
// PROVIDED ON AN ?AS-IS? BASIS AND POINT GREY RESEARCH INC. MAKES NO 
// REPRESENTATIONS OR WARRANTIES ABOUT THE SOFTWARE, EITHER EXPRESS 
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO ANY IMPLIED WARRANTIES OR 
// CONDITIONS OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR 
// NON-INFRINGEMENT. POINT GREY RESEARCH INC. SHALL NOT BE LIABLE FOR ANY 
// DAMAGES, INCLUDING BUT NOT LIMITED TO ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES, OR ANY LOSS OF PROFITS, 
// REVENUE, DATA OR DATA USE, ARISING OUT OF OR IN CONNECTION WITH THIS 
// SOFTWARE OR OTHERWISE SUFFERED BY YOU AS A RESULT OF USING, MODIFYING 
// OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

/**
*  @example Spinnaker2CVMat.cpp
*
*  @brief Spinnaker2CVMat.cpp shows how to capture images with the camera and takes the image pointer data and puts it into an OpenCV Mat container. It relies on information
*	provided in the Enumeration, Acquisition, Trigger, and NodeMapInfo examples.
*
*/

#include "stdafx.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <sstream> 

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace cv;


int ConfigureTrigger(INodeMap & nodeMap)
{
	int result = 0;
	cout << endl << endl << "*** CONFIGURING TRIGGER ***" << endl << endl;
	try
	{
		// Ensure trigger mode off
		//
		// *** NOTES ***
		// The trigger must be disabled in order to configure whether the source
		// is software or hardware.
		//
		CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
		if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
		{
			cout << "Unable to disable trigger mode (node retrieval). Aborting..." << endl;
			return -1;
		}

		CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
		if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
		{
			cout << "Unable to disable trigger mode (enum entry retrieval). Aborting..." << endl;
			return -1;
		}

		ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());

		cout << "Trigger mode disabled..." << endl;

		CEnumerationPtr ptrTriggerSource = nodeMap.GetNode("TriggerSource");
		if (!IsAvailable(ptrTriggerSource) || !IsWritable(ptrTriggerSource))
		{
			cout << "Unable to set trigger mode (node retrieval). Aborting..." << endl;
			return -1;
		}

		CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Software");
		if (!IsAvailable(ptrTriggerSourceSoftware) || !IsReadable(ptrTriggerSourceSoftware))
		{
			cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
			return -1;
		}

		ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());

		cout << "Trigger source set to software..." << endl;

		// Turn trigger mode on
		//
		// *** LATER ***
		// Once the appropriate trigger source has been set, turn trigger mode 
		// on in order to retrieve images using the trigger.
		//
		CEnumEntryPtr ptrTriggerModeOn = ptrTriggerMode->GetEntryByName("On");
		if (!IsAvailable(ptrTriggerModeOn) || !IsReadable(ptrTriggerModeOn))
		{
			cout << "Unable to enable trigger mode (enum entry retrieval). Aborting..." << endl;
			return -1;
		}

		ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());

		cout << "Trigger mode turned back on..." << endl << endl;
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}
	return result;
}


ImagePtr AcquireImage(CameraPtr pCam, INodeMap & nodeMap)
{
	// Set acquisition mode to continuous
	CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
	if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
	{
		cout << "Unable to set acquisition mode to continuous (node retrieval). Aborting..." << endl << endl;
		return -1;
	}

	CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
	if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
	{
		cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval). Aborting..." << endl << endl;
		return -1;
	}

	int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

	ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

	cout << "Acquisition mode set to continuous..." << endl;

	// Begin acquiring images
	pCam->BeginAcquisition();


	// Get user input
	cout << "Press the Enter key to get an image." << endl;
	getchar();

	// Execute software trigger
	CCommandPtr ptrSoftwareTriggerCommand = nodeMap.GetNode("TriggerSoftware");
	if (!IsAvailable(ptrSoftwareTriggerCommand) || !IsWritable(ptrSoftwareTriggerCommand))
	{
		cout << "Unable to execute trigger. Aborting..." << endl;
		return -1;
	}

	ptrSoftwareTriggerCommand->Execute();

	// Retrieve the next received image
	ImagePtr pResultImage = pCam->GetNextImage();

	if (pResultImage->IsIncomplete())
	{
		cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl << endl;
	}
	else
	{
		// Print image information
		cout << "Grabbed image: width = " << pResultImage->GetWidth() << ", height = " << pResultImage->GetHeight() << endl;
	}
	
	return pResultImage;
}

/*
 * This function shows how to convert between Spinnaker ImagePtr container to CVmat container used in OpenCV.
*/
int ConvertToCVmat(ImagePtr pImage)
{
	int result = 0;
	ImagePtr convertedImage = pImage->Convert(PixelFormat_BGR8, NEAREST_NEIGHBOR);

	unsigned int XPadding = convertedImage->GetXPadding();
	unsigned int YPadding = convertedImage->GetYPadding();
	unsigned int rowsize = convertedImage->GetWidth();
	unsigned int colsize = convertedImage->GetHeight();

	//image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding. 
	Mat cvimg = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
	namedWindow("current Image", CV_WINDOW_AUTOSIZE);
	imshow("current Image", cvimg);
	resizeWindow("current Image", rowsize / 2, colsize / 2);
	waitKey(1);//otherwise the image will not display...
	
	return result;
}

int RunSingleCamera(CameraPtr pCam)
{
	int result = 0;
	int err = 0;
	try
	{
		// Initialize camera
		pCam->Init();

		// Retrieve GenICam nodemap
		INodeMap & nodeMap = pCam->GetNodeMap();
		//set trigger to software trigger
		result = ConfigureTrigger(nodeMap);

		ImagePtr pImage;
		//take an image
		pImage = AcquireImage(pCam, nodeMap);
		
		//convert to CVmat format
		result = ConvertToCVmat(pImage);
		pImage->Release();
		pCam->EndAcquisition();
		// Deinitialize camera
		pCam->DeInit();
	}

	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}
	return result;
}


int main()
{
	int result = 0;
	// Retrieve singleton reference to system object
	SystemPtr system = System::GetInstance();

	// Retrieve list of cameras from the system
	CameraList camList = system->GetCameras();

	unsigned int numCameras = camList.GetSize();

	// Finish if there are no cameras
	if (numCameras == 0)
	{
		// Clear camera list before releasing system
		camList.Clear();

		// Release system
		system->ReleaseInstance();

		cout << "Not enough cameras!" << endl;
		cout << "Done! Press Enter to exit..." << endl;
		getchar();

		return -1;
	}

	// Run example on each camera
	for (unsigned int i = 0; i < numCameras; i++)
	{
		cout << endl << "Running example for camera " << i << "..." << endl;

		result = result | RunSingleCamera(camList.GetByIndex(i));

		cout << "Camera " << i << " example complete..." << endl << endl;
	}

	// Clear camera list before releasing system
	camList.Clear();

	// Release system
	system->ReleaseInstance();

	cout << endl << "Done! Press Enter to exit..." << endl;
	getchar();

	return result;
}

