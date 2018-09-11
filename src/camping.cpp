#include <WalabotAPI.h>
#include <iostream>
#include <stdio.h>
#include <string>

#define CHECK_WALABOT_RESULT(result, func_name)					\
{																\
	if (result != WALABOT_SUCCESS)								\
	{															\
		unsigned int extended = Walabot_GetExtendedError();		\
		const char* errorStr = Walabot_GetErrorString();		\
		std::cout << std::endl << "Error at " __FILE__ << ":"	\
                  << std::dec << __LINE__ << " - "				\
				  << func_name << " result is 0x" << std::hex	\
                  << result << std::endl;						\
																\
		std::cout << "Error string: " << errorStr << std::endl; \
																\
		std::cout << "Extended error: 0x" << std::hex			\
                  << extended << std::endl << std::endl;		\
																\
		std::cout << "Press enter to continue ...";				\
		std::string dummy;										\
		std::getline(std::cin, dummy);							\
		return;													\
	}															\
}

// This function store the collected data that comes from walabot sensor, the storage data will be
// transmitted to the iot-cloud via another sorce program (inside the folder thinger-Linux-Client).
// Remeber to change the text file path according to your system.
void setValue(int value)
{
	FILE *fptr;
	fptr = (fopen("/home/pi/Documents/walabot_project/program.txt", "w")); //  you need to change this path
	fprintf(fptr,"%d", value);
	fclose(fptr);
}
void PrintSensorTargets(SensorTarget* targets, int numTargets)
{
	int targetIdx;

    #ifdef __LINUX__
		printf("\033[2J\033[1;1H");
	#else
	    system("cls");
	#endif

	if (numTargets > 0)
	{
		for (targetIdx = 0; targetIdx < numTargets; targetIdx++)
		{
			printf("Target #%d: \nX = %lf \nY = %lf \nZ = %lf \namplitude = %lf\n\n\n ",
				targetIdx,
				targets[targetIdx].xPosCm,
				targets[targetIdx].yPosCm,
				targets[targetIdx].zPosCm,
				targets[targetIdx].amplitude);
		}
		
		//upload data to cloud
		//Call a function for uploading the sensor data to a text file.
		setValue(numTargets);

	


	}
	else
	{
		printf("No target detected\n");
		setValue(0);
	}
}

void SensorCode_SampleCode()
{
	// --------------------
	// Variable definitions
	// --------------------
	WALABOT_RESULT res;

	// Walabot_GetSensorTargets - output parameters
	SensorTarget* targets;
	int numTargets;

	// Walabot_GetStatus - output parameters
	APP_STATUS appStatus;
	double calibrationProcess; // Percentage of calibration completed, if status is STATUS_CALIBRATING

   // Walabot_GetRawImageSlice - output parameters
	int*	rasterImage;
	int		sizeX;
	int		sizeY;
	double	sliceDepth;
	double	power;

	// ------------------------
	// Initialize configuration
	// ------------------------

	// Walabot_SetArenaR - input parameters
	double minInCm = 30;
	double maxInCm = 200;
	double resICm = 3;

	// Walabot_SetArenaTheta - input parameters
	double minIndegrees = -15;
	double maxIndegrees = 15;
	double resIndegrees = 5;

	// Walabot_SetArenaPhi - input parameters
	double minPhiInDegrees = -60;
	double maxPhiInDegrees = 60;
	double resPhiInDegrees = 5;

	// ----------------------
	// Sample Code Start Here
	// ----------------------

	/*
		For an image to be received by the application, the following need to happen :
		1) Connect
		2) Configure
		3) Calibrate
		4) Start
		5) Trigger
		6) Get action
		7) Stop/Disconnect
	*/

	bool mtiMode = true;

	// Configure Walabot database install location
	#ifdef __LINUX__
	    res = Walabot_SetSettingsFolder((char*)"/var/lib/walabot");
	#else
	    res = Walabot_SetSettingsFolder("C:/ProgramData/Walabot/WalabotSDK");
	#endif
	
	CHECK_WALABOT_RESULT(res, "Walabot_SetSettingsFolder");	

	//	1) Connect : Establish communication with Walabot.
	//	==================================================
	res = Walabot_ConnectAny();
	CHECK_WALABOT_RESULT(res, "Walabot_ConnectAny");

	//  2) Configure : Set scan profile and arena
	//	=========================================

	// Set Profile - to Sensor. 
	//			Walabot recording mode is configure with the following attributes:
	//			-> Distance scanning through air; 
	//			-> high-resolution images
	//			-> slower capture rate 
	res = Walabot_SetProfile(PROF_SENSOR);
	CHECK_WALABOT_RESULT(res, "Walabot_SetProfile");

	// Setup arena - specify it by Cartesian coordinates(ranges and resolution on the x, y, z axes); 
	//	In Sensor mode there is need to specify Spherical coordinates(ranges and resolution along radial distance and Theta and Phi angles).
	res = Walabot_SetArenaR(minInCm, maxInCm, resICm);
	CHECK_WALABOT_RESULT(res, "Walabot_SetArenaR");

	// Sets polar range and resolution of arena (parameters in degrees).
	res = Walabot_SetArenaTheta(minIndegrees, maxIndegrees, resIndegrees);
	CHECK_WALABOT_RESULT(res, "Walabot_SetArenaTheta");

	// Sets azimuth range and resolution of arena.(parameters in degrees).
	res = Walabot_SetArenaPhi(minPhiInDegrees, maxPhiInDegrees, resPhiInDegrees);
	CHECK_WALABOT_RESULT(res, "Walabot_SetArenaPhi");

	FILTER_TYPE filterType = mtiMode ?
		FILTER_TYPE_MTI :		//Moving Target Identification: standard dynamic-imaging filter
		FILTER_TYPE_NONE;

	res = Walabot_SetDynamicImageFilter(filterType);
	CHECK_WALABOT_RESULT(res, "Walabot_SetDynamicImageFilter");

	//	3) Start: Start the system in preparation for scanning.
	//	=======================================================
	res = Walabot_Start();
	CHECK_WALABOT_RESULT(res, "Walabot_Start");

	//	4) Trigger: Scan(sense) according to profile and record signals to be available
	//	for processing and retrieval.
	//	================================================================================
	if (!mtiMode) // if MTI mode is not set - start calibrartion
	{
		// calibrates scanning to ignore or reduce the signals
		res = Walabot_StartCalibration();
		CHECK_WALABOT_RESULT(res, "Walabot_StartCalibration");
	}

	bool recording = true;

	while (recording)
	{
		// calibrates scanning to ignore or reduce the signals
		res = Walabot_GetStatus(&appStatus, &calibrationProcess);
		CHECK_WALABOT_RESULT(res, "Walabot_GetStatus");

		//	5) Trigger: Scan(sense) according to profile and record signals to be 
		//	available for processing and retrieval.
		//	====================================================================
		res = Walabot_Trigger();
		CHECK_WALABOT_RESULT(res, "Walabot_Trigger");

		//	6) 	Get action : retrieve the last completed triggered recording 
		//	================================================================
		res = Walabot_GetSensorTargets(&targets, &numTargets);
		CHECK_WALABOT_RESULT(res, "Walabot_GetSensorTargets");

		res = Walabot_GetRawImageSlice(&rasterImage, &sizeX, &sizeY, &sliceDepth, &power);
		CHECK_WALABOT_RESULT(res, "Walabot_GetRawImageSlice");

		//	******************************
		//	TODO: add processing code here
		//	******************************
		PrintSensorTargets(targets, numTargets);
	}

	//	7) Stop and Disconnect.
	//	======================
	res = Walabot_Stop();
	CHECK_WALABOT_RESULT(res, "Walabot_Stop");

	res = Walabot_Disconnect();
	CHECK_WALABOT_RESULT(res, "Walabot_Disconnect");
}


#ifndef _SAMPLE_CODE_
int main()
{
	SensorCode_SampleCode();
}
#endif
