
#include "Move.h"
#include <godot_cpp/core/class_db.hpp>

// OPTOTRAK INCLUDES

// Include all the main cpp functionality stuff
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <sys/time.h>

#include <string.h>


// Include all the libraries in api
#include <ndtypes.h>
#include <ndhost.h>
#include <ndpack.h>
#include <ndopto.h>
// C code stuff for OPTOTRAK

extern "C" {
	#include "certus_aux.h"
	#include "ot_aux.h"
}
#define PI 3.1415926535898

#define SAMPLE_MARKERFREQ	4600.0f
#define SAMPLE_FRAMEFREQ	650.0f //860.0f // 860 Hz frames
#define SAMPLE_DUTYCYCLE	0.35f//0.35f
#define SAMPLE_VOLTAGE		7.0f
#define SAMPLE_STREAMDATA	0
#define NUM_MARKERS	    	6 //total markers
#define RIGID_BODY_1        	0 //ID associated with this rigid body
#define RIGID_BODY_2        	1 //ID associated with this rigid body
#define NUM_RIGID_BODIES    	2 //total rigid numbers
#define NUM_MINIMUM_MARKER_FOR_RIGID_BODY  3 //minimum maker number for rb calc

#define TICKMAX                 1000 // one second of data at a time


#define r1_x	RigidBodyData.pRigidData[0].transformation.rotation.translation.x
#define r1_y	RigidBodyData.pRigidData[0].transformation.rotation.translation.y
#define r1_z	RigidBodyData.pRigidData[0].transformation.rotation.translation.z
#define r1_xx	RigidBodyData.pRigidData[0].transformation.rotation.matrix[0][0]
#define r1_xy	RigidBodyData.pRigidData[0].transformation.rotation.matrix[1][0]
#define r1_xz	RigidBodyData.pRigidData[0].transformation.rotation.matrix[2][0]
#define r1_yx	RigidBodyData.pRigidData[0].transformation.rotation.matrix[0][1]
#define r1_yy	RigidBodyData.pRigidData[0].transformation.rotation.matrix[1][1]
#define r1_yz	RigidBodyData.pRigidData[0].transformation.rotation.matrix[2][1]
#define r1_zx	RigidBodyData.pRigidData[0].transformation.rotation.matrix[0][2]
#define r1_zy	RigidBodyData.pRigidData[0].transformation.rotation.matrix[1][2]
#define r1_zz	RigidBodyData.pRigidData[0].transformation.rotation.matrix[2][2]

#define r2_x	RigidBodyData.pRigidData[1].transformation.rotation.translation.x
#define r2_y	RigidBodyData.pRigidData[1].transformation.rotation.translation.y
#define r2_z	RigidBodyData.pRigidData[1].transformation.rotation.translation.z
#define r2_xx	RigidBodyData.pRigidData[1].transformation.rotation.matrix[0][0]
#define r2_xy	RigidBodyData.pRigidData[1].transformation.rotation.matrix[1][0]
#define r2_xz	RigidBodyData.pRigidData[1].transformation.rotation.matrix[2][0]
#define r2_yx	RigidBodyData.pRigidData[1].transformation.rotation.matrix[0][1]
#define r2_yy	RigidBodyData.pRigidData[1].transformation.rotation.matrix[1][1]
#define r2_yz	RigidBodyData.pRigidData[1].transformation.rotation.matrix[2][1]
#define r2_zx	RigidBodyData.pRigidData[1].transformation.rotation.matrix[0][2]
#define r2_zy	RigidBodyData.pRigidData[1].transformation.rotation.matrix[1][2]
#define r2_zz	RigidBodyData.pRigidData[1].transformation.rotation.matrix[2][2]

// Optotrak rigidbody datastruct
typedef struct RigidBodyDataStruct
{
    struct OptotrakRigidStruct  pRigidData[NUM_RIGID_BODIES];
    Position3d                  p3dData[NUM_MARKERS];
} RigidBodyDataType;

// Input OPTOTRAK settings

OptotrakSettings				dtSettings;
ApplicationDeviceInformation	*pdtDevices;
DeviceHandle					*pdtDeviceHandles;
DeviceHandleInfo				*pdtDeviceHandlesInfo;
unsigned int					uFlags,uElements,uFrameNumber;
RigidBodyDataType				RigidBodyData;
int uRigidCnt,ni, nIterations,nCurDevice,nCurMarker,nDevices,nMarkersToActivate;

double trajdata[TICKMAX][56];
int counter,trajcounter,callbackcounter,command;


int i,j;
FILE *fplog;

char s[] = "system";
char f[] = "optotrak";

char szNDErrorString[MAX_ERROR_STRING_LENGTH + 1];
char szProperty[32];
int nCurProperty;
int nCurFrame;

char *pChar;
char szInput[10];

Vector3 correction;
using namespace std;

void Move::_bind_methods() {
	ClassDB::bind_method(D_METHOD("initScanner"), & Move:: initScanner);
	ClassDB::bind_method(D_METHOD("calibrate"), & Move:: calibrate);
}

int Move::initScanner() {

		//optotrak variables init*****************************************
	callbackcounter=0; counter = 0; trajcounter=0; command = 0;

	pdtDevices = NULL;
	pdtDeviceHandles = NULL;
	pdtDeviceHandlesInfo = NULL;
	dtSettings.nMarkers = NUM_MARKERS;
	dtSettings.fFrameFrequency = SAMPLE_FRAMEFREQ;
	dtSettings.fMarkerFrequency = SAMPLE_MARKERFREQ;
	dtSettings.nThreshold = 30;//30
	dtSettings.nMinimumGain = 160;//
	dtSettings.nStreamData = SAMPLE_STREAMDATA;
	dtSettings.fDutyCycle = SAMPLE_DUTYCYCLE;
	dtSettings.fVoltage = SAMPLE_VOLTAGE;
	dtSettings.fCollectionTime = 1.0;
	dtSettings.fPreTriggerTime = 0.0;
	nDevices = 0;
	nMarkersToActivate = NUM_MARKERS;

	// Run init api commands
	cout << "Loading system...\n";

    if (TransputerLoadSystem( s ) != OPTO_NO_ERROR_CODE) goto error;

	cout << "System loaded!\n";
	cout << "Initializing system...\n";
    if(TransputerInitializeSystem(OPTO_LOG_ERRORS_FLAG|OPTO_LOG_MESSAGES_FLAG)!=OPTO_NO_ERROR_CODE) goto error;

	cout << "System initialized!\n";
	cout << "Determining system...\n";
	if(uDetermineSystem()!= OPTOTRAK_CERTUS_FLAG ) goto error;

	cout << "Determined system!\n";
	cout << "Determine strober config...\n";
	if(DetermineStroberConfiguration(&pdtDeviceHandles,&pdtDeviceHandlesInfo,&nDevices)!=OPTO_NO_ERROR_CODE ) goto error;

	ApplicationStoreDeviceProperties(&pdtDevices,pdtDeviceHandlesInfo,nDevices);

	printf("device number %d\n",pdtDevices[0].nDeviceType);

	cout << "Setting processing flags...\n";

	if(OptotrakSetProcessingFlags(OPTO_LIB_POLL_REAL_DATA|OPTO_CONVERT_ON_HOST|OPTO_RIGID_ON_HOST)) goto error;
	cout << "Processing flags set.\n";
	cout << "Loading camera params...\n";
    if(OptotrakLoadCameraParameters("standard")!=OPTO_NO_ERROR_CODE ) goto error;
        cout << "Camera params loaded.\n";
	OptotrakEnableWireless(-1,1); // needed or not????

    cout << "Marker activation... \n";
	// printf("%d\n", nMarkersToActivate);

	if(SetMarkersToActivateForDevice( &(pdtDevices[0]), pdtDeviceHandlesInfo[0].pdtHandle->nID, nMarkersToActivate ) != OPTO_NO_ERROR_CODE) goto error;

	ApplicationDetermineCollectionParameters(nDevices,pdtDevices,&dtSettings);

	cout << "Setting up collection...\n";	
	printf(".....%d, %.2f, %.0f, %d, %d, %d, %.2f, %.2f, %.0f, %.0f\n",
		dtSettings.nMarkers,dtSettings.fFrameFrequency,dtSettings.fMarkerFrequency,
		dtSettings.nThreshold,dtSettings.nMinimumGain,dtSettings.nStreamData,
		dtSettings.fDutyCycle,dtSettings.fVoltage,
		dtSettings.fCollectionTime,dtSettings.fPreTriggerTime );
	if(OptotrakSetupCollection(dtSettings.nMarkers, dtSettings.fFrameFrequency, 
		dtSettings.fMarkerFrequency, dtSettings.nThreshold, dtSettings.nMinimumGain, 
		dtSettings.nStreamData, dtSettings.fDutyCycle, dtSettings.fVoltage,
		dtSettings.fCollectionTime, dtSettings.fPreTriggerTime,
		OPTOTRAK_NO_INTERPOLATION_FLAG|OPTOTRAK_BUFFER_RAW_FLAG|OPTOTRAK_NO_FIRE_MARKERS_FLAG)
		!= OPTO_NO_ERROR_CODE ) goto error;

	fflush(stdout);

	//SetMarkersToActivateForDevice( &(pdtDevices[0]), pdtDeviceHandlesInfo[0].pdtHandle->nID, nMarkersToActivate );

	this_thread::sleep_for(std::chrono::milliseconds(1000)); // replaced C sleep()

		// Get markers going
	if( OptotrakActivateMarkers()!=OPTO_NO_ERROR_CODE) goto error;

    cout << "Success!!\n";

	cout << "Importing rigid body file\n";
  if( RigidBodyAddFromFile(
	RIGID_BODY_1,
	1,
	"fingercoil",
	0 ) ) goto error; 
  if( RigidBodyChangeSettings(RIGID_BODY_1,NUM_MINIMUM_MARKER_FOR_RIGID_BODY,
  		90,             /* Cut off angle for marker inclusion in calcs.*/
  		(float)2.5,    /* Maximum 3-D marker error for this rigid body. */
  		(float)2.5,     /* Maximum raw sensor error for this rigid body. */
  		(float)2.5,     /* Maximum 3-D RMS marker error for this rigid body. */
  		(float)2.5,     /* Maximum raw sensor RMS error for this rigid body. */
  		OPTOTRAK_QUATERN_RIGID_FLAG | OPTOTRAK_RETURN_MATRIX_FLAG ) ) goto error;
  if( RigidBodyAddFromFile(RIGID_BODY_2,4,"fingercoil",0 ) ) goto error;
  if( RigidBodyChangeSettings(RIGID_BODY_2,NUM_MINIMUM_MARKER_FOR_RIGID_BODY,
  		90,             /* Cut off angle for marker inclusion in calcs.*/
  		(float)2.5,    /* Maximum 3-D marker error for this rigid body. */
  		(float)2.5,     /* Maximum raw sensor error for this rigid body. */
  		(float)2.5,     /* Maximum 3-D RMS marker error for this rigid body. */
  		(float)2.5,     /* Maximum raw sensor RMS error for this rigid body. */
  		OPTOTRAK_QUATERN_RIGID_FLAG | OPTOTRAK_RETURN_MATRIX_FLAG ) ) goto error;


	printf("Initialized!\n");


    return 0;

	error:
	if( OptotrakGetErrorString( szNDErrorString,
                                MAX_ERROR_STRING_LENGTH + 1 ) == 0 )
    {
		cout << "\033[1mAn error was detected. Message:\033[0m\n";
        cout << szNDErrorString;
		cout << "\n";
		return -1;
	}
}

void Move::calibrate(){
	// get latest position
	this_thread::sleep_for(std::chrono::milliseconds(1000)); // replaced C sleep()
	if(DataGetLatestTransforms( &uFrameNumber, &uElements, &uFlags, &RigidBodyData )) printf("Failed to get transform data.");

	correction = Vector3(-r1_x,-r1_y,-r1_z);
}

void Move::_ready() {
	UtilityFunctions::print("cpp started");
	initScanner();
	calibrate();
}


void Move::_process(double delta){
	//update rbody data
	if(DataGetLatestTransforms( &uFrameNumber, &uElements, &uFlags, &RigidBodyData )) printf("Failed to get transform data.");
	
	// transform the new position by translating to the initial coordinate and then converting from mm to m.
	Vector3 newPosition = 0.01 * Vector3(r1_x+correction.x,r1_y+correction.y,r1_z+correction.z);
	// transform from optotrak (z up) to godot (y up)
	double y = newPosition.y;
	newPosition.y = newPosition.z;
	newPosition.z = -y;

	set_position(newPosition);

	// now prepare to rotate using the quaternions
	// flip y and z axis to transform from optotrak to godot

	Basis basis(
		Vector3(r1_xx, r1_yx, r1_zx),
		Vector3(r1_xy, r1_yy, r1_zy),
		Vector3(r1_xz, r1_yz, r1_zz)
	);

	Basis mirrorY(
		Vector3(1, 0, 0),
		Vector3(0, -1, 0),
		Vector3(0, 0, -1)
	);
	Basis mirrored = mirrorY * basis * mirrorY;
	Quaternion rotation = Quaternion(mirrored);



	set_quaternion(rotation);
}

Move::Move() {
	printf("yeet");
}

