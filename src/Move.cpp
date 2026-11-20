
#include "Move.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/input.hpp>
#include <godot_cpp/classes/input_event_key.hpp>
#include <godot_cpp/classes/engine.hpp>
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


// Optotrak setup stuff

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




// Optotrak rigidbody datastruct
typedef struct RigidBodyDataStruct
{
    struct OptotrakRigidStruct  pRigidData[NUM_RIGID_BODIES];
    Position3d                  p3dData[NUM_MARKERS];
} RigidBodyDataType;

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
char type[] = "standard";
char rigidBodyFile[] = "fingercoil";
char f[] = "optotrak";

char szNDErrorString[MAX_ERROR_STRING_LENGTH + 1];
char szProperty[32];
int nCurProperty;
int nCurFrame;

char *pChar;
char szInput[10];

using namespace std;

void Move::_bind_methods() {
	ClassDB::bind_method(D_METHOD("initScanner"), & Move:: initScanner);
	ClassDB::bind_method(D_METHOD("calibrate"), & Move:: calibrate);
	ClassDB::bind_method(D_METHOD("updateCoords"), & Move:: updateCoords);

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
    if(OptotrakLoadCameraParameters(type)!=OPTO_NO_ERROR_CODE ) goto error;
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
	rigidBodyFile,
	0 ) ) goto error; 
  if( RigidBodyChangeSettings(RIGID_BODY_1,NUM_MINIMUM_MARKER_FOR_RIGID_BODY,
  		90,             /* Cut off angle for marker inclusion in calcs.*/
  		(float)2.5,    /* Maximum 3-D marker error for this rigid body. */
  		(float)2.5,     /* Maximum raw sensor error for this rigid body. */
  		(float)2.5,     /* Maximum 3-D RMS marker error for this rigid body. */
  		(float)2.5,     /* Maximum raw sensor RMS error for this rigid body. */
  		OPTOTRAK_QUATERN_RIGID_FLAG | OPTOTRAK_RETURN_MATRIX_FLAG ) ) goto error;
  if( RigidBodyAddFromFile(RIGID_BODY_2,4,rigidBodyFile,0 ) ) goto error;
  if( RigidBodyChangeSettings(RIGID_BODY_2,NUM_MINIMUM_MARKER_FOR_RIGID_BODY,
  		90,             /* Cut off angle for marker inclusion in calcs.*/
  		(float)2.5,    /* Maximum 3-D marker error for this rigid body. */
  		(float)2.5,     /* Maximum raw sensor error for this rigid body. */
  		(float)2.5,     /* Maximum 3-D RMS marker error for this rigid body. */
  		(float)2.5,     /* Maximum raw sensor RMS error for this rigid body. */
  		OPTOTRAK_QUATERN_RIGID_FLAG | OPTOTRAK_RETURN_MATRIX_FLAG ) ) goto error;


	printf("Initialized!\n");

	fflush(stdout);

    return 0;

	error:
	if( OptotrakGetErrorString( szNDErrorString,
                                MAX_ERROR_STRING_LENGTH + 1 ) == 0 )
    {
		cout << "\033[1mAn error was detected. Message:\033[0m\n";
        cout << szNDErrorString;
		cout << "\n";
	}
	return -1;

}

void Move::calibrate(){

	// get latest position
	this_thread::sleep_for(std::chrono::milliseconds(1000)); // replaced C sleep()
	
	//Get children for godot to move child objects
	printf("Getting Godot child objects...\n");
	fingers = {get_node<Node3D>("Finger1"), get_node<Node3D>("Finger2")};

	updateCoords();

	cout << "Calibrating\n";
	for (int n = 0; n < fingers.size(); n++) {
		corrections[n] = -coords[n];
	}

}

void Move::_ready() {
	UtilityFunctions::print("cpp started");
	initScanner();
	calibrate();
}

void Move::updateCoords(){
	if(DataGetLatestTransforms( &uFrameNumber, &uElements, &uFlags, &RigidBodyData )) printf("Failed to get transform data.");
	//store in arrays
	coords = {Vector3(r1_x, r1_y, r1_z), Vector3(r2_x, r2_y, r2_z)};
	bases = {Basis(
				Vector3(r1_xx, r1_yx, r1_zx),
				Vector3(r1_xy, r1_yy, r1_zy),
				Vector3(r1_xz, r1_yz, r1_zz)
			), Basis(
				Vector3(r2_xx, r2_yx, r2_zx),
				Vector3(r2_xy, r2_yy, r2_zy),
				Vector3(r2_xz, r2_yz, r2_zz)
			)};
}

void Move::_process(double delta){
	int fps = Engine::get_singleton()->get_frames_per_second();

	printf("FPS: %d\n", fps);
	
	//update rbody data
	
	updateCoords();
	
	for (int n = 0; n < fingers.size(); n++) {
		if(!fingers[n]){
			printf("Error: finger child object not detected!");
		}
		else{
			// transform the new position by translating to the initial coordinate and then converting from mm to m.
			Vector3 newPosition = 0.01 * (coords[n] + corrections[n]);
			// transform from optotrak (z up) to godot (y up)
			double y = newPosition.y;
			newPosition.y = newPosition.z;
			newPosition.z = -y;

			fingers[n]->set_position(newPosition);

			// now prepare to rotate using the quaternions

			// flip y and z axis to transform from optotrak standards to godot

			Basis switchYZ(
				Vector3(1, 0, 0),
				Vector3(0, 0, -1),
				Vector3(0, 1, 0)
			);
			// matmul mirror basis to transform original basis
			Basis mirrored = switchYZ * bases[n].transposed() * switchYZ.inverse();

			fingers[n]->set_basis(mirrored);
		}
	}

}

Move::Move() {

}

void Move::_input(const Ref<InputEvent> &event) {
    Ref<InputEventKey> key_event = event;

    if (key_event.is_valid() && key_event->is_pressed() && !key_event->is_echo()) {
        if (key_event->get_keycode() == Key::KEY_SPACE) {
            // Do something when SPACE is pressed
			cout << "this works";
			// initScanner();
		}
    }
}