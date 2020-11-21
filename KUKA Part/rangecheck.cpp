#include <sys/time.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include "PositionControlClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <time.h>
#include <sys/shm.h>
#include <eigen3/Eigen/Dense>
#include "UdpServer.h"
#include "TrignoEmgClient.h"
#include <algorithm>
/* Boost filesystem */
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>


using namespace std;
using namespace KUKA::FRI;
using namespace Eigen;
using namespace boost::filesystem;

/* IP Address/Port for KONI Connection */
#define DEFAULT_PORTID 30200
#define DEFAULT_IP "192.170.10.2"

#define DEFAULT_TRIALNUMBER 0

/* GUI UDP Server Address/Port */
const std::string 	udp_addr_gui("192.168.0.102");
const int 			udp_port_gui = 50000;




/* Shared Memory Function Prototype */
template<typename T>
T * InitSharedMemory(std::string shmAddr, int nElements);
void CreateOrOpenKukaDataFile(boost::filesystem::ofstream & ofs, path kukaDataFilePath);


// Main
int main(int argc, char** argv)
{
	// UDP Server address and port hardcoded now -- change later
	UDPServer udp_server(udp_addr_gui, udp_port_gui);

	/* Command line arguments */
	if (argc < 3)
	{
		printf("Use: 2DVDC <SubjectNumber> <BlockNumber> <optional: TrialNumber>\n");
		exit(1);
	}
	int subjectNumber 	= atoi(argv[1]);
	int blockNum 	= atoi(argv[2]);

	/* Check Inputted Subject Number */
	if (subjectNumber < 0)
	{
		printf("Subject Number Must Be Nonnegative\n");
		printf("Inputted Subject Number: %d\n", subjectNumber);
		exit(1);
	}
	else
	{
		printf("Subject: %d\n", subjectNumber);
	}

	/* Parse the rest of the command line args */
	int trialNum;
	std::string emgIpAddr;
	const double DEFAULT_KP_AP = 63.48;
	const double DEFAULT_KN_AP = 32.81;
	const double DEFAULT_KP_ML = 63.48;
	const double DEFAULT_KN_ML = 32.81;
	MatrixXd kp(2, 1);
	MatrixXd kn(2, 1);

	bool useEmg = false;
	bool kpInputted_ML = false;
	bool kpInputted_AP = false;
	bool knInputted_ML = false;
	bool knInputted_AP = false;
	bool trialNumberInputted = false;
	bool tuneInputted = false;

	std::string argKey;
	std::string argVal;
	for (int iArg=3; iArg<(argc-1); iArg+=2)
	{
		// get key and val
		argKey = std::string(argv[iArg]);
		argVal = std::string(argv[iArg+1]);
		// set key uppercase 
		std::transform(argKey.begin(), argKey.end(), argKey.begin(), ::toupper);

		if (argKey.compare("TN") == 0)
		{
			trialNumberInputted = true;
			trialNum = std::stoi(argVal);
		}
		else if (argKey.compare("EMG") == 0)
		{
			useEmg = true;
			emgIpAddr = argVal;
		}
		else if (argKey.compare("KPAP") == 0)
		{
			kpInputted_AP = true;
			kp(0) = std::stod(argVal);
		}
		else if (argKey.compare("KNAP") == 0)
		{
			knInputted_AP = true;
			kn(0) = std::stod(argVal);
		}
		else if (argKey.compare("KPML") == 0)
		{
			kpInputted_ML = true;
			kp(1) = std::stod(argVal);
		}
		else if (argKey.compare("KNML") == 0)
		{
			knInputted_ML = true;
			kn(1) = std::stod(argVal);
		}
		else if (argKey.compare("TUNE") == 0)
		{
			tuneInputted = true;
		}
		else
		{
			printf("Key: %s not understood\n", argKey.c_str());
		}
	}

	// Check kp and kn values
	if (!kpInputted_ML)
	{
		kp(1) = DEFAULT_KP_ML;
	}
	if (!knInputted_ML)
	{
		kn(1) = DEFAULT_KN_ML;
	}
	if (!kpInputted_AP)
	{
		kp(0) = DEFAULT_KP_AP;
	}
	if (!knInputted_AP)
	{
		kn(0) = DEFAULT_KN_AP;
	}

	// Check Inputted Trial Number
	if (!trialNumberInputted)
	{
		trialNum = (int) DEFAULT_TRIALNUMBER;
	}

	// Check EMG use
	TrignoEmgClient emgClient;
	if (useEmg)
	{
		emgClient.SetIpAddress(emgIpAddr);
		emgClient.ConnectDataPort();
		emgClient.ConnectCommPort();
		if (emgClient.IsCommPortConnected())
		{
			// Check if sensors are paired
			emgClient.IsSensorPaired(1);
			emgClient.IsSensorPaired(2);
			emgClient.IsSensorPaired(3);
			emgClient.IsSensorPaired(4);
			emgClient.IsSensorPaired(5);
			emgClient.IsSensorPaired(6);
			
			/* Turns on backwards compatibility and upsampling, then prints the confrimation that it is on, also prints sampling rate*/
			emgClient.CheckBackwards(2); 
			emgClient.CheckBackwards(3); 
			emgClient.CheckBackwards(4); 
			emgClient.CheckBackwards(5);

			emgClient.SendCommand(1); // this command signals the emg server to send readings to Data Port
			std::thread emgReceiveThread(&TrignoEmgClient::ReceiveDataStream, &emgClient);
			emgReceiveThread.detach();
		}
	}

	// Setup damping modes
	MatrixXd groupDampingModes(1, 14);
	MatrixXd pathNumber(1, 14); 
	MatrixXd directionMatrix(1, 14);
	MatrixXd b_LBMatrix(2, 14);
	if(!tuneInputted)
	{
		//groupDampingModes << 1, 1, 2, 2, 1, 2, 1, 2, 1, 2, 2, 1, 1, 2;
		//pathNumber << 0, 2, 3, 4, 1, 2, 4, 1, 3, 0, 6, 5, 6, 5;
		groupDampingModes << 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 2;
		pathNumber << 0, 2, 3, 4, 1, 2, 4, 1, 5, 6, 0, 5, 3, 6;
	}
	else
	{
		printf("Tuning\n");
		groupDampingModes << 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2; 
		pathNumber << 0, 2, 1, 3, 1, 3, 2, 5, 0, 4, 3, 1, 4, 5; 
		directionMatrix << 1, 2, 1, 2, 1, 2, 2, 2, 1, 1, 2, 1, 1, 2;
		b_LBMatrix << -10, -10, -10, -10, -15, -10, -10, -10, -10, -25, -10, -20, -5, -10,
					  -30, -30, -30, -30, -15, -20, -30, -50, -30, -30, -40, -30, -30, -10;

	}
	int pathNum = pathNumber(blockNum-1);
	int groupDamping;
	int direction = directionMatrix(blockNum-1);

	/* Check Inputted Block Number */
	if (blockNum > (int) groupDampingModes.cols() || blockNum < 1)
	{
		printf("Block Number Out of Acceptable Range\n");
		printf("Inputted Block Number: %d\n", blockNum);
		printf("Min: 1\n");
		printf("Max: %d\n", (int) groupDampingModes.cols()-1);
		exit(1);
	}
	else
	{
		groupDamping = groupDampingModes(blockNum-1);
		printf("Block: %d\n", blockNum);
	}

	// Definition of target points
	MatrixXd targetx(7, 11); 
	MatrixXd targety(7, 11); 
	
	if (!tuneInputted)
	{
		targetx << 0,-8.7,-1.1,9,-7.9,-0.6,7.2,-5.1,3.9,-8.2,0,
					0,7,-7.3,7.5,-8.5,-1.5,-6.8,2.9,-2.3,9.2,0,
					0,-7.5,9.5,-7.1,5.8,0.4,7.5,1.8,-3.2,9.8,0,
					0,8.7,-7,3.6,9.4,4.1,-9.7,-4.2,6.7,-8.5,0,
					0,-8.8,4.5,-7.7,7.3,-0.6,-6.2,0,-9.9,5.6,0,
					0,-8.7,-1.1,9,-7.9,-0.6,7.2,-5.1,3.9,-8.2,0,
					0,-7.5,9.5,-7.1,5.8,0.4,7.5,1.8,-3.2,9.8,0;
	
		targety << 0,-5.3,9.5,0.4,8.8,-2.3,3.7,-4.9,5.9,-10,0,
					0,6.1,-7.3,9.2,-4.4,1.4,-5,8.4,1.9,-8.2,0,
					0,-8.1,-1,8.3,-7.9,8.7,-0.5,-7.1,-1.7,9.9,0,
					0,-6.1,9,2.6,-3.5,-8.7,9.5,-9.9,-3.4,6.8,0,
					0,9.2,-3.5,5,-7.6,7.6,0.6,9.5,-1,-8.2,0,
					0,6.1,-7.3,9.2,-4.4,1.4,-5,8.4,1.9,-8.2,0,
					0,-6.1,9,2.6,-3.5,-8.7,9.5,-9.9,-3.4,6.8,0;
	}
	else
	{
		targetx << 0, 10, 10, -10, 10, -10, 10, -10, 10, -10, -10,
											0, -10, 10, 10, -10, -10, 10, -10, 10, 10, -10,
											0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											0, -10, 10, -10, 10, -10, -10, 10, 10, -10, 10,
											0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											0,-8.7,-1.1,9,-7.9,-0.6,7.2,-5.1,3.9,-8.2,0;

		targety << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											0, 10, 10, -10, 10, -10, 10, -10, 10, -10, -10,
											0, -10, 10, 10, -10, -10, 10, -10, 10, 10, -10,
											0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
											0, -10, -10, -10, 10, 10, -10, 10, 10, -10, 10,
											0,-5.3,9.5,0.4,8.8,-2.3,3.7,-4.9,5.9,-10,0;
	}
	

	int nTrialsPerBlock = targetx.cols();
	if (trialNum >= nTrialsPerBlock || trialNum < 0)
	{
		printf("Trial Number Out of Acceptable Range\n");
		printf("Inputted Trial Number: %d\n", trialNum);
		printf("Min: 0\n");
		printf("Max: %d\n", nTrialsPerBlock -1);
		exit(1);
	}

	//***** path
	// Saving file
	path p_base = current_path();
	// subject directory
	std::string subjectDir = "Subject" + std::to_string(subjectNumber);
	path p_subject = path(p_base.string()) /= path(subjectDir);
	create_directory(p_subject);
	
	//path p_direction;
	// if(tuneInputted)
	// {
	// 	std::string tuneDir = "Tuning";
	// 	path p_tune = path(p_subject.string()) /= path(tuneDir);
	// 	create_directory(p_tune);

	// 	std::string groupDir = "group" + std::to_string(groupDamping);
	// 	p_group = path(p_tune.string()) /= path(groupDir);
	// 	create_directory(p_group);
	// }
	// else
	// {
	std::string diectionDir = "Direction" + std::to_string(direction);
	path p_direction = path(p_subject.string()) /= path(diectionDir);
	create_directory(p_direction);
	//}

	std::string pathDir = "Block" + std::to_string(blockNum);
	path p_block = path(p_direction.string()) /= path(pathDir);
	create_directory(p_block);

	std::string trialDir;
	path p_trial;
	path p_kukadata;
	path p_emgdata;
	boost::filesystem::ofstream OutputFile;

	// EMG File
	std::string emgfilename = "EmgData.txt";

	// Kuka Data File
	std::string kukafilename = "KukaData.txt";

	// Measured Torque
	double meas_torque[7];

	// Variables related to target reaching
	MatrixXd endEffectorXY(2, 1); endEffectorXY << 0, 0;
	MatrixXd neutralXY(2, 1); neutralXY << 0, 0;
	MatrixXd targetXY(2, 1); targetXY << 0, 0;
	MatrixXd targetXYold(2, 1); targetXYold << 0, 0;
	MatrixXd temptarget(2, 1); temptarget << 0.1, 0;
	double ycenter = 0.76;
	bool withinErrorBound;
	bool targetReached = false;

	//Variables related to defining new trial
	bool endBlock = false;
	bool startBlock = false;
	bool readyTrials = false;
	bool startTrial = false;
	int trialSettleIterCount = 0;
	int const trialEndIterCount = 2000;
	int trialWaitCounter = 0;
	int trialWaitTime = rand() % 1000 + 500;
	int beep_flag = 0;
	int beep_count = 0;
	int beep = 0;
	int beep_count1 = 0;
	

	// Command line arguments
	const char* hostname =  DEFAULT_IP;
	int port = DEFAULT_PORTID;

	// Force Related Varibles
	double ftx;						// Force x-direction (filtered)
	double fty;						// Force y-direction (filtered)
	double ftx_un;					// Force x-direction (unfiltered)
	double fty_un;					// Force y-direction (unfiltered)
	double zerox = 0;				// Force x-baseline
	double zeroy = 0;				// Force y-baseline
	double ftx_0 = 0.0;				// Part of force filtering calc
	double fty_0 = 0.0;  			// Part of force filtering calc
	double al = 0.5;				// exponential moving average alpha level

	// Shared Memory Setup
	std::string shmAddr("/home/justin/Desktop/FRI-Client-SDK_Cpp/example/PositionControl2/shmfile");
	int shmnElements = 2;
	int * data = InitSharedMemory<int>(shmAddr, shmnElements);

	// Force baseline variables/flags 
	int firstIt = 0;			// first iteration flag
	int steady = 0;				// Flag to collect average first 2000 samples of forces without moving KUKA

	
	// Variables related to variable dampings
	float dt = 0.001;
	MatrixXd b_var(2, 1); b_var << 30, 30;					// Variable damping
	MatrixXd b_LB(2, 1); //b_LB << -30, -10;				// Lower bound of damping
	b_LB(0) = b_LBMatrix(0, blockNum-1);
	b_LB(1) = b_LBMatrix(1, blockNum-1);
	MatrixXd b_UB(2, 1); b_UB << 60, 60;				// Upper bound of damping
	MatrixXd Bgroups(2, 1);
	const double DEFAULT_Damping = 30.0;
	
	MatrixXd x_new_filt(6, 1); x_new_filt << 0, 0, 0, 0, 0, 0;
	MatrixXd x_new_filt_old(6, 1); x_new_filt_old << 0, 0, 0, 0, 0, 0;
	MatrixXd xdot_filt(6, 1); xdot_filt << 0, 0, 0, 0, 0, 0;
	MatrixXd xdot_filt_old(6, 1); xdot_filt_old << 0, 0, 0, 0, 0, 0;
	MatrixXd xdotdot_filt(6, 1); xdotdot_filt << 0, 0, 0, 0, 0, 0;
	MatrixXd xdotdot_filt_old(6, 1); xdot_filt_old << 0, 0, 0, 0, 0, 0;
	

	// Variables related to filter of position, velocity and acceleration
	double CUTOFF = 20.0;
	double RC = (CUTOFF * 2 * 3.14);
	double df = 1.0 / dt;
	double alpha_filt = RC / (RC + df);
	

	// Euler Angles 
	double phi_euler = 0;
	double theta_euler = 0;
	double psi_euler = 0;


	// ----------------------Initial DH Parameters------------------------
	MatrixXd alpha(1, 7); alpha << M_PI / 2, -M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2, -M_PI / 2, 0;
	MatrixXd a(1, 7); a << 0, 0, 0, 0, 0, 0, 0;
	MatrixXd d(1, 7); d << 0.36, 0, 0.42, 0, 0.4, 0, 0.126;
	MatrixXd theta(1, 7); theta << 0, 0, 0, 0, 0, 0, 0;



	MatrixXd qc(6, 1); qc << 0, 0, 0, 0, 0, 0;					// calculated joint space parameter differences (between current and prev iteration)
	MatrixXd delta_q(6, 1); delta_q << 0, 0, 0, 0, 0, 0;		// (current - initial) joint space parameters
	MatrixXd q_init(6, 1); q_init << 0, 0, 0, 0, 0, 0;			// initial joint space parameter (after steady parameter is met)
	MatrixXd q_freeze(6, 1); q_freeze << 0, 0, 0, 0, 0, 0;		// joint parameters when the trials are ended and it will be freezed here

	MatrixXd x_e(6, 1); x_e << 0, 0, 0, 0, 0, 0;				// end effector equilibrium position
	MatrixXd force(6, 1); force << 0, 0, 0, 0, 0, 0;			// force vector (filtered)
	MatrixXd q_new(6, 1); q_new << 0, 0, 0, 0, 0, 0;			// joint space parameters
	MatrixXd x_new(6, 1); x_new << 0, 0, 0, 0, 0, 0;			// calculated current position and pose
	MatrixXd xdot(6, 1);  xdot << 0, 0, 0, 0, 0, 0;				// first derative of x_new
	MatrixXd xdotdot(6, 1); xdotdot << 0, 0, 0, 0, 0, 0;		// second derativive of x_new

	MatrixXd x_old(6, 1); x_old << 0, 0, 0, 0, 0, 0;			// one iteration old position and pose
	MatrixXd x_oldold(6, 1); x_oldold << 0, 0, 0, 0, 0, 0;		// two iteration old position and pose

	// GUI variables
	double gui_data[16];											// xy coordinates that are send to gui
	memset(gui_data, 0, sizeof(double) * 16);						// xy coordinates that are send to gui

	double radius_e = 0.005;
	double rangex_ = -0.18;
	double rangex = 0.18;
	double rangey_ = -0.18;
	double rangey =0.18;
	double d_r;
	if(tuneInputted)
	{
	  if (blockNum <= 6)
	  {
	    d_r = 0;
	  }
	  else
	  {
	    //d_r = (rangex * 2) / 15;
	    d_r = 0;
	  }
	}
	else
	{
	  //d_r = (rangex * 2) / 15;
	  d_r = 0.01;
	}
	//double ex_r = (rangex * 2) / 15;
	double ex_r = 0.01;
	double u_r = 0.005;//ex_r - radius_e;
	int guiMode = 2;
	int guiMode2;
	if(tuneInputted)
	{
	  if(blockNum <= 6)
	  {
	    guiMode2 = 1;
	  }
	  else
	  {
	    guiMode2 = 1;
	  }
	}
	else
	{
	  guiMode2 = 2;
	}
	

	int count = 0;					// iteration counter
	float sampletime = 0.001;
	double MJoint[7] = { 0 };		// measured joint position

	double MaxRadPerSec[7] = { 1.7104,1.7104,1.7453,2.2689,2.4435,3.14159,3.14159 }; //absolute max velocity (no load from KUKA manual for iiwa 800)																					 //double MaxRadPerSec[7]={1.0,1.0,1.0,1.0,1.0,1.0,1.0}; //more conservative velocity limit
	double MaxRadPerStep[7] = { 0 };	// will be calculated
	double MaxJointLimitRad[7] = { 2.9671,2.0944,2.9671,2.0944,2.9671,2.0944,3.0543 };//Max joint limits in radians (can be changed to further restrict motion of robot)
	double MinJointLimitRad[7] = { -2.9671,-2.0944,-2.9671,-2.0944,-2.9671,-2.0944,-3.0543 }; //Min joint limits in radians (can be changed to further restrict motion of robot)


	//calculate max step value
	for (int i = 0; i<7; i++)
	{
		MaxRadPerStep[i] = sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
	}

	// create new joint position client
	PositionControlClient client;
	client.InitValues(MaxRadPerStep, MaxJointLimitRad, MinJointLimitRad);

	// create new udp connection for FRI
	UdpConnection connection;

	// pass connection and client to a new FRI client application
	ClientApplication app(connection, client);

	// connect client application to KUKA Sunrise controller
	app.connect(port, hostname);


	// Initialize stiffness, damping, and inertia matrices
	MatrixXd inertia(6, 6);
	MatrixXd stiffness(6, 6);
	MatrixXd damping(6, 6);

	// Initial Joint Angles
	client.NextJoint[0] = -1.5708;
	client.NextJoint[1] = 1.5708;
	client.NextJoint[2] = 0;
	client.NextJoint[3] = 1.5708;
	client.NextJoint[4] = 0;
	client.NextJoint[5] = -1.5708;
	client.NextJoint[6] = -0.958709;
	memcpy(client.LastJoint, client.NextJoint, 7 * sizeof(double));



	while (true)
	{

		app.step();//step through program

		if (client.KukaState == 4)
		{
			count++; //count initialized at 0
			if (count == 1)//first time inside
			{
				sampletime = client.GetTimeStep();
				//calculate max step value
				for (int i = 0; i < 7; i++)
				{
					client.MaxRadPerStep[i] = sampletime*MaxRadPerSec[i]; //converts angular velocity to discrete time step
				}

			}

			// Update measured joint angle values
			memcpy(MJoint, client.GetMeasJoint(), sizeof(double) * 7);

			// Forward Kinematic
			theta << MJoint[0], MJoint[1], MJoint[2], MJoint[3], MJoint[4], MJoint[5], MJoint[6];

			MatrixXd A1(4, 4); A1 << cos(theta(0, 0)), -sin(theta(0, 0))*cos(alpha(0, 0)), sin(theta(0, 0))*sin(alpha(0, 0)), a(0, 0)*cos(theta(0, 0)),
									 sin(theta(0, 0)), cos(theta(0, 0))*cos(alpha(0, 0)), -cos(theta(0, 0))*sin(alpha(0, 0)), a(0, 0)*sin(theta(0, 0)),
									 0, sin(alpha(0, 0)), cos(alpha(0, 0)), d(0, 0),
									 0, 0, 0, 1;
			MatrixXd A2(4, 4); A2 << cos(theta(0, 1)), -sin(theta(0, 1))*cos(alpha(0, 1)), sin(theta(0, 1))*sin(alpha(0, 1)), a(0, 1)*cos(theta(0, 1)),
									 sin(theta(0, 1)), cos(theta(0, 1))*cos(alpha(0, 1)), -cos(theta(0, 1))*sin(alpha(0, 1)), a(0, 1)*sin(theta(0, 1)),
									 0, sin(alpha(0, 1)), cos(alpha(0, 1)), d(0, 1),
									 0, 0, 0, 1;
			MatrixXd A3(4, 4); A3 << cos(theta(0, 2)), -sin(theta(0, 2))*cos(alpha(0, 2)), sin(theta(0, 2))*sin(alpha(0, 2)), a(0, 2)*cos(theta(0, 2)),
									 sin(theta(0, 2)), cos(theta(0, 2))*cos(alpha(0, 2)), -cos(theta(0, 2))*sin(alpha(0, 2)), a(0, 2)*sin(theta(0, 2)),
									 0, sin(alpha(0, 2)), cos(alpha(0, 2)), d(0, 2),
									 0, 0, 0, 1;
			MatrixXd A4(4, 4); A4 << cos(theta(0, 3)), -sin(theta(0, 3))*cos(alpha(0, 3)), sin(theta(0, 3))*sin(alpha(0, 3)), a(0, 3)*cos(theta(0, 3)),
									 sin(theta(0, 3)), cos(theta(0, 3))*cos(alpha(0, 3)), -cos(theta(0, 3))*sin(alpha(0, 3)), a(0, 3)*sin(theta(0, 3)),
									 0, sin(alpha(0, 3)), cos(alpha(0, 3)), d(0, 3),
									 0, 0, 0, 1;
			MatrixXd A5(4, 4); A5 << cos(theta(0, 4)), -sin(theta(0, 4))*cos(alpha(0, 4)), sin(theta(0, 4))*sin(alpha(0, 4)), a(0, 4)*cos(theta(0, 4)),
									 sin(theta(0, 4)), cos(theta(0, 4))*cos(alpha(0, 4)), -cos(theta(0, 4))*sin(alpha(0, 4)), a(0, 4)*sin(theta(0, 4)),
									 0, sin(alpha(0, 4)), cos(alpha(0, 4)), d(0, 4),
									 0, 0, 0, 1;
			MatrixXd A6(4, 4); A6 << cos(theta(0, 5)), -sin(theta(0, 5))*cos(alpha(0, 5)), sin(theta(0, 5))*sin(alpha(0, 5)), a(0, 5)*cos(theta(0, 5)),
									 sin(theta(0, 5)), cos(theta(0, 5))*cos(alpha(0, 5)), -cos(theta(0, 5))*sin(alpha(0, 5)), a(0, 5)*sin(theta(0, 5)),
									 0, sin(alpha(0, 5)), cos(alpha(0, 5)), d(0, 5),
									 0, 0, 0, 1;

			MatrixXd T01(4, 4); T01 << A1;
			MatrixXd T02(4, 4); T02 << T01*A2;
			MatrixXd T03(4, 4); T03 << T02*A3;
			MatrixXd T04(4, 4); T04 << T03*A4;
			MatrixXd T05(4, 4); T05 << T04*A5;
			MatrixXd T06(4, 4); T06 << T05*A6;

			// Inverse Kinematic
			phi_euler = atan2(T06(1, 2), T06(0, 2));
			theta_euler = atan2(sqrt(pow(T06(1, 2), 2) + pow(T06(0, 2), 2)), T06(2, 2));
			psi_euler = atan2(T06(2, 1), -T06(2, 0));

			MatrixXd z0(3, 1); z0 << 0, 0, 1;
			MatrixXd z1(3, 1); z1 << T01(0, 2), T01(1, 2), T01(2, 2);
			MatrixXd z2(3, 1); z2 << T02(0, 2), T02(1, 2), T02(2, 2);
			MatrixXd z3(3, 1); z3 << T03(0, 2), T03(1, 2), T03(2, 2);
			MatrixXd z4(3, 1); z4 << T04(0, 2), T04(1, 2), T04(2, 2);
			MatrixXd z5(3, 1); z5 << T05(0, 2), T05(1, 2), T05(2, 2);
			MatrixXd z6(3, 1); z6 << T06(0, 2), T06(1, 2), T06(2, 2);

			MatrixXd p0(3, 1); p0 << 0, 0, 0;
			MatrixXd p1(3, 1); p1 << T01(0, 3), T01(1, 3), T01(2, 3);
			MatrixXd p2(3, 1); p2 << T02(0, 3), T02(1, 3), T02(2, 3);
			MatrixXd p3(3, 1); p3 << T03(0, 3), T03(1, 3), T03(2, 3);
			MatrixXd p4(3, 1); p4 << T04(0, 3), T04(1, 3), T04(2, 3);
			MatrixXd p5(3, 1); p5 << T05(0, 3), T05(1, 3), T05(2, 3);
			MatrixXd p6(3, 1); p6 << T06(0, 3), T06(1, 3), T06(2, 3);

			MatrixXd J1(6, 1); J1 << z0(1, 0)*(p6(2, 0) - p0(2, 0)) - z0(2, 0)*(p6(1, 0) - p0(1, 0)),
									-z0(0, 0)*(p6(2, 0) - p0(2, 0)) + z0(2, 0)*(p6(0, 0) - p0(0, 0)),
									 z0(0, 0)*(p6(1, 0) - p0(1, 0)) - z0(1, 0)*(p6(0, 0) - p0(0, 0)),
									 z0(0, 0), z0(1, 0), z0(2, 0);
			MatrixXd J2(6, 1); J2 << z1(1, 0)*(p6(2, 0) - p1(2, 0)) - z1(2, 0)*(p6(1, 0) - p1(1, 0)),
									-z1(0, 0)*(p6(2, 0) - p1(2, 0)) + z1(2, 0)*(p6(0, 0) - p1(0, 0)),
									 z1(0, 0)*(p6(1, 0) - p1(1, 0)) - z1(1, 0)*(p6(0, 0) - p1(0, 0)),
									 z1(0, 0), z1(1, 0), z1(2, 0);
			MatrixXd J3(6, 1); J3 << z2(1, 0)*(p6(2, 0) - p2(2, 0)) - z2(2, 0)*(p6(1, 0) - p2(1, 0)),
									-z2(0, 0)*(p6(2, 0) - p2(2, 0)) + z2(2, 0)*(p6(0, 0) - p2(0, 0)),
									 z2(0, 0)*(p6(1, 0) - p2(1, 0)) - z2(1, 0)*(p6(0, 0) - p2(0, 0)),
									 z2(0, 0), z2(1, 0), z2(2, 0);
			MatrixXd J4(6, 1); J4 << z3(1, 0)*(p6(2, 0) - p3(2, 0)) - z3(2, 0)*(p6(1, 0) - p3(1, 0)),
									-z3(0, 0)*(p6(2, 0) - p3(2, 0)) + z3(2, 0)*(p6(0, 0) - p3(0, 0)),
									 z3(0, 0)*(p6(1, 0) - p3(1, 0)) - z3(1, 0)*(p6(0, 0) - p3(0, 0)),
									 z3(0, 0), z3(1, 0), z3(2, 0);
			MatrixXd J5(6, 1); J5 << z4(1, 0)*(p6(2, 0) - p4(2, 0)) - z4(2, 0)*(p6(1, 0) - p4(1, 0)),
									-z4(0, 0)*(p6(2, 0) - p4(2, 0)) + z4(2, 0)*(p6(0, 0) - p4(0, 0)),
									 z4(0, 0)*(p6(1, 0) - p4(1, 0)) - z4(1, 0)*(p6(0, 0) - p4(0, 0)),
									 z4(0, 0), z4(1, 0), z4(2, 0);
			MatrixXd J6(6, 1); J6 << z5(1, 0)*(p6(2, 0) - p5(2, 0)) - z5(2, 0)*(p6(1, 0) - p5(1, 0)),
									-z5(0, 0)*(p6(2, 0) - p5(2, 0)) + z5(2, 0)*(p6(0, 0) - p5(0, 0)),
									 z5(0, 0)*(p6(1, 0) - p5(1, 0)) - z5(1, 0)*(p6(0, 0) - p5(0, 0)),
									 z5(0, 0), z5(1, 0), z5(2, 0);


			MatrixXd Jg(6, 6); Jg << J1, J2, J3, J4, J5, J6; 	// Geometric Jacobian
			MatrixXd Tphi(6, 6); Tphi << 1, 0, 0, 0, 0, 0,
										 0, 1, 0, 0, 0, 0,
										 0, 0, 1, 0, 0, 0,
										 0, 0, 0, 0, -sin(phi_euler), cos(phi_euler)*sin(theta_euler),
										 0, 0, 0, 0, cos(phi_euler), sin(phi_euler)*sin(theta_euler),
										 0, 0, 0, 1, 0, cos(theta_euler);

			MatrixXd Ja(6, 6); Ja << Tphi.inverse()*Jg;			// Analytical Jacobian

			// Set end effector position
			/*
			// This is the conversion of frame when you seat in front of the KUKA
			endEffectorXY(0) = -1*T06(0,3);
			endEffectorXY(1) = -1*T06(2,3) + 2*ycenter;
			*/
			// This is the conversion of fram when you seat with 90 degree rotation from the KUKA
			endEffectorXY(0) = T06(2,3) - ycenter;
			endEffectorXY(1) = -1*T06(0,3); //+ycenter

			steady++;
			if (steady < 2000)
			{
				if (firstIt == 0)	//first time inside
				{
					firstIt = 1;
					// Initialize equilibrium position and pose
					x_e << T06(0, 3), T06(1, 3), T06(2, 3), phi_euler, theta_euler, psi_euler;
					x_old << x_e;
					x_oldold << x_e;
					x_new << x_e;

					// Initializing Stiffness Damping and Inertia
					stiffness << 0, 0, 0, 0, 0, 0, //toward varun desk
													0, 10000000, 0, 0, 0, 0, //up
													0, 0, 0, 0, 0, 0, //out toward workshop
													0, 0, 0, 1000000, 0, 0,
													0, 0, 0, 0, 1000000, 0,
													0, 0, 0, 0, 0, 1000000;

					damping << 30, 0, 0, 0, 0, 0,
													0, 100, 0, 0, 0, 0,
													0, 0, 30, 0, 0, 0,
													0, 0, 0, 0.5, 0, 0,
													0, 0, 0, 0, 0.5, 0,
													0, 0, 0, 0, 0, 0.5;

					inertia << 10, 0, 0, 0, 0, 0,
						    0, 0.000001, 0, 0, 0, 0,
						    0, 0, 10, 0, 0, 0,
						    0, 0, 0, 0.0001, 0, 0,
						    0, 0, 0, 0, 0.0001, 0,
						    0, 0, 0, 0, 0, 0.0001;
				}

				force << 0, 0, 0, 0, 0, 0;

			    zerox = al*(double)data[0] / 1000000 + (1 - al)*zerox;
			    zeroy = al*(double)data[1] / 1000000 + (1 - al)*zeroy;

			    q_new(0) = -1.5708;
			    q_new(1) = 1.5708;
			    q_new(2) = 0;
			    q_new(3) = 1.5708;
			    q_new(4) = 0;
			    q_new(5) = -1.5708;
				
				q_init << q_new;
			}
			else  //after 2 seconds
			{
				// Get force data from shared memory, convert to Newtons
				ftx = (double)data[0] / 1000000 - zerox; //toward varun desk
				ftx_un = (double)data[0] / 1000000 - zerox;

				fty = (double)data[1] / 1000000 - zeroy;
				fty_un = (double)data[1] / 1000000 - zeroy;

				// Filter force data with exponential moving average
				ftx = al*ftx + (1 - al)*ftx_0;
				ftx_0 = ftx;
				fty = al*fty + (1 - al)*fty_0;
				fty_0 = fty;

				force << ftx,0,fty,0,0,0;

				// Bgp should be set based on +ve -ve or variable
				if(groupDamping == 1) // 1 is group damping for positive damping
				{
					Bgroups << b_UB;
				}
				else if(groupDamping == 2) // 2 is group damping for negative damping
				{
					Bgroups << b_var;
					if (tuneInputted)
					{
						if (pathNum == 0 || pathNum == 1 || pathNum == 4)
						{
							Bgroups(0) = DEFAULT_Damping;
						}
						else if (pathNum == 2 || pathNum == 3 || pathNum == 5)
						{
							Bgroups(1) = DEFAULT_Damping;
						}
					}
				}
				else if (groupDamping == 3) // 3 is group damping for variable damping
				{
					Bgroups << b_LB;
					/*
					if (tuneInputted)
					{
						if (!targetx(pathNum, trialNum))
						{
							Bgroups(1) = DEFAULT_Damping;
						}
						else
						{
							Bgroups(0) = DEFAULT_Damping;
						}
					}*/
				}
				else if (groupDamping == 0)
				{
					Bgroups << 0, 0;
					if (tuneInputted)
					{
						if (pathNum == 0 || pathNum == 1 || pathNum == 4)
						{
							Bgroups(0) = DEFAULT_Damping;
						}
						else if (pathNum == 2 || pathNum == 3 || pathNum == 5)
						{
							Bgroups(1) = DEFAULT_Damping;
						}
					}
				}
				

				if (startBlock)
				{
					
					targetXY(0) = (targetx(pathNum, trialNum))*0.01;
					targetXY(1) = (targety(pathNum, trialNum))*0.01;

					withinErrorBound = (pow((endEffectorXY(0) - targetXY(0)),2) + pow((endEffectorXY(1) - targetXY(1)),2) <= pow(radius_e,2));

					if (!readyTrials)
					{
						if (withinErrorBound)
						{
							readyTrials = true;
						}
					}
					else
					{
						if (startTrial)
						{	
							if(beep == 1)
							{
							  beep_count1++;
							}
							if(beep_count1 >= 10)
							{
							  beep = 0;
							  beep_flag = 1;
							  beep_count1 = 0;
							}
							if(beep_flag == 1)
							{
							  beep_count++;
							}
							if(beep_count >=200)
							{
							  beep_count = 0;
							  beep_flag = 0;
							}
							
							targetXYold(0) = (targetx(pathNum, trialNum - 1))*0.01;
							targetXYold(1) = (targety(pathNum, trialNum - 1))*0.01;
							if (!tuneInputted)
							{
								d_r = ex_r;
							}
							else
							{
								d_r = 0.0;
							}
							
							damping(0, 0) = Bgroups(0);
							damping(2, 2) = Bgroups(1);

							memcpy(meas_torque, client.GetMeasTorque(), sizeof(double)*7);
							OutputFile	<< count << " "	
										<< MJoint[0] << " "
										<< MJoint[1] << " "
										<< MJoint[2] << " "
										<< MJoint[3] << " "
										<< MJoint[4] << " "
										<< MJoint[5] << " "
										<< MJoint[6] << " "
										<< force(0)  << " "
										<< force(2)  << " "
										<< x_new(0)  << " "
										<< x_new(1)  << " "
										<< x_new(2)  << " "
										<< x_new(3)  << " "
										<< x_new(4)  << " "
										<< x_new(5)  << " "
										<< damping(0, 0) << " "
										<< damping(2, 2) << " "
										<< xdot_filt(0)<< " " 
										<< xdot_filt(2) << " " 
										<< xdotdot_filt(0) << " " 
										<< xdotdot_filt(2) << " "
										<< targetXY(0) << " "
										<< targetXY(1) << " "
										<< targetXYold(0) << " "
										<< targetXYold(1) << " "
										<< endEffectorXY(0) << " "
										<< endEffectorXY(1) << " "
										<< groupDamping << " "
										<< meas_torque[0] << " "
										<< meas_torque[1] << " "
										<< meas_torque[2] << " "
										<< meas_torque[3] << " "
										<< meas_torque[4] << " "
										<< meas_torque[5] << " "
										<< meas_torque[6] << " "
										<< b_LB(0) << " "
										<< b_LB(1) << " "
										<< direction << std::endl;
							
							if (!targetReached && withinErrorBound)
							{
								targetReached = true;
								trialSettleIterCount = 0;
							}

							// Keep iteration count after target is first reached 
							if (targetReached)
							{
								trialSettleIterCount++;

								// End Trial --- This occurs 2 secs after the target was reached 
								if (trialSettleIterCount >= trialEndIterCount)
								{
									targetReached = false;
									startTrial = false;

									// Stop emg recording if necessary
									if (useEmg)
									{
										emgClient.StopWritingFileStream();
									}
								}
							}	
						}
						else
						{
							// damping can be set here to default value****
							damping(0, 0) = DEFAULT_Damping;
							damping(2, 2) = DEFAULT_Damping;

							if (tuneInputted)
							{
							  //if(blockNum <= 6)
							  //{
							  targetXY(0) = neutralXY(0);
							  targetXY(1) = neutralXY(1);
							  withinErrorBound = (pow((endEffectorXY(0) - targetXY(0)),2) + pow((endEffectorXY(1) - targetXY(1)),2) <= pow(radius_e,2));
							  //}
							}

							// Triggered when trial is not running
							if (!targetReached && withinErrorBound)
							{
								targetReached = true;
								trialWaitCounter = 0;
								trialWaitTime = rand() % 1000 + 500;
							}

							// Keep iteration count after target is first reached 
							if (targetReached)
							{
								trialWaitCounter++;

								// Start Trial --- This occurs 0.5-1.5 seconds after the neutral position is first reached 
								if (trialWaitCounter >= trialWaitTime)
								{
									beep = 1;
									trialNum++; 
									if (trialNum < nTrialsPerBlock)
									{
										targetReached = false;
										startTrial = true;
										printf("Starting trial %d\n", trialNum);

										// Make trial directory 
										trialDir = std::string("Trial") + std::to_string(trialNum);
										p_trial = path(p_block.string()) /= path(trialDir);
										create_directory(p_trial);

										// Create kuka data trial files
										p_kukadata = path(p_trial.string()) /= path(kukafilename);
										CreateOrOpenKukaDataFile(OutputFile, p_kukadata);

										// Create emg hdf5 file
										if (useEmg)
										{
											p_emgdata = path(p_trial.string()) /= path(emgfilename);
											emgClient.StartWritingFileStream(p_emgdata);
										}
									}
									else
									{
										q_freeze << q_new;
										endBlock = true;
										startBlock = false;
									}
								}
							}
						}
					}
				}
				else
				{
					targetXY(0) = temptarget(0);
					targetXY(1) = temptarget(1);
					/*if (tuneInputted)
					{
					  d_r = 0.0;
					}*/
					//d_r = 0.0;
					if ((pow((endEffectorXY(0) - targetXY(0)),2) + pow((endEffectorXY(1) - targetXY(1)),2) <= pow(radius_e,2)))
					{
						startBlock = true;
					}
				}

				// Shift old position/pose vectors, calculate new
				x_oldold << x_old;
				x_old << x_new;
				x_new << (inertia/(0.000001) + damping/(0.001) + stiffness).inverse()*(force + (inertia/(0.000001))*(x_old - x_oldold) + stiffness*(x_e - x_old)) + x_old;

				if (x_new(2) >= 0.94)
			    {
			      x_new(2) = 0.94;
			    }

			    if (x_new(2) <= 0.58)
			    {
			      x_new(2) = 0.58;
			    }

			    if (x_new(0) >= 0.18)
			    {
			      x_new(0) = 0.18;
			    }

			    if (x_new(0) <= -0.18)
			    {
			      x_new(0) = -0.18;
			    }

				qc << Ja.inverse()*(x_new - x_old);
				delta_q << delta_q +qc;
				q_new << delta_q +q_init;

				// Filter 
				x_new_filt_old 		= x_new_filt;
				x_new_filt 			= x_new_filt_old + (alpha_filt*(x_new - x_new_filt_old));

				xdot_filt_old 		= xdot_filt;
				xdot 				= (x_new_filt - x_new_filt_old) / dt;
				xdot_filt 			= xdot_filt_old + (alpha_filt*(xdot - xdot_filt_old));

				xdotdot_filt_old 	= xdotdot_filt;
				xdotdot 			= (xdot_filt - xdot_filt_old) / dt;
				xdotdot_filt 		= xdotdot_filt_old + (alpha_filt*(xdotdot - xdotdot_filt_old));

				// calculating variable damping
				if (xdot_filt(0)*xdotdot_filt(0) >= 0)					
				{
					b_var(0) = (2 * b_LB(0) / (1 + exp(-kp(0) * xdot_filt(0)*xdotdot_filt(0))) - b_LB(0));
				}
				else					
				{
					b_var(0) = -(2 * b_UB(0) / (1 + exp(-kn(0) * xdot_filt(0)*xdotdot_filt(0))) - b_UB(0));
				}
				if (xdot_filt(2)*xdotdot_filt(2) >= 0)					
				{
					b_var(1) = (2 * b_LB(1) / (1 + exp(-kp(1) * xdot_filt(2)*xdotdot_filt(2))) - b_LB(1));
				}
				else					
				{
					b_var(1) = -(2 * b_UB(1) / (1 + exp(-kn(1) * xdot_filt(2)*xdotdot_filt(2))) - b_UB(1));
				}

				if (endBlock)
				{
					q_new << q_freeze;
				}

			}
			
			// Register new joint angles with KUKA
			client.NextJoint[0] = q_new(0);
			client.NextJoint[1] = q_new(1);
			client.NextJoint[2] = q_new(2);
			client.NextJoint[3] = q_new(3);
			client.NextJoint[4] = q_new(4);
			client.NextJoint[5] = q_new(5);
			client.NextJoint[6] = -0.958709;
			
			

			// Send data to visualizer gui
			gui_data[0] = (double) guiMode;
			gui_data[1] = targetXYold(0);
			gui_data[2] = targetXYold(1);
			gui_data[3] = d_r;
			gui_data[4] = endEffectorXY(0);
			gui_data[5] = endEffectorXY(1);
			gui_data[6] = u_r;
			gui_data[7] = targetXY(0);
			gui_data[8] = targetXY(1);
			gui_data[9] = ex_r;
			gui_data[10] = damping(2, 2);
			gui_data[11] = (float) (trialNum);
			gui_data[12] = (float) (nTrialsPerBlock - 1);
			gui_data[13] = damping(0, 0);
			gui_data[14] = (double) beep_flag;
			gui_data[15] = (double) guiMode2;
			
			udp_server.Send(gui_data, 16);
		}
	}

	
	usleep(10000000);//microseconds //wait for close on other side

	// disconnect from controller
	app.disconnect();

	return 1;
}

void CreateOrOpenKukaDataFile(boost::filesystem::ofstream & ofs, path kukaDataFilePath) 
{
	/* deconstruct kuka file path into path, filename, extension */
	path pp = kukaDataFilePath.parent_path();
	path fname_stem = kukaDataFilePath.stem();
	path fname_ext = kukaDataFilePath.extension();

	/* Make a path to rename old file with same path, and rename if necessary */
	path p_unsuc = path(kukaDataFilePath.string());
	int unsuc_count = 1;
	std::string fname_unsuc;
	if (is_regular_file(p_unsuc)) 
	{
		while (is_regular_file(p_unsuc)) {
			//fname_unsuc = fname_stem.string() + std::string("_unsuccessful_") + std::to_string(unsuc_count) + fname_ext.string();
			fname_unsuc = fname_stem.string() + std::string("_") + std::to_string(unsuc_count) + fname_ext.string();
			p_unsuc = path(pp.string()) /= path(fname_unsuc);
			unsuc_count++;
		}
		rename(kukaDataFilePath, p_unsuc);
	}

	/* Make file stream */
	ofs.close();
	ofs.open(kukaDataFilePath);
}

// Shared Memory-------------------------------------------------------
template<typename T>
T * InitSharedMemory(std::string shmAddr, int nElements){
	key_t key;
	int shmid;
	size_t shmSize = nElements*sizeof(T);
	T * shm = (T *) malloc(shmSize);
	/* make the key */
	if ((key = ftok(shmAddr.c_str(), 'R')) == -1)
	{
		perror("ftok-->");
		exit(1);
	}

	if ((shmid = shmget(key, shmSize, 0666 | IPC_CREAT)) == -1)
	{
		perror("shmget");
		exit(1);
	}

	shm = (T *) shmat(shmid, (void *)0, 0);

	if (shm == (T *)(-1))
	{
		perror("shmat");
		exit(1);
	}

	for (int i = 0; i<nElements; i++)
	{
		shm[i] = 0.0;
	}

	return shm;
}
