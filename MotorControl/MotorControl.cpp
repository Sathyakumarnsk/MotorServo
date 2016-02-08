// MotorControl.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <windows.h>
#include<time.h>
#include <conio.h>
#include "oldaapi.h" //For Keithley
#include <USD_USB.h> //For USDigital Encoder
#include <sys/types.h>
#include <sys/timeb.h>
#include <string.h>
float Ts = 0.028;

#define NZEROS 2
#define NPOLES 2


static float xv[NZEROS + 1] = { 0,0,0 };
static float yv[NPOLES + 1] = { 0,0,0 };

static float xv1[NZEROS + 1] = { 0,0,0 };
static float yv1[NPOLES + 1] = { 0,0,0 };


/*
#define GAIN   4.840925170e+00
//fc=20Hz, Fs=80Hz
static float filterloop(float x)
{
float y;
//for (;;)
{
xv[0] = xv[1]; xv[1] = xv[2];
xv[2] = x / GAIN;
yv[0] = yv[1]; yv[1] = yv[2];
yv[2] = (xv[0] + xv[2]) + 2 * xv[1]
+ (-0.1958157127 * yv[0]) + (0.3695273774 * yv[1]);
y = yv[2];
}
return y;
}


static float filter(float x)
{
float y;
//for (;;)
{
xv1[0] = xv1[1]; xv1[1] = xv1[2];
xv1[2] = x / GAIN;
yv1[0] = yv1[1]; yv1[1] = yv1[2];
yv1[2] = (xv1[0] + xv1[2]) + 2 * xv1[1]
+ (-0.1958157127 * yv1[0]) + (0.3695273774 * yv1[1]);
y = yv1[2];
}
return y;
}*/


//fc=40Hz Fs=100Hz
#define GAIN   1.565078650e+00
static float filterloop(float x)
{
	float y;
	{
		xv[0] = xv[1]; xv[1] = xv[2];
		xv[2] = x / GAIN;
		yv[0] = yv[1]; yv[1] = yv[2];
		yv[2] = (xv[0] + xv[2]) + 2 * xv[1]
			+ (-0.4128015981 * yv[0]) + (-1.1429805025 * yv[1]);
		y = yv[2];
	}
	return y;
}
static float filter(float x)
{
	float y;
	{
		xv1[0] = xv1[1]; xv1[1] = xv1[2];
		xv1[2] = x / GAIN;
		yv1[0] = yv1[1]; yv1[1] = yv1[2];
		yv1[2] = (xv1[0] + xv1[2]) + 2 * xv1[1]
			+ (-0.4128015981 * yv1[0]) + (-1.1429805025 * yv1[1]);
		y = yv1[2];
	}
	return y;
}



//using namespace System;
//using namespace System::Windows::Forms;
float Derivative(float e1, float e0)
{
	float e_deri;
	e_deri = (e1 - e0) / Ts;
	return e_deri;
}


\

int value[200];
	int NumberOfUSB1s = 0;
	unsigned char ModuleAddress;
	bool blnResult;
	DBL volt = 2.5, gain, min, max, gain1, min1, max1;
	HDEV hDev = NULL;
	HDASS hDA = NULL, hDA1 = NULL;
	LNG iValue;
	UINT resolution, encoding, resolution1, encoding1, channel[2] = { 0,1 };
	//char tmpbuf[128], timebuf[26], ampm[] = "AM";
	time_t ltime;
	struct _timeb tstruct;

	DWORD start, finish, duration;

	int i, j,k=0;
	const int m = 5000;
	float e[2][m], u[2][m];
	float Kp[2] = { 3.0, 5.0 }, Kd[2] = { 0.1, 0.1 };
	float theta[2][m], dtheta[2][m], ddtheta[2][m], de[2][m], thetad[2][m], dthetad[2][m], ddthetad[2][m];
	long lVal[2]; //Get USB Encoder value
	char lpszDriverName1[100], lpszDriverName2[100];
	float tempd = 0;
	float tempdd = 0;

	//char lpszDriverName1[100], lpszDriverName2[100];
	//UINT resolution, encoding, resolution1, encoding1, channel[2] = { 0,1 };
	HDEV hDevice[2];
	char Bname[2][100];
	using namespace std;

	void EnumBrdProc(LPSTR lpszBrdName, LPSTR lpszDriverName, LPARAM lParam)
	{
		// Make sure we can Init Board
		//MessageBox::Show("Hollo");
		(olDaInitialize((PTSTR)lpszBrdName, (LPHDEV)lParam));

		// Make sure Board has an A/D Subsystem 
		UINT uiCap = 0;
		olDaGetDevCaps(*((LPHDEV)lParam), OLDC_DAELEMENTS, &uiCap);

		//printf("Called%d\n",cnt);

	}

	void init(void) {
		olDaEnumBoards((DABRDPROC)EnumBrdProc, (LPARAM)&hDevice[0]);
		(olDaGetDeviceName(hDevice[0], (PTSTR)lpszDriverName1, 100));
		olDaGetDASS(hDevice[0], OLSS_DA, 0, &hDA);
		olDaSetDataFlow(hDA, OL_DF_SINGLEVALUE);
		olDaSetChannelListEntry(hDA, 0, channel[0]);
		olDaSetChannelListEntry(hDA, 0, channel[1]);
		olDaConfig(hDA);
		olDaGetResolution(hDA, &resolution);
		olDaGetEncoding(hDA, &encoding);
		olDaGetGainListEntry(hDA, 0, &gain);
		olDaGetRange(hDA, &max, &min);
		olDaVoltsToCode(min, max, gain, resolution, encoding, volt, &iValue);
		olDaPutSingleValue(hDA, iValue, channel[0], gain);
		olDaPutSingleValue(hDA, iValue, channel[1], gain);

	}
	void stopmotor()
	{
		(olDaVoltsToCode(min, max, gain, resolution, encoding, 2.5, &iValue));  //To Stop Motor
		(olDaPutSingleValue(hDA, iValue, channel[0], gain));
		(olDaPutSingleValue(hDA, iValue, channel[1], gain));
	}


	void samplefun(double kp1, double kd1, double kp2, double kd2, double angle)
	{
		// Position control 
		Kp[0] = kp1;
		Kp[1] = kp2;
		Kd[0] = kd1;
		Kd[1] = kd2;
		int ik=0;

		//MessageBox::Show("Given values are\n:KP1="+Kp[0].ToString()+"\nKp2="+Kp[1].ToString()+"\nKd1="+Kd[0].ToString()+"\nKd2="+Kd[1].ToString()+"\nDesired Angle="+angle.ToString());			 

	///	FILE *fp; //for Sample reading
	//	fopen_s(&fp, "D:\Sample.txt", "w");


		//CHECKERROR(olDaVoltsToCode(min,max,gain,resolution,encoding,volt,&iValue));  //To Stop Motor
		//CHECKERROR(olDaPutSingleValue(hDA,iValue,channel[0],gain));

		//CHECKERROR(olDaVoltsToCode(min,max,gain,resolution,encoding,volt,&iValue));  //To Stop Motor
		//CHECKERROR(olDaPutSingleValue(hDA,iValue,channel[1],gain));


		//CHECKERROR(olDaVoltsToCode(min,max,gain,resolution,encoding,volt,&iValue));  //To Stop Motor
		//CHECKERROR(olDaPutSingleValue(hDA1,iValue,channel[1],gain));

		FILE *fp; //for Sample reading
		fopen_s(&fp, "D:\Sample.txt", "w");
		//fp = _fsopen("D:\\outfile.txt", "w", _SH_DENYRD);
		//fp = fopen("D:\\outfile.txt", "w+");
		NumberOfUSB1s = USB1Init();
		//printf("Number of USB1s = %d\n", NumberOfUSB1s);
		//MessageBox::Show("No of USB1 connected:"+NumberOfUSB1s.ToString());

		blnResult = USB1ReturnModuleAddress(0, &ModuleAddress);
	if (blnResult == false) printf("Cannot read Module Address!");
		else
		{
			(olDaVoltsToCode(min, max, gain, resolution, encoding, volt, &iValue));  //To Stop Motor
			(olDaPutSingleValue(hDA, iValue, channel[0], gain));
			(olDaPutSingleValue(hDA, iValue, channel[1], gain));
			//MessageBox::Show("Motor Stopped, Plz set the position of motor and press ok...");			

		//	USB1SetIncMaxCount(ModuleAddress, 0, 64000); //Reset the USBEncoder
		//	USB1SetIncMaxCount(ModuleAddress, 1, 64000);

			// ************ Position Control Reference Trajectories *********
			for (i = 0; i < m; i++)
			{
				for (j = 0; j<2; j++)
				{
					thetad[j][i] = angle;
					dthetad[j][i] = 0.0;
					u[j][i] = 0.0;
					de[j][i] = 0.0;
				}
			}

			for (j = 0; j < 2; j++)
			{
				e[j][0] = angle;
				USB1GetIncPosition(ModuleAddress, j, &lVal[j]);
				theta[j][0] = 360.0*((float(lVal[j] % 64000)) / 64000.0);
				dtheta[j][0] = 0.0;
			//printf("theta=%f \t Encoder=%ld\n", theta[j][0], lVal[j]);
				//	fprintf(fp, "lVal=%f\t theta= %f\n", lVal[j], theta[j][0]);

			}
			//	_strtime_s(tmpbuf, 128);

			start = GetTickCount();


			//fprintf(fp, "Theta[1]\tdTheta[1]\tddtheta\tu[i]\n");

			for (i = 1; i < m; i++)
			{
				for (j = 0; j < 2; j++)
				{
					USB1GetIncPosition(ModuleAddress, j, &lVal[j]);
					theta[j][i] = 360.0*((float(lVal[j] % 64000)) / 64000.0);
					
						
					
					//printf("theta=%f \t Encoder=%ld\n", theta[0][i], lVal[0]);
				    //fprintf(fp, "%f\n", theta[j][i]);
				     //fprintf(fp, "%f\n", theta[0][i]);
					//	dtheta[j][i] = (theta[j][i] - theta[j][i - 1]) / Ts;
					tempd = (theta[j][i] - theta[j][i - 1]) / Ts;
					dtheta[j][i] = filterloop(tempd);


					//ddtheta[j][i] = (dtheta[j][i] - dtheta[j][i - 1]) / Ts;
					tempdd = (dtheta[j][i] - dtheta[j][i - 1]) / Ts;
					ddtheta[j][i] = filter(tempdd);


					e[j][i] = thetad[j][i] - theta[j][i];
					

							u[j][i] = (Kp[j] * e[j][i]) / 100.0;
							//printf("theta=%f \t Encoder=%ld\t u[j][i]=%f\n", theta[j][0], lVal[j], u[j][i]);
							//printf("%f\n",u[j][i]);
				//	u[j][i] = -(Kp[j] * e[j][i] + Kd[j] * Derivative(e[j][i], e[j][i - 1])) / 100;
						if (u[j][i] > 2.5)
				u[j][i] = 2.5;
							else if (u[j][i] < -2.5)
							u[j][i] = -2.5;
						u[j][i] = u[j][i] + (float)2.5;
					//u[j][i] =2.4;
						

					(olDaVoltsToCode(min, max, gain, resolution, encoding, u[j][i], &iValue));
					(olDaPutSingleValue(hDA, iValue, channel[j], gain));
				//	ik++;
					//	printf("theta [0][i]=%f\t    theta[1][i]= %f\n",theta[0][i],theta[1][i]);
					
				
				}
				fprintf(fp, "%f\t%f\t%f\t%f\n", theta[0][i], dtheta[0][i], ddtheta[0][i], u[0][i]);

			}

		//	fprintf(fp, "%f\t%f\t%f\t%f\n", theta[0][i], dtheta[0][i], ddtheta[0][i], resolution);
			//	_strtime_s(tmpbuf, 128);
			//	fprintf(fp,"OS time:\t\t\t\t%s\n", tmpbuf);
			finish = GetTickCount();
			duration = finish - start;
			Ts = (float)duration / m;
			printf("%ld\n", duration);
			//fprintf(fp, "%f\t%d\n", duration, m);
			fclose(fp);
		}
	//fclose(fp);

	//stopmotor();
	}

	



int main()
{
	init();
	int degree=270;
	stopmotor();
	Sleep(500);
	//FILE *fp;
	USB1SetIncMaxCount(ModuleAddress, 0, 64000);
	//while (1) {
	//for (int j = 0; j <= 200;j++){


		//printf("******POSITION CONTROL*******ENTER 500 TO EXIT\n");
	//	printf("Enter the angle in degrees (0-360)\n");
		//scanf_s("%d", &degree);
		samplefun(38.0, 0, 2.0, 0, 60);
	//	Sleep(5000);
		//samplefun(2.0, 0, 2.0, 0, 180);
	//	stopmotor();
		//Sleep(5000);
		//stopmotor();
		//Sleep(1500);
		
		

		//if (degree == 500)
		//	break;
	//stopmotor();
			
//}
	stopmotor();

    return 0;
}

