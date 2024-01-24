#ifndef PID_BV_H
#define PID_BV_H
#include "Arduino.h"

class PID_BV
{
	public:
		
		//Constructor
		PID_BV(double* Input, double* Output, double* Setpoint, double Kp, double Ti, double Td, bool ForwardActingController);
			
		void SetOutputLimits(double Min, double Max);
		void ForwardActionController(bool mode);
		void SetTunings(double Kp, double Ti, double Td);
		bool Compute();
		void SetSampleTime(int);
		void SetAutoState(bool newAutoState);
		double debug;


		double GetKp();
		double GetTi();
		double GetTd();
		double GetIntegral();
		bool inAutoState();
		bool isForwardDirection();


	private:
		
		void Initialize();
		double *myInput;              
		double *myOutput;             
		double *mySetpoint;           
		
		double kp;                  // * 
		double ti;                  // * [seconds]
		double td;                  // * [seconds]
		bool isDirect;
		
		
		
		double dispKp;				// * 
		double dispTi;				//   
		double dispTd;				//
		
		

									  
				  
		unsigned long lastTime;
		double lastInput;
		double integralSum;

		unsigned long SampleTime;
		double outMin, outMax;
		bool inAuto;
  
};
#endif
