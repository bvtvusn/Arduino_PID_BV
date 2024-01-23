#include "Arduino.h"
#include "PID_BV.h"


PID_BV::PID_BV(double* Input, double* Output, double* Setpoint, double Kp, double Ti, double Td, bool ForwardActingController)
   : myOutput(Output), myInput(Input), mySetpoint(Setpoint), inAuto(true), SampleTime(100), lastTime(millis() - SampleTime)
{
	
	
	PID_BV::SetOutputLimits(0, 255);
			
	PID_BV::ForwardActionController(ForwardActingController);
	
	PID_BV::SetTunings(Kp, Ti, Td);
	
	
}

void PID_BV::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;

   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;

	   // For later: Check that no values in the program exceed the limits. 
   }
}

void PID_BV::ForwardActionController(bool mode)
{
	isDirect = mode;
}


void PID_BV::SetTunings(double Kp, double Ti, double Td)
{
   if (Kp<0 || Ti<0 || Td<0) return;

   kp = Kp; 
   ti = Ti; 
   td = Td;
   
}

bool PID_BV::Compute()
{
	if(!inAuto) return false;
	unsigned long now = millis();
	unsigned long timeChange = (now - lastTime);
	
	if(timeChange>=SampleTime)
	{
		double input = *myInput;
		double setpoint = *mySetpoint;
	    double error = setpoint - input;
		double timeChange_Seconds = (timeChange / 1000.0);
		
		//Serial.print("Input: ");
		//Serial.println(input);
		
		//Serial.print("Setpoint: ");
		//Serial.println(setpoint);

		double internalKp = kp;
		if(!isDirect){
			internalKp = -kp;
		}
	    
		double out_p = error * internalKp;
		integralSum += error * (internalKp / ti) * timeChange_Seconds;
		
		debug = error;
		// Antiwindup start
		double overClip = (integralSum+out_p) - outMax;
		double underClip = outMin-(integralSum+out_p);
		if (overClip > 0){
			integralSum -= overClip;
		}
		else if(underClip > 0){
			integralSum+= underClip;
		}
		// Antiwindup end
		
		
		double out_d = - internalKp * td * (input - lastInput) / timeChange_Seconds;
		
		double PIDsum = out_p + integralSum + out_d;
		*myOutput = max(outMin,min(outMax, PIDsum));
		
		
		//Serial.print("Output: ");
      //Serial.println(*myOutput);
		
		
	  lastInput = input;
      lastTime = now;
	  return true;
	}
	else return false;
}