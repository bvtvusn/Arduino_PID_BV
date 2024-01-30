#include "PID_BV.h"

double PID1_Setpoint, PID1_Input, PID1_Output;

double Kp = 0.5, Ti = 10000, Td = 0;
PID_BV myController(&PID1_Input, &PID1_Output, &PID1_Setpoint, Kp, Ti, Td, true);

unsigned long sim_lastTime = 0;
float sim_tankLevel = 0;
int sampleTimeMS = 200;
int delayTimeMS = 2000;  // Set your desired delay time

// Function prototypes
float SimulateWaterTank(float controlSignal);
void UpdateControlSignalQueue(float controlSignal);

// Circular buffer for control signals
float* controlSignalQueue;
int controlSignalQueueSize;
int rear = 0;  // Declare rear as a global variable

void setup() {
  Serial.begin(9600);
  myController.SetOutputLimits(-100, 100);
  myController.SetSampleTime(sampleTimeMS);

  // Initialize controlSignalQueue with dynamic memory allocation
  controlSignalQueueSize = delayTimeMS / sampleTimeMS;
  controlSignalQueue = new float[controlSignalQueueSize];

  // Initialize the circular buffer with zeros
  for (int i = 0; i < controlSignalQueueSize; i++) {
    controlSignalQueue[i] = 0.0;
  }
}

void loop() {
  PID1_Setpoint = 50.0;  // Setpoint water level

  bool doCompute = myController.Compute();

  if (millis() - sim_lastTime > sampleTimeMS) {
    // Update the control signal queue
    UpdateControlSignalQueue(PID1_Output);

    // Use the delayed control signal from the queue
    PID1_Input = SimulateWaterTank(controlSignalQueue[rear]);

    Serial.print("Setpoint: ");
    Serial.print(PID1_Setpoint);
    Serial.print(", WaterLevel: ");
    Serial.print(PID1_Input);
    Serial.print(", ControlSignal: ");
    Serial.println(PID1_Output);
  }
}

float SimulateWaterTank(float controlSignal) {
  unsigned long now = millis();
  float dt = float(now - sim_lastTime) / 1000.0; // time in seconds
  sim_lastTime = now;

  float effectiveControlSignal = controlSignal;
  float inflowRate = effectiveControlSignal * 0.5;  // Arbitrary factor for simulation
  float outflowRate = 0.1;

  sim_tankLevel += (inflowRate - outflowRate) * dt;  // Time step of the simulation

  // Ensure the water level doesn't go negative
  if (sim_tankLevel < 0.0) {
    sim_tankLevel = 0.0;
  }
  return sim_tankLevel;
}

void UpdateControlSignalQueue(float controlSignal) {
  controlSignalQueue[rear] = controlSignal;
  rear = (rear + 1) % controlSignalQueueSize;
}
