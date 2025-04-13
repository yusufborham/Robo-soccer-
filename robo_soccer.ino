#include <Arduino.h>

// Define motor driver pins for horizontal thrusters
// Motor driver pins for horizontal thrusters (A, B, C, D)
#define motor1PwmA 14
#define motor1PwmB 27

#define motor2PwmA 26
#define motor2PwmB 15

#define motor3PwmA 19
#define motor3PwmB 18

#define motor4PwmA 5
#define motor4PwmB 17

// Define Motor Positions for the robot

// B ###################### A //      \    /
//                            //       \  /
//                            //        \/
//                            //        /\
// C ###################### D //       /  \
//                                    /    \

// pwmV(motor number)_(motor pin)
// A
#define pwmV1_1 motor1PwmA
#define pwmV1_2 motor1PwmB

// B
#define pwmV2_1 motor2PwmA
#define pwmV2_2 motor2PwmB

// C
#define pwmV3_1 motor3PwmA
#define pwmV3_2 motor3PwmB

// D
#define pwmV4_1 motor4PwmA
#define pwmV4_2 motor4PwmB

// Input forces (Fx, Fy, Tau)
double inputH[3] = {0, 0, 0}; // Fx , Fy , Tau

// Pseudoinverse matrices
double T_inverse_Horizontal[4][3] = {
    {0.25, 0.25, 0.25},
    {0.25, -0.25, -0.25},
    {0.25, 0.25, -0.25},
    {0.25, -0.25, 0.25}};

float outputHorizontalThrusters[4] = {0, 0, 0, 0};
// Vertical thrust motor driver pins
int HorizontalThrusterSp1[4] = {pwmV1_1, pwmV2_1, pwmV3_1, pwmV4_1}; // all pins for forward motion
int HorizontalThrusterSp2[4] = {pwmV1_2, pwmV2_2, pwmV3_2, pwmV4_2}; // all pins for backward motion

// Apply constraints to the thruster forces
// if the maximum absolute force exceeds the max allowed force, scale down
void applyConstraints(float *thruster_forces, int size, float max_force)
{
  float max_abs_force = 0;

  // Find the maximum absolute value of the thruster forces
  for (int i = 0; i < size; i++)
  {
    if (abs(thruster_forces[i]) > max_abs_force)
    {
      max_abs_force = abs(thruster_forces[i]);
    }
  }

  // If the maximum absolute force exceeds the max allowed force, scale down
  if (max_abs_force > max_force)
  {
    float scaling_factor = max_force / max_abs_force;
    for (int i = 0; i < size; i++)
    {
      thruster_forces[i] *= scaling_factor; // Apply the scaling factor
    }
  }
}

void ComputeHorizontalThrustForces(double *input, double T_inverse[4][3], float *outputThrusters)
{

  // Perform matrix multiplication outputThrusters = T_inverse * input

  for (int i = 0; i < 4; i++)
  {
    outputThrusters[i] = 0;
    for (int j = 0; j < 3; j++)
    {
      outputThrusters[i] += T_inverse[i][j] * input[j];
    }
  }

  float max_force = 255;
  applyConstraints(outputThrusters, 4, max_force);
}

void controlHmotors()
{
  for (int num = 0; num <= 3; num++)
  {
    if (outputHorizontalThrusters[num] >= 0)
    {
      analogWrite(HorizontalThrusterSp1[num], int(abs(outputHorizontalThrusters[num]))); // control speed
      analogWrite(HorizontalThrusterSp2[num], 0);                                        // control speed
    }
    else
    {
      analogWrite(HorizontalThrusterSp1[num], 0);                                        // control speed
      analogWrite(HorizontalThrusterSp2[num], int(abs(outputHorizontalThrusters[num]))); // control speed
    }
  }
}

void setup_H_motors()
{
  for (int num = 0; num <= 3; num++)
  {
    pinMode(HorizontalThrusterSp1[num], OUTPUT); // Set the pin as output
    pinMode(HorizontalThrusterSp2[num], OUTPUT); // Set the pin as output
  }
}

void setup()
{
  Serial.begin(115200); // Initialize serial communication for debugging
  setup_H_motors();     // Setup horizontal motors
                        // Initialize other components as needed
}

void loop()
{
  // Read input forces (Fx, Fy, Tau) from sensors or other sources
  // For example, you can read from serial or use predefined values for testing
  inputH[0] = 10; // Example value for Fx
  inputH[1] = 5;  // Example value for Fy
  inputH[2] = 2;  // Example value for Tau

  // Compute horizontal thrust forces
  ComputeHorizontalThrustForces(inputH, T_inverse_Horizontal, outputHorizontalThrusters);

  // Control horizontal motors based on computed thrust forces
  controlHmotors();

  delay(100); // Add a delay to avoid overwhelming the loop
}