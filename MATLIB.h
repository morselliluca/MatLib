#include "Arduino.h" 
#include <PID_v1.h>
#include <STM32SD.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

class Lambda{
public:

  Lambda(uint8_t _PIN_VGND, uint8_t _PIN_VS, uint8_t _PIN_IPA, uint8_t _POUT_VS, uint8_t _POUT_HEATER, uint8_t _POUT_IP);
  uint16_t lambdaValue();


private:

  float nernstResistanceVolts();
  float lambdaAmp(float amps);

/*
  uint8_t PIN_VGND; // Analog In - Level of the virtual ground
  uint8_t PIN_VS; // Analog In - Reads Nernst Cell voltage
  uint8_t PIN_IPA; // Analog In - Measures pump current via differential amp
*/

  uint8_t POUT_VS; // Digital Out - Pulses Nernst Cell to calculate internal resistance
  uint8_t POUT_HEATER; // Analog Out - Heater control, keeps Nernst cell at right temperature
  uint8_t POUT_IP; // Analog Out - Pump cell current output, keeps Nernst cell in stoich range

  uint8_t PINS[3]; 

  double heatIn, heatOut;
  double pumpIn, pumpOut;

  const float targetLambda = 1.0; // Narrowband simulation, cross point in lambda
  const float widebandTop = 2.0; // Where the +5v of the linear wideband output is

  const int warmUpDelay = 20000; // Initial delay we wait before starting heating of the sensor, in millis

  double heatTarget = 2.3 / 1300.0 * 200.0; // Target voltage, this is pulse voltage divided by total target resistance, times the target cell resistance
  int heatSampleTime = 500; // How often the heater is sampled for PID, in milliseconds
  double pumpTarget = 0.45; // Target Nernst cell voltage
  int pumpSampleTime = 40; 
  
  PID heater;
  PID pump;

  #define LAMBDA_LOOKUP_SIZE 42
  const float lambdaLookup[LAMBDA_LOOKUP_SIZE] = {
    -2.00, 0.65,
    -1.602, 0.70,
    -1.243, 0.750,
    -0.927, 0.800,
    -0.800, 0.822,
    -0.652, 0.850,
    -0.405, 0.900,
    -0.183, 0.950,
    -0.106, 0.970,
    -0.040, 0.990,
    0.00, 1.003,
    0.015, 1.010,
    0.097, 1.100,
    0.250, 1.132,
    0.329, 1.179,
    0.671, 1.429,
    0.938, 1.701,
    1.150, 1.990,
    1.385, 2.434,
    2.000, 5.391,
    2.54, 4.99
  };

};

class AnalogInputs{

public: 

  AnalogInputs(uint8_t* _pins);

  int analogvalue(uint8_t pin);
  float tovolts(float value);
  float linearizedanalogvalue2point(float value, float min, float max);
  float linearizedanalogvaluearray(float value, float *arrayvolt, float *arrayvalue);
  uint8_t digitalwithisteresis(float value, float valmin, float valmax, float outlow, float outhigh, float lastval);

private:

  uint8_t* PINS;

};

class Datalogging{

  public: 

  Datalogging();

  void log(float value, int frequency);

  private:

  File dataFile;

};

class Batterytest{

  public: 

  Batterytest(uint8_t pinvoltage, uint8_t pinmosfet);

  float testcapacity(uint8_t pinvoltage, uint8_t pinmosfet, float resistor, bool log);

  private:

  uint8_t PINS[2];

};

class Filters{


  public:

  Filters();
  Filters(uint8_t _firdimension);
  float IIR(float value,  float _iiralpha);
  float FIR(float value);
  float MA(float value);
  float RMs(float value);

  private:

  float iirvalue;
  uint8_t firdimension = 16;
  float* firvalues;
  uint8_t firindex = 0;
  float* maimpresponse;

}