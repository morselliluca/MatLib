#include "Arduino.h"
#include "MATLIB.h"

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

float readVoltages(byte pin){
  float value;
  for (int i = 0; i < 200; i++){
    value = value + analogRead(pin);
    delayMicroseconds(1);
  }
  return value / 200;
}

//LAMBDA

float Lambda::nernstResistanceVolts() {
    int before = AnalogInputs.analogvalue(pins[1]); // Read nernst before pulse
    digitalWrite(POUT_VS, 1); // Pulse it
    delayMicroseconds(30); // Wait for the pulse cap to reach full voltage
    int after = AnalogInputs.analogvalue(pins[1]); // Read during the pulse
    digitalWrite(POUT_VS, 0); // Restore it
    delayMicroseconds(200); // Give the nernst cell and cap some time to equalise back out after
    return (mapfloat(after - before, 0, 4095, 0, 3.3)*26)/16*1000;
}

float Lambda::lambdaAmp(float amps) {
  if (amps < lambdaLookup[0]) return 0.0; // Too rich for sensor
  if (amps > lambdaLookup[LAMBDA_LOOKUP_SIZE-2]) return 9.99; // To lean to care
  for (int x = 0; x < LAMBDA_LOOKUP_SIZE-2; x = x + 2) {
    if (amps >= lambdaLookup[x] && amps <= lambdaLookup[x+2]) {
      // Interpolate the lookup
      return (amps - lambdaLookup[x]) / (lambdaLookup[x+2] - lambdaLookup[x]) * (lambdaLookup[x+3] - lambdaLookup[x+1]) + lambdaLookup[x+1];
    }
  }
}

Lambda::Lambda(uint8_t _PIN_VGND, uint8_t _PIN_VS, uint8_t _PIN_IPA, uint8_t _POUT_VS, uint8_t _POUT_HEATER, uint8_t _POUT_IP): 
heater(&heatIn, &heatOut, &heatTarget, 800, 100, 1, REVERSE) ,  pump(&pumpIn, &pumpOut, &pumpTarget, 800, 100, 0.001, DIRECT) {

  PINS[0] = _PIN_VGND;
  PINS[1] = _PIN_VS; 
  PINS[2] = _PIN_IPA;
  POUT_VS = _POUT_VS; 
  POUT_HEATER = _POUT_HEATER; 
  POUT_IP = _POUT_IP;

  AnalogInputs(PINS);

  pinMode(POUT_VS, OUTPUT);
  digitalWrite(POUT_VS, 0);
  pinMode(POUT_IP, OUTPUT);
  analogWrite(POUT_IP, 128);
  pinMode(POUT_HEATER, OUTPUT);
  analogWrite(POUT_HEATER, 0);

  heater=PID(&heatIn,&heatOut,&heatTarget,800,100,1,REVERSE);
  pump=PID(&pumpIn,&pumpOut,&pumpTarget,800,100,0.001,DIRECT);

  heater.SetOutputLimits(0, 250);
  heater.SetSampleTime(heatSampleTime);
  heater.SetMode(AUTOMATIC);
  pump.SetOutputLimits(0, 250);
  pump.SetSampleTime(pumpSampleTime);
  pump.SetMode(AUTOMATIC);
}

uint16_t Lambda::lambdaValue() {
  heatIn = nernstResistanceVolts();
  heater.Compute();
  analogWrite(POUT_HEATER, heatOut);
  
  pumpIn = AnalogInputs.analogvalue(pins[1]) - AnalogInputs.analogvalue(pins[0]);
  pump.Compute();
  analogWrite(POUT_IP, pumpOut);

  return (int)(lambdaAmp(-((mapfloat(0, 4095, 0, AVREF, AnalogInputs.analogvalue(pins[2]))/16)/69.1)*1000)*1000);
}

//Fine Lambda

//Analogici

AnalogInputs::AnalogInputs(uint8_t *_pins){
  PINS =  _pins;
  for(int i = 0; i < sizeof(PINS); i++){
    pinMode(PINS[i], INPUT);
  }
}

float AnalogInputs::analogvalue(uint8_t pin){
  return readVoltages(pin);
}

float AnalogInputs::tovolts(float value){
  return mapfloat(0, 4095, 0, AVREF, value);
}

float AnalogInputs::linearizedanalogvalue2point(float value, float min, float max){
  return mapfloat(0, AVREF, min, max, value);
}

float AnalogInputs::linearizedanalogvaluearray(float value, float *arrayvolt, float *arrayvalue){
  for(int i = 0; i < sizeof(arrayvolt); i++){
    if(value > arrayvolt[i]){
      return value*((arrayvalue[i]-arrayvalue[i-1])/(arrayvolt[i]-arrayvolt[i-1]));
    }
  }
  return arrayvalue[sizeof(arrayvolt)];
}

uint8_t AnalogInputs::digitalwithisteresis(float value, float valmin, float valmax, float outlow, float outhigh, float lastval){
  if(value >=  valmax){return outhigh;}
  else if(value >=  valmax){return outlow;}
  else{return lastval;}
}

//Fine Analogici

//Datalogging

Datalogging::Datalogging(){
  while (!SD.begin(SD_DETECT_PIN)) {
    delay(10);
  }
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.seek(dataFile.size());
  }
}

//Fine Datalogging

//Batterytest

Batterytest::Batterytest(uint8_t pinvoltage, uint8_t pinmosfet){

  pinMode(pinvoltage, OUTPUT);
  pinMode(pinmosfet, INPUT);

}

float Batterytest::testcapacity(uint8_t pinvoltage, uint8_t pinmosfet, float resistor, float minvolt, bool log){
  PINS[0] = pinvoltage;
  PINS[1] = pinmosfet;
  unsigned long starttime;
  unsigned long endtime;
  unsigned long amph;

  AnalogInputs(PINS);

  while(AnalogInputs.analogvalue(pins[0]) >= minvolt){
    starttime = millis();
    digitalWrite(PINS[1], HIGH);
    delay(500);
    endtime = millis();
    amph += (starttime-endtime)/(1000*3600)*AnalogInputs.analogvalue(pins[0])/resistor;
  }
}

//Fine Batterytest

//Filters

Filters::Filters(){
  maimpresponse[firdimension];
  for(int i = 0; i < firdimension; i++){maimpresponse[i] = 1/firdimension;}
}

Filters::Filters(uint8_t _firdimension){
  firdimension = _firdimension;
  maimpresponse[firdimension];
  for(int i = 0; i < firdimension; i++){maimpresponse[i] = 1/firdimension;}
}

float Filters::IIR(float value, float _iiralpha){

  if(_iiralpha < 1 && _iiralpha > 0){iirvalue = iirvalue*_iiralpha+(11-_iiralpha)*value;return iirvalue;}
  else{return -1;}

}

float Filters::FIR(float value, float* firimpresponse){

  firvalues[firindex] = value;
  firindex++;
  uint8_t sumindex = firindex;
  float out;

  if(firindex == firdimension){firindex = 0;}

  for(int i = 0; i < firdimension; i++){
    if(sumindex>0){
      sumindex--;
    }else{
      sumindex = firdimension-1;
    }
    out += firimpresponse[i]*firvalues[sumindex];
  }

  return out;
}

float Filters::MA(float value){
  return FIR(value,maimpresponse);
}

float Filters::RMS(float value){
  sqrt(MA(sq(value)));
}

//Fine Filters