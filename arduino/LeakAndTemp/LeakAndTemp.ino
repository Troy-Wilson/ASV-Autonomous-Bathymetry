/*
  Analog Input
 
 The circuit:
 * leak sensor analog output attached to pins A0 and A1
 * ground pins to gound vcc pins to 3.3V
 * Data returned over serial port emulator through USB
 * port ACM0 in test case. data returned at 1Hz.
 * thermister appears calibrated to 5V. Using values from
 * supplier, probably needs calibration, but better than nothing
 */

#include <math.h>
double Thermister(int RawADC) {
double Temp;
//this is all from supplier, probably need calibration
Temp = log(((10240000/RawADC) - 10000));
Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
Temp = Temp - 273.15;// Convert Kelvin to Celcius
return Temp;
}

int leaksensorA = A0;    
int leaksensorB = A1;   
int thermisterPort = A2;

int sensorVal1 = 0;
int sensorVal2 = 0;
double sensorVal3 = 0;
char tmp[10];

void setup() {
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  sensorVal1 = analogRead(leaksensorA);  
  sensorVal2 = analogRead(leaksensorB);  
  sensorVal3 = Thermister(analogRead(thermisterPort));     
  dtostrf(sensorVal3,1,1,tmp); //converts float to str [input,mindigits,digitsAfterDecimal,output]
  Serial.println("A," + String(sensorVal1));  
  Serial.println("B," + String(sensorVal2));  
  Serial.println("temp," + String(tmp));
  delay(1000); //delay for a second
}
