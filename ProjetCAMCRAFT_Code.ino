#include <ParticleSoftSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <SoftwareSerial.h>
#include <NewSoftSerial.h>
#include <Particle.h>

const int mpuAddress = 0x68;          // I2C address of the MPU-6050
 
float xByGyro, yByGyro, zByGyro;      // Global variables for the rotation by gyro

// The serial connection to the GPS module
SoftwareSerial ss(4, 3);

//This is a character buffer that will store the data from the serial port
char rxData[20];
char rxIndex=0;

//Variables to hold the speed and RPM data.
int vehicleSpeed=0;
int vehicleRPM=0;


void setup()
{
  Serial.begin( 115200);
 
  Wire.begin();
 

  // Initialize the MPU-6050 and test if it is connected.
  Wire.beginTransmission( mpuAddress);
  Wire.write( 0x6B);                           // PWR_MGMT_1 register
  Wire.write( 0);                              // set to zero (wakes up the MPU-6050)
  auto error = Wire.endTransmission();
  if( error != 0)
  {
    Serial.println(F( "Error, MPU-6050 not found"));
    for(;;);                                   // halt the sketch if error encountered
  }

{
  Serial.begin(115200);
  ss.begin(115200);
}
 //Reset the OBD-II-UART
  Serial.println("ATZ");
  //Wait for a bit before starting to send commands after the reset.
  delay(2000);
  
  //Delete any data that may be in the serial port before we begin.
  Serial.flush();


}
 
 
void loop()
{
  Wire.beginTransmission( mpuAddress);
  Wire.write( 0x3B);                   // Starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission( false);        // No stop condition for a repeated start
 
  // The MPU-6050 has the values as signed 16-bit integers.
  // There are 7 values in 14 registers.
  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
 
  Wire.requestFrom( mpuAddress, 14);   // request a total of 14 bytes
  AcX = Wire.read()<<8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY = Wire.read()<<8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read()<<8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read()<<8 | Wire.read();  // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  GyX = Wire.read()<<8 | Wire.read();  // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read()<<8 | Wire.read();  // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read()<<8 | Wire.read();  // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)
 
  // The acceleration is directly mapped into the angles.
  // That is rather artificial.
  // The combined gravity could be used for an angle, while ignoring the strength.
  //
  // The gyro sets the rotation speed.
  // The angle created by the rotation speed is added to the angle by the accelerometer.
  //
  // The conversion from the sensor values to the rotation is just a value
  // that makes it look good on display.
 
  float xByAccel = (float) AcX * 0.0001;      // static angle by accelerometer
  float yByAccel = (float) AcY * 0.0001;
  float zByAccel = (float) AcZ * 0.0001;
 
  xByGyro += (float) GyX * 0.00001;           // moving angle by gyro
  yByGyro += (float) GyY * 0.00001;
  zByGyro += (float) GyZ * 0.00001;
 
  float x = xByAccel + xByGyro;               // combine both angles
  float y = yByAccel + yByGyro;
  float z = zByAccel + zByGyro;
 
  // Keep the radians in range (although the cos/sin functions accept every value)
  if( x < 0.0)
    x += 2.0 * M_PI;
  else if( x > 2.0 * M_PI)
    x -= 2.0 * M_PI;
  if( y < 0.0)
    y += 2.0 * M_PI;
  else if( y > 2.0 * M_PI)
    y -= 2.0 * M_PI;
  if( z < 0.0)
    z += 2.0 * M_PI;
  else if( z > 2.0 * M_PI)
    z -= 2.0 * M_PI;
 
  
 
 
  
  // Temperature by the MPU-6050 is -40 to 85.
  // According to the datasheet:
  //   Temperature in Celsius = (raw_value / 340) + 36.53
  float Celsius = ((float) Tmp / 340.00) + 36.53;

  //GPS Info
  
  while (ss.available() > 0)
  {
    // get the byte data from the GPS
    byte gpsData = ss.read();
    Serial.write(gpsData);
  }
  
 //OBDII
  //Delete any data that may be in the serial port before we begin.  
  Serial.flush();
  void loop(){
  //Delete any data that may be in the serial port before we begin.  
  Serial.flush();
  //Set the cursor in the position where we want the speed data.
  serial.printin(254, BYTE);
  serial.printin(128+8, BYTE);
  //Clear out the old speed data, and reset the cursor position.
  serial.printin("        ");
  serial.printin(254, BYTE);
  serial.printin(128+8, BYTE);
  //Query the OBD-II-UART for the Vehicle Speed
  Serial.println("010D");
  //Get the response from the OBD-II-UART board. We get two responses
  //because the OBD-II-UART echoes the command that is sent.
  //We want the data in the second response.
  getResponse();
  getResponse();
  //Convert the string data to an integer
  vehicleSpeed = strtol(&rxData[6],0,16);
  //Print the speed data to the lcd
  serial.printin(vehicleSpeed);
  serial.printin(" km/h");
  delay(100);
  
  //Delete any data that may be left over in the serial port.
  Serial.flush();
  //Move the serial cursor to the position where we want the RPM data.
  serial.printin(254, BYTE);
  serial.printin(128 + 69, BYTE);
  //Clear the old RPM data, and then move the cursor position back.
  serial.printin("          ");
  serial.printin(254, BYTE);
  serial.printin(128+69, BYTE);

  //Query the OBD-II-UART for the Vehicle rpm
  Serial.println("010C");
  //Get the response from the OBD-II-UART board
  getResponse();
  getResponse();
  //Convert the string data to an integer
  //NOTE: RPM data is two bytes long, and delivered in 1/4 RPM from the OBD-II-UART
  vehicleRPM = ((strtol(&rxData[6],0,16)*256)+strtol(&rxData[9],0,16))/4;
  //Print the rpm data to the lcd
  lcd.print(vehicleRPM); 
  
  //Give the OBD bus a rest
  delay(100);
  
}

void getResponse(void){
  char inChar=0;
  //Keep reading characters until we get a carriage return
  while(inChar != '\r'){
    //If a character comes in on the serial port, we need to act on it.
    if(Serial.available() > 0){
      //Start by checking if we've received the end of message character ('\r').
      if(Serial.peek() == '\r'){
        //Clear the Serial buffer
        inChar=Serial.read();
        //Put the end of string character on our data string
        rxData[rxIndex]='\0';
        //Reset the buffer index so that the next character goes back at the beginning of the string.
        rxIndex=0;
      }
      //If we didn't get the end of message character, just add the new character to the string.
      else{
        //Get the new character from the Serial port.
        inChar = Serial.read();
        //Add the new character to the string, and increment the index variable.
        rxData[rxIndex++]=inChar;
      }
    }
  }
}
}
 

 