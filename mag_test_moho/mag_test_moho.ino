#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

//HMC5883L compass; //Copy the folder "HMC5883L" in the folder "C:\Program Files\Arduino\libraries" and restart the arduino IDE.
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

float xv, yv, zv;




void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  /*Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");*/
  delay(500);
} 

void setup()
{   
  Serial.begin(9600);
  //Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
  //  Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
    /* Display some basic information on this sensor */
  displaySensorDetails();
        
}

void loop()
{
 // getHeading();
  sensors_event_t event; 
  mag.getEvent(&event);
  Serial.flush();
  Serial.print(event.magnetic.x); 
  Serial.print(",");
  Serial.print(event.magnetic.y);
  Serial.print(",");
  Serial.print(event.magnetic.z);
  Serial.println();
  
  delay(150); 
} 

void getHeading()
{
  sensors_event_t event; 
  mag.getEvent(&event);
  xv = event.magnetic.x;
  yv= event.magnetic.y;
  zv= event.magnetic.z;
  
}
