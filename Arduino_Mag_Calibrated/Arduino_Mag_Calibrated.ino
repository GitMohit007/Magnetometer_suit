#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
//HMC5883L compass; //Copy the folder "HMC5883L" in the folder "C:\Program Files\Arduino\libraries" and restart the arduino IDE.
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float xv, yv, zv;
float headingDegrees;
//calibrated_values[3] is the global array where the calibrated data will be placed
//calibrated_values[3]: [0]=Xc, [1]=Yc, [2]=Zc
float calibrated_values[3];   
//transformation(float uncalibrated_values[3]) is the function of the magnetometer data correction 
//uncalibrated_values[3] is the array of the non calibrated magnetometer data
//uncalibrated_values[3]: [0]=Xnc, [1]=Ync, [2]=Znc
void transformation(float uncalibrated_values[3])    
{
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] = 
  {
    {0.083832,0.001101, -0.010638},
    {0.001101,0.096019,-0.018923},
    {-0.010638,-0.018923, 0.113036}  
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
    11.008123,
    -22.149114,
    -1.271789
  };   
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}

//vector_length_stabilasation() - is the function of the magnetometer vector length stabilasation (stabilisation of the sphere radius)
float scaler;
boolean scaler_flag = false;
float normal_vector_length;
void vector_length_stabilasation(){
  //calculate the normal vector length
  if (scaler_flag == false)
  {
    getHeading();
    normal_vector_length = sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
    scaler_flag = true;
  } 
  //calculate the current scaler
  scaler = normal_vector_length/sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
  //apply the current scaler to the calibrated coordinates (global array calibrated_values)
  calibrated_values[0] = calibrated_values[0]*scaler;
  calibrated_values[1] = calibrated_values[1]*scaler;
  calibrated_values[2] = calibrated_values[2]*scaler;
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
//  delay(50);
} 

void setup()
{   
  Serial.begin(9600);
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    while(1);
  }
 // mag.begin();  
 //compass = HMC5883L();  
  //setupHMC5883L();       
  displaySensorDetails();
}

void loop()
{
  float values_from_magnetometer[3];
  getHeading();
  values_from_magnetometer[0] = xv;
  values_from_magnetometer[1] = yv;
  values_from_magnetometer[2] = zv;
  transformation(values_from_magnetometer);
  
  vector_length_stabilasation();

  Serial.flush(); 
  Serial.print(calibrated_values[0]); 
  Serial.print(",");
  Serial.print(calibrated_values[1]);
  Serial.print(",");
  Serial.print(calibrated_values[2]);//----------------------------------------------
  Serial.print(",");
  float heading = atan2(calibrated_values[0], calibrated_values[1]);
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  headingDegrees = heading * 180/M_PI; 
 Serial.print(headingDegrees);
  Serial.println();
  
 // Serial.println();

  delay(100); 
} 


 
void getHeading()
{ 
  sensors_event_t event; 
  mag.getEvent(&event);
  xv=event.magnetic.x;
  yv=event.magnetic.y;
  zv=event.magnetic.z;
}
