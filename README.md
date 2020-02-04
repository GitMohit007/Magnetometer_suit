# Magnetometer_suit
package for calibrating , testing and usage of magnetometer
PreRequisits:
  Python27
  Arduino
  I2C Magnetometer
Steps:

1: Upload the mag_test_moho code to arduino

2: Go to "avr_helper_magn_docalibration_01" , Run magn_do_calibration(Python27 Required!) using console and the following commands     can be used :

   "python magn_do_calibration.py -h" - for help
   
   "python magn_do_calibration.py -d" - to debug the coming magnetometer input
   
   "python magn_do_calibration.py -g" - to gather data
   
   
3: the gathered data is stored in "magnpoints.txt"

4: go to magneto , run "magneto12.exe"

5: open "magnpoints.txt" in "magneto.exe"

6: press calibrate to get hard iron biases

7: check "raw_gravitation_field_info.txt" to mitigate soft iron descripencies

8: copy and replace the hard iron biases and soft iron matrix values set in 
Arduino_mag_calibrated/mag_.py.
 
Attention! : copyright to "avr_helper_magn_docalibration_01" folder belongs to Davide Gironi, 2012 , http://davidegironi.blogspot.it/ . 
