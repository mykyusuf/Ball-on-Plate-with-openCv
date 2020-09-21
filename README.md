# Ball on Plate with openCv
 Ball on Plate system with Stm32F407VG Discovery
 
 System uses opencv to catch ball position from the camera and sends its coordinates to STM32F407VG microcontroller via bluetooth. Microcontroller calculates and defines PID Controller to keep ball in balance.
 
# How to Run

 First run the opencv program and put a table tennis ball (white color is recommended) and calibrate the best Hue, Saturation and Value settings for your system. Then connect yoor pc and microcontroller via bluetooth (used HC-05). Then run the STM32F407VG and see results. 
 
# Simple Design

![alt text](https://github.com/mykyusuf/Ball-on-Plate-with-openCv/blob/master/BallOnPlate/OpenCv/gif.gif?raw=true )
