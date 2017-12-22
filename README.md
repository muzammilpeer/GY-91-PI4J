Welcome to the GY-91-PI4J wiki!

GY-91, 10 DOF, Accelerometer, Gyroscope, Magnetometer, Pressure sensor
Having 2 modules inside it MPU-9250 and BMP-280.

Converted the c code into java to work for my project.
Using pi4j.

To detect the GY-91 sensor with raspberry pi type in terminal.
sudo i2cdetect -y 1

OR 

sudo i2cdetect -y 0 

<img width="693" alt="screen shot 2017-12-22 at 8 32 44 pm" src="https://user-images.githubusercontent.com/859865/34304518-f6403756-e75b-11e7-8572-58ed4c003ca9.png">


Check the sensors addresses:
And change if required.
I found x68 address for MPU-9250
and x76 address for BMP-280.

then run this command in terminal.
first move to jar directory 'project/out/artificts/sensors/' using cd.

sudo java -jar sensors.jar 

<img width="690" alt="screen shot 2017-12-22 at 8 33 00 pm" src="https://user-images.githubusercontent.com/859865/34304543-0c541562-e75c-11e7-8f3d-4bb7561dbcd5.png">





![img_20171222_211234](https://user-images.githubusercontent.com/859865/34304828-373873da-e75d-11e7-859f-bc15c9649916.jpg)


Special Thanks to Referenced c-language source code: (If you want the c version of the code you can use this zip)
http://roboturka.com/wp-content/uploads/GY-91.zip
or you can find the c code in this repo as well.

compile using gcc:
gcc -Wall -o gy91 gy91.c -lwiringPi -lm

run:
./gy91



Wire Connection:

Connect only 4 pins if using raspberry pi.

VCC -> 5v pin of raspberry pi

GND -> Ground Pin of raspberry pi

SCL -> SCL pin of raspberry pi

SDA -> SDA pin of raspberry pi


That's it.




