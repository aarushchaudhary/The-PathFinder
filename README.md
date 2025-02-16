# The PathFinder

![isometric](https://github.com/user-attachments/assets/151eff5d-a2d8-45b1-aa68-562f9d0d12c8)

## What is 'The PathFinder'
The PathFinder as the names suggests is a rover, and it aims to bring the cost of automation down by offering an alternative to proprietary hardware. It uses a Raspberry Pi 3 Model B which is open source and cheap, also the entire thing runs on python.
## How does it work
For the main board it uses a Raspberry Pi 3 Model B, L298N motor driver for its movement, for motors and wheels geared BO motors are being used. The motor driver is connected to a 18650 battery pack delivering about 12V power, and two servo motors with a pan and tilt mount control the movement for face tracking through Raspberry Pi’s official camera (currently my Raspberry Pi camera is broken so i have used a Logitech C270 webcam that also has a mic inbuilt) and the pan and tilt mount also houses a mic for voice command.
As of now python is being used to run the motor driver and pan and tilt mount, for the voice detection and wakeword detection again python is being used with speech recognition and pttsx3 library (currently the voice recognition is not fully fucntioning, you can currently only control PathFinder's movement through your voice completely).
## In Action
It currently has 6 functions in present release-
1. Voicecar- Control the movement of The PathFinder using your voice, commands are- front, back, left, right, stop. (filename- voice_control_main.py)
2. Facetrack- Pan and tilt face tracking using mediapipe library of Google. (filename- face_tracking_pantilt.py)
3. Follow Me- Control the movement of The PathFinder using your face, where ever you go it will follow you using its pan and tilt, motors and camera.
          
   (Has two files, face_tracking_nopan_main_control.py is currently working fine but has no pan in camera, face_tracking_main_control.py has both pan and tilt but is currently not working as expected)
4. Voice Menu- All voice commands are present here, you can execute any function using your voice, currently under development. (filename- voice_detection_main.py)
5. Wakeword- Main function, works like your personal assistant, the keyword is 'pathfinder', on hearing the keyword the Voice Menu opens, currently under development. (filename- wakeword_main.py)
6. Game Car- Control PathFinder's movement using Xbox compatible controller with a camera feed. (Right joystick for steer, RT for speed, Left joystick for camera pan and tilt)

Apart from this a centre_servo.py file is there to center the servo motor if needed.
## Required dependencies
Install these requirements using terminal,
```
sudo apt-get update
sudo apt-get install pigpio
sudo apt-get install python-pigpio python3-pigpio
sudo apt-get install pulseaudio
sudo apt-get install python3-rpi.gpio
```
Install these python libraries using terminal, if you face any error replace 'pip' with 'pip3'
```
pip install opencv-python
pip install mediapipe
pip install python-time
pip install statistics
pip install SpeechRecognition
pip install pyttsx3
pip install pygame
pip install os-sys
pip install python-dateutil
pip install subprocess.run
pip install thread6
```
## How to run and Connections
After installing all dependencies and libraries open whatever function file that you wish to use, note that the Voice Menu and Wakeword files are not working as expected.
### Connection Diagram
![circuit_image](https://github.com/user-attachments/assets/e41170c2-53f1-4e83-a02c-1d89ed03bcaa)
### Connections
* ENA- 25
* IN1- 27
* IN2- 22
* ENB- 17
* IN3- 23
* IN4- 24
(These are Raspberry Pi GPIO)
---
* OUT1- Motor A +ve
* OUT2- Motor A -ve
* OUT3- Motor B -ve
* OUT4- Motor B +ve
---
* +12V- Battery +ve
* GND- Battery -ve, Servo 1 -ve, Servo 2 -ve, RPi Ground GPIO, Speaker Module -ve
* +5V- Servo 1 +ve, Servo 2 +ve, Speaker Module +ve
---
* Servo 1 PWM (PAN Control)- 12
* Servo 2 PWM (TILT Control)- 13
(These are Raspberry Pi GPIO)
---
Apart from this connect the webcam to the usb port on the Raspberry Pi and connect bluetooth speaker module using bluetooth (default the pulseaudio output to bluetooth)

### Pinout Diagram
![raspberry-pi-pinout](https://github.com/user-attachments/assets/6ca5c245-9c44-480d-bee9-041be4a1e4f4)

## Future
The Future of The PathFinder looks promising considering an inclusion of a robotic arm with object tracking in it, that could be used for picking up and placing back objects that will make it more useful.

## License
