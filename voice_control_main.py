#importing all libraries required
import speech_recognition as sr
import pyttsx3
import pygame
import RPi.GPIO as GPIO
import os
import time
from datetime import datetime

#initializing pyttsx3 engine
engine = pyttsx3.init()
engine.setProperty('rate', 150)

try:
    GPIO.setwarnings(False)  # Disable warnings
    GPIO.setmode(GPIO.BOARD)

    # Motor control pins
    PINS = {
        'ENA': 11,
        'IN1': 13,
        'IN2': 15,
        'IN3': 16,
        'IN4': 18,
        'ENB': 22
    }

    # Setup all pins
    for pin in PINS.values():
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, False)  # Initialize all pins to OFF

    # Setup PWM for motor speed control
    pwm_a = GPIO.PWM(PINS['ENA'], 100)
    pwm_b = GPIO.PWM(PINS['ENB'], 100)
    pwm_a.start(50)
    pwm_b.start(50)
except Exception as e:
    print(f"Failed to initialize GPIO: {e}")
    exit(1)

def speak_and_print(text):
    """Helper function to both speak and print messages"""
    print(text)
    engine.say(text)
    engine.runAndWait()
    time.sleep(0.5)  # Added delay after speaking

def movefwd():
    speak_and_print("Moving Forward")
    GPIO.output(PINS['IN1'], False)
    GPIO.output(PINS['IN2'], True)
    GPIO.output(PINS['IN3'], False)
    GPIO.output(PINS['IN4'], True)

def movebwd():
    speak_and_print("Moving Backward")
    GPIO.output(PINS['IN1'], True)
    GPIO.output(PINS['IN2'], False)
    GPIO.output(PINS['IN3'], True)
    GPIO.output(PINS['IN4'], False)

def moverwd():
    speak_and_print("Turning Right")
    GPIO.output(PINS['IN1'], True)
    GPIO.output(PINS['IN2'], False)
    GPIO.output(PINS['IN3'], False)
    GPIO.output(PINS['IN4'], True)

def movelwd():
    speak_and_print("Turning Left")
    GPIO.output(PINS['IN1'], False)
    GPIO.output(PINS['IN2'], True)
    GPIO.output(PINS['IN3'], True)
    GPIO.output(PINS['IN4'], False)

def stop():
    speak_and_print("Stopping")
    for pin in [PINS['IN1'], PINS['IN2'], PINS['IN3'], PINS['IN4']]:
        GPIO.output(pin, False)

def cleanup():
    """Cleanup GPIO on program exit"""
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()

def main():
    # Create a speech recognizer object.
    recognizer = sr.Recognizer()

    with sr.Microphone() as mic:
        print("Start recording")
        engine.say("What direction do you want to go")
        engine.runAndWait()
        audio = recognizer.listen(mic, phrase_time_limit=10)
        print("Stop recording")

        # Recognize the speech.
    try:
        # The text transcription of the audio.
        text = recognizer.recognize_google(audio)
    except sr.UnknownValueError:
        engine.say("Could not understand audio")
        engine.runAndWait()
        print("Could not understand audio")
    except sr.RequestError:
        engine.say("Could not connect to Google Speech API")
        engine.runAndWait()
        print("Could not connect to Google Speech API")

        # Open the file corresponding to the recognized text.
    if "front" in text:
        movefwd()
        time.sleep(2)
        stop()
        main()
    elif "back" in text:
        movebwd()
        time.sleep(2)
        stop()
        main()
    elif "right" in text:
        moverwd()
        time.sleep(2)
        stop()
        main()
    elif "left" in text:
        movelwd()
        time.sleep(2)
        stop()
        main()
    elif "Stop" in text:
        stop()
    else:
        engine.say("Command not found")
if __name__ == "__main__":
    main()

