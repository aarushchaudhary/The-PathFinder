import pyttsx3 as tts
engine=tts.init()
engine.setProperty('rate', 150)
engine.say("Follow sequence initiated")
engine.runAndWait()
print("Follow sequence initiated")
with open("face_tracking_nopan_main_control.py") as f:
    exec(f.read())
