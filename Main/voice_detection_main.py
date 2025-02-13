import speech_recognition as sr
import pyttsx3

engine = pyttsx3.init()

def main():
    """The main function."""
    # Create a speech recognizer object.
    recognizer = sr.Recognizer()

    # Slow down the speech rate
    engine.setProperty('rate', 150)  # Adjust the value (lower = slower)

    # Get the audio from the microphone.
    with sr.Microphone() as mic:
        print("Start recording")
        engine.say("How may I assist you")
        engine.runAndWait()
        audio = recognizer.listen(mic, phrase_time_limit=7)
        print("Stop recording")

    # Recognize the speech.
    try:
        # The text transcription of the audio.
        text = recognizer.recognize_google(audio)
        print("You said:", text)  # Print what the user said
    except sr.UnknownValueError:
        engine.say("Could not understand audio")
        engine.runAndWait()
        print("Could not understand audio")
        return  # Exit the function if speech is not understood

    except sr.RequestError:
        engine.say("Could not connect to Google Speech API")
        engine.runAndWait()
        print("Could not connect to Google Speech API")
        return  # Exit the function if there's a request error

    # Open the file corresponding to the recognized text.
    if "follow me" in text.lower():  # make it case insensitive
        engine.say("Follow sequence initiated")
        file_name = "face_tracking_nopan_main_control.py.py"
    elif "track my face" in text.lower():
        engine.say("Tracking Face")
        file_name = "face_tracking_pantilt.py"
    elif "enter voice car" in text.lower():
        engine.say("Entering Voice Car Mode")
        file_name = "voice_control_main.py"
    else:
        print("No file to open")
        engine.say("No command given")
        with open ("wakeword_main.py") as f:
            exec(f.read())
        return  # Exit if no file to open

    try:
        with open(file_name) as f:
            exec(f.read())
    except FileNotFoundError:
        engine.say(f"File '{file_name}' not found.")
        engine.runAndWait()
        print(f"File '{file_name}' not found.")
    except Exception as e:  # Catch any other exceptions during file execution
        engine.say(f"An error occurred: {e}")
        engine.runAndWait()
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    main()