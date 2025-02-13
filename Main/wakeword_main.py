import speech_recognition as sr
import subprocess
import threading
import time
import os


class WakeWordDetector:
    def __init__(self, wake_word="pathfinder"):
        self.wake_word = wake_word.lower()
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.is_running = True

        # Adjust for ambient noise when starting up
        with self.microphone as source:
            print("Calibrating for ambient noise...")
            self.recognizer.adjust_for_ambient_noise(source, duration=2)
            print("Calibration complete!")

    def process_audio(self):
        while self.is_running:
            try:
                with self.microphone as source:
                    print("Listening for wake word...")
                    audio = self.recognizer.listen(source, timeout=None)

                try:
                    # Convert speech to text
                    text = self.recognizer.recognize_google(audio).lower()
                    print(f"Detected: {text}")

                    # Check if wake word is in the detected text
                    if self.wake_word in text:
                        print(f"Wake word '{self.wake_word}' detected!")
                        self.trigger_action()

                except sr.UnknownValueError:
                    # Speech was unintelligible
                    continue
                except sr.RequestError as e:
                    print(f"Could not request results; {e}")
                    continue

            except Exception as e:
                print(f"Error in audio processing: {e}")
                continue

    def trigger_action(self):
        """
        Triggers the main voice detection program when wake word is detected
        """
        try:
            # Start the main voice detection program in a separate process
            subprocess.Popen(['python', 'voice_detection_main.py'])
        except Exception as e:
            print(f"Error launching voice detection program: {e}")

    def run_in_background(self):
        """
        Starts the wake word detection in a background thread
        """
        thread = threading.Thread(target=self.process_audio)
        thread.daemon = True  # Thread will exit when main program exits
        thread.start()
        return thread

    def stop(self):
        """
        Stops the wake word detection
        """
        self.is_running = False


def main():
    # Create and start the wake word detector
    detector = WakeWordDetector(wake_word="pathfinder")

    # Start detection in background
    detection_thread = detector.run_in_background()

    try:
        # Keep the main program running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        # Handle graceful shutdown
        print("\nShutting down wake word detector...")
        detector.stop()
        detection_thread.join(timeout=2)
        print("Shutdown complete")


if __name__ == "__main__":
    main()