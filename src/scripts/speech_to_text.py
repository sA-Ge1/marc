import speech_recognition as sr

# Initialize the recognizer
r = sr.Recognizer()

# Speech-to-text loop
while True:
    with sr.Microphone() as source:
        print("Please say something...")
        audio = r.listen(source)
        try:
            # Recognize speech using Google Web Speech API
            text = r.recognize_google(audio)
            print("You said: " + text)
            
            # Stop the loop if the user says "stop"
            if "stop" in text.lower():
                print("Stopping...")
                break
        except sr.UnknownValueError:
            print("Sorry, I could not understand the audio.")
        except sr.RequestError:
            print("Could not request results from Google Speech Recognition Service.")
