from picamera2 import Picamera2


# Initialize camera
picam2 = Picamera2()

# Configure for still image capture
picam2.configure(picam2.create_still_configuration())

# Start the camera and capture an image
picam2.start()
picam2.capture_file("picam_test.jpg")
picam2.stop()

