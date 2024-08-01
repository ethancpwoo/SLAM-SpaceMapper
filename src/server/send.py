from flask import Flask, send_file
from picamera2 import Picamera2
import time
import os

app = Flask(__name__)

# read from image DIR

picam2 = PiCamera2()
picam2.start_preview()

time.sleep(2)

@app.route('/latest')
def latest_img(): 
    timestamp = time.now
    filename = os.path.join(IMAGE_DIR, f'image_{timestamp}.jpg')
    picam2.capture_file(filename)

    return send_file(filename, mimetype='image/jpeg')

@app.route('/')
def index():
    return 'Visit /latest'

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)