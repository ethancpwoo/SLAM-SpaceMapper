import numpy as np
import urllib
import cv2

def url_to_image(url):
    resp = urllib.urlopen(url)
    image = np.asarray(bytearray(resp.read()), dtype="uint8")
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    return image

if __name__ == '__main__':
    img = url_to_image('192.168.0.36:5000')