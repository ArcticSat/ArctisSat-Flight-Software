import cv2 as cv
import numpy as np

# Open the binary file
with open('outImage', 'rb') as file:
    # Read the file content into a numpy array
    file_content = np.fromfile(file, dtype=np.uint8)

# Assuming the image dimensions are known, e.g., 640x480
image_width = 60
image_height = 80

# Reshape the array to the image dimensions
image = file_content.reshape((image_height, image_width))

# Create a resizable window
cv.namedWindow('Image', cv.WINDOW_NORMAL)

# Display the image using OpenCV
cv.imshow('Image', image)
cv.waitKey(0)
cv.destroyAllWindows()