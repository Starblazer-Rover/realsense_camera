import numpy as np
import cv2

# Create a simple image (e.g., a black square on a white background)
image_size = (256, 256)  # Specify the size of the image (width, height)
image = np.ones((image_size[1], image_size[0], 3), dtype=np.uint8) * 255  # Create a white background
square_size = min(image_size) // 2
center = (image_size[0] // 2, image_size[1] // 2)
cv2.rectangle(image, (center[0] - square_size // 2, center[1] - square_size // 2),
              (center[0] + square_size // 2, center[1] + square_size // 2),
              (0, 0, 0), thickness=-1)  # Draw a black square

# Display the generated image
cv2.imshow('Generated Image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
