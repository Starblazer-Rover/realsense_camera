import cv2
import socket
import numpy as np

BUFF_SIZE = 65536
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)
host_ip = 'localhost'  # Change this to the server's IP
port = 8080
message = b'Hello'

client_socket.sendto(message, (host_ip, port))

# Initialize variables to store received image parts
left_half = None
right_half = None

while True:
    # Receive left half of the image
    left_data, _ = client_socket.recvfrom(BUFF_SIZE)
    # Receive right half of the image
    right_data, _ = client_socket.recvfrom(BUFF_SIZE)

    # Convert received bytes to numpy arrays
    left_npdata = np.frombuffer(left_data, dtype=np.uint8)
    right_npdata = np.frombuffer(right_data, dtype=np.uint8)

    # Decode the image parts
    left_frame = cv2.imdecode(left_npdata, 1)
    right_frame = cv2.imdecode(right_npdata, 1)

    # Check if both parts are received
    if left_frame is not None and right_frame is not None:
        # Concatenate left and right halves to reconstruct the full image
        full_frame = np.concatenate((left_frame, right_frame), axis=1)

        # Display the reconstructed full image
        cv2.imshow("RECEIVING VIDEO", full_frame)

        # Exit on 'q' key press
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    else:
        print("Received incomplete image parts. Retrying...")

# Close the socket and destroy any OpenCV windows
client_socket.close()
cv2.destroyAllWindows()
