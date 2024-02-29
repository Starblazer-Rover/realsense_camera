import socket
import numpy as np

def main():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Use SOCK_DGRAM for UDP
    host_ip = 'localhost'  # Change this to the server's IP
    port = 8080
    server_address = (host_ip, port)

    try:
        client_socket.connect(server_address)
        print("Connected to the server")
    except socket.error as e:
        print(f"Error connecting to the server: {e}")
        return

    left_half = None
    right_half = None

    while True:
        # Receive the image data
        print('booooo')

        message, _ = client_socket.recvfrom(4096)

        print("yeeeer")
        image_data = np.frombuffer(message, dtype=np.int32)

        # Split the image data into left and right halves
        if left_half is None:
            left_half = image_data
        else:
            right_half = image_data

            # Combine the left and right halves
            full_image = np.concatenate((left_half, right_half))

            # Print the entire image data
            np.set_printoptions(threshold=np.inf)  # Print the entire array without truncation
            print("Received full image:")
            print(full_image)

            # Reset the halves for the next image
            left_half = None
            right_half = None

if __name__ == '__main__':
    main()
