import cv2
import numpy as np

# Input from user
video_path  = "./video_calibracao2.mp4"
rows        = int(input("Numbers of row in the chessboard(Default is 8): ").strip() or "8")
columns     = int(input("Numbers of columns in the chessboard(Default is 5`): ").strip() or "5")
square_size = float(input("Size of the square in the chessboard(in meters)(Default is 0.0255): ").strip() or "0.0255")

# Define the chessboard size (number of corners in rows and columns)
chessboard_size = (rows, columns)

# Prepare object points like (0,0,0), (1,0,0), (2,0,0), ..., (rows - 1, columns - 1, 0)
objp = np.zeros((np.prod(chessboard_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)*square_size

# Arrays to store object points and image points from all frames
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

ZED = False

# Define frame rate reduction - process every nth frame
frame_skip = 10  # e.g., process every 10th frame
frame_count = 0

total_frames = 0
processed_frames = 0
detected_frames = 0

# Open the video file
cap = cv2.VideoCapture(video_path)  # Replace with your video file path
while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame_count += 1
    total_frames += 1
    
    # Process only every nth frame
    if frame_count % frame_skip == 0:
        processed_frames += 1
        if ZED:
            height, width, _ = frame.shape
            half_width = width // 2
            frame = frame[:, half_width:]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if ret:
            detected_frames += 1
            objpoints.append(objp)  # Add object points
            imgpoints.append(corners)  # Add image points

            # Draw and display the corners
            cv2.drawChessboardCorners(frame, chessboard_size, corners, ret)
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    print(f"Total Frames: {total_frames}, Processed Frames: {processed_frames}, Detected Frames: {detected_frames}", end='\r')

cap.release()
cv2.destroyAllWindows()

# Perform camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save calibration results
print(f"Camera matrix:\n{mtx}")
print(f"Distortion coefficients:\n{dist}")
np.savez('config/calibration_data_movel.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)

# Print out the results
