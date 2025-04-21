import cv2
import numpy as np

# Set the desired ArUco dictionary
desired_aruco_dictionary = "DICT_5X5_100"

# The different ArUco dictionaries built into the OpenCV library
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# Example calibration matrix and distortion coefficients
matrix_coefficients = np.array([
    [1.37993424e+03, 0.00000000e+00, 6.97533136e+02],
    [0.00000000e+00, 1.38551911e+03, 4.10583251e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

distortion_coefficients = np.array([
    [0.1330664, -0.84474222, -0.00719269, -0.00640081, 1.18991264]
])

def compute_relative_transform(base_rvec, base_tvec, target_rvec, target_tvec):
    """
    Compute the relative transformation (rotation and translation) between two markers.

    Parameters:
    base_rvec, base_tvec: Rotation and translation vectors of the base marker.
    target_rvec, target_tvec: Rotation and translation vectors of the target marker.

    Returns:
    relative_rvec, relative_tvec: Relative rotation and translation vectors.
    """
    # Convert rotation vectors to rotation matrices
    base_rmat, _ = cv2.Rodrigues(base_rvec)
    target_rmat, _ = cv2.Rodrigues(target_rvec)

    # Ensure base_tvec and target_tvec are reshaped to (3,)
    base_tvec = base_tvec.reshape(3)
    target_tvec = target_tvec.reshape(3)

    # Compute relative rotation matrix and translation vector
    relative_rmat = target_rmat @ base_rmat.T
    relative_tvec = target_tvec - (relative_rmat @ base_tvec)

    # Convert relative rotation matrix back to rotation vector
    relative_rvec, _ = cv2.Rodrigues(relative_rmat)
    return relative_rvec, relative_tvec

def main():
    if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
        print(f"[INFO] ArUCo tag of '{desired_aruco_dictionary}' is not supported")
        return

    # Load the ArUco dictionary
    print(f"[INFO] Detecting '{desired_aruco_dictionary}' markers...")
    this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
    this_aruco_parameters = cv2.aruco.DetectorParameters()

# OpenCV 비디오 캡처 객체 생성 (카메라 2번 장치 사용)
    cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FPS, 25)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # 해상도값 출력
    print(cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    if not cap.isOpened():
        print("[ERROR] Could not open camera.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Could not read frame.")
            break

        # Detect ArUco markers in the video frame
        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame, this_aruco_dictionary, parameters=this_aruco_parameters)

        # Draw detected markers and pose axes
        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            marker_transforms = {}

            for marker_corner, marker_id in zip(corners, ids):
                # Estimate pose of the marker
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    marker_corner, 0.05, matrix_coefficients, distortion_coefficients)

                # Draw pose axes on the marker
                cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05)

                # Save the pose (rvec, tvec) for each marker
                marker_transforms[marker_id] = (rvec, tvec)

            # Compute relative transforms for all markers relative to id == 0
            if 0 in marker_transforms:
                base_rvec, base_tvec = marker_transforms[0]
                for marker_id, (rvec, tvec) in marker_transforms.items():
                    if marker_id != 0:
                        # Compute relative transform
                        relative_rvec, relative_tvec = compute_relative_transform(
                            base_rvec[0], base_tvec[0], rvec[0], tvec[0]
                        )

                        # Display the relative transform next to the marker
                        position = tuple(corners[ids.tolist().index(marker_id)][0][0].astype(int))
                        text = f"dx: {float(relative_tvec[0]):.2f}, dy: {float(relative_tvec[1]):.2f}, dz: {float(relative_tvec[2]):.2f}\n" \
                               f"dr: {np.degrees(float(relative_rvec[0])):.2f}, dp: {np.degrees(float(relative_rvec[1])):.2f}, dy: {np.degrees(float(relative_rvec[2])):.2f}"
                        cv2.putText(frame, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        # Display the frame
        cv2.imshow("ArUco Detection", frame)

        # Exit the loop on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
