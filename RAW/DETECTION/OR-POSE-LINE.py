import cv2
import cv2.aruco as aruco
import numpy as np

def detect_aruco_and_draw_arrow(image_path):
    # Load the image
    image = cv2.imread(image_path)
    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Initialize the detector ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    # Detect markers in the image
    corners, ids, rejected_points = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)

    # Calculate the orientation of each marker
    angles = []
    if ids is not None:
        for i, corner in zip(ids, corners):
            # Calculate the angle of the marker
            """
                   1_________2
                    |       |
                    | aruco |
                   0|_______|3       
            """


            vector_1 = corner[0][1] - corner[0][0]
            print(corner[0][0])
            print(image.shape)
            cv2.circle(image,(int(corner[0][1][0]),int(corner[0][1][1])), 25, (0,0,255),-1)
            cv2.circle(image,(int(corner[0][0][0]),int(corner[0][0][1])), 25, (0,255,255),-1)

            vector_1_norm = vector_1 / np.linalg.norm(vector_1)
            angle = np.arctan2(vector_1_norm[1], vector_1_norm[0])
            angles.append(np.degrees(angle))

        # Assuming the first marker (with ID 0) as the reference
        reference_idx = np.where(ids == 0)[0][0]
        reference_angle = angles[reference_idx]

        # Calculate the error for each marker relative to the reference
        orientation_errors = [angle - reference_angle for angle in angles]

        # ...
        # Draw detected markers and information on the image
        for idx, (id_val, corner, error) in enumerate(zip(ids, corners, orientation_errors)):
            # Calculate the center point of the ArUco marker
            center = corner[0].mean(axis=0)
            #Ubah nilai posisi x dan y dalam bentuk integer
            center_x, center_y = int(center[0]), int(center[1])  # Separate the center coordinates
            angle_deg = angles[idx]
            if id_val == 0:
                reference_angle = angle_deg  # Assign reference angle from marker ID 0
            else:
                error = angle_deg - reference_angle  # Calculate error with respect to marker ID 0
            text_pos = f"Pos: x={center_x:}, y={center_y:}"
            text_ori = f"Ori: {angle_deg:.2f}"
            text_error = f"Error: {error:.2f}" if idx != reference_idx else "Ref"
            text_x = int(center_x)
            text_y = int(center_y)
            cv2.putText(image, text_pos, (text_x - 900, text_y - 350), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (100, 200, 150), 5)
            cv2.putText(image, text_ori, (text_x - 600, text_y - 250), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (100, 200, 150), 5)
            cv2.putText(image, text_error, (text_x - 600, text_y - 150), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (100, 200, 150), 5)
            arrow_length = 400
            end_point_x = int(center_x + arrow_length * np.cos(np.radians(angle_deg)))
            end_point_y = int(center_y + arrow_length * np.sin(np.radians(angle_deg)))
            cv2.arrowedLine(image, (text_x, text_y), (end_point_x, end_point_y), (0, 0, 255), 10, tipLength=0.5)
            cv2.line(image, (text_x, text_y), (int(center_x + 700), text_y), (0, 0, 0), 10)
        # ...


    # Resize the image to 720p (1280x720)
    resized_image = cv2.resize(image, (1280, 720))
    # Display the image with annotations
    cv2.imshow('Detected ArUco markers with Arrow and Error', resized_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
# Path to the image
image_path = 'D:\SKRIPSI related\PY\ORITEST\oritest1.jpg'
detect_aruco_and_draw_arrow(image_path)