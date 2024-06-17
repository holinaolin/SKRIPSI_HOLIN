import cv2
import numpy as np
import os

ARUCO_DICT = cv2.aruco.DICT_6X6_250
SQUARES_VERTICALLY = 7 # row
SQUARES_HORIZONTALLY = 5 #col
SQUARE_LENGTH = 0.046 # ukuran kotak catur dalam meter
MARKER_LENGTH = 0.027 # ukuran aruco dalam meter

PATH_TO_YOUR_IMAGES = r"D:\SKRIPSI related\PY\CAMERA CALIBRATION\\NYK\\NYK-Calibrate"

all_rvecs = []
all_tvecs = []
all_corners = []
all_ids = []
dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard(size=(SQUARES_HORIZONTALLY, SQUARES_VERTICALLY), squareLength=SQUARE_LENGTH, markerLength=MARKER_LENGTH, dictionary=dictionary)
params = cv2.aruco.DetectorParameters()
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
print('params: ', params)

def calibrate_and_save_parameters():
    # Load images from folder
    print([i for i in os.listdir(PATH_TO_YOUR_IMAGES)])
    image_files = [PATH_TO_YOUR_IMAGES+"\\"+f for f in os.listdir(PATH_TO_YOUR_IMAGES) if f.endswith(".jpg")]
    image_files.sort()  # Ensure files are in order

    all_charuco_corners = []
    all_charuco_ids = []
    print(image_files)

    for image_file in image_files:
        print("image_files:", image_files)
        print(f"image_file:{image_file}")
        image = cv2.imread(image_file)
        image_copy = image.copy()
        [marker_corners, marker_ids, _] = cv2.aruco.detectMarkers(image, board.getDictionary(), parameters=params)
        print(f"markercorner:{marker_corners}")
        # If at least one marker is detected
        if marker_ids is not None and len(marker_ids) >= 4 :
            cv2.aruco.drawDetectedMarkers(image_copy, marker_corners, marker_ids)
            retval,charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, image, board)
            if retval:
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
            if charuco_ids is not None and len(charuco_ids) > 0:
                cv2.aruco.drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids, (255, 0, 0))
        cv2.imshow('image', image_copy)
        cv2.waitKey(1)
          
    print("Corner Count:", len(all_charuco_corners))
    print("ID Count:", len(all_charuco_ids))

    # Verifikasi bahwa setiap entri memiliki ID yang sesuai
    for corners, ids in zip(all_charuco_corners, all_charuco_ids):
        print("Corners:", len(corners), "IDs:", len(ids))
        assert len(corners) == len(ids), "Jumlah sudut dan ID tidak cocok."

        # print(all_charuco_corners)
        # print(all_charuco_ids)

    # print("ini", all_charuco_corners[0].shape)
        # Calibrate camera
        # retval, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(all_charuco_corners, all_charuco_ids, board, image.shape[:2], None, None)
        retval, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(all_charuco_corners, all_charuco_ids, board, image.shape[:2], None, None)
        all_rvecs.append(rvecs)
        all_tvecs.append(tvecs)

    # Save rvecs and tvecs
    np.save(r'D:\SKRIPSI related\PY\CAMERA CALIBRATION\\NYK\NYK-Calibrate\\rvecs.npy', rvecs)
    np.save(r'D:\SKRIPSI related\PY\CAMERA CALIBRATION\\NYK\NYK-Calibrate\\tvecs.npy', tvecs)
    np.save(r'D:\SKRIPSI related\PY\CAMERA CALIBRATION\\NYK\NYK-Calibrate\cam-mat-NYK.npy', camera_matrix)
    np.save(r'D:\SKRIPSI related\PY\CAMERA CALIBRATION\\NYK\NYK-Calibrate\dist-coeffs-NYK.npy', dist_coeffs)
    
    return all_charuco_corners, all_charuco_ids, camera_matrix, dist_coeffs, board

def calculate_reprojection_error(all_charuco_corners, all_charuco_ids, camera_matrix, dist_coeffs, board, all_rvecs, all_tvecs):
    print("charuco")
    mean_error = 0
    print("len: ", len(all_charuco_corners))

    for i in range(len(all_charuco_corners)):
        print('gambar ke-: ', i)
        charuco_points = all_charuco_corners[i]
        charuco_ids = all_charuco_ids[i]
        rvecs = all_rvecs[i]
        tvecs = all_tvecs[i]
        rvecs = np.array(rvecs)
        tvecs = np.array(tvecs)
        status, rvecs, tvecs = cv2.aruco.estimatePoseCharucoBoard(charuco_points, charuco_ids, board, camera_matrix, dist_coeffs, rvecs, tvecs)
        print("estimate pose charuco ","berhasil" if status else "gagal")
        obj_points = np.array(board.getObjPoints(), dtype=np.float32)
        print("obj points: ",obj_points)
        obj_points = obj_points.reshape(-1, 3)  # Menyesuaikan dimensi obj_points
        # print("sebelum project points")
        imgpoints, _ = cv2.projectPoints(obj_points, all_rvecs[i][0], all_tvecs[i][0], camera_matrix, dist_coeffs)
        # print("stelah project points")
        # Mengubah all_corners[i] dan imgpoints ke format 2D
        corners_2d = all_charuco_corners[i].reshape(-1, 2)
        
        imgpoints_2d = imgpoints.squeeze()
        # print("squeeze done")

        # Menyesuaikan ukuran array jika perlu
        min_len = min(len(corners_2d), len(imgpoints_2d))
        corners_2d = corners_2d[:min_len]
        imgpoints_2d = imgpoints_2d[:min_len]

        # Menghitung norma dengan format yang benar
        error = cv2.norm(corners_2d, imgpoints_2d, cv2.NORM_L2) / len(corners_2d)
        mean_error += error
        print("MEAN ERROR : ", mean_error)
        print(len(all_charuco_corners))

    return mean_error/len(all_charuco_corners)

def detect_pose(image, camera_matrix, dist_coeffs):
    # Undistort the image
    undistorted_image = cv2.undistort(image, camera_matrix, dist_coeffs)

    # Define the aruco dictionary and charuco board
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
    params = cv2.aruco.DetectorParameters()

    # Detect markers in the undistorted image
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(undistorted_image, dictionary, parameters=params)

    # If at least one marker is detected
    if len(marker_ids) > 0:
        # Interpolate CharUco corners
        charuco_retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, undistorted_image, board)

        # If enough corners are found, estimate the pose
        if charuco_retval:
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(charuco_corners, charuco_ids, board, camera_matrix, dist_coeffs, None, None)

            # If pose estimation is successful, draw the axis
            if retval:
                cv2.drawFrameAxes(undistorted_image, camera_matrix, dist_coeffs, rvec, tvec, length=0.1, thickness=15)
    return undistorted_image

def main():
    camera_matrix = np.load(r'D:\SKRIPSI related\PY\CAMERA CALIBRATION\NYK\NYK-Calibrate\cam-mat-NYK.npy')
    dist_coeffs = np.load(r'D:\SKRIPSI related\PY\CAMERA CALIBRATION\NYK\NYK-Calibrate\dist-coeffs-NYK.npy')
    print("cam2: ", camera_matrix)
    print("dist2: ", dist_coeffs)

    # Iterate images in the folder
    image_files = [os.path.join(PATH_TO_YOUR_IMAGES, f) for f in os.listdir(PATH_TO_YOUR_IMAGES) if f.endswith(".jpg")]
    image_files.sort()  # Ensure files are in order

    for image_file in image_files:
        # Load an image
        image = cv2.imread(image_file)

        # Detect pose and draw axis
        pose_image = detect_pose(image, camera_matrix, dist_coeffs)

        # Show the image
        cv2.imshow('Pose Image', pose_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()
    all_charuco_corners, all_charuco_ids, camera_matrix, dist_coeffs, board = calibrate_and_save_parameters()
    hasil = calculate_reprojection_error(all_charuco_corners, all_charuco_ids, camera_matrix, dist_coeffs, board, all_rvecs, all_tvecs)
    print("hasil : ",hasil)