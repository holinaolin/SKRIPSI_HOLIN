import cv2
import numpy as np

# Definisikan kamus dan papan Charuco
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
SQUARES_VERTICALLY = 7  # jumlah baris
SQUARES_HORIZONTALLY = 5  # jumlah kolom
SQUARE_LENGTH = 0.046  # ukuran kotak catur dalam meter
MARKER_LENGTH = 0.021  # ukuran marker aruco dalam meter
PATH_TO_YOUR_IMAGES = r"D:\SKRIPSI related\PY\CAMERA CALIBRATION\NYK\NYK-Calibrate"

def calibrate_and_save_parameters():
    # Buat papan Charuco
    charuco_board = cv2.aruco.CharucoBoard(size=(SQUARES_HORIZONTALLY, SQUARES_VERTICALLY), squareLength=SQUARE_LENGTH, markerLength=MARKER_LENGTH, dictionary=ARUCO_DICT)

    # Parameter deteksi marker
    params = cv2.aruco.DetectorParameters()

    # Kumpulkan sudut dan ID dari markers
    all_charuco_corners = []
    all_charuco_ids = []

    # Asumsikan hanya menggunakan satu gambar untuk demonstrasi
    image_file = r"D:\SKRIPSI related\PY\CAMERA CALIBRATION\NYK\NYK-Calibrate\\25.jpg"  # Anda perlu mengganti ini dengan path gambar yang sebenarnya
    image = cv2.imread(image_file)
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Deteksi markers
    corners, ids, _ = cv2.aruco.detectMarkers(image, ARUCO_DICT, parameters=params)
    
    # Jika ada marker yang terdeteksi, lanjutkan dengan deteksi sudut Charuco
    if ids is not None:
        retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=image,
            board=charuco_board
        )
        if charuco_corners is not None and charuco_ids is not None:
            all_charuco_corners.append(charuco_corners)
            all_charuco_ids.append(charuco_ids)

    # Jika tidak ada cukup marker, tidak bisa melakukan kalibrasi
    if not all_charuco_corners:
        raise ValueError("Tidak cukup marker yang terdeteksi untuk melakukan kalibrasi.")

    # Kalibrasi kamera
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_charuco_corners,
        charucoIds=all_charuco_ids,
        board=charuco_board,
        imageSize=image.shape[:2],
        cameraMatrix=None,
        distCoeffs=None
    )

    # Simpan data kalibrasi
    np.save(r'D:\SKRIPSI related\PY\CAMERA CALIBRATION\NYK\NYK-Calibrate\cam-mat-NYK2.npy', camera_matrix)
    np.save(r'D:\SKRIPSI related\PY\CAMERA CALIBRATION\NYK\NYK-Calibrate\dist-coeffs-NYK2.npy', dist_coeffs)

    resized_image = cv2.resize(image, (640, 360))

    # Tampilkan gambar asli dan gambar yang sudah di-distorsi
    undistorted_image = cv2.undistort(resized_image, camera_matrix, dist_coeffs)
    comparison_image = np.hstack((resized_image, undistorted_image))

    # Tampilkan gambar
    cv2.imshow('Original (Left) vs Undistorted (Right)', comparison_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Jalankan fungsi kalibrasi
calibrate_and_save_parameters()
camera_matrix = np.load(r'D:\SKRIPSI related\PY\CAMERA CALIBRATION\NYK\NYK-Calibrate\cam-mat-NYK2.npy')
dist_coeffs = np.load(r'D:\SKRIPSI related\PY\CAMERA CALIBRATION\NYK\NYK-Calibrate\dist-coeffs-NYK2.npy')

# Tampilkan hasil kalibrasi
print("Camera matrix: ", camera_matrix)
print("\n")
print("Distortion coefficients: ", dist_coeffs)