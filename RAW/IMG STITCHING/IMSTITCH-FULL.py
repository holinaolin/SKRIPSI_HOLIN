import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load images for stitching
image1 = cv2.imread('D:\SKRIPSI related\PY\IMG STITCHING\DATA TEST STITCHING\P1.png')
image2 = cv2.imread('D:\SKRIPSI related\PY\IMG STITCHING\DATA TEST STITCHING\P2.png')

# Tampilkan gambar asli secara bersebelahan
fig, ax = plt.subplots(1, 2, figsize=(14, 10))
ax[0].imshow(image1)
ax[0].set_title("PIC 1 (un)")
ax[0].axis("off")
ax[1].imshow(image2)
ax[1].set_title("PIC 2 (un)")
ax[1].axis("off")
plt.show()

camera_matrix = np.load(r'D:\SKRIPSI related\PY\CAMERA CALIBRATION\NYK\NYK-Calibrate\cam-mat-NYK.npy')
dist_coeffs = np.load(r'D:\SKRIPSI related\PY\CAMERA CALIBRATION\NYK\NYK-Calibrate\dist-coeffs-NYK.npy')

undistorted_image1 = cv2.undistort(image1, camera_matrix, dist_coeffs)
cv2.imshow('Undistorted Image 1', undistorted_image1)
cv2.waitKey(0)
cv2.destroyAllWindows()
undistorted_image2 = cv2.undistort(image2, camera_matrix, dist_coeffs)
cv2.imshow('Undistorted Image 2', undistorted_image2)
cv2.waitKey(0)

# Konversi gambar menjadi grayscale
gray_image1 = cv2.cvtColor(undistorted_image1, cv2.COLOR_BGR2GRAY)
gray_image2 = cv2.cvtColor(undistorted_image2, cv2.COLOR_BGR2GRAY)

# Create a Stitcher object
stitcher = cv2.Stitcher_create()
status, stitched_image = stitcher.stitch((undistorted_image1, undistorted_image2))
if status == cv2.Stitcher_OK:
    cv2.imshow('Stitched Image', stitched_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print('Image stitching failed!')


