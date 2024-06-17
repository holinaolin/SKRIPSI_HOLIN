import cv2
import numpy as np

# Load the images
image1 = cv2.imread('D:\SKRIPSI related\PY\CAMERA CALIBRATION\DATA w PAK TIO\IMstitching\I0.jpg') #KANAN
image2 = cv2.imread('D:\SKRIPSI related\PY\CAMERA CALIBRATION\DATA w PAK TIO\IMstitching\I1.jpg') #KIRI

## Show both image
# cv2.imshow('Image 1', image1)
# cv2.imshow('Image 2', image2)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# Convert images to grayscale
gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

# Initialize ORB detector
orb = cv2.ORB_create()

# Find the keypoints and descriptors with ORB
keypoints1, descriptors1 = orb.detectAndCompute(gray1, None)
keypoints2, descriptors2 = orb.detectAndCompute(gray2, None)

# Create BFMatcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Match descriptors
matches = bf.match(descriptors1, descriptors2)

# Sort them in the order of their distance
matches = sorted(matches, key=lambda x: x.distance)

# Calculate Homography
src_pts = np.float32([keypoints1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
dst_pts = np.float32([keypoints2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

# Warp image1 to get the overlapping area
warp_img = cv2.warpPerspective(image1, M, (image1.shape[1] + image2.shape[1], image1.shape[0]))
warp_img[0:image2.shape[0], 0:image2.shape[1]] = image2

# Create binary masks of the images
mask1 = cv2.warpPerspective(np.ones_like(gray1), M, (image1.shape[1] + image2.shape[1], image1.shape[0]))
mask2 = np.zeros((image1.shape[0], image1.shape[1] + image2.shape[1]), dtype=np.uint8)
mask2[0:image2.shape[0], 0:image2.shape[1]] = 1

# Ensure mask1 and mask2 are single-channel
mask1 = mask1[:, :, 0] if mask1.ndim == 3 else mask1
mask2 = mask2[:, :, 0] if mask2.ndim == 3 else mask2

# Find overlap area by comparing the masks
overlap_mask = np.logical_and(mask1 > 0, mask2 > 0)
overlap_area = np.sum(overlap_mask)

# Convert warp_img to grayscale to threshold and find contours
gray_warp_img = cv2.cvtColor(warp_img, cv2.COLOR_BGR2GRAY)
ret, thresh_warp_img = cv2.threshold(gray_warp_img, 1, 255, cv2.THRESH_BINARY)

# Find contours to calculate the area
contours, hierarchy = cv2.findContours(thresh_warp_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Assuming largest contour is the total stitched area
max_area = 0
for cnt in contours:
    area = cv2.contourArea(cnt)
    if area > max_area:
        max_area = area

# Calculate the percentage of overlap
overlap_percentage = (overlap_area / max_area) * 100

# Reformat the overlap percentage and areas with two decimal places and append the respective units
overlap_percentage_formatted = "{:.2f}%".format(overlap_percentage)
max_area_formatted = "{:.2f} pixels".format(max_area)
overlap_area_formatted = "{:.2f} pixels".format(overlap_area)

print('OVERLAP : ', overlap_percentage_formatted)
print('MAX AREA : ', max_area_formatted)
print('OVERLAP AREA : ', overlap_area_formatted)
