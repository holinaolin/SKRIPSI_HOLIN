import cv2
import numpy as np
import matplotlib.pyplot as plt
from random import randrange

# Load images for stitching
image1 = cv2.imread(r'D:\SKRIPSI related\PY\IMG STITCHING\DATA TEST STITCHING\P1.png')
image2 = cv2.imread(r'D:\SKRIPSI related\PY\IMG STITCHING\DATA TEST STITCHING\P2.png')

# Tampilkan gambar asli secara bersebelahan
fig, ax = plt.subplots(1, 2, figsize=(14, 10))
ax[0].imshow(image1)
ax[0].set_title("PIC 1 - kiri (asli)")
ax[0].axis("off")
ax[1].imshow(image2)
ax[1].set_title("PIC 2 - kanan (asli)")
ax[1].axis("off")
plt.show()

# Function to calculate the mean and std deviation of the image in the LAB color space
def get_color_stats(image):
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    mean_l, std_l = cv2.meanStdDev(l)
    mean_a, std_a = cv2.meanStdDev(a)
    mean_b, std_b = cv2.meanStdDev(b)
    return (mean_l[0][0], mean_a[0][0], mean_b[0][0]), (std_l[0][0], std_a[0][0], std_b[0][0])

# Function to apply the white balance from source image stats to the target image
def apply_white_balance(target_image, src_means, src_stds):
    target_lab = cv2.cvtColor(target_image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(target_lab)
    target_means, target_stds = get_color_stats(target_image)

    l = ((l - target_means[0]) * (src_stds[0] / (target_stds[0] + 1e-6))) + src_means[0]
    a = ((a - target_means[1]) * (src_stds[1] / (target_stds[1] + 1e-6))) + src_means[1]
    b = ((b - target_means[2]) * (src_stds[2] / (target_stds[2] + 1e-6))) + src_means[2]

    l = np.clip(l, 0, 255).astype('uint8')
    a = np.clip(a, 0, 255).astype('uint8')
    b = np.clip(b, 0, 255).astype('uint8')

    balanced_lab = cv2.merge([l, a, b])
    balanced_bgr = cv2.cvtColor(balanced_lab, cv2.COLOR_LAB2BGR)
    return balanced_bgr

# Get color statistics from image B
image_b_means, image_b_stds = get_color_stats(image2)

# Apply the white balance from image B to image A
WB_IMAGE = apply_white_balance(image1, image_b_means, image_b_stds)

# Tampilkan gambar asli secara bersebelahan
fig, ax = plt.subplots(1, 2, figsize=(14, 10))
ax[0].imshow(WB_IMAGE)
ax[0].set_title("1 - kiri")
ax[0].axis("off")
ax[1].imshow(image2)
ax[1].set_title("2 - kanan")
ax[1].axis("off")
plt.show()

# Create a Stitcher object
stitcher = cv2.Stitcher_create()
print("stitcher func created")
# stitcher = cv2.createStitcher()

# Stitch images
image1 = cv2.cvtColor(WB_IMAGE, cv2.COLOR_BGR2RGB)
image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)
print("images converted")

# Tampilkan gambar asli secara bersebelahan
fig, ax = plt.subplots(1, 2, figsize=(14, 10))
ax[0].imshow(image1)
ax[0].set_title("1 GRAY")
ax[0].axis("off")
ax[1].imshow(image2)
ax[1].set_title("2 GRAY")
ax[1].axis("off")
plt.show()

print("stitching in progress")
status, stitched_image = stitcher.stitch((image1, image2))
if status == cv2.Stitcher_OK:
    # Convert stitched image to RGB format
    print("stitching done")
    size = stitched_image.shape
    stitched_image = cv2.resize(stitched_image, (size[1]//2, size[0]//2))
    stitched_image = cv2.cvtColor(stitched_image, cv2.COLOR_BGR2RGB)
    
    # Display stitched image with original color
    plt.figure(figsize=(12, 10))
    plt.imshow(stitched_image)
    plt.title('Stitched Image (Back to Ori Color)')
    plt.axis('off')
    plt.show()

else:
    print('Image stitching failed!')

def find_largest_rectangle(contours):
    """Find the largest rectangular contour in the list."""
    largest_area = 0
    largest_rectangle = None
    for contour in contours:
        # Approximate the contour to a polygon and check if it has four sides
        epsilon = 0.05 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) == 4:
            # Check if this is the largest rectangle so far
            area = cv2.contourArea(contour)
            if area > largest_area:
                largest_area = area
                largest_rectangle = approx
    return largest_rectangle

# Convert the image to grayscale and threshold to find the black border
gray = cv2.cvtColor(stitched_image, cv2.COLOR_BGR2GRAY)
_, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

# Find contours in the thresholded image
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Find the largest rectangular contour which should be the content area
largest_rectangle = find_largest_rectangle(contours)

if largest_rectangle is not None:
    def order_points(pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=2)

        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        diff = np.diff(pts, axis=2)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]

        return rect

    # Reorder the points for the perspective warp
    ordered_rectangle = order_points(largest_rectangle)

    # Determine the dimensions of the new perspective warped image
    (tl, tr, br, bl) = ordered_rectangle
    
    img_copy = stitched_image.copy()
    cv2.drawContours(img_copy, [largest_rectangle], -1, (0, 255, 0), 2)
    print(tl, tr, br, bl)
    cv2.rectangle(img_copy, (int(tl[0]),int(tl[1])),(int(br[0]),int(br[1])), (255, 0, 0), 2)
    cv2.imshow("Largest Rectangle", img_copy)
    cv2.waitKey(0)

    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    # Define the destination points which will map the rectangle's points to a straight rectangle
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype="float32")

    # Calculate the perspective transform matrix and warp the perspective to grab the screen
    M = cv2.getPerspectiveTransform(ordered_rectangle, dst)
    warp = cv2.warpPerspective(stitched_image, M, (maxWidth, maxHeight))
else:
    # If no rectangle is found, return the original image
    warp = stitched_image

warp = cv2.cvtColor(warp, cv2.COLOR_RGB2BGR)

# Displaying the image inline with matplotlib
plt.imshow(warp)
plt.title('Warped Image (Original Color)')
plt.axis('off')
plt.show()