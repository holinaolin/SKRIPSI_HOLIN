import cv2
import numpy as np
import time
from threading import Thread
import os
import platform

class Camera:
    def __init__(self,cam1=0,cam2=1, height=720, width=1280):
        if platform.system() == "Linux":
            print("Linux detected")
            self.cap1 = cv2.VideoCapture(0)
            self.cap2 = cv2.VideoCapture(1)
        else:
            self.cap1 = cv2.VideoCapture(cam1, cv2.CAP_DSHOW)
            self.cap2 = cv2.VideoCapture(cam2, cv2.CAP_DSHOW)

        if not self.cap1.isOpened() and not self.cap2.isOpened():
            print("Error opening camera")
        else :
            print("Camera opened")

        self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap1.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap1.set(cv2.CAP_PROP_FPS, 30)
        print("cam NYK-KIRI set")
        self.cap2.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap2.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap2.set(cv2.CAP_PROP_FPS, 30)
        print("cam NYK-KANAN set")

        path = os.getcwd()

        cammat = os.path.join(path, "data-CAL-NYK2/cam-mat-NYK2.npy")
        dist = os.path.join(path, "data-CAL-NYK2/dist-coeffs-NYK2.npy")

        self.camera_matrix = np.load(cammat)
        self.distortion_coefficient = np.load(dist)
        print("camera matrix NYK2 loaded")

        self.stitcher = cv2.Stitcher().create(cv2.Stitcher_PANORAMA)
        self.stitcher_flags = True
        self.stopped = False
        self.frame1 = np.empty((720,1280,3),dtype=np.uint8)
        self.frame2 = np.empty((720,1280,3),dtype=np.uint8)
        self.ret1 = False
        self.ret2 = False
        self.stitched_image = np.empty((720,1280,3),dtype=np.uint8)
        self.wbmeans = np.load(r"means.npy")
        self.wbstds = np.load(r"stds.npy")
        self.wbmeans = np.load(r"D:\SKRIPSI related\PY\gogogo\means.npy")
        self.wbstds = np.load(r"D:\SKRIPSI related\PY\gogogo\stds.npy")
        time.sleep(10)
        # self.check_frame()

    def check_frame(self):
        while not self.ret1 and not self.ret2:
            for _ in range(10):  # Read 10 frames to allow the camera to stabilize
                self.ret1, img1 = self.cap1.read()
                self.ret2, img2 = self.cap2.read()
                # cv2.imshow("img1", img1)
                # cv2.imshow("img2", img2)
                # cv2.waitKey(1)
            if not self.ret1:
                print("Camera 1 not ready")
            if not self.ret2:
                print("Camera 2 not ready")
            if not self.ret1 or not self.ret2:
                time.sleep(1)
            else:
                break
            
    # Function to calculate the mean and std deviation of the image in the LAB color space
    # def get_color_stats(self, image):
    #     lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    #     l, a, b = cv2.split(lab)
    #     mean_l, std_l = cv2.meanStdDev(l)
    #     mean_a, std_a = cv2.meanStdDev(a)
    #     mean_b, std_b = cv2.meanStdDev(b)
    #     return (mean_l[0][0], mean_a[0][0], mean_b[0][0]), (std_l[0][0], std_a[0][0], std_b[0][0])

    # # Function to apply white balance
    # def apply_white_balance(self, target_image, src_means, src_stds):
    #     target_lab = cv2.cvtColor(target_image, cv2.COLOR_BGR2LAB)
    #     l, a, b = cv2.split(target_lab)
    #     target_means, target_stds = self.get_color_stats(target_image)

    #     l = ((l - target_means[0]) * (src_stds[0] / (target_stds[0] + 1e-6))) + src_means[0]
    #     a = ((a - target_means[1]) * (src_stds[1] / (target_stds[1] + 1e-6))) + src_means[1]
    #     b = ((b - target_means[2]) * (src_stds[2] / (target_stds[2] + 1e-6))) + src_means[2]

    #     l = np.clip(l, 0, 255).astype('uint8')
    #     a = np.clip(a, 0, 255).astype('uint8')
    #     b = np.clip(b, 0, 255).astype('uint8')

    #     balanced_lab = cv2.merge([l, a, b])
    #     balanced_bgr = cv2.cvtColor(balanced_lab, cv2.COLOR_LAB2BGR)
    #     return balanced_bgr

    def start(self):
        self.t1 = Thread(target=self.update_cap1)
        self.t1.daemon = True
        self.t1.start()
        print("Thread 1 started")

        self.t2 = Thread(target=self.update_cap2)
        self.t2.daemon = True
        self.t2.start()
        print("Thread 2 started")
        time.sleep(5)
        self.t3 = Thread(target=self.update_stitch)
        self.t3.daemon = True
        self.t3.start()
        print("Thread 3 started")

    def stop(self):
        self.stopped = True
        self.t1.setDaemon(False)
        self.t2.setDaemon(False)
        self.t3.setDaemon(False)
        self.cap1.release()
        self.cap2.release()

    def start_visualize(self):
        # self.t4 = Thread(target=self.update_imshow)
        self.t4.daemon = True
        self.t4.start()
        return self

    # def update_imshow(self):
    #     while True:
    #         img = np.empty((720,1280,3),dtype=np.uint8)
    #         if self.stopped:
    #             break
    #         img1 = cv2.resize(self.frame1, (640, 360))
    #         img2 = cv2.resize(self.frame2, (640, 360))
    #         img[0:360,0:640] = img1
    #         img[0:360,640:1280] = img2
    #         img3 = cv2.resize(self.stitched_image, (1280, 720))
    #         img[360:720,0:640] = img3
    # cv2.imshow("img", img)
    # if cv2.waitKey(1) & 0xFF == 27:
    #     break

    def update_cap1(self):
        while True:
            start = time.time_ns()
            if self.stopped:
                print("stopped")
                break
            self.ret1, frame = self.cap1.read()
            if self.ret1:
                frame = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficient)
                percentage = 0
                # Crop frame1
                frame1_height = frame.shape[0]
                frame1_width = frame.shape[1]
                up = int(frame1_height * percentage/2)
                down = int(frame1_height*(1-percentage/2))
                left = int(frame1_width * percentage/2)
                right = int(frame1_width*(1-percentage/2))
                frame1_cropped = frame[up:down, left:right]
                self.frame1 = frame1_cropped

                # ambil wb
                # means,stds =  self.get_color_stats(self.frame1)
                # print(means, stds)
                # np.save(r"D:\SKRIPSI related\PY\SKRIPSI HOLIN\means_baru.npy", np.array(means))
                # np.save(r"D:\SKRIPSI related\PY\SKRIPSI HOLIN\stds_baru.npy", np.array(stds))

                # # load and apply wb
                # self.frame1 = self.apply_white_balance(self.frame1, self.wbmeans, self.wbstds)
            else:
                print("FAILED to cap NYK-KIRI")

    def update_cap2(self):
        while True:
            start = time.time_ns()
            if self.stopped:
                print("stopped")
                break

            # self.ret2, frame = self.cap2.read()
            # frame = self.apply_white_balance(
            #     frame, self.wbmeans, self.wbstds)

            if self.ret2:
                # self.frame2 = frame
                frame = cv2.undistort(frame, self.camera_matrix, self.distortion_coefficient)
                percentage = 0
                # Crop frame1
                frame2_height = frame.shape[0]
                frame2_width = frame.shape[1]
                up = int(frame2_height * percentage / 2)
                down = int(frame2_height * (1 - percentage / 2))
                left = int(frame2_width * percentage / 2)
                right = int(frame2_width * (1 - percentage / 2))
                frame2_cropped = frame[up:down, left:right]
                self.frame2 = frame2_cropped

            else:
                print("FAILED to cap NYK-KANAN")

    def update_stitch(self):
        while True:
            start = time.time_ns()
            if self.stopped:
                break
            if self.ret1 and self.ret2:
                self.stitched_image = self.get_stitched_image(self.frame1, self.frame2)
                print("stitched img shape: ", self.stitched_image.shape)
                # cv2.imshow("stitched", self.stitched_image)
                # cv2.waitKey(1)
            end = time.time_ns()
            elapsed = (end-start)/1000000
            print("update stitch Elapsed time: ", elapsed, "ms")
        cv2.destroyWindow("stitched")

    def get_image(self):
        return self.stitched_image

    def order_points(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=2)

        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]

        diff = np.diff(pts, axis=2)
        rect[1] = pts[np.argmin(diff)]
        rect[3] = pts[np.argmax(diff)]
        return rect

    def find_largest_rectangle(self, contours, max_area=None):
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
                    if max_area is not None and area > max_area:
                        continue
                    largest_area = area
                    largest_rectangle = approx

        return largest_rectangle

    def get_stitched_image(self, frame1: np.array, frame2: np.array):
        # Create an empty image for display (unused in this function)
        img_show = np.empty((720, 1280, 3), dtype=np.uint8)
        if self.stitcher_flags:
            status, stitched = self.stitcher.stitch((frame1, frame2))
            if status == cv2.Stitcher_OK:
                self.stitcher_flags = False
                # pass

        else:
            status, stitched = self.stitcher.composePanorama((frame1, frame2))

        if status == 0:
            print("Stitching success")
            # Show the stitched image
            # cv2.imshow("stitched", stitched)
            # cv2.waitKey(1)

            threshold_rect = 1
            img = stitched
            max_area = (img.shape[0]) * (img.shape[1])

            # Convert image to grayscale
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Apply binary thresholding
            _, thresh = cv2.threshold(gray, threshold_rect, 255, cv2.THRESH_BINARY)
            # Find contours
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Find the largest rectangle from the contours
            largest_rectangle = self.find_largest_rectangle(contours, max_area=max_area)

            if largest_rectangle is not None:
                # Order the points of the largest rectangle
                ordered_rect = self.order_points(largest_rectangle)
                (tl, tr, br, bl) = ordered_rect
                # Compute the width of the new image
                widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
                widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
                maxWidth = max(int(widthA), int(widthB))

                # Compute the height of the new image
                heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
                heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
                maxHeight = max(int(heightA), int(heightB))

                # Define destination points for perspective transform
                dst = np.array([
                    [0, 0],
                    [maxWidth - 1, 0],
                    [maxWidth - 1, maxHeight - 1],
                    [0, maxHeight - 1]], dtype="float32")

                # Get the perspective transform matrix
                M = cv2.getPerspectiveTransform(ordered_rect, dst)
                # Apply the perspective warp to get the final image
                final_image = cv2.warpPerspective(img, M, (maxWidth, maxHeight))

            else:
                print("NO RECTANGLE FOUND")
                final_image = img

            # Display the final stitched image
            img_show = final_image
            # cv2.imshow("stitched final", cv2.resize(img_show,(img_show.shape[1]//2,img_show.shape[0]//2)))
            # cv2.waitKey(1)

            return final_image

        else:
            print("Stitching failed")
            return self.stitched_image

if __name__ == "__main__":
    cam = Camera()
    time.sleep(5)
    cam.start()
    time.sleep(5)

    while True:
        img = cam.get_image()
        img1 = cam.frame1
        img2 = cam.frame2
        if cv2.waitKey(0) & 0xFF == 27:
            break