import cv2
import time
import os
from copy import deepcopy

ARUCO_DICT = cv2.aruco.DICT_6X6_250
SQUARES_VERTICALLY = 7 # row
SQUARES_HORIZONTALLY = 5 #col
SQUARE_LENGTH = 0.046 # ukuran kotak catur dalam meter
MARKER_LENGTH = 0.027 # ukuran aruco dalam meter

dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard(size=(SQUARES_HORIZONTALLY, SQUARES_VERTICALLY), squareLength=SQUARE_LENGTH, markerLength=MARKER_LENGTH, dictionary=dictionary)
params = cv2.aruco.DetectorParameters()
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
print('params: ', params)

camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)
print("video capture started")
camera.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)
camera.set(cv2.CAP_PROP_FPS, 30)
print("resolution set")

ret, img = camera.read()
print(img.shape)
path_time = time.strftime("%m-%d-%H.%M")
path = r"D:\SKRIPSI related\PY\CAMERA CALIBRATION\NYK"
path = path +"\\"+ path_time + "\\"
os.mkdir(path)
count = 0
start_time = time.time()
time_elapsed = 0
while True:
    name = path+str(count)+".jpg"
    ret, img = camera.read()
    img_copy = deepcopy(img)
    [marker_corners, marker_ids, _] = cv2.aruco.detectMarkers(img, board.getDictionary(), parameters=params)
    print(f"markercorner:{marker_corners}")
    # If at least one marker is detected
    if marker_ids is not None and len(marker_ids) > 0:
        cv2.aruco.drawDetectedMarkers(img_copy, marker_corners, marker_ids)
        retval,charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, img, board)
            # all_charuco_corners.append(charuco_corners)
            # all_charuco_ids.append(charuco_ids)
        if charuco_ids is not None and len(charuco_ids) > 0:
            cv2.aruco.drawDetectedCornersCharuco(img_copy, charuco_corners, charuco_ids, (255, 0, 0))
        time_elapsed = time.time() - start_time
        if retval and time_elapsed > 3:
            cv2.imwrite(name, img)  
            count += 1
            # cv2.imshow("img", img_copy)
            print("captured", count)
            # print("delay 3 seconds")
            start_time = time.time()
            
    cv2.putText(img_copy, f"Captured: {count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(img_copy, f"Time: {time_elapsed:.1f}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow("img", img_copy)
    key = cv2.waitKey(1) & 0xFF

    if  key == ord('c'):
        print("captured", count)
        cv2.imwrite(name, img)
        # cv2.imshow("img", img)
        count += 1
    elif  key == ord('q'):
            break