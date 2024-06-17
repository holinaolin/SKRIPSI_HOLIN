import cv2
import os
import cv2.aruco as aruco
import numpy as np
from copy import deepcopy
from scipy.spatial import KDTree
import random

class Position:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
    def __add__(self, other):
        return Position(self.x + other.x, self.y + other.y)
    def __str__(self) -> str:
        return f"x: {self.x}, y: {self.y}"
    def __getitem__(self, index):
        return self.x if index == 0 else self.y
    def __len__(self):
        return 2
    def from_tuple(self, pos):
        self.x = pos[0]
        self.y = pos[1]

class RRT:
    def __init__(self, posisi_robot, posisi_tujuan, posisi_objek,
                obstacle_radius=50,
                avoidance_radius=50,
                min_step_length=100,
                max_step_length=200,
                max_iterations=100000,
                img_width=1280,
                img_height=720) -> None:
        self.posisi_robot = tuple(posisi_robot)
        self.posisi_tujuan = tuple(posisi_tujuan)
        self.posisi_obstacle = [tuple(obj) for obj in posisi_objek]
        self.obstacle_radius = obstacle_radius
        self.avoidance_radius = avoidance_radius
        self.min_step_length = min_step_length
        self.max_step_length = max_step_length
        self.max_iterations = max_iterations
        self.margin = 50
        self.width = img_width - self.margin 
        self.height = img_height - self.margin 
        self.nodes = [self.posisi_robot]
        self.node_parents = {self.posisi_robot: None}
        self.edges = []
        self.path = []
        self.path_optimal = []
        self.image = np.empty((self.height, self.width, 3), dtype=np.uint8)

    def closest_node(self, nodes, new_point):
        tree = KDTree(nodes)
        dist, index = tree.query(new_point)
        return nodes[index]

    def get_new_point(self, point1, point2, min_length, max_length):
        direction = np.array(point2) - np.array(point1)
        distance = np.linalg.norm(direction)
        if distance > max_length:
            direction = direction / distance * max_length
        elif distance < min_length:
            direction = direction / distance * min_length
        return tuple(map(int, np.array(point1) + direction))

    def line_circle_intersection(self, node_prev, node_new, circle_center, r):
        a, b = node_prev
        dx, dy = np.array(node_new) - np.array(node_prev)
        h, k = circle_center

        A = dx**2 + dy**2
        B = 2 * (dx * (a - h) + dy * (b - k))
        C = (a - h) ** 2 + (b - k) ** 2 - r**2

        discriminant = B**2 - 4 * A * C

        p1x, p1y = node_prev
        p2x, p2y = node_new
        cx, cy = circle_center
        (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
        dx, dy = (x2 - x1), (y2 - y1)
        dr = np.sqrt(dx**2 + dy**2)
        big_d = x1 * y2 - x2 * y1
        discriminant = r**2 * dr**2 - big_d**2

        if discriminant < 0:
            return False
        
        elif discriminant == 0:
            t = -B / (2 * A)
            # return (a + t * dx, b + t * dy)
            return True
        else:
            t1 = (-B + np.sqrt(discriminant)) / (2 * A)
            t2 = (-B - np.sqrt(discriminant)) / (2 * A)
            return ((a + t1 * dx, b + t1 * dy), (a + t2 * dx, b + t2 * dy))
            return True
        
    def check_collision(self, point, obstacle_points, obstacle_radius, avoidance_radius):
        for obstacle in obstacle_points:
            if cv2.norm(np.array(point) - np.array(obstacle)) < (obstacle_radius + avoidance_radius):
                return True
        return False

    def check_collision_line(self, point1, point2, obstacle_points, obstacle_radius, avoidance_radius=100):
        for obstacle in obstacle_points:
            is_intersect = self.line_circle_intersection(point1, point2, obstacle, obstacle_radius + avoidance_radius)
            if is_intersect:
                return True
            print("is_intersect", is_intersect)
        return False

    def draw_obstacles(self):
        for obstacle in self.posisi_obstacle:
            cv2.circle(self.image, obstacle, self.obstacle_radius + self.avoidance_radius, (0, 0, 255), 2)  # Red circles

    def generate_tree(self, image, filename=None):
        for i in range(self.max_iterations):
            rnd_point = (random.randint(self.margin, self.width), random.randint(self.margin, self.height))
            nearest_node = self.closest_node(self.nodes, rnd_point)
            new_point = self.get_new_point(nearest_node, rnd_point, self.min_step_length, self.max_step_length)
            
            if not self.check_collision_line(nearest_node, new_point, self.posisi_obstacle, self.obstacle_radius, self.avoidance_radius):
                # check if the point position over the goal
                if(cv2.norm(np.array(new_point) - np.array(self.posisi_robot)) > cv2.norm(np.array(self.posisi_robot) - np.array(self.posisi_tujuan))):
                    continue

                self.nodes.append(new_point)
                self.node_parents[new_point] = nearest_node
                cv2.line(image, nearest_node, new_point, (255, 200, 255), 2)
                # cv2.imshow('RRT Nodes progress', cv2.resize(image, (1280, 720)))
                
                # cv2.waitKey(0)
                if cv2.norm(np.array(new_point) - np.array(self.posisi_tujuan)) <= self.max_step_length:
                    if not self.check_collision_line(new_point, self.posisi_tujuan, self.posisi_obstacle, self.obstacle_radius, self.avoidance_radius):
                        self.nodes.append(self.posisi_tujuan)
                        self.node_parents[self.posisi_tujuan] = new_point
                        cv2.line(image, new_point, self.posisi_tujuan, (255, 200, 255), 2)
                        break
        self.image = image
        #panggil fungsi untuk gambar circle obstacle
        self.draw_obstacles()
        cv2.imwrite(f'D:\SKRIPSI related\PY\\RRT Testing\{filename}-progress.jpg', self.image)
        return self.image

    def get_path(self):
        path = [self.posisi_tujuan]
        while path[-1] != self.posisi_robot:
            parent = self.node_parents[path[-1]]
            path.append(parent)
        path = path[::-1]
        self.path = path
        return self.path

    def visualize(self, image = None):
        if image is None:
            image = self.image
        path = self.get_path()
        print("path", path)
        dark_pink_color = (147, 20, 255)
        for i in range(len(path) - 1):
            cv2.line(image, path[i], path[i+1], dark_pink_color, 5)

        circle_color = (255, 0, 0)
        #Tampilkan gambar hasilnya tapi
        #  meresize dulu ke setengah ukuran.
        circle_radius = 7
        for point in path:
            cv2.circle(image, point, circle_radius, circle_color, -1)
        image = cv2.resize(image, (int(image.shape[1]/2), int(image.shape[0]/2)))
        #Tampilkan gambar hasilnya
        cv2.imshow('Optimal RRT Path Result', image)
        cv2.waitKey(0)


def get_stitched_rect_tmp(images=None):
        """
        get_stitched_rect_tmp() : crop stitched image to square (function outside camera class)

        """
        threshold_rect = 220
        img = images
        max_area = (img.shape[0]-40) *( img.shape[1]-40)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, threshold_rect, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_rectangle = find_largest_rectangle(contours, max_area=max_area)

        if largest_rectangle is not None:
            ordered_rect = order_points(largest_rectangle)
            (tl, tr, br, bl) = ordered_rect
            widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
            widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
            maxWidth = max(int(widthA), int(widthB))

            heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
            heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
            maxHeight = max(int(heightA), int(heightB))

            dst = np.array([
                [0, 0],
                [maxWidth - 1, 0],
                [maxWidth - 1, maxHeight - 1],
                [0, maxHeight - 1]], dtype="float32")
            
            M = cv2.getPerspectiveTransform(ordered_rect, dst)
            final_image = cv2.warpPerspective(img, M, (maxWidth, maxHeight))
            return final_image
        else:
            print("NO RECTANGLE FOUND")
            final_image = img
            return final_image

def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=2)

    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    diff = np.diff(pts, axis=2)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    return rect


def find_largest_rectangle(contours, max_area=None):
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

def detect_aruco_markers(image, aruco_dict = aruco.DICT_6X6_250):
    """ 
    desc: deteksi single/multiple aruco 
    params: image
    return: annotated image, ids, corners 
    ex: annotated, ids, corners = detect_aruco_markers(img)
    """
    image_copy = deepcopy(image)
    # Convert to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Initialize the detector ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco_dict)
    parameters = aruco.DetectorParameters()
    # Detect markers in the image
    corners, ids, rejected_points = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)

    # Check if any markers were detected
    if ids is not None:
        # Flatten the IDs list
        ids = ids.flatten()
        
        # Initialize dictionary to store marker info
        marker_info = {}

        # Go through each detected marker
        for i, corner in zip(ids, corners):
            # Calculate the center point of the ArUco marker
            center = corner[0].mean(axis=0)
            # Store the position and orientation of each marker in the dictionary
            marker_info[i] = {
                'center': center,
                'orientation': np.rad2deg(np.arctan2(corner[0][0][1] - center[1], corner[0][0][0] - center[0]))
            }

        # Sort markers by their x-coordinate
        sorted_ids = sorted(marker_info.keys(), key=lambda k: marker_info[k]['center'][0])

        # Label the extreme markers and the ones in between
        labels = ['AGV'] + ['OBJ']*(len(sorted_ids)-2) + ['TUJUAN']

        # Annotate markers with their IDs, positions, and specific labels
        for i, label in zip(sorted_ids, labels):
            center = marker_info[i]['center']
            ori = marker_info[i]['orientation']
            x, y = int(center[0]), int(center[1])
            ## label_text = tampilkan ID beserta posisi x dan y
            label_text = f"{label} {i} ({x}, {y})"
            #Tampikan gambar hasilnya berupa ID lalu enter ke bawahnya tampilkan X, Y
            cv2.putText(image_copy, label_text, (x, y), cv2.FONT_ITALIC, 1, (250, 50, 250), 3)
            cv2.imshow('Annotated Image', image_copy)

    return image_copy, ids, corners

def pose_estimation(image, camera_matrix, dist_coeffs, aruco_dict = cv2.aruco.DICT_6X6_250)->np.ndarray:
    """
        pose_estimation() : call aruco_detect_markers() first then estimate position
        params: image, camera_matrix, dist_coeffs hasil kalibrasi setelah stitch
        return: img_copy, rvec, tvec, markerPoints
    """

    img_copy, ids, corners = detect_aruco_markers(image, aruco_dict)
    rvecs = []
    tvecs = []
    markersPoints = []
    if len(corners) > 0:
        for i in range(0, len(ids)):
           
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, camera_matrix, dist_coeffs)
            rvecs.append(rvec)
            tvecs.append(tvec)
            markersPoints.append(markerPoints)
            img_copy = cv2.aruco.drawDetectedMarkers(img_copy, corners) 

            img_copy = cv2.drawFrameAxes(img_copy, camera_matrix, dist_coeffs, rvec, tvec, 0.01)  
    else:
        print("no aruco corners")
    return img_copy, rvecs, tvecs, markersPoints

def get_aruco_orientation(corner):
    """
    Calculate the orientation of the ArUco marker relative to the horizontal line
    params: corner
    return: angle in degrees
            
       1_________2
        |       |
        | aruco |
       0|_______|3       
            
    """
    
    vector_1 = corner[0][1] - corner[0][0]
    vector_1_norm = vector_1 / np.linalg.norm(vector_1)
    angle = np.arctan2(vector_1_norm[1], vector_1_norm[0])
    return np.degrees(angle)

def get_2d_orientations(image, ids, corners):
    """Calculate the orientations of each marker relative to the horizontal line
    params: image, ids, corners
    return: image, angles
    """
    image_copy = deepcopy(image)
    angles = []
    if ids is not None:
        for i, corner in zip(ids, corners):
            # Calculate the angle of the marker
            vector_1 = corner[0][1] - corner[0][0]
            cv2.circle(image_copy,(int(corner[0][1][0]),int(corner[0][1][1])), 25, (0,0,255),-1)
            cv2.circle(image_copy,(int(corner[0][0][0]),int(corner[0][0][1])), 25, (0,255,255),-1)
            vector_1_norm = vector_1 / np.linalg.norm(vector_1)
            angle = np.arctan2(vector_1_norm[1], vector_1_norm[0])
            angles.append(np.degrees(angle))
    
    # Draw detected markers and information on the image
        for idx, (id_val, corner) in enumerate(zip(ids, corners)):
            # Calculate the center point of the ArUco marker
            center = corner[0].mean(axis=0)
            center_x, center_y = center[0], center[1]  # Separate the center coordinates
            angle_deg = angles[idx]
            
            text_pos = f"Pos: x={center_x:.2f}, y={center_y:.2f}"
            text_ori = f"Ori: {angle_deg:.2f}"
            text_x = int(center_x)
            text_y = int(center_y)
            cv2.putText(image_copy, text_pos, (text_x - 900, text_y - 450), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (100, 20, 150), 5)
            cv2.putText(image_copy, text_ori, (text_x - 600, text_y - 350), cv2.FONT_HERSHEY_SIMPLEX, 2.5, (100, 20, 150), 5)
            arrow_length = 400
            end_point_x = int(center_x + arrow_length * np.cos(np.radians(angle_deg)))
            end_point_y = int(center_y + arrow_length * np.sin(np.radians(angle_deg)))
            cv2.arrowedLine(image_copy, (text_x, text_y), (end_point_x, end_point_y), (0, 0, 255), 10, tipLength=0.5)
            cv2.line(image_copy, (text_x, text_y), (int(center_x + 700), text_y), (0, 0, 0), 10)    
    return image_copy, angles

def get_aruco_center(corner):
    """
    Calculate the center of the ArUco marker
    params: corner of single aruco marker
    return: center
    """
    # center = [(corner[0][0]+corner[2][0])//2+(corner[0][1]+corner[2][1])//2]
    center = corner[0].mean(axis=0)

    return (int(center[0]), int(center[1]))