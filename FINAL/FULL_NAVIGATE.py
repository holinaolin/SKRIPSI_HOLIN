import time
from helperbaru import *
import numpy as np
import os
import matplotlib.pyplot as plt
from CONTROL import PIDHeading, PID, Robot
from math import sqrt, atan2, degrees
import cv2
from CAM_CLASS import Camera
# from threading import Thread

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

    def to_tuple(self):
        return (self.x, self.y)

class Mission:
    def __init__(self, robot) -> None:
        self.velocity = [0,0]
        self.angular_velocity = 0
        self.use_prediction = False
        self.prev_orientation = 0
        self.prev_pose = [0,0]
        self.cur_orientation = 0
        self.cur_pose = [0,0]
        self.last_time = time.time_ns()
        img_height = 720
        img_width = 1280
        print("Starting main function")
        self.cam = Camera()
        # self.cam.check_frame()
        print("initializing camera")
        self.last_image = np.empty((720, 1280, 3), dtype=np.uint8)
        self.img_annotated = np.empty((720, 1280, 3), dtype=np.uint8)
        time.sleep(5)
        self.cam.start()
        time.sleep(1)
        self.robot = robot
        self.run = True
        # self.start_update_pose()
        self.formatted = time.strftime("%m-%d-%H.%M.%S", time.localtime())

        os.makedirs(f"LOG_IMG\\{self.formatted}", exist_ok=True)
        self.robot_pose_over_time = []
        self.robot_orientation_over_time = []
        self.robot_timestamp = []

    def update_pose_orientation(self):
        image = self.cam.get_image()
        self.last_image = image

        if image is None:
            print("Failed to get stitched image")

        self.img_annotated, ids, corners = detect_aruco_markers(image)
        if ids is None:
            print("No aruco markers detected")
            return

        cv2.imshow("img anot", cv2.resize(self.img_annotated,(self.img_annotated.shape[1]//2,self.img_annotated.shape[0]//2)))
        cv2.waitKey(5)

        try:
            robot_index = np.where(ids == 0)[0][0]  # Assuming ID 0 is the robot's marker
            robot_corner = corners[robot_index]
            cur_pose = get_aruco_center(robot_corner)
            orientation = get_aruco_orientation(robot_corner)
            self.velocity[0] = (cur_pose[0]-self.prev_pose[0])/(time.time_ns()-self.last_time)
            self.velocity[1] = (cur_pose[1]-self.prev_pose[1])/(time.time_ns()-self.last_time)
            self.angular_velocity = (orientation-self.prev_orientation)/(time.time_ns()-self.last_time)
            self.last_time = time.time_ns()
            self.prev_pose = cur_pose
            self.prev_orientation = orientation
            self.cur_pose = cur_pose
            self.cur_orientation = orientation
            self.use_prediction = False
        except:
            print("no robot detected")
            print("predicting position")
            # if this is not working simply comment it and replace with the following line:
            # return self.prev_orientation, self.prev_pose

            dt = time.time_ns()-self.last_time
            cur_pred_pose = [int(self.prev_pose[0]+self.velocity[0]*(dt)),int(self.prev_pose[1]+self.velocity[1]*(dt))]
            cur_pred_orientation = self.prev_orientation+self.angular_velocity*dt
            self.cur_pose = cur_pred_pose
            self.cur_orientation = cur_pred_orientation
            self.use_prediction = True

        self.robot_timestamp.append(time.time())
        self.robot_pose_over_time.append(self.cur_pose)
        self.robot_orientation_over_time.append(self.cur_orientation)
        print(f"Current Pose: {self.cur_pose} orientation: {self.cur_orientation}")

    def get_target_orientation_from_position(self, target_posisi):
        # get target orientation based on target position and current position
        self.update_pose_orientation()
        posisi = self.cur_pose
        target_orientation = degrees(
            atan2(target_posisi[1] - posisi[1], target_posisi[0] - posisi[0])
        )
        return target_orientation

    def get_target_distance_from_position(self, target_posisi):
        self.update_pose_orientation()
        posisi = self.cur_pose
        dist = sqrt(
            (target_posisi[0] - posisi[0]) ** 2 + (target_posisi[1] - posisi[1]) ** 2
        )
        return dist

    def adjust_heading(self, target_orientation):
        global pid_heading
        # adjust heading facing to target orientation
        self.update_pose_orientation()
        current_orientation = self.cur_orientation
        while abs(target_orientation - current_orientation) > 5:
            self.update_pose_orientation()
            current_orientation = self.cur_orientation
            correction = pid_heading.update(target_orientation, current_orientation)
            rpm_left = max(-20, min(20, -correction))
            rpm_right = -(max(-20, min(20, correction)))
            self.robot.set_rpm([rpm_left, rpm_right])
            print(
                f"RPM Left: {rpm_left}, RPM Right: {rpm_right}, Current Orientation: {current_orientation}"
            )
            time.sleep(0.05)
        self.robot.set_rpm([0, 0])


    def move_to_target(self, target_posisi):
        global pid_heading, pid

        # move to target position and adjust heading facing to target position

        # get target orientation based on target position and current position
        self.update_pose_orientation()
        img = self.last_image
        current_orientation = self.cur_orientation
        posisi = self.cur_pose

        target_posisi = (int(target_posisi[0]), int(target_posisi[1]))
        target_orientation = self.get_target_orientation_from_position(target_posisi)

        dist = self.get_target_distance_from_position(target_posisi)

        if current_orientation is None:
            return  # Exit if no orientation could be fetched

        time_elapsed = 0
        delta_time = 0
        print(f"Initial Orientation: {current_orientation}")
        print(f"Target Orientation: {target_orientation}")
        while time_elapsed < 30:
        # while True:
            self.update_pose_orientation()
            img = self.last_image
            img = cv2.circle(img, posisi, 10, (0, 0, 255), -1)
            img = cv2.circle(img, target_posisi, 10, (0, 0, 255), -1)
            # cv2.resize(img, (1280, 720))
            # cv2.imshow("img", img)
            # cv2.waitKey(10)

            print(f"Target Orientation: {target_orientation}")
            print(f"Current Orientation: {current_orientation}")

            start_time = time.time()
            correction = pid_heading.update(target_orientation, current_orientation)
            # correction_posisi = pid.update(0, dist)
            rpm_left = max(-20, min(20, -correction))
            rpm_right = -(max(-20, min(20, correction)))
            print(f"correction: {correction}, rpm_left: {rpm_left}, rpm_right: {rpm_right}")

            self.robot.set_rpm([rpm_left, rpm_right])
            try:
                current_orientation = self.cur_orientation
                posisi = self.cur_pose
            except:
                pass
            if current_orientation is None:
                break

            print(
                f"RPM Left: {rpm_left}, RPM Right: {rpm_right}, Current Orientation: {current_orientation}"
            )

            time.sleep(0.05)

            if abs(target_orientation - current_orientation) < 3:
                print("Target orientation Reached")
                break

            end_time = time.time()
            delta_time = end_time - start_time
            time_elapsed += delta_time

        self.robot.set_rpm([0, 0])

        time_elapsed = 0
        delta_time = 0

        posisi = self.cur_pose
        current_orientation = self.cur_orientation
        while time_elapsed < 30:
            self.update_pose_orientation()
            img = self.last_image
            img = cv2.line(img, posisi, target_posisi, (0, 0, 255), 2)
            img = cv2.circle(img, posisi, 10, (0, 0, 255), -1)
            img = cv2.circle(img, target_posisi, 10, (0, 0, 255), -1)
            # cv2.imshow("img", img)
            # cv2.waitKey(5)

            print(f"Initial Position: {posisi}")
            print(f"Target Position: {target_posisi}")


            start_time = time.time()

            dist = self.get_target_distance_from_position( target_posisi)

            target_orientation = self.get_target_orientation_from_position( target_posisi)

            posisi = self.cur_pose
            current_orientation = self.cur_orientation

            if abs(current_orientation) > 45:
                self.adjust_heading(target_orientation) 
                

            correction = pid_heading.update(target_orientation, current_orientation)

            if abs(dist) > 200:
                correction*=2
            correction_posisi = pid.update(0, dist)


            print(f"correction: {correction}, correction_posisi: {correction_posisi}")

            val_left = 5 + correction_posisi + correction
            val_right = 5 + correction_posisi - correction

            if target_orientation - current_orientation > 90:
                val_left = -val_left
                val_right = -val_right

            rpm_left = -max(-20, min(20, val_left))
            rpm_right = max(-20, min(20, val_right))

            print(
                f"correction: {correction}, correction_posisi: {correction_posisi}, rpm_left: {rpm_left}, rpm_right: {rpm_right}"
            )

            self.robot.set_rpm([rpm_left, rpm_right])

            if current_orientation is None:
                break
            try:
                self.cur_pose = posisi
                self.cur_orientation = current_orientation
            except:
                pass

            print(
                f"RPM Left: {rpm_left}, RPM Right: {rpm_right}, Current Orientation: {current_orientation}"
            )

            dist = self.get_target_distance_from_position( target_posisi)
            if abs(dist) < 50:
                print("Target Reached")
                break

            time.sleep(0.1)

            end_time = time.time()
            delta_time = end_time - start_time
            time_elapsed += delta_time
        self.robot.set_rpm([0, 0])

    def save_image_node(self, i):
        pathfile = f"LOG_IMG\\{self.formatted}\\"
        detected = False
        while not detected:
            self.update_pose_orientation()
            img = self.last_image
            img_anot, ids, corners = detect_aruco_markers(img)
            if 0 not in ids:
                continue
            else:
                detected = True
        robot_index = np.where(ids == 0)[0][0]
        robot_corner = corners[robot_index]

        pose = get_aruco_center(robot_corner)
        w,h = img.shape[:2]
        safe_margin = 50
        if(pose[0] < safe_margin):
            text_x = safe_margin
        elif(pose[0] > w-safe_margin):
            text_x = w-safe_margin
        else:
            text_x = pose[0]
        if(pose[1] < safe_margin):
            text_y = safe_margin
        elif(pose[1] > h-safe_margin):
            text_y = h-safe_margin
        else:
            text_y = pose[1]
        pathfiletmp = pathfile + f"node-{i}.jpg"
        stat = cv2.imwrite(pathfiletmp,img)
        if not stat:
            print(f"Failed to save image at {pathfiletmp}")
        else:
            print(f"Image saved at {pathfiletmp}")
        cv2.putText(img_anot, f"Node-{i}{self.path[i]}", (text_x,text_y-50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        cv2.putText(img_anot, f"{pose}", (text_x,text_y-25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        cv2.circle(img_anot, pose, 10, (255, 0, 0), -1)
        cv2.circle(img_anot, self.path[i], 10, (0, 0, 255), -1)
        pathfile += f"node-{i}_anottated.jpg"
        stat = cv2.imwrite(pathfile , img_anot)
        if not stat:
            print(f"Failed to save image at {pathfile}")
        else:
            print(f"Image saved at {pathfile}")

    def main(self):
    # cam.start_visualize()
        stitched_image = self.last_image
        self.update_pose_orientation()
        anot_img, ids, corners = detect_aruco_markers(stitched_image)
        robot_pos = Position()
        goal_pos = Position()

        while ids is None or len(ids) < 4:
            self.update_pose_orientation()
            stitched_image = self.last_image
            anot_img, ids, corners = detect_aruco_markers(stitched_image)
            robot_pos = Position()
            goal_pos = Position()

        if ids is not None and len(ids) > 0:
            robot_index = np.where(ids == 0)[0][0]
            robot_corner = corners[robot_index]
            robot_pos.from_tuple(get_aruco_center(robot_corner))
            goal_index = np.where(ids == 1)[0][0]
            goal_corner = corners[goal_index]
            goal_pos.from_tuple(get_aruco_center(goal_corner))
            obstacles_pos = []
            obstacles_corners = []
            obstacle_index = np.where(ids == 2)[0][0]
            obstacle_corner = corners[obstacle_index]
            obstacles_pos.append(get_aruco_center(obstacle_corner))
            obstacles_corners.append(obstacle_corner)
            obstacle_index = np.where(ids == 3)[0][0]
            obstacle_corner = corners[obstacle_index]
            obstacles_pos.append(get_aruco_center(obstacle_corner))
            obstacles_corners.append(obstacle_corner)

            print(f"Robot Position: {robot_pos}")
            print(f"Goal Position: {goal_pos}")
            print(f"Obstacle Positions: {obstacles_pos}")

        # get image shape
        height, width = stitched_image.shape[:2]
        print(f"w:{width} h:{height}")

        # generate RRT
        rrt = RRT(
            posisi_robot=robot_pos.to_tuple(),
            posisi_tujuan=goal_pos.to_tuple(),
            posisi_objek=obstacles_pos,
            img_height=height,
            img_width=width,
            min_step_length=30,
            max_step_length=200,
            max_iterations=10000,
            obstacle_radius=60,
            avoidance_radius=60,
        )
        rrt.generate_tree(anot_img, "A//"+self.formatted)

        path = rrt.get_path()
        self.path = path
        filepath = f"Path_{self.formatted}.txt"
        with open(filepath, "+w") as f:
            f.write(str(path))
        print(f"Path: {path}")
        rrt.visualize()

        cv2.imwrite(f"LOG_IMG\\{self.formatted}\\anot_img.jpg", anot_img)
        
        print("Starting to move")
        start_time = time.time_ns()
        for i, node in enumerate(path):
            print(f"Node-{i}")
            if i == 0:
                continue  # skip the first node because it is the robot's current position
            self.update_pose_orientation()
            print(f"Moving to {node}")
            self.move_to_target(node)
            print(f"Arrived at {node}")
            self.save_image_node(i)
            if i == (len(path) - 1):

                print("Arrived at destination")
                self.adjust_heading(0)
                break

        end_time = time.time_ns()
        print(f"Total time: {(end_time - start_time) / 1e9} seconds")
        filepath = f"time_{self.formatted}.txt"
        with open(filepath, "+w") as f:
            f.write(f"Total time: {(end_time - start_time) / 1e9} seconds")

        cv2.destroyAllWindows()
        exit()

if __name__ == "__main__":
    robot = Robot("192.168.18.94", 12345, 50)
    print(time.time())

    ##UNTUK MANUAL DENGAN TANGAN
    # from control import RobotDummy
    # robot = RobotDummy()
    # Inisialisasi PID Controllers

    pid_heading = PIDHeading(
        kp=0.3, ki=0.1, kd=0.1, max_output=30, integral_saturation=5
    )
    pid = PID(kp=0.3, ki=0.1, kd=0, max_output=50, integral_saturation=10)
    misi = Mission(robot)
    misi.main()
