import cv2

# Define the points
start_point = (960, 527) 
# object_point1 = (572, 152) #obstacle
# object_point2 = (573, 321) #obstacle
# goal_point = (915, 150)

# Image size in pixels
image_path = 'D:\SKRIPSI related\PY\ORITEST\oritest5.jpg'  # Update this path as per your image location
image = cv2.imread(image_path)
height, width = image.shape[:2]
print(height)
print(width)
image_size_pixels = height, width # (height, width)

# Actual size in cm
image_size_cm = (60, 120) # (height, width)

# Calculate the conversion factors from pixels to cm
conversion_factor_height = image_size_cm[0] / image_size_pixels[0]
conversion_factor_width = image_size_cm[1] / image_size_pixels[1]

# Function to convert points from pixels to cm
def convert_point_to_cm(point, conversion_factor_height, conversion_factor_width):
    return (point[0] * conversion_factor_width, point[1] * conversion_factor_height)

# Convert each point to cm
start_point_cm = convert_point_to_cm(start_point, conversion_factor_height, conversion_factor_width)
# object_point1_cm = convert_point_to_cm(object_point1, conversion_factor_height, conversion_factor_width)
# object_point2_cm = convert_point_to_cm(object_point2, conversion_factor_height, conversion_factor_width)
# goal_point_cm = convert_point_to_cm(goal_point, conversion_factor_height, conversion_factor_width)

print('titik awal : ', start_point_cm)
# print('titik objek 1 : ', object_point1_cm)
# print('titik objek 2 : ', object_point2_cm)
# print('titik tujuan : ', goal_point_cm)

# Manual measurement points in cm
manual_start_point = (60, 35)
# manual_object_point1 = (15, 4)
# manual_object_point2 = (15, 8)
# manual_goal_point = (40, 8)

# Calculated points in cm (rounded to two decimal places for comparison)
calculated_start_point = (round(start_point_cm[0], 2), round(start_point_cm[1], 2))
# calculated_object_point1 = (round(object_point1_cm[0], 2), round(object_point1_cm[1], 2))
# calculated_object_point2 = (round(object_point2_cm[0], 2), round(object_point2_cm[1], 2))
# calculated_goal_point = (round(goal_point_cm[0], 2), round(goal_point_cm[1], 2))

# Function to calculate the error
def calculate_error(manual_point, calculated_point):
    return (abs(manual_point[0] - calculated_point[0]), abs(manual_point[1] - calculated_point[1]))

# Calculate the errors
error_start_point = calculate_error(manual_start_point, calculated_start_point)
# error_object_point1 = calculate_error(manual_object_point1, calculated_object_point1)
# error_object_point2 = calculate_error(manual_object_point2, calculated_object_point2)
# error_goal_point = calculate_error(manual_goal_point, calculated_goal_point)

# print('ERR titik awal : ', error_start_point)
# print('ERR titik objek 1 : ', error_object_point1)
# print('ERR titik objek 2 : ', error_object_point2)
# print('ERR titik tujuan : ', error_goal_point)

# Function to calculate the percentage error
def calculate_percentage_error(manual_point, calculated_point):
    return (abs(manual_point[0] - calculated_point[0]) / manual_point[0] * 100,
            abs(manual_point[1] - calculated_point[1]) / manual_point[1] * 100)

# Calculate the percentage errors for each point
percentage_error_start_point = calculate_percentage_error(manual_start_point, calculated_start_point)
# percentage_error_object_point1 = calculate_percentage_error(manual_object_point1, calculated_object_point1)
# percentage_error_object_point2 = calculate_percentage_error(manual_object_point2, calculated_object_point2)
# percentage_error_goal_point = calculate_percentage_error(manual_goal_point, calculated_goal_point)

# Sum the absolute errors for x and y to calculate cumulative errors
cumulative_error_x = sum([error_start_point[0]])
cumulative_error_y = sum([error_start_point[1]])

# Calculate the cumulative error in percentage
total_manual_x = sum([manual_start_point[0]])
total_manual_y = sum([manual_start_point[1]])
cumulative_percentage_error_x = (cumulative_error_x / total_manual_x) * 100
cumulative_percentage_error_y = (cumulative_error_y / total_manual_y) * 100

print('\nERR titik awal : ', percentage_error_start_point)
# print('ERR titik objek 1 : ', percentage_error_object_point1)
# print('ERR titik objek 2 : ', percentage_error_object_point2)
# print('ERR titik tujuan : ', percentage_error_goal_point)

print('\nERR X : ', cumulative_percentage_error_x, '%')
print('ERR Y : ', cumulative_percentage_error_y, '%')

#HITUNG RERATA ERROR X DAN Y
rerata_error = (cumulative_error_x + cumulative_error_y)/2
print('RERATA ERROR : ', rerata_error)