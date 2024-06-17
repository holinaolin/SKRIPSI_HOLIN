import cv2
import cv2.aruco as aruco
import numpy as np

def detect_aruco_and_display_info_orientation(image_path):
    # Muat gambar
    image = cv2.imread(image_path)

    # Ubah gambar ke grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Inisialisasi detektor ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()

    # Deteksi marker dalam gambar
    corners, ids, rejected_points = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)

    if ids is not None:
        aruco.drawDetectedMarkers(image, corners, ids)
        for idx, corner in zip(ids, corners):
            # Hitung titik tengah marker
            center = corner[0].mean(axis=0)
            
            # Menghitung orientasi marker
            vector_1 = corner[0][1] - corner[0][0]
            vector_1_norm = vector_1 / np.linalg.norm(vector_1)
            angle = np.arctan2(vector_1_norm[1], vector_1_norm[0])
            angle_deg = np.degrees(angle)

            # Teks yang akan ditampilkan untuk posisi
            text_pos = f"Pos: x={int(center[0]): }, y={int(center[1]): }"
            
            # Teks yang akan ditampilkan untuk orientasi
            text_ori = f"Ori: {angle_deg:.2f}"

            # Hitung posisi untuk teks
            text_x = int(corner[0][:,0].min())
            text_y = int(corner[0][:,1].min())

            # Tampilkan posisi dan orientasi ID marker pada gambar
            cv2.putText(image, text_pos, (text_x + 100, text_y - 200), cv2.FONT_HERSHEY_SIMPLEX, 
                        2, (255, 50, 0), 2, cv2.LINE_AA)
            cv2.putText(image, text_ori, (text_x + 100, text_y - 300), cv2.FONT_HERSHEY_SIMPLEX, 
                        2, (255, 50, 0), 2, cv2.LINE_AA)

    # Resize gambar menjadi 720p (1280x720)
    resized_image = cv2.resize(image, (1280, 720))

    # Tampilkan gambar yang telah diresize
    cv2.imshow('Detected ArUco markers with Orientation', resized_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # Path ke gambar yang ingin Anda proses
    image_path = r'D:\SKRIPSI related\PY\ORITEST\oritest1.jpg' # Ganti dengan path yang sesuai
    detect_aruco_and_display_info_orientation(image_path)

    cammat = np.load(r'D:\SKRIPSI related\PY\cammat.npy')
    distcoeff = np.load(r'D:\SKRIPSI related\PY\distcoef.npy')
    print(cammat)
    print(distcoeff)