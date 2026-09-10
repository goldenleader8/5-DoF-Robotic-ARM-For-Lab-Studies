import os
import sys
import math
import time
import numpy as np
import random
from dynamixel_sdk import * 

COM_PORT = 'COM6'
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112

DXL_RAW_MAX = 4095      
DXL_DEG_RANGE = 360.0   

ID_BASE    = 1
ID_BAHU    = 2
ID_SIKU2   = 18   
ID_WRIST   = 3    
ID_POINTER = 15   

MID_ID1  = 180.00    # Titik Home Fisik Base 
MID_ID2  = 100.00    # Titik Home Fisik Bahu
MID_ID3  = 358.00    # Titik Home Fisik Wrist
MID_ID15 = 0.5       # Titik Home Pointer
MID_ID18 = 1.50      # Titik Home Fisik Siku2

MID_MAP = {
    ID_BASE: 0.0,
    ID_BAHU: 0.0,
    ID_SIKU2: 0.0,
    ID_WRIST: 0.0,
    ID_POINTER: 0.0,
}

SERVO_LIMITS = {
    ID_BASE:    (0.09, 359.91),
    ID_BAHU:    (30.00, 200.00),
    ID_SIKU2:   (1.14, 185.00),
    ID_WRIST:   (0.70, 358.59),
    ID_POINTER: (0.09, 143.96),
}

#DH Parameters
d1, theta_off1, a1, alpha1_deg = 0.021, 0.0 - MID_ID1, 0.0, 0.0
d2, theta2_deg, a2, alpha2_deg = 0.006448, 90.0, 0.0075, -90.0
d3, theta_off3, a3, alpha3_deg = 0.0215, -102.529 + MID_ID2, 0.036878, 0.0
d4, theta4_deg, a4, alpha4_deg = -0.0051, 16.3429, 0.120266, 0.0
d5, theta_off5, a5, alpha5_deg = 0.005, -16.3429 - MID_ID18, 0.036878, 0.0
d6, theta6_deg, a6, alpha6_deg = 0.015, 102.529, 0.008, 90.0
d7, theta_off7, a7, alpha7_deg = 0.0735, -102.529 - MID_ID3, 0.036878, 0.0
d8, theta8_deg, a8, alpha8_deg = 0.0465, 102.529, 0.008, 90.0

alpha1 = alpha1_deg * math.pi / 180.0
alpha2 = alpha2_deg * math.pi / 180.0
alpha3 = alpha3_deg * math.pi / 180.0
alpha4 = alpha4_deg * math.pi / 180.0
alpha5 = alpha5_deg * math.pi / 180.0
alpha6 = alpha6_deg * math.pi / 180.0
alpha7 = alpha7_deg * math.pi / 180.0
alpha8 = alpha8_deg * math.pi / 180.0

theta2 = theta2_deg * math.pi / 180.0
theta4 = theta4_deg * math.pi / 180.0
theta6 = theta6_deg * math.pi / 180.0
theta8 = theta8_deg * math.pi / 180.0

def mat4x4():
    return [
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0]
]
    
def dh_matrix(theta, alpha, d, a):
    return [
    [math.cos(theta), (-1) * math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
    [math.sin(theta), math.cos(theta) * math.cos(alpha), (-1) * math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
    [0, math.sin(alpha), math.cos(alpha), d],
    [0, 0, 0, 1]
]

portHandler = PortHandler(COM_PORT)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def kalibrasi_awal():
    home1, home2, home3, home4 = MID_ID1, MID_ID2, MID_ID18, MID_ID3
    
    if not cek_tabrakan_meja(home1, home2, home3, home4):
        print("! Kalibrasi awal DIBATALKAN: posisi HOME akan menabrak meja !")
        return
    
    jawab = input("Gerakkan lengan ke posisi HOME sekarang untuk kalibrasi awal? (y/n): ").strip().lower()
    
    if jawab != 'y':
        print("Kalibrasi awal dilewati. Lengan tetap di posisi terakhir (torque tetap aktif/menahan).")
        return
    
    x, y, z, T_total = forwardkin(home1, home2, home3, home4)
    send_all(home1, home2, home3, home4)

    sudut_POINTER = MID_ID15
    senddata(ID_POINTER, sudut_POINTER)
    
    print("Kalibrasi awal selesai -> lengan di posisi HOME (pointer diatur ke atas).")

def init():
    ids = [ID_BASE, ID_BAHU, ID_SIKU2, ID_WRIST, ID_POINTER]
    for dxl_id in ids:
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_ACCELERATION, 20)
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, 40)
        
        result, error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
        if result != COMM_SUCCESS:
            print(f"[ID {dxl_id}] GAGAL komunikasi: {packetHandler.getTxRxResult(result)}")
        elif error != 0:
            print(f"[ID {dxl_id}] Servo error: {packetHandler.getRxPacketError(error)}")
        else:
            print(f"[ID {dxl_id}] Torque enabled OK")
    
    kalibrasi_awal()

def identity(mat):
    for i in range(4):
      for j in range(4):
        mat[i][j] = 1.0 if i == j else 0.0
    return mat

matA = mat4x4()
matB = mat4x4()
matC = mat4x4()

def multiply(matA, matB, matC):
    for i in range(4):
      for j in range(4):
        matC[i][j] = 0
        for k in range(4):
          matC[i][j] += matA[i][k] * matB[k][j]

#mengalikan semua matriks transformasi berurutan → menghasilkan posisi (x,y,z) end-effector dari 4 sudut input.
def forwardkin(theta1_deg, theta2_deg_in, theta3_deg, theta4_deg):
    tetha1 = (theta_off1 + theta1_deg) * math.pi / 180.0
    tetha3 = (theta_off3 - theta2_deg_in) * math.pi / 180.0 
    tetha5 = (theta_off5 + theta3_deg) * math.pi / 180.0
    tetha7 = (theta_off7 + theta4_deg) * math.pi / 180.0

    matT1 = dh_matrix(tetha1, alpha1, d1, a1)   # revolute
    matT2 = dh_matrix(theta2, alpha2, d2, a2)   # fixed
    matT3 = dh_matrix(tetha3, alpha3, d3, a3)   # revolute
    matT4 = dh_matrix(theta4, alpha4, d4, a4)  # fixed
    matT5 = dh_matrix(tetha5, alpha5, d5, a5)   # revolute
    matT6 = dh_matrix(theta6, alpha6, d6, a6)   # fixed
    matT7 = dh_matrix(tetha7, alpha7, d7, a7)   # revolute
    
    matT_tcp = [
        [0.9761863, -0.2169337, 0.0000000, -0.0204999],
        [0.2169337, 0.9761863, -0.0000000, -0.0045549],
        [-0.0000000, -0.0000000, 1.0000000, 0.1075027],
        [0.0, 0.0, 0.0, 1.0]
    ]

    T_total = mat4x4()
    temp = mat4x4()

    identity(T_total)
    multiply(T_total, matT1, temp)
    multiply(temp, matT2, T_total)
    multiply(T_total, matT3, temp)
    multiply(temp, matT4, T_total)
    multiply(T_total, matT5, temp)
    multiply(temp, matT6, T_total)
    multiply(T_total, matT7, temp)
    multiply(temp, matT_tcp, T_total)

    x = T_total[0][3]
    y = T_total[1][3]
    z = T_total[2][3]

    return x, y, z, T_total

Z_FLOOR = 0.025  # tinggi minimum aman dari meja/lantai (meter), samain dengan data yang dihasilkan dari test collision

def position_all_z(theta1_deg, theta2_deg_in, theta3_deg, theta4_deg): #FK yang mengembalikan ketinggian Z di setiap sendi (bahu, siku2, wrist, tip)
    tetha1 = (theta_off1 + theta1_deg) * math.pi / 180.0
    tetha3 = (theta_off3 - theta2_deg_in) * math.pi / 180.0  
    tetha5 = (theta_off5 + theta3_deg) * math.pi / 180.0
    tetha7 = (theta_off7 + theta4_deg) * math.pi / 180.0

    matT1 = dh_matrix(tetha1, alpha1, d1, a1)
    matT2 = dh_matrix(theta2, alpha2, d2, a2)
    matT3 = dh_matrix(tetha3, alpha3, d3, a3)
    matT4 = dh_matrix(theta4, alpha4, d4, a4)
    matT5 = dh_matrix(tetha5, alpha5, d5, a5)
    matT6 = dh_matrix(theta6, alpha6, d6, a6)
    matT7 = dh_matrix(tetha7, alpha7, d7, a7)
    matT_tcp = [
        [0.9761863, -0.2169337, 0.0000000, -0.0204999],
        [0.2169337, 0.9761863, -0.0000000, -0.0045549],
        [-0.0000000, -0.0000000, 1.0000000, 0.1075027],
        [0.0, 0.0, 0.0, 1.0]
    ]

    T_total = mat4x4()
    temp = mat4x4()
    identity(T_total)

    z_checkpoints = []

    multiply(T_total, matT1, temp)
    multiply(temp, matT2, T_total)

    multiply(T_total, matT3, temp)      # setelah bahu
    z_checkpoints.append(temp[2][3])
    multiply(temp, matT4, T_total)

    multiply(T_total, matT5, temp)      # setelah siku2
    z_checkpoints.append(temp[2][3])
    multiply(temp, matT6, T_total)

    multiply(T_total, matT7, temp)      # setelah wrist
    z_checkpoints.append(temp[2][3])

    multiply(temp, matT_tcp, T_total)   # ujung capit TCP
    z_checkpoints.append(T_total[2][3])

    x = T_total[0][3]
    y = T_total[1][3]
    z_tip = T_total[2][3]

    return x, y, z_tip, z_checkpoints

def cek_tabrakan_meja(theta1_deg, theta2_deg, theta3_deg, theta4_deg): #Return True kalau aman, False kalau berpotensi nabrak
    x, y, z_ee, z_checkpoints = position_all_z(theta1_deg, theta2_deg, theta3_deg, theta4_deg)
    return all(z >= Z_FLOOR for z in z_checkpoints)

def get_jacobian(theta1, theta2, theta3, theta4): #turunan numerik (finite difference)
    delta = 0.01 
    J = [
        [0.0, 0.0, 0.0, 0.0], 
        [0.0, 0.0, 0.0, 0.0], 
        [0.0, 0.0, 0.0, 0.0], 
    ]
    
    thetas_original = [theta1, theta2, theta3, theta4]
    x0, y0, z0, _ = forwardkin(*thetas_original)
    
    for j in range(4):
        thetas_new = thetas_original.copy()
        thetas_new[j] += delta
        x1, y1, z1, _ = forwardkin(*thetas_new)
        J[0][j] = (x1 - x0) / delta
        J[1][j] = (y1 - y0) / delta
        J[2][j] = (z1 - z0) / delta
    return J

def inverse_3x3(M):
    a, b, c = M[0]
    d, e, f = M[1]
    g, h, i = M[2]
    
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    
    if abs(det) < 1e-9:
        det = 1e-9 if det >= 0 else -1e-9
    
    inv = [
        [(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
        [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
        [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]
    ]
    
    return inv

def matmul_3x3_3x1(M, v):
    return [
        M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
        M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
        M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2],
    ]

def transpose_3x4(mat):
    T = [[0.0] * 3 for _ in range(4)]
    for i in range(3):
        for j in range(4):
            T[j][i] = mat[i][j]
    return T

def inversekin(x_target, y_target, z_target, thetas_original): #Damped Least Squares (pseudo-invers Jacobian yang diminimalkan norma-2 kecepatan sendi sambil mengikuti target, dengan tambahan redaman λ dekat singularitas)
    max_iteration = 10000
    
    ids_urut = [ID_BASE, ID_BAHU, ID_SIKU2, ID_WRIST]
    
    home_seed = [
        MID_ID1 - MID_MAP[ID_BASE],
        MID_ID2 - MID_MAP[ID_BAHU],
        MID_ID18 - MID_MAP[ID_SIKU2],
        MID_ID3 - MID_MAP[ID_WRIST]
    ]
    
    seeds_to_try = [
        list(thetas_original),
        home_seed,
        [
            random.uniform(SERVO_LIMITS[ID_BASE][0], SERVO_LIMITS[ID_BASE][1]) - MID_MAP[ID_BASE],
            random.uniform(SERVO_LIMITS[ID_BAHU][0], SERVO_LIMITS[ID_BAHU][1]) - MID_MAP[ID_BAHU],
            random.uniform(SERVO_LIMITS[ID_SIKU2][0], SERVO_LIMITS[ID_SIKU2][1]) - MID_MAP[ID_SIKU2],
            random.uniform(SERVO_LIMITS[ID_WRIST][0], SERVO_LIMITS[ID_WRIST][1]) - MID_MAP[ID_WRIST]
        ]
    ]

    best_thetas = None
    best_error = float('inf')
    
    for seed_idx, current_seed in enumerate(seeds_to_try):
        thetas_list = list(current_seed)
        jaraktotal = float('inf')
        
        for iteration in range(max_iteration):
            x_now, y_now, z_now, _ = forwardkin(*thetas_list)
            err_x = x_target - x_now
            err_y = y_target - y_now
            err_z = z_target - z_now
            
            jaraktotal = math.sqrt((err_x * err_x) + (err_y * err_y) + (err_z * err_z))
            if jaraktotal < 0.001:
                break
            
            J = get_jacobian(*thetas_list)
            J_T = transpose_3x4(J)
            
            lam = 0.05
            JJt = [[sum(J[a][k] * J[b][k] for k in range(4)) for b in range(3)] for a in range(3)]
            for a in range(3):
                JJt[a][a] += lam * lam
            
            JJt_inv = inverse_3x3(JJt)
            temp_vec = matmul_3x3_3x1(JJt_inv, [err_x, err_y, err_z])    
            
            for i in range(4):
                delta_theta = (J_T[i][0] * temp_vec[0]) + (J_T[i][1] * temp_vec[1]) + (J_T[i][2] * temp_vec[2])
                thetas_list[i] += delta_theta
        
        if jaraktotal < 0.01:
            best_thetas = thetas_list
            best_error = jaraktotal
            print(f"IK sukses di Seed {seed_idx+1} dalam {iteration} iterasi. Error sisa: {jaraktotal:.5f} m")
            break
        
        if jaraktotal < best_error:
            best_error = jaraktotal
            best_thetas = list(thetas_list)
        
        if seed_idx < 2:
            print(f"! [IK Fallback] Seed {seed_idx+1} gagal konvergen (Error {jaraktotal:.5f}m). Mencoba rute alternatif...")
    
    if best_error >= 0.01:
        print(f"! IK GAGAL setelah 3 percobaan. Target mungkin di luar jangkauan. Error sisa: {best_error:.5f} m")
        print("  Servo TIDAK akan digerakkan ke solusi ini karena tidak akurat.")
        return best_thetas[0], best_thetas[1], best_thetas[2], best_thetas[3], False
    
    if not cek_tabrakan_meja(*best_thetas):
        print("! IK DITOLAK: solusi ditemukan, tapi akan membuat lengan menabrak meja!")
        print("  Servo TIDAK digerakkan. Coba target lain (misal naikkan nilai Z).")
        return best_thetas[0], best_thetas[1], best_thetas[2], best_thetas[3], False
    
    return best_thetas[0], best_thetas[1], best_thetas[2], best_thetas[3], True

def hitung_pointdegree(T_total):
    zx = T_total[0][2]
    zy = T_total[1][2]
    zz = T_total[2][2]

    horizontal = math.sqrt(zx*zx + zy*zy)
    pitch_dunia_deg = math.degrees(math.atan2(horizontal, zz))

    degree_pointer_theta = MID_ID15 - pitch_dunia_deg   # tanpa +90.0

    point_min_theta = SERVO_LIMITS[ID_POINTER][0] - MID_MAP[ID_POINTER]
    point_max_theta = SERVO_LIMITS[ID_POINTER][1] - MID_MAP[ID_POINTER]
    degree_pointer_theta = max(point_min_theta, min(point_max_theta, degree_pointer_theta))
    return degree_pointer_theta
    
def cekbatas(dxl_id, theta_deg):
    sudut_fisik = MID_MAP[dxl_id] + theta_deg
    min_lim, max_lim = SERVO_LIMITS[dxl_id]
    aman = (min_lim <= sudut_fisik <= max_lim)
    return aman, sudut_fisik, min_lim, max_lim

def inputvalidation(prompt, dxl_id):
    while True:
        nilai_str = input(prompt)
        try:
            theta_deg = float(nilai_str)
        except ValueError:
            print("Input harus berupa angka (boleh desimal, misal 12.5)! Coba lagi.")
            continue
        
        aman, sudut_fisik, min_lim, max_lim = cekbatas(dxl_id, theta_deg)
        if not aman:
            theta_min_valid = min_lim - MID_MAP[dxl_id]
            theta_max_valid = max_lim - MID_MAP[dxl_id]
            print(f"! Di luar jangkauan servo! Sudut fisik hasil ({sudut_fisik:.2f}°) "
                  f"harus antara {min_lim}° - {max_lim}°.")
            print(f"  Artinya theta yang valid: {theta_min_valid:.2f}° sampai {theta_max_valid:.2f}°. Coba lagi.")
            continue
        return theta_deg
    
def senddata(dxl_id, theta_deg):
    truedegree = MID_MAP[dxl_id] + theta_deg
    min_lim, max_lim = SERVO_LIMITS[dxl_id]
    sudut_fisik = max(min_lim, min(max_lim, truedegree))
    raw_position = degreetoraw(sudut_fisik)
    
    result, error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, raw_position)
    if result != COMM_SUCCESS:
        print(f"[ID {dxl_id}] GAGAL kirim goal: {packetHandler.getTxRxResult(result)}")
    elif error != 0:
        print(f"[ID {dxl_id}] Servo error: {packetHandler.getRxPacketError(error)}")
    else:
        print(f"[ID {dxl_id}] Goal terkirim -> raw={raw_position} ({sudut_fisik:.2f}°)")

def send_all(theta1_deg, theta2_deg, theta3_deg, theta4_deg):
    senddata(ID_BASE, theta1_deg)
    senddata(ID_BAHU, theta2_deg)
    senddata(ID_SIKU2, theta3_deg)
    senddata(ID_WRIST, theta4_deg)
    
def rawtodegree(raw_value):
    return raw_value * (DXL_DEG_RANGE / DXL_RAW_MAX)

def degreetoraw(deg_value):
    return round(deg_value * (DXL_RAW_MAX / DXL_DEG_RANGE))

def read_theta(dxl_id):
    raw_pos, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"[ID {dxl_id}] GAGAL baca posisi: {packetHandler.getTxRxResult(dxl_comm_result)}")
        raise RuntimeError(f"Gagal membaca posisi servo ID {dxl_id} (komunikasi error)")
    if dxl_error != 0:
        print(f"[ID {dxl_id}] Servo error saat baca posisi: {packetHandler.getRxPacketError(dxl_error)}")
        raise RuntimeError(f"Servo ID {dxl_id} melaporkan error saat dibaca")

    sudut_fisik = rawtodegree(raw_pos)
    theta = sudut_fisik - MID_MAP[dxl_id]
    return theta
    
def readall():
    theta1_deg = read_theta(ID_BASE)
    theta2_deg = read_theta(ID_BAHU)
    theta3_deg = read_theta(ID_SIKU2)
    theta4_deg = read_theta(ID_WRIST) 
    return theta1_deg, theta2_deg, theta3_deg, theta4_deg

def main():
    if portHandler.openPort():
        print("Berhasil membuka port COM!")
    else:
        print("Gagal membuka port!")
        sys.exit()

    if portHandler.setBaudRate(BAUDRATE):
        print("Berhasil set baudrate!")
    
    init()    
    
    while True:
        print("==[MENU]==")
        print("1. Forward Kinematics")
        print("2. Read Current Servo Positions")
        print("3. Return to Home Position")
        print("4. Inverse Kinematics")
        print("5. Random Movements")
        print("6. Lingkaran")
        print("0. Exit")
        inputmenu = int(input("Pilihan menu: "))
        
        if inputmenu == 1:
            print("===[FORWARD KINEMATICS]===")
            theta1_deg = inputvalidation("Masukkan sudut pertama (base): ", ID_BASE)
            theta2_deg = inputvalidation("Masukkan sudut kedua (bahu): ", ID_BAHU)
            theta3_deg = inputvalidation("Masukkan sudut ketiga (siku2): ", ID_SIKU2)
            theta4_deg = inputvalidation("Masukkan sudut keempat (wrist): ", ID_WRIST)
            
            x, y, z, T_total = forwardkin(theta1_deg, theta2_deg, theta3_deg, theta4_deg)
            print(f"Posisi end-effector -> x: {x:.5f} m, y: {y:.5f} m, z: {z:.5f} m")
            
            if not cek_tabrakan_meja(theta1_deg, theta2_deg, theta3_deg, theta4_deg):
                print("! DITOLAK: Kombinasi sudut ini akan membuat lengan menabrak meja!")
                print("  Servo TIDAK digerakkan. Coba sudut lain.")
                continue
            
            send_all(theta1_deg, theta2_deg, theta3_deg, theta4_deg)
            sudut_POINTER = hitung_pointdegree(T_total)
            print(f"Sudut POINTER (otomatis): {sudut_POINTER:.2f}°")
            senddata(ID_POINTER, sudut_POINTER)
            
        elif inputmenu == 2:
            print("===[SUDUT SAAT INI]===")
            try:
                theta1_deg, theta2_deg, theta3_deg, theta4_deg = readall()
                print(f"Sudut terbaca -> 1:{theta1_deg:.2f} 2:{theta2_deg:.2f} 3:{theta3_deg:.2f} 4:{theta4_deg:.2f}")
            except RuntimeError as e:
                print(f"! Gagal membaca posisi servo: {e}")
                print("  Cek koneksi kabel/power servo, lalu coba lagi.")
        
        elif inputmenu == 3:
            home1, home2, home3, home4 = MID_ID1, MID_ID2, MID_ID18, MID_ID3
            x, y, z, T_total = forwardkin(home1, home2, home3, home4)
            print(f"Posisi end-effector -> x: {x:.5f} m, y: {y:.5f} m, z: {z:.5f} m")

            if not cek_tabrakan_meja(home1, home2, home3, home4):
                print("! DITOLAK: Posisi HOME saat ini akan membuat lengan menabrak meja!")
                print("  Servo TIDAK digerakkan. Cek ulang nilai home1..home4 di atas.")
                continue

            send_all(home1, home2 ,home3, home4)
            
            # Gunakan sudut mutlak MID_ID15 untuk pointer di posisi HOME
            sudut_POINTER = MID_ID15
            print(f"Sudut POINTER (HOME): {sudut_POINTER:.2f}°")
            senddata(ID_POINTER, sudut_POINTER)
            
        elif inputmenu == 4:
            print("===[INVERSE KINEMATICS]===")
            x_target = float(input("Masukkan target X (meter): "))
            y_target = float(input("Masukkan target Y (meter): "))
            z_target = float(input("Masukkan target Z (meter): "))
            
            try:
                theta1_original, theta2_original, theta3_original, theta4_original = readall()
            except RuntimeError as e:
                print(f"! Gagal membaca posisi servo saat ini: {e}")
                print("  IK dibatalkan (butuh posisi awal yang valid sebagai seed). Cek koneksi servo lalu coba lagi.")
                continue
            thetas_original = [theta1_original, theta2_original, theta3_original, theta4_original]
            
            print("Menghitung Inverse Kinematics...")
            th1, th2, th3, th4, ik_sukses = inversekin(x_target, y_target, z_target, thetas_original)
            
            print(f"Hasil Sudut IK -> 1:{th1: .2f}° 2:{th2: .2f}° 3:{th3: .2f}° 4:{th4: .2f}°")

            if ik_sukses:
                send_all(th1, th2, th3, th4)
                x_verif, y_verif, z_verif, T_total_verif = forwardkin(th1, th2, th3, th4)
                sudut_POINTER = hitung_pointdegree(T_total_verif)
                print(f"Sudut POINTER (otomatis): {sudut_POINTER:.2f}°")
                senddata(ID_POINTER, sudut_POINTER)
            else:
                print("Servo tidak digerakkan karena IK gagal konvergen ke target.")
        
        elif inputmenu == 5:
            print("===[Random Movements]===")
            
            movements = [
                (303.3, 153.8, 46.8, 358),
                (117, 132, 12.6, 294.9),
                (50.1, 110.3, 76.4, 358),
                (233.5, 44.7, 1.1, 304.9),
                (240.0, 182.6, 73.7, 298.7),
                (143.3, 36.7, 57, 63.7),
                (202.5, 78.9, 83.9, 286.4),
                (31.6, 142.6, 97.7, 184.1),
                (72.1, 40.6, 1.1, 204.1),
                (303.3, 153.8, 46.8, 358),
                (117, 132, 12.6, 294.9),
                (50.1, 110.3, 76.4, 358),
                (233.5, 44.7, 1.1, 304.9),
                (240.0, 182.6, 73.7, 298.7),
                (143.3, 36.7, 57, 63.7),
                (202.5, 78.9, 83.9, 286.4),
                (31.6, 142.6, 97.7, 184.1),
                (72.1, 40.6, 1.1, 204.1),
                (303.3, 153.8, 46.8, 358),
                (117, 132, 12.6, 294.9),
                (50.1, 110.3, 76.4, 358),
                (233.5, 44.7, 1.1, 304.9),
                (240.0, 182.6, 73.7, 298.7)
            ]
            
            for i, (theta1_deg, theta2_deg, theta3_deg, theta4_deg) in enumerate (movements):
                send_all(theta1_deg, theta2_deg, theta3_deg, theta4_deg)
                
                x, y, z, T_total = forwardkin(theta1_deg, theta2_deg, theta3_deg, theta4_deg)
                sudut_POINTER = hitung_pointdegree(T_total)
                senddata(ID_POINTER, sudut_POINTER)
                time.sleep(2)
            
            home1, home2, home3, home4 = MID_ID1, MID_ID2, MID_ID18, MID_ID3
            send_all(home1, home2 ,home3, home4)
            sudut_POINTER = MID_ID15
            senddata(ID_POINTER, sudut_POINTER)
            
        elif inputmenu == 6:
            print("===[Lingkaran]===")
            
            movements = [
                (158.56, 110.54, 89.44, 358.59),
                (161.16, 103.96, 83.47, 358.59),
                (164.41, 97.94, 77.52, 358.59),
                (168.12, 92.66, 71.92, 358.59),
                (172.11, 88.30, 67.03, 358.59),
                (176.29, 85.05, 63.22, 358.59),
                (180.55, 83.09, 60.86, 358.59),
                (184.82, 82.55, 60.20, 358.59),
                (189.02, 83.31, 61.07, 358.59),
                (193.06, 85.63, 63.86, 358.59),
                (196.86, 89.20, 68.00, 358.59),
                (200.29, 93.83, 73.14, 358.59),
                (203.20, 99.33, 78.88, 358.59),
                (205.38, 105.53, 84.88, 358.59),
                (206.54, 112.22, 90.81, 358.59),
                (206.27, 119.17, 96.36, 358.59),
                (204.04, 126.02, 101.22, 358.59),
                (199.21, 132.22, 105.12, 358.59),
                (191.47, 137.01, 107.82, 358.59),
                (181.46, 139.52, 109.12, 358.59),
                (171.32, 139.13, 108.93, 358.59),
                (163.43, 135.99, 107.31, 358.59),
                (158.63, 130.81, 104.35, 358.59),
                (156.74, 124.37, 100.18, 358.59),
                (156.94, 117.46, 95.11, 358.59),
                (161.16, 103.96, 83.47, 358.59),
                (164.41, 97.94, 77.52, 358.59),
                (168.12, 92.66, 71.92, 358.59),
                (172.11, 88.30, 67.03, 358.59),
                (176.29, 85.05, 63.22, 358.59),
                (180.55, 83.09, 60.86, 358.59),
                (184.82, 82.55, 60.20, 358.59),
                (189.02, 83.31, 61.07, 358.59),
                (193.06, 85.63, 63.86, 358.59),
                (196.86, 89.20, 68.00, 358.59)
            ]
            
            for i, (theta1_deg, theta2_deg, theta3_deg, theta4_deg) in enumerate (movements):
                send_all(theta1_deg, theta2_deg, theta3_deg, theta4_deg)
                
                x, y, z, T_total = forwardkin(theta1_deg, theta2_deg, theta3_deg, theta4_deg)
                sudut_POINTER = hitung_pointdegree(T_total)
                senddata(ID_POINTER, sudut_POINTER)
                time.sleep(0.05)
            
            print("Lingkaran selesai")
    
        elif inputmenu == 0:
            print("Menonaktifkan torque semua servo sebelum keluar...")
            
            for dxl_id in [ID_BASE, ID_BAHU, ID_SIKU2, ID_WRIST, ID_POINTER]:
                packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            portHandler.closePort()
            
            print("Torque nonaktif, port ditutup. Program selesai.")
            break
        
        else:
            print("Pilihan tidak valid!")
        
if __name__ == '__main__':
    main()