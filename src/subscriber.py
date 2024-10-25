#!/usr/bin/env python3

import os
import rospy
import cv2
import tf
import random
from math import *
import numpy as np
from sensor_msgs.msg import Range, Image, NavSatFix
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped, PoseWithCovarianceStamped
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

last_save_time = 0

save_interval = 0.05  #tempo de salvamento (segundos), altere aqui se quiser alterar o intervalo de salvamento

# Parâmetros de ruído para latitude, longitude e altitude
noise_std_lat = 0.00005  # Exemplo de desvio padrão para latitude
noise_std_lon = 0.00005  # Exemplo de desvio padrão para longitude
noise_std_alt = 0.05     # Exemplo de desvio padrão para altitude

# Parâmetros de ruído para velocidade linear e angular
noise_std_linear = 0.05  # Exemplo de desvio padrão para velocidade linear
noise_std_angular = 0.05  # Exemplo de desvio padrão para velocidade angular

# Parâmetro de ruído para sonar_height
noise_std_sonar = 0.05  # Exemplo de desvio padrão para sonar_height
## ********************************************** ##

#Variaveis globais
sonar_data = None
image_data = None
velocity_data = None
wind_data = None
gps_data = None
odometry_data = None
initial_pose_data = None

#Funcao que converte e salva imagem
def save_image(simulation_time, image_data):
    if image_data is None:
        rospy.logwarn("Dados de imagem ausentes. Não é possível salvar a imagem.")
        return

    folder_path = "/home/lucas-alves/hector_ws/src/print_info_hector/pasta_imagens" #caminho para a pasta de imagens .png
    p = "0" * 12
    decimal_formatado = f"{simulation_time:.3f}"
    tempo_string = str(decimal_formatado)
    tempo_formatado = p[:(len(p) - len(tempo_string))] + tempo_string
    filename = f"image_{tempo_formatado}.png"
    full_path = os.path.join(folder_path, filename)
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_data, desired_encoding="rgb8")
    cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    cv2.imwrite(full_path, cv_image_rgb)
    rospy.loginfo(f"Imagem salva em: {full_path}")

#funcoes que salvam os parametros
def gps_callback(data):
    global gps_data
    gps_data = data

def image_callback(data):
    global image_data
    image_data = data

def sonar_height_callback(data):
    global sonar_data
    sonar_data = data

def velocity_callback(data):
    global velocity_data
    velocity_data = data

def wind_callback(data):
    global wind_data
    wind_data = data

def odometry_callback(data):
    global odometry_data
    odometry_data = data

def initial_pose_callback(data):
    global initial_pose_data
    initial_pose_data = data

#funcao para criar o intervalo e iniciar as funcoes de salvamento
def clock_callback(msg):
    global last_save_time, sonar_data, velocity_data, wind_data, image_data, gps_data, odometry_data, initial_pose_data
    simulation_time = rospy.get_time() 
    
    if simulation_time - last_save_time >= save_interval:
        last_save_time = simulation_time
        save_topic_info(simulation_time, sonar_data, velocity_data, wind_data, gps_data, odometry_data, initial_pose_data) 
        save_image(simulation_time, image_data)

#funcao que salva os parametros no arquivo
def save_topic_info(simulation_time, sonar_data, velocity_data, wind_data, gps_data, odometry_data, initial_pose_data):
    folder_path = "/home/lucas-alves/hector_ws/src/print_info_hector/pasta_informacoes"  #caminho para a pasta dos arquivos .txt
    p = "0" * 12
    decimal_formatado = f"{simulation_time:.3f}"
    tempo_string = str(decimal_formatado)
    tempo_formatado = p[:(len(p) - len(tempo_string))] + tempo_string
    filename = f"simulation_{tempo_formatado}.txt" 
    full_path = os.path.join(folder_path, filename)
    
    with open(full_path, "w") as file_handle:
        if sonar_data:
            file_handle.write(f"Sonar Height: {sonar_data.range}\n")
            file_handle.write(f"Sonar Height with noise: {sonar_data.range + random.gauss(0, noise_std_sonar)}\n")

        if velocity_data:
            file_handle.write(f"\nVelocity - Linear: x={velocity_data.vector.x}, y={velocity_data.vector.y}, z={velocity_data.vector.z}\n")
            # Adicionando ruído gaussiano à velocidade linear
            noisy_linear_x = velocity_data.vector.x + random.gauss(0, noise_std_linear)
            noisy_linear_y = velocity_data.vector.y + random.gauss(0, noise_std_linear)
            noisy_linear_z = velocity_data.vector.z + random.gauss(0, noise_std_linear)
            file_handle.write(f"Noisy Velocity - Linear: x={noisy_linear_x}, y={noisy_linear_y}, z={noisy_linear_z}\n")
            
        if wind_data:
            file_handle.write(f"\nWind - Linear: x={wind_data.x}, y={wind_data.y}, z={wind_data.z}\n")

        if gps_data:
            file_handle.write(f"\nGPS without noise:\n")
            file_handle.write(f"Altitude: {gps_data.altitude} Latitude: {gps_data.latitude}, Longitude: {gps_data.longitude}\n")
            file_handle.write(f"\nGPS with noise:\n")
            #Adicionando ruído gaussiano
            noisy_latitude = gps_data.latitude + random.gauss(0, noise_std_lat)
            noisy_longitude = gps_data.longitude + random.gauss(0, noise_std_lon)
            noisy_altitude = gps_data.altitude + random.gauss(0, noise_std_alt)
            file_handle.write(f"Altitude: {noisy_altitude} Latitude: {noisy_latitude}, Longitude: {noisy_longitude}\n")

        if odometry_data:
            orientation_quaternion = odometry_data.pose.pose.orientation
            orientation_list = [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w]
            roll, pitch, yaw = euler_from_quaternion(orientation_list)
            file_handle.write(f"\nOrientation quartenion: \n")
            file_handle.write(f"Orientation quartenion x: {orientation_quaternion.x}\n")
            file_handle.write(f"Orientation quartenion y: {orientation_quaternion.y}\n")
            file_handle.write(f"Orientation quartenion z: {orientation_quaternion.z}\n")
            file_handle.write(f"Orientation quartenion w: {orientation_quaternion.w}\n")
            file_handle.write(f"\nRoll: {roll}\n")
            file_handle.write(f"Pitch: {pitch}\n")
            file_handle.write(f"Yaw: {yaw}\n")

            file_handle.write(f"\nOdometry pose: \n")
            file_handle.write(f"X: {odometry_data.pose.pose.position.x}\n")
            file_handle.write(f"Y: {odometry_data.pose.pose.position.y}\n")
            file_handle.write(f"Z: {odometry_data.pose.pose.position.z}\n")

        if initial_pose_data:
            file_handle.write(f"\n Initial pose: \n")
            file_handle.write(f"Positioin: \n")
            file_handle.write(f"x: {initial_pose_data.pose.pose.position.x}")
            file_handle.write(f"y: {initial_pose_data.pose.pose.position.y}")
            file_handle.write(f"z: {initial_pose_data.pose.pose.position.z}")
            

#subscribers
def hector_listener():
    rospy.init_node('hector_listener', anonymous=True)
    rospy.Subscriber("/clock", Float64, clock_callback)
    rospy.Subscriber("/sonar_height", Range, sonar_height_callback)
    rospy.Subscriber("/fix_velocity", Vector3Stamped, velocity_callback)
    rospy.Subscriber("/wind", Vector3, wind_callback)
    rospy.Subscriber("/fix", NavSatFix, gps_callback)
    rospy.Subscriber("/ground_truth/state", Odometry, odometry_callback)
    rospy.Subscriber("/front_cam/camera/image", Image, image_callback)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initial_pose_callback)
    rospy.spin()

#criacao das pastas e chamada do subscriber
if __name__ == '__main__':
    #trecho para verificar se a pasta foi criada
    os.makedirs("/home/lucas-alves/hector_ws/src/print_info_hector/pasta_informacoes", exist_ok=True) #caminho para a pasta de arquivos .txt
    os.makedirs("/home/lucas-alves/hector_ws/src/print_info_hector/pasta_imagens", exist_ok=True)  #caminho para a pasta de arquivos .png
    hector_listener()