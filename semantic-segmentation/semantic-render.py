import os
import sys
import numpy as np
from collada import *
from PIL import  Image, ImageFilter

from OpenGL.GL import *
from glfw.GLFW import *
from glfw import _GLFWwindow as GLFWwindow
import glm
from shader_s import Shader

import platform, ctypes

import math
import re

import numpy as np

import matplotlib.pyplot as plt

Image.MAX_IMAGE_PIXELS = None

def calculate_fov_y(fy,height):
    fov_y = 2 * np.arctan(height / (2 * fy))
    return fov_y

def processar_arquivo_txt(caminho_arquivo):
    dados = {}
    lendo = 0 #0: Não Lendo, 1: Lendo gps gt, 2: lendo gps, 3: lendo orientacao, 4: lendo pose
    lidos = 0

    with open(caminho_arquivo, 'r') as f:
        linhas = f.readlines()

        # Processando os dados
        for linha in linhas:
            if lendo == 0:
                if linha.startswith('Sonar Height:'):
                    dados['Sonar_gt'] = float(linha.split(':')[1].strip())
                if linha.startswith('Sonar Height with'):
                    dados['Sonar'] = float(linha.split(':')[1].strip())
                elif linha.startswith('Velocity - Linear:'):
                    partes_velocidade = linha.split(':')[1].split(',')
                    dados['Velocity Linear'] = {
                        'x': float(partes_velocidade[0].split('=')[1]),
                        'y': float(partes_velocidade[1].split('=')[1]),
                        'z': float(partes_velocidade[2].split('=')[1])
                    }
                elif linha.startswith('GPS without noise:'):
                    lendo = 1
                elif linha.startswith('Orientation quartenion:'):
                    lendo = 3
                    lidos = 0
                    dados['Orientation Quaternion'] = {
                        'X': 0.0,
                        'Y': 0.0,
                        'Z': 0.0,
                        'W': 0.0
                    }
                elif linha.startswith('Roll:'):
                    dados['Roll'] = float(linha.split(':')[1].strip())
                elif linha.startswith('Pitch:'):
                    dados['Pitch'] = float(linha.split(':')[1].strip())
                elif linha.startswith('Yaw:'):
                    dados['Yaw'] = float(linha.split(':')[1].strip())
                elif linha.startswith('Odometry pose:'):
                    dados['Odometry Pose'] = {
                        'X': 0.0,
                        'Y': 0.0,
                        'Z': 0.0
                    }
                    lendo = 4
                    lidos = 0
            elif lendo == 1:
                partes_gps = linha.split(',')
                #dados['GPS'] = {
                #        'Altitude': float(partes_gps[0].split()[1]),
                #        'Latitude': float(partes_gps[1].split()[1]),
                #        'Longitude': float(partes_gps[2].split()[1])
                #    }
                lendo = 0
            elif lendo == 2:
                partes_gps = linha.split(',')
                #dados['GPS_gt'] = {
                #        'Altitude': float(partes_gps[0].split()[1]),
                #        'Latitude': float(partes_gps[1].split()[1]),
                #        'Longitude': float(partes_gps[2].split()[1])
                #    }
                lendo = 0
            elif lendo == 3:
                if lidos == 0:
                    dados['Orientation Quaternion']['X'] = float(linha.split(':')[1])
                elif lidos == 1:
                    dados['Orientation Quaternion']['Y'] = float(linha.split(':')[1])
                elif lidos == 2:
                    dados['Orientation Quaternion']['Z'] = float(linha.split(':')[1])
                elif lidos == 3:
                    dados['Orientation Quaternion']['W'] = float(linha.split(':')[1]) 
                    lendo = 0
                lidos += 1  
            elif lendo == 4:
                if lidos == 0:
                    dados['Odometry Pose']['X'] = float(linha.split(':')[1])
                elif lidos == 1:
                    dados['Odometry Pose']['Y'] = float(linha.split(':')[1])
                elif lidos == 2:
                    dados['Odometry Pose']['Z'] = float(linha.split(':')[1])
                    lendo = 0
                lidos += 1 

    return dados

def ler_arquivos_txt_em_pasta(pasta):
    dados_arquivos = {}
    # Verifica se o caminho da pasta existe
    if os.path.exists(pasta):
        # Itera sobre todos os arquivos na pasta
        for arquivo in os.listdir(pasta):
            # Verifica se é um arquivo .txt
            if arquivo.endswith(".txt"):
                # Constrói o caminho completo do arquivo
                caminho_completo = os.path.join(pasta, arquivo)
                # Processa o arquivo e armazena os dados em um dicionário
                dados_arquivos[arquivo] = processar_arquivo_txt(caminho_completo)
    else:
        print(f"A pasta '{pasta}' não existe.")

    return dados_arquivos

def processar_arquivo_camera(caminho_arquivo):
    dados = {}
    with open(caminho_arquivo, 'r') as f:
        linhas = f.readlines()

        for linha in linhas:
            if ':' in linha:
                chave, valor = linha.split(':', 1)
                #print (chave, valor)
                if 'height' == chave.strip():
                    dados[chave.strip()] = int(valor.strip())
                elif 'width' == chave.strip():
                    dados[chave.strip()] = int(valor.strip())
                elif 'D' == chave.strip():
                    dados[chave.strip()] = [float(i) for i in valor.split('[')[1].split(']')[0].split(',')]
                elif 'P' == chave.strip():
                    dados[chave.strip()] = [float(i) for i in valor.split('[')[1].split(']')[0].split(',')]
                elif 'K' == chave.strip():
                    dados[chave.strip()] = [float(i) for i in valor.split('[')[1].split(']')[0].split(',')]
                elif 'R' == chave.strip():
                    dados[chave.strip()] = [float(i) for i in valor.split('[')[1].split(']')[0].split(',')]
                elif 'distortion_model' == chave.strip():
                    dados[chave.strip()] = valor.strip()[1:-1]                               
                elif 'roi' == chave.strip():
                    return dados


    return dados

def processar_arquivo_geometria(caminho_arquivo):
    dados = {}
    # Processar o arquivo de geometria (se necessário)
    return dados

def processar_arquivo_textura(caminho_arquivo):
    dados = {}
    # Processar o arquivo de textura (se necessário)
    return dados

def processar_arquivo_config_robo(caminho_arquivo):
    dados = {}
    with open(caminho_arquivo, 'r') as f:
        linhas = f.readlines()
        for linha in linhas:
            if ':' in linha:
                chave, valor = linha.split(':', 1)
                dados[chave.strip()] = float(valor.strip())

    return dados

def processar_arquivo_translacao_mundo(caminho_arquivo):
    dados = {}
    with open(caminho_arquivo, 'r') as f:
        linhas = f.readlines()
        for linha in linhas:
            if ':' in linha:
                chave, valor = linha.split(':', 1)
                dados[chave.strip()] = float(valor.strip())
    
    return dados


# process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
# ---------------------------------------------------------------------------------------------------------
def processInput(window: GLFWwindow) -> None:
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS):
        glfwSetWindowShouldClose(window, True)

# glfw: whenever the window size changed (by OS or user resize) this callback function executes
# ---------------------------------------------------------------------------------------------
def framebuffer_size_callback(window: GLFWwindow, width: int, height: int) -> None:
    # make sure the viewport matches the new window dimensions; note that width and 
    # height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height)


if __name__ == "__main__":
    if len(sys.argv) != 8:
        print("Uso: python script.py <caminho_da_pasta> <arquivo_camera> <arquivo_geometria> <arquivo_textura> <arquivo_config_robo> <arquivo_translacao_mundo> <caminho_da_pasta_saida>")
        sys.exit(1)

    pasta = sys.argv[1]
    arquivo_camera = sys.argv[2]
    arquivo_geometria = sys.argv[3]
    arquivo_textura = sys.argv[4]
    arquivo_config_robo = sys.argv[5]
    arquivo_config_translacao_mundo = sys.argv[6]
    out_path = sys.argv[7]

    dados_arquivos = ler_arquivos_txt_em_pasta(pasta)
    dados_camera = processar_arquivo_camera(arquivo_camera)
    #print (dados_camera)
    dados_robo = processar_arquivo_config_robo(arquivo_config_robo)
    dados_translacao_mundo = processar_arquivo_translacao_mundo(arquivo_config_translacao_mundo)
    # Importar objeto COLLADA
    cena = Collada(arquivo_geometria)

    # Recuperar dados da malha
    
    for geometry in cena.geometries:
        for primitive in geometry.primitives:
            vertices = np.array(primitive.vertex)            
            texcoords = np.reshape(np.array(primitive.texcoordset),(-1,2))            
            normals = np.array(primitive.normal)            
            faces = np.array(primitive.vertex_index) 

    mapa = []
       
    for i in range(vertices.shape[0]):
        mapa.append(vertices[i,0])
        mapa.append(vertices[i,1])
        mapa.append(vertices[i,2])
        mapa.append(normals[i,0])
        mapa.append(normals[i,1])
        mapa.append(normals[i,2])
        mapa.append(texcoords[i,0])
        mapa.append(texcoords[i,1])

    SCR_WIDTH = dados_camera['width']
    SCR_HEIGHT = dados_camera['height']

    CAMERA_MIN = 0.1
    CAMERA_MAX = 1000.0
    
    # glfw: initialize and configure
    # ------------------------------
    # glfw: initialize and configure
    # ------------------------------
    glfwInit()
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3)
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE)

    if (platform.system() == "Darwin"): # APPLE
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE)

    # glfw window creation
    # --------------------
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", None, None)
    if (window == None):

        print("Failed to create GLFW window")
        glfwTerminate()
        exit()

    glfwMakeContextCurrent(window)
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback)

    glEnable(GL_DEPTH_TEST)

    # set up vertex data (and buffer(s)) and configure vertex attributes
    # ------------------------------------------------------------------
   #my_geometry = np.array([0.5,  0.5, 0.0,   1.0, 0.0, 0.0,   1.0, 1.0, # top right
   #                         0.5, -0.5, 0.0,   0.0, 1.0, 0.0,   1.0, 0.0, # bottom right
   #                        -0.5, -0.5, 0.0,   0.0, 0.0, 1.0,   0.0, 0.0, # bottom left
   #                        -0.5,  0.5, 0.0,   1.0, 1.0, 0.0,   0.0, 1.0], dtype=np.float32)  # top left
   # 

  

    my_geometry = np.array(mapa, dtype=np.float32)

    #indices = np.array([ 0, 1, 3,1, 2, 3],dtype=np.uint32)
    #    0, 1, 3, # first triangle
    #    1, 2, 3  # second triangle
    #)
    indices = np.array(faces.flatten(),dtype=np.uint32)
    

    VAO = glGenVertexArrays(1)
    VBO = glGenBuffers(1)
    EBO = glGenBuffers(1)

    glBindVertexArray(VAO)

    glBindBuffer(GL_ARRAY_BUFFER, VBO)
    glBufferData(GL_ARRAY_BUFFER, my_geometry.nbytes, my_geometry.data, GL_STATIC_DRAW)

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO)
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices.data, GL_STATIC_DRAW)

    # position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * glm.sizeof(glm.float32), None)
    glEnableVertexAttribArray(0)
    # color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * glm.sizeof(glm.float32), ctypes.c_void_p(3 * glm.sizeof(glm.float32)))
    glEnableVertexAttribArray(1)
    # texture coord attribute
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * glm.sizeof(glm.float32), ctypes.c_void_p(6 * glm.sizeof(glm.float32)))
    glEnableVertexAttribArray(2)


    # load and create a texture 
    # -------------------------
    texture = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, texture) # all upcoming GL_TEXTURE_2D operations now have effect on this texture object
    # set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)	# set texture wrapping to GL_REPEAT (default wrapping method)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
    # set texture filtering parameters
    # glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
    # glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    
    # load image, create texture and generate mipmaps
    try:
        img = Image.open(arquivo_textura)
        #img = np.flipud(img)
        #import pdb
        #pdb.set_trace()
        out = img.transpose(Image.FLIP_TOP_BOTTOM)
    
        #out.save('/home/lucas-alves/transpose-output.png')
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, out.width, out.height, 0, GL_RGB, GL_UNSIGNED_BYTE, out.tobytes())
        glGenerateMipmap(GL_TEXTURE_2D)

        img.close()

    except:
        print("Failed to load texture")


    #Gerando a matriz da câmera
    projection = glm.perspective(calculate_fov_y(dados_camera['K'][4],dados_camera['height']), dados_camera['width']/dados_camera['height'], CAMERA_MIN, CAMERA_MAX)
 
    #print (projection)
    if dados_camera['distortion_model'] == 'plumb_bob':
        # Distortion correction
        K_zero = True
        for d in dados_camera['D']:
            if d != 0.0:
                K_zero = False
            if not K_zero:
                print ("Modelo de distorção não implementado")
                exit(-1)

    else:
        print ("Modelo de distorção não conhecido")
        exit(-1)
        
      
    # render loop with texture
    #-------------------------------------------------------------------------------------------------------------
    # build and compile our shader zprogram
    # ------------------------------------
    ourShader = Shader("texture.vs", "texture.fs") 
    
    
    # -----------
    for key in sorted(dados_arquivos.keys()):
        if glfwWindowShouldClose(window):
            break

        # input
        # -----
        processInput(window)

        #print (key)

        # render
        # ------
        glClearColor(1.0, 1.0, 1.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        # bind Texture
        glBindTexture(GL_TEXTURE_2D, texture)

        # create transformations       
        # get matrix's uniform location and set matrix
        ourShader.use()
        projectionLoc = glGetUniformLocation(ourShader.ID, "projection")
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm.value_ptr(projection))

        # Rotação do robô
        # Crie matrizes de rotação individuais para cada eixo
        rotation_x = glm.rotate(glm.mat4(1.0), dados_arquivos[key]['Roll'], glm.vec3(1, 0, 0))
        rotation_y = glm.rotate(glm.mat4(1.0), dados_arquivos[key]['Pitch'], glm.vec3(0, 1, 0))
        rotation_z = glm.rotate(glm.mat4(1.0), dados_arquivos[key]['Yaw'], glm.vec3(0, 0, 1))

        robo_rotation_matrix = rotation_z * rotation_y * rotation_x

        posi_robo = glm.vec3(dados_arquivos[key]['Odometry Pose']['X'],dados_arquivos[key]['Odometry Pose']['Y'], dados_arquivos[key]['Odometry Pose']['Z'])
        camera_posi = robo_rotation_matrix*glm.vec4(dados_robo['x'],dados_robo['y'],dados_robo['z'],1.0)
        camera_posi = glm.vec3(camera_posi.x/camera_posi.w,camera_posi.y/camera_posi.w,camera_posi.z/camera_posi.w) + posi_robo


        # Rotação do câmera
        # Crie matrizes de rotação individuais para cada eixo
        rotation_x = glm.rotate(glm.mat4(1.0), dados_robo['r'], glm.vec3(1, 0, 0))
        rotation_y = glm.rotate(glm.mat4(1.0), dados_robo['p'], glm.vec3(0, 1, 0))
        rotation_z = glm.rotate(glm.mat4(1.0), dados_robo['y'], glm.vec3(0, 0, 1))
       
        camera_rotation_matrix = rotation_z * rotation_y * rotation_x

        robo_direction = glm.vec4(1.0, 0.0, 0.0,1.0)  #ros orientation
        camera_direction = robo_rotation_matrix*camera_rotation_matrix*robo_direction
        camera_direction = glm.vec3(camera_direction.x/camera_direction.w,camera_direction.y/camera_direction.w,camera_direction.z/camera_direction.w)

        up = robo_rotation_matrix*camera_rotation_matrix*glm.vec4(0.0, 0.0, 1.0,1.0)
        up = glm.vec3(up.x/up.w,up.y/up.w,up.z/up.w)
       

        view = glm.lookAt(camera_posi, camera_posi + camera_direction , up)

        viewLoc = glGetUniformLocation(ourShader.ID, "view")
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm.value_ptr(view))

        # Crie matrizes de transformaçãpo para o mundo
        rotation_x_world = glm.rotate(glm.mat4(1.0), dados_translacao_mundo['roll_world'], glm.vec3(1, 0, 0))
        rotation_y_world = glm.rotate(glm.mat4(1.0), dados_translacao_mundo['pitch_world'], glm.vec3(0, 1, 0))
        rotation_z_world = glm.rotate(glm.mat4(1.0), dados_translacao_mundo['yaw_world'], glm.vec3(0, 0, 1))
        scale_world = glm.scale(glm.mat4(1.0), glm.vec3(dados_translacao_mundo['scale_x_world'], dados_translacao_mundo['scale_y_world'], dados_translacao_mundo['scale_z_world']))
        translate_world = glm.translate(glm.mat4(1.0), glm.vec3(dados_translacao_mundo['translate_x_world'], dados_translacao_mundo['translate_y_world'], dados_translacao_mundo['translate_z_world']))        

        model_transformation =  translate_world*scale_world*rotation_z_world*rotation_y_world*rotation_x_world

        model_transformationLoc = glGetUniformLocation(ourShader.ID, "model_transformation")
        glUniformMatrix4fv(model_transformationLoc, 1, GL_FALSE, glm.value_ptr(model_transformation))

        # render container        
        glBindVertexArray(VAO)
        glDrawElements(GL_TRIANGLES,indices.shape[0] , GL_UNSIGNED_INT, None)
        # Leitura dos pixels do framebuffer
        pixels = glReadPixels(0, 0, SCR_WIDTH, SCR_HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE)
        image_data = np.frombuffer(pixels, dtype=np.uint8).reshape(SCR_HEIGHT, SCR_WIDTH, 4).copy()

        # Inversão da imagem para que a parte superior seja no topo
        image_data = np.flipud(image_data)

        # Descartar os canais vermelho e verde, mantendo apenas o azul
        image_data[:, :, 0] = 0  # Descartar canal vermelho
        image_data[:, :, 1] = 0  # Descartar canal verde

        # Criar uma nova imagem a partir dos dados modificados, mantendo a estrutura RGBA
        image = Image.fromarray(image_data, 'RGBA')

        # Converter a imagem para RGB, mantendo apenas o canal azul
        blue_channel = image_data[:, :, 2]  # Extrai apenas o canal azul
        image = Image.fromarray(blue_channel, 'L')  # 'L' para imagem em escala de cinza (um único canal)

        # Salvar a imagem
        image.save(out_path + "/" + key + ".png", format='PNG', compress_level=0)

        # glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        # -------------------------------------------------------------------------------
        glfwSwapBuffers(window)
        glfwPollEvents()

    #End of render loop with texture
    


    # render loop depth map
    # -----------

    # build and compile our shader program
    # ------------------------------------
    ourShader = Shader("depth.vs", "depth.fs") 
    
    for key in sorted(dados_arquivos.keys()):
        if glfwWindowShouldClose(window):
            break

        # input
        # -----
        processInput(window)

        #print (key)

        # render
        # ------
        glClearColor(0.2, 0.3, 0.3, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        # bind Texture
        glBindTexture(GL_TEXTURE_2D, texture)

        # create transformations       
        # get matrix's uniform location and set matrix
        ourShader.use()
        projectionLoc = glGetUniformLocation(ourShader.ID, "projection")
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm.value_ptr(projection))

        # Rotação do robô
        # Crie matrizes de rotação individuais para cada eixo
        rotation_x = glm.rotate(glm.mat4(1.0), dados_arquivos[key]['Roll'], glm.vec3(1, 0, 0))
        rotation_y = glm.rotate(glm.mat4(1.0), dados_arquivos[key]['Pitch'], glm.vec3(0, 1, 0))
        rotation_z = glm.rotate(glm.mat4(1.0), dados_arquivos[key]['Yaw'], glm.vec3(0, 0, 1))

        robo_rotation_matrix = rotation_z * rotation_y * rotation_x

        posi_robo = glm.vec3(dados_arquivos[key]['Odometry Pose']['X'],dados_arquivos[key]['Odometry Pose']['Y'], dados_arquivos[key]['Odometry Pose']['Z'])
        camera_posi = robo_rotation_matrix*glm.vec4(dados_robo['x'],dados_robo['y'],dados_robo['z'],1.0)
        camera_posi = glm.vec3(camera_posi.x/camera_posi.w,camera_posi.y/camera_posi.w,camera_posi.z/camera_posi.w) + posi_robo


        # Rotação do câmera
        # Crie matrizes de rotação individuais para cada eixo
        rotation_x = glm.rotate(glm.mat4(1.0), dados_robo['r'], glm.vec3(1, 0, 0))
        rotation_y = glm.rotate(glm.mat4(1.0), dados_robo['p'], glm.vec3(0, 1, 0))
        rotation_z = glm.rotate(glm.mat4(1.0), dados_robo['y'], glm.vec3(0, 0, 1))
       
        camera_rotation_matrix = rotation_z * rotation_y * rotation_x

        robo_direction = glm.vec4(1.0, 0.0, 0.0,1.0)  #ros orientation
        camera_direction = robo_rotation_matrix*camera_rotation_matrix*robo_direction
        camera_direction = glm.vec3(camera_direction.x/camera_direction.w,camera_direction.y/camera_direction.w,camera_direction.z/camera_direction.w)

        up = robo_rotation_matrix*camera_rotation_matrix*glm.vec4(0.0, 0.0, 1.0,1.0)
        up = glm.vec3(up.x/up.w,up.y/up.w,up.z/up.w)
       

        view = glm.lookAt(camera_posi, camera_posi + camera_direction , up)

        viewLoc = glGetUniformLocation(ourShader.ID, "view")
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm.value_ptr(view))

        # Crie matrizes de transformaçãpo para o mundo
        rotation_x_world = glm.rotate(glm.mat4(1.0), dados_translacao_mundo['roll_world'], glm.vec3(1, 0, 0))
        rotation_y_world = glm.rotate(glm.mat4(1.0), dados_translacao_mundo['pitch_world'], glm.vec3(0, 1, 0))
        rotation_z_world = glm.rotate(glm.mat4(1.0), dados_translacao_mundo['yaw_world'], glm.vec3(0, 0, 1))
        scale_world = glm.scale(glm.mat4(1.0), glm.vec3(dados_translacao_mundo['scale_x_world'], dados_translacao_mundo['scale_y_world'], dados_translacao_mundo['scale_z_world']))
        translate_world = glm.translate(glm.mat4(1.0), glm.vec3(dados_translacao_mundo['translate_x_world'], dados_translacao_mundo['translate_y_world'], dados_translacao_mundo['translate_z_world']))        

        model_transformation =  translate_world*scale_world*rotation_z_world*rotation_y_world*rotation_x_world

        model_transformationLoc = glGetUniformLocation(ourShader.ID, "model_transformation")
        glUniformMatrix4fv(model_transformationLoc, 1, GL_FALSE, glm.value_ptr(model_transformation))

        # render container        
        glBindVertexArray(VAO)
        glDrawElements(GL_TRIANGLES,indices.shape[0] , GL_UNSIGNED_INT, None)

        # Ler a profundidade como floats
        depth_pixels = glReadPixels(0, 0, SCR_WIDTH, SCR_HEIGHT, GL_DEPTH_COMPONENT, GL_FLOAT)
        depth_data = np.frombuffer(depth_pixels, dtype=np.float32).reshape(SCR_HEIGHT, SCR_WIDTH)
        depth_data = np.flipud(depth_data)

        # Salvar os dados originais como binário compactado NumPy
        z_ndc = 2 * depth_data - 1.0
        A = projection[2][2]
        B = projection[3][2]
        z_eye = B / (A + z_ndc)
        #print (np.min(z_eye))
        #print (np.max(z_eye))

        #np.savez_compressed(out_path + "/" + key + "_depth_data.npz", depth_data=z_eye)


        # Normalizar os valores de profundidade para a faixa [0, 1]
        
        depth_data_normalized = (depth_data - np.min(depth_data)) / (np.max(depth_data) - np.min(depth_data))


        # Aplicar o colormap 'coolwarm' para colorir a profundidade
        colored_depth = plt.cm.coolwarm(depth_data_normalized)

        # Remover o canal alfa
        colored_depth = (colored_depth[:, :, :3] * 255).astype(np.uint8)

        # Converter para imagem PIL
        image = Image.fromarray(colored_depth)
        #image.convert("RGB").save(out_path + "/" + key + "_depth.jpg")

        # glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        # -------------------------------------------------------------------------------
        glfwSwapBuffers(window)
        glfwPollEvents()







    # optional: de-allocate all resources once they've outlived their purpose:
    # ------------------------------------------------------------------------
    glDeleteVertexArrays(1, (VAO,))
    glDeleteBuffers(1, (VBO,))
    glDeleteBuffers(1, (EBO,))

    # glfw: terminate, clearing all previously allocated GLFW resources.
    # ------------------------------------------------------------------
    glfwTerminate()