# 📡 Algoritmos de Navegación y Localización - 2025-2

## 🪶 Estudiantes:
* Juan Camilo Gomez Robayo
* Andres Camilo Torres-Cajamarca

## 👨‍🏫 Profesores:
* PhD. Ing. Ricardo Emiro Ramírez Heredia
* PhD. Ing. Pedro Fabian Cárdenas Herrera

## 📚 Indice

- [📡 Algoritmos de Navegación y Localización - 2025-2](#-algoritmos-de-navegación-y-localización---2025-2)
  - [🪶 Estudiantes:](#-estudiantes)
  - [👨‍🏫 Profesores:](#-profesores)
  - [📚 Indice](#-indice)
  - [🎯 Objetivos](#-objetivos)
  - [🚗 Conociendo al SDV](#-conociendo-al-sdv)
  - [🔢 Procedimiento](#-procedimiento)
    - [🏗️ Arquitectura en ROS Noetic](#️-arquitectura-en-ros-noetic)
    - [💻 Firmware de Tiva](#-firmware-de-tiva)
    - [🤖 Arquitectura en ROS2 Humble](#-arquitectura-en-ros2-humble)
    - [⚙️ Cinemática del SDV](#️-cinemática-del-sdv)
  - [📖 Bibliografia](#-bibliografia)


## 🎯 Objetivos

## 🚗 Conociendo al SDV

En el proceso de migración del SDVUN1 a ROS2 es necesario conocer de primera mano el funcionamiento y operación de los componentes del robot. Lo primero por descubrir es el rol de la Tiva en el proceso de comunicación entre los drivers de los motores y la NUC.
Primero iniciamos con las conexiones fisicas entre los motores y los encoders a los drivers, a continuación se presenta el esquema de conexiones descritas.





La Tiva se conecta a los drivers por puerto SATA a traves de un shield diseñado en el laboratorio y se encarga de enviar los parámetros de velocidad a los drivers de los motores.

Caracterización de los motores
Para este proceso se enviaron valores de PWM a la tiva iniciando en 20 y en paso de 10 hasta 60 y se contó el número de revoluciones para mismos periodos de tiempo en cada prueba como se muestra en el siguiente video:
https://github.com/user-attachments/assets/3671fbed-32fd-4de0-80da-c60e88005442

A partir de la información obtenida se elaboró la siguiente tabla que muestra los valores obtenidos:

| % PWM | RPM RUEDAS | 
|    :---:     |     :---:     |  
| 20   | 2    | 
| 30   | 4    | 
| 40   | 6    |
| 50   | 8    |
| 60   | 10   |


## 🔢 Procedimiento

### 🏗️ Arquitectura en ROS Noetic

### 💻 Firmware de Tiva

### 🤖 Arquitectura en ROS2 Humble

Una vez se comprendió la comunicación entre la NUC y la tiva para el envio de comandos al driver de los motores se procedió con la actualización de los nodos descritros en [Arquitectura en ROS Noetic](#arquitectura-en-ros-noetic). Acontinuación se describen los nodos actualizados

* **_SDV_Serial:_**  Permite la comunicación con la Tiva por medio del puerto serial. Los comandos enviados siguen la tabla descrita en la sección [Firmware de Tiva](#firmware-de-tiva)
* **_SDV_Control:_**  Realiza la cinematica inversa del robot, por medio de la transformación de las velocidades lineales y angulares a valores PWM para cada rueda. Para mayor información ir a [Cinemática del SDV](#Cinematica-SDV)


### ⚙️ Cinemática del SDV


## 📖 Bibliografia

