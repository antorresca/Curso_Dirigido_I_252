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

## 🔢 Procedimiento

### 🏗️ Arquitectura en ROS Noetic

### 💻 Firmware de Tiva

### 🤖 Arquitectura en ROS2 Humble

Una vez se comprendió la comunicación entre la NUC y la tiva para el envio de comandos al driver de los motores se procedió con la actualización de los nodos descritros en [Arquitectura en ROS Noetic](#arquitectura-en-ros-noetic). Acontinuación se describen los nodos actualizados

* **_SDV_Serial:_**  Permite la comunicación con la Tiva por medio del puerto serial. Los comandos enviados siguen la tabla descrita en la sección [Firmware de Tiva](#firmware-de-tiva)
* **_SDV_Control:_**  Realiza la cinematica inversa del robot, por medio de la transformación de las velocidades lineales y angulares a valores PWM para cada rueda. Para mayor información ir a [Cinemática del SDV](#Cinematica-SDV)


### ⚙️ Cinemática del SDV


## 📖 Bibliografia

