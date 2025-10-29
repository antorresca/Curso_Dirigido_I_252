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
    - [🧱 Componentes implementados](#-componentes-implementados)
      - [🎛️ Tiva](#️-tiva)
      - [🚏 Driver y Encoder](#-driver-y-encoder)
      - [🚘 Motores](#-motores)
      - [🧠 NUC](#-nuc)
      - [📶 Lidar](#-lidar)
  - [🔢 Procedimiento](#-procedimiento)
    - [🏗️ Arquitectura en ROS Noetic](#️-arquitectura-en-ros-noetic)
    - [💻 Firmware de Tiva](#-firmware-de-tiva)
    - [🤖 Arquitectura en ROS2 Humble](#-arquitectura-en-ros2-humble)
    - [⚙️ Cinemática del SDV](#️-cinemática-del-sdv)
      - [🧾 Pruebas iniciales](#-pruebas-iniciales)
      - [📝 Caracterización de motores](#-caracterización-de-motores)
      - [🔧Cambio en cinemática](#cambio-en-cinemática)
      - [✅ Validación de cinemática](#-validación-de-cinemática)
    - [📡 Lidar](#-lidar-1)
  - [📖 Bibliografia](#-bibliografia)


## 🎯 Objetivos

## 🚗 Conociendo al SDV

En el proceso de migración del SDVUN1 a ROS2 es necesario conocer de primera mano el funcionamiento y operación de los componentes del robot. Lo primero por descubrir es el rol de la Tiva en el proceso de comunicación entre los drivers de los motores y la NUC.
Primero iniciamos con las conexiones fisicas entre los motores y los encoders a los drivers, a continuación se presenta el esquema de conexiones descritas.

La Tiva se conecta a los drivers por puerto SATA a traves de un shield diseñado en el laboratorio y se encarga de enviar los parámetros de velocidad a los drivers de los motores.

### 🧱 Componentes implementados 

#### 🎛️ Tiva

<!---Poner informacion del LaunchPad--->

#### 🚏 Driver y Encoder

<!---Poner informacion del driver--->

#### 🚘 Motores

<!---Poner informacion de los motores--->

#### 🧠 NUC

<!---Poner informacion de la NUC y sus caracteristicas de hardware--->

#### 📶 Lidar

<!--Informacion general del lidar-->

## 🔢 Procedimiento

### 🏗️ Arquitectura en ROS Noetic

<!---Mostrar RQT de ROS1--->

### 💻 Firmware de Tiva

<!--Informacion general del firmware-->

### 🤖 Arquitectura en ROS2 Humble

Una vez se comprendió la comunicación entre la NUC y la tiva para el envio de comandos al driver de los motores se procedió con la actualización de los nodos descritros en [Arquitectura en ROS Noetic](#arquitectura-en-ros-noetic). Acontinuación se describen los nodos actualizados

* **_SDV_Serial:_**  Permite la comunicación con la Tiva por medio del puerto serial. Los comandos enviados siguen la tabla descrita en la sección [Firmware de Tiva](#firmware-de-tiva)
* **_SDV_Control:_**  Realiza la cinematica inversa del robot, por medio de la transformación de las velocidades lineales y angulares a valores PWM para cada rueda. Para mayor información ir a [Cinemática del SDV](#Cinematica-SDV)

### ⚙️ Cinemática del SDV

#### 🧾 Pruebas iniciales

Para comprobar el correcto funcionamiento del robot, se verificó la cinematica implementada en la version inicial (Con ROS noetic) para ello se tomó un video enviadole una velocidad lineal de $0.1\tfrac{m}{s}$ y se procesó con Tracker

Al realizar el analisis con los datos recolectados con el programa dio un promedio de $0.06\tfrac{m}{s}$ dando un error absoluto en la velocidad lineal de aproximadamente 40%. Teniendo en cuenta esto, no se realizó la prueba angular y se siguió con la caracterización de los motores y la implementación de una nueva cinemática.

<!-- colocar el codigo o la ecuacion que habian empleado-->

#### 📝 Caracterización de motores

Para este proceso se enviaron valores de PWM a la tiva iniciando en 20 y en paso de 10 hasta 60 y se contó el número de revoluciones para mismos periodos de tiempo en cada prueba como se muestra en el siguiente video:

<div align ='center'>
   <video src='https://github.com/user-attachments/assets/3671fbed-32fd-4de0-80da-c60e88005442'>
</div>

A partir de la información obtenida se elaboró la siguiente tabla que muestra los valores obtenidos:
<div align ='center'>
   
| % PWM | RPM RUEDAS | 
|    :---:     |     :---:     |  
| 20   | 2    | 
| 30   | 4    | 
| 40   | 6    |
| 50   | 8    |
| 60   | 10   |

</div>

<!---Colocar la regresión lineal-->

$$\text{RPM} = 1.2\text{PWM}-12$$

Cabe resaltar que se asume que la ganacia lineal y desface de los motores para generar torque es aproximadamente igual en ambos sentidos de giro

#### 🔧Cambio en cinemática

Con la regresion lineal hallada en la [Caracterización de motores](#-caracterización-de-motores) se implementó en el código teniendo las siguientes consideraciones:

- La velocidad enviada por el tópico será en metros por segundo $m/s$
- La conversion debe ser a PWM y se debe tener en cuenta que la regresion es PWM $vs$ RPM
- Al considerarse igualdad en la ganancia en ambos sentidos de giro se debe tomar el valor absoluto de la velocidad y solo cambia el signo para aplicar el cambio de giro

Por lo cual la ecuacion cambia a

$$\text{PWM} = 0.8333(\tfrac{30}{\pi})v_{\text{rueda}} + 10$$

Dicha ecuacion se implementó en el código como se observa acontinuación:

```cpp
int getPWM(double V, double W,bool Side){
        /*Function to get PWM for each wheel 
        <args>
        V -> Lineal Velocity
        W -> Angular Velocity
        Side -> Side of wheel (True: Right | False: Left)
        */
        double wheel_radio = 0.075; //radio of wheels in meters
        double wheel_base = 0.32;  //distance between wheel (RL) in meter
        int factor = 0;

        double w_wheel =  V/wheel_radio;

        if(Side){
            w_wheel -= (wheel_base*W)/wheel_radio; 
        }else{
            w_wheel += (wheel_base*W)/wheel_radio;
        };

        if(w_wheel>0){
        factor = 1;
        }else{
        factor = -1;
        };

        return (0.8333*(30/3.141592)*abs(w_wheel)+10)*factor;
    }
```

#### ✅ Validación de cinemática

Una vez cambiada la cinematica implementada, se verificó la velocidad lineal y angular, para ello se tomaron videos y se procesaron con el sofware Tracker

Dando como resultado:

<div align ='center'>

|Velocidad|Enviada|Medida|Error|
|---|----|---|---|
|Lineal ($v$)|$0.1\tfrac{m}{s}$|$0.09\tfrac{m}{s}$|$7\%$|
|Angular ($\omega$)|$0.5\tfrac{rad}{s}$|$0.43\tfrac{rad}{s}$|$13\%$|

</div>

### 📡 Lidar

Para conectar el lidar, en un principio se empleó _SOPAS ET_ para poder verificacar el modo en el que se encuentra el LIDAR, con la finalidad de evitar posibles fallos en la conexión



una vez verificado se siguió el procedimiento del paquete oficial para ROS2 creado por el fabricante ([sick_scan_xd](https://docs.ros.org/en/iron/p/sick_scan_xd/)), dicho paquete fue incluido en el _workspace_ y se eliminaron los archivos no necesarios. Una vez con este paquete fue necesario realizar cambios en el archivo "_sick_nav_350.launch_" puesto que la IP que trae por defecto el Lidar fue modificada para evitar que se pueda acceder directamente desde WiFi

```python
<arg name="hostname" default="169.254.7.16"/>
```

Aparte de esto es necesario recalcar que se debe cambiar la IP del puerto Ethernet (Eth0) de la NUC a "169.254.7.15" para poder realizar la comunicación con el Lidar. Al cambiar las IPs se realiza Ping al Lidar para comprobar la comunicación. Ya con esta verificación se puede realizar el compilado, el cual la primera vez que se ejecute se debe realizar de la siguiente manera

```bash
colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" " -DLDMRS=0" --event-handlers console_direct+
. install/setup.bash
```

Esto con el fin de que se instalen las dependencias necesarias para su correcto funcionamiento. Al terminal al compilacion se realiza el lanzamiento de los nodos 

```bash
ros2 launch sick_scan_xd sick_nav_350.launch.py
```

dichos nodos permiten la comunicación con el Lidar y la habilitación del topico "_/scan_" el cual manda mensajes de tipo "_sensor_msgs/LaserSensor_" <!--verificar--> para visualizar el funcionamiento del Lidar una vez esté conectado a ROS2, se puede ejecutar RViz2 como se ve acontinuación:

Aca se puede ver el entorno que el lidar puede percibir

## 📖 Bibliografia

