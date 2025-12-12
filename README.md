# 📡 Algoritmos de Navegación y Localización - 2025-2 <!-- omit from toc -->

<div align='center'>
      <img src="https://github.com/user-attachments/assets/ee060fb8-cf1f-4fa6-b386-5f3007993c33" 
           alt="Escudo UNAL" 
           width=100>
</div>

## 🪶 Estudiantes: <!-- omit from toc -->
* [Juan Camilo Gomez Robayo](juagomezro@unal.edu.co)
* [Andres Camilo Torres-Cajamarca](antorresca@unal.edu.co)

## 👨‍🏫 Profesores: <!-- omit from toc -->
* PhD. Ing. Ricardo Emiro Ramírez Heredia
* PhD. Ing. Pedro Fabian Cárdenas Herrera

## 🕹️ Manual de Usuario <!-- omit from toc -->

Para emplear el SDV 1 con ROS2 Humble, se deben seguir los siguientes pasos:

### 0. Requisitos

* Ubuntu 22.04
* ROS2 Humble
* RViz2
* Acceso red del laboratorio
* Robot SDV1

### 1. Configuración inicial <!-- omit from toc -->

Para utilizar este proyecto, asegúrese de que el robot esté encendido, con las baterías LiPo cargadas y correctamente conectadas, y con los tres switches activados (Driver, Lidar y NUC).

Además, el robot debe estar ubicado en el HOME definido en el laboratorio.

Si tiene dudas sobre este procedimiento, consulte con el personal del laboratorio.

### 2. Conexión por SHH con el SDV 1 <!-- omit from toc -->

Conéctese a la red del laboratorio *LabFabEx* (o alguna de sus subredes habilitadas) y ejecute en la terminal:

```bash
ssh sdvr2@192.168.1.11
```

El sistema pedirá la contraseña, la cual debe solicitar al personal del laboratorio o al correo: ```labfabex_fibog@unal.edu.co```

Una vez conectado al robot, ejecute los siguientes comandos:

```bash
cd ~/CursoDirigido/SDV_UN_ROS2
git checkout master
git pull
colcon build
. install/setup.bash
```

Esto garantiza que cuenta con la versión más actualizada y estable del proyecto.

### 3. Lanzamiento de nodos <!-- omit from toc -->

Desde el robot, ejecutar el siguiente comando

```bash
ros2 launch sdv_nav sdv_nav.launch.py
```

Este comando iniciará todos los nodos necesarios para el funcionamiento del sistema.

Por otro lado, en el PC debe cargar la configuración de RViz2 [sdv_nav.rviz](misc/sdv_nav.rviz) Puede hacerlo de dos maneras:

1. **Desde la intrefaz de usuario**

Para abrir la configuración realizada en RViz2, ejecutar:

```bash
rviz2
```

Luego, en el menú superior seleccione **File → Open Config** y cargue el archivo *sdv_nav.rviz*

2. **Abrir el archivo desde terminal**

Si se quiere abrir el archivo solo por lineas de comandos, ejecutar:

```bash
git clone https://github.com/antorresca/Curso_Dirigido_I_252.git
cd Curso_Dirigido_I_252
rviz2 -d /misc/sdv_nav.rviz
```

Esto abrirá RViz2 con la configuración preestablecida.

**Nota:** Verificar en RViz2 que los siguientes items mínimos estén cargados:

* Map

  * Tópico: /map

  * QoS:

    Reliability: Reliable

    Durability: Transient Local

* TF cargado correctamente.

## 4. Puesta en marcha <!-- omit from toc -->

Con todos los nodos en ejecución y RViz2 mostrando el estado del robot, la interfaz debería verse de manera similar a la imagen de referencia:
<div align="center">
<img width="1919" height="1015" alt="RViz" src="https://github.com/user-attachments/assets/b8a7de93-0641-4e53-a8e6-dc907bbec9db" />
</div>

Para enviar un objetivo al robot, seleccione **2D Goal Pose** y haga clic en el punto al que desea que llegue.

<div align="center"
<img width="928" height="236" alt="Goal" src="https://github.com/user-attachments/assets/178a4d26-5b86-4d27-ae45-daa029583946" />
</div>

## 5. Cierre del programa <!-- omit from toc -->

En la terminal del robot se puede finalizar con la combinación de teclado "Ctrl+C", de igual manera en la terminal del PC para cerrar el RViz.

----
----

# ℹ️ Más información  <!-- omit from toc -->

A continuación se detalla el desarrollo del proyecto.

## 📚 Indice <!-- omit from toc -->

<details>
    <summary>🗂️ Tabla de Contenido</summary>

- [1. 🎯 Objetivos](#1--objetivos)
- [2. 🚗 Conociendo al SDV](#2--conociendo-al-sdv)
  - [2.1. 🧱 Componentes implementados](#21--componentes-implementados)
    - [2.1.1. 🎛️ Tiva](#211-️-tiva)
    - [2.1.2. 🚏 Driver y Encoder](#212--driver-y-encoder)
    - [2.1.3. 🚘 Motores](#213--motores)
    - [2.1.4. 🧠 NUC](#214--nuc)
    - [2.1.5. 📶 Lidar](#215--lidar)
- [3. 🔢 Procedimiento](#3--procedimiento)
  - [3.1. 🏗️ Arquitectura en ROS Melodic](#31-️-arquitectura-en-ros-melodic)
  - [3.2. 💻 Firmware de Tiva](#32--firmware-de-tiva)
  - [3.3. 🤖 Arquitectura en ROS2 Humble](#33--arquitectura-en-ros2-humble)
  - [3.4. ⚙️ Programación de la Cinemática del SDV](#34-️-programación-de-la-cinemática-del-sdv)
    - [3.4.1. 🧾 Pruebas iniciales](#341--pruebas-iniciales)
    - [3.4.2. 📝 Caracterización de motores](#342--caracterización-de-motores)
    - [3.4.3. 🔧Cambio en programación de la cinemática](#343-cambio-en-programación-de-la-cinemática)
    - [3.4.4. ✅ Validación de la programación de la cinemática](#344--validación-de-la-programación-de-la-cinemática)
  - [3.5. 🖥️ Simulación](#35-️-simulación)
  - [3.6. 📡 Lidar](#36--lidar)
  - [3.7. 🗺️ Mapa Global](#37-️-mapa-global)
  - [3.8. 🌎 Odometría](#38--odometría)
  - [3.9. 📍 Localización](#39--localización)
  - [3.10. 🗃️ Planeación](#310-️-planeación)
    - [3.10.1. ⭐ A\*](#3101--a)
    - [3.10.2. 🖥️ Implementación](#3102-️-implementación)
  - [3.11. 🕹️ Control](#311-️-control)
    - [3.11.1. ⏭️ Pure Pursuit](#3111-️-pure-pursuit)
    - [3.11.2. 🖥️ Implementación](#3112-️-implementación)
- [4. 🧪 Resultados](#4--resultados)
  - [4.1. Pipeline obtenido](#41-pipeline-obtenido)
  - [4.2. Arbol de TF](#42-arbol-de-tf)
  - [4.3. Arquitectura ROS2 Humble](#43-arquitectura-ros2-humble)
  - [4.4. Robot físico](#44-robot-físico)
- [5. 🔚 Conclusiones](#5--conclusiones)
- [6. 🔜 Trabajo a futuro](#6--trabajo-a-futuro)
- [7. 📖 Bibliografia](#7--bibliografia)
- [8. 📒 Contacto](#8--contacto)

</details>


## 1. 🎯 Objetivos

1. Estudiar y comprender diversos algoritmos de navegación aplicados a robots móviles terrestres.

2. Implementar algoritmos de control, localización, mapeo y planeación en un robot móvil con arquitectura diferencial.

3. Actualizar el SDV 1 del laboratorio, migrándolo de ROS Melodic a ROS 2 Humble para mejorar su funcionalidad y compatibilidad.

## 2. 🚗 Conociendo al SDV

En el proceso de migración del SDVUN1 a ROS2 es necesario conocer de primera mano el funcionamiento y operación de los componentes del robot. Lo primero por descubrir es el rol de la Tiva en el proceso de comunicación entre los drivers de los motores y la NUC.
<div align="center">
<img width="700" height="463" alt="LazoControl" src="https://github.com/user-attachments/assets/863656fd-fdd0-49fb-8037-90483a1c4678" />
</div>

<div align="center">
  <img width="300"  alt="Blank diagram - Page 23" src="https://github.com/user-attachments/assets/84c4a94c-67f7-480a-b780-e1941fe37414" />
</div>

Primero iniciamos con las conexiones fisicas entre los motores y los encoders a los drivers, a continuación se presenta el esquema de conexiones descritas.

### 2.1. 🧱 Componentes implementados 

#### 2.1.1. 🎛️ Tiva
El SDV utiliza una placa de desarrollo launchpad TIVA de National Instruments, la cual se encarga de configurar la comunicación entre la NUC y los motores del vehículo para la ejecución de un movimiento controlado, a continuación se presenta la imagen de la tiva que además está montada sobre una PCB desarrollada para la hacer la conexión por puerto SATA con los drivers de los respectivos motores:
<div align="center">
<img width="400"  alt="Conexión" src="https://github.com/user-attachments/assets/9c5efe28-f632-4d6a-85b1-192cad82ea40" />
</div>
Además, se desarrolló esta PCB que se encarga del mapeo de las conexiones en el bus SATA como se muestra a continuación:
<div align="center">
<img width="300"  alt="PCBEscon" src="https://github.com/user-attachments/assets/aaf3ffca-48dc-428a-867f-3db05c0106f4" />
</div>


<!---Poner informacion del LaunchPad--->

#### 2.1.2. 🚏 Driver y Encoder
El driver utilizado es un driver de EsconMotor referencia 50/5 el cual se comunica por puerto serial con la tiva, que le envía los valores de PWM para el motor que controla y se realimenta con el encoder, esta realimentación la usa para realizar el control de velocidad en el motor correspondiente. Tiene diferentes entradas y salidas, entre ellas una entrada para las señales digitales del encoder y un puerto de comunicación con la tiva a traves de un cable SATA. 

<div align="center">
<img width="400"  alt="Conexion PCBEscon" src="https://github.com/user-attachments/assets/c97b2dc5-3c7d-4005-9fc5-ffeb69124703" />
</div>

El encoder utilizado tiene una resolucón de 1200 PPR, lo cual brinda una resolución más que suficiente para el control del motor

<div align="center">
<img width="400"  alt="Encoder" src="https://github.com/user-attachments/assets/2074c60a-afdb-44dd-bc8b-95d6504c9bb3" />
</div>


<!---Poner informacion del driver--->

#### 2.1.3. 🚘 Motores
Los motores tambien son de la marca Maxon Motors, son motores DC con un sistema de engranajes que generan una reducción de 57/1 y elevan el torque del motor. 
<div align="center">
<img width="400"  alt="Motor" src="https://github.com/user-attachments/assets/9ff65ddc-7f57-4008-aff4-1093654eaa7f" />
</div>


<!---Poner informacion de los motores--->

#### 2.1.4. 🧠 NUC
El procesamiento en general corre sobre una Intel NUC que posee un procesador Core I7 con 8 Núcleos, 8 GB de memoria Ram y un SSD SATA de 240 GB, se conecta a la red local a través de la red WIFI de laboratorio.
<div align= "center">
<img width="400"  alt="NUC" src="https://github.com/user-attachments/assets/814236b2-b625-4bf7-a34b-7d7d41c1503f") />
</div>

<!---Poner informacion de la NUC y sus caracteristicas de hardware--->

#### 2.1.5. 📶 Lidar
El lidar implementado es un Sick Nav 350-3232 el cual tiene una capacidad de detección de 360° se alimenta con 2 Baterías LiPo de 4 celdas cada una y se conecta a la NUC a traves del puerto Ethernet, es necesario mencionar que la IP del adaptador de Red debe estár en el mismo rango de IP que el LiDar ya que una mala configuración no permite que se inicie la comunicación entre el LiDar y la NUC
<!--Informacion general del lidar-->
<div align= "center">
<img width="400" height="547" alt="LiDar" src="https://github.com/user-attachments/assets/1c6af9ea-e131-40e6-aec6-217a148aaa9f")/>
</div>

## 3. 🔢 Procedimiento

### 3.1. 🏗️ Arquitectura en ROS Melodic

Se quiere comprender el funcionamiento inicial del robot con la arquitectura realizada previamente por el grupo DIMA fue implementada en ROS Melodic 1.14.12. Para ello se analizan los nodos y tópicos implementados, para obtener una visión general de la arquitectura, se muestran a continuación en el grafo RQT.

<div align="center">
<img width="2960" height="1224" alt="Group" src="https://github.com/user-attachments/assets/19fe4194-843f-41f9-9f89-4250225912e9" />
</div>

**Nota:** Esta arquitectura se encuentra bajo derechos de autor por lo cual no puede ser compartida en su totalidad.

Los nodos y su descripción general se pueden comprender en la siguiente tabla:

A partir de esto, se puede decir que:

* Existe un nodo dedicado a la comunicación con la Tiva.
* Se emplea el paquete oficial de ROS "*move_base*" para la navegación.
* Se utiliza Hector Mapping para el SLAM.
* Hay nodos dedicados a la comunicación en red y con Firebase.
* Existen nodos exclusivos para el uso del Lidar.
  
Teniendo en cuenta lo anterior se decidió empezar con la actualización a ROS2 de las siguientes características:

* Comunicación serial a la Tiva
* Navegación con "*Nav2*" de ROS2 ("*Move Base*" no existe para ROS2)
* Uso de Lidar con paquetes oficiales de SICK

Cabe aclarar que se tiene acceso a los archivos originales de los SDV por lo cual se puede reutilizar archivos de declaraciones (en C++) y simulaciones (con archivos DAE).

### 3.2. 💻 Firmware de Tiva

para acceder al firmware de la tiva se ejecuta el siguiente comando

```bash
screen /dev/ttyACM0 921600
```

**Nota:** Se debe tener instalado "screen" (ejecutar ```sudo apt install screen```)

dentro de screen se envia el comando ```h``` para poder revisar la información de los comandos disponibles para usarlo

<div align="center">

<img width="591" height="547" alt="Screenshot_2025-10-30_09-59-37" src="https://github.com/user-attachments/assets/237ad5b4-42c9-4e8d-95d9-b5fc4b87f575" />

</div>

Aca se puede observar los deversos comandos que se pueden enviar a la tiva, el necesario para el uso de los motores es ```m``` con los argumentos de habilitación y velocidades con signo de cada motor. Con ello se concluye que:

* al enviar ```m 1 +V1 +V2``` los motores se habilitan y giran en sentido que avance el robot
* al enviar ```m 0 +/-V1 +/-V2``` los motores se deshabilitan y el robot se detiene
* Las velocidades que se envian tiene en cuenta el marco de referencia del robot ($+x$ en sentido de avance) mas no el marco de referencia de cada rueda ($+z$ saliendo de la rueda, provocando que la rueda derecha deba girar en sentido negativo a este marco para lograr avance lineal)

### 3.3. 🤖 Arquitectura en ROS2 Humble

Una vez se comprendió la comunicación entre la NUC y la tiva para el envio de comandos al driver de los motores se procedió con la actualización de los nodos descritros en [Arquitectura en ROS Noetic](#️-arquitectura-en-ros-noetic). Acontinuación se describen los nodos actualizados

* **_SDV_Serial:_**  Permite la comunicación con la Tiva por medio del puerto serial. Los comandos enviados siguen la tabla descrita en la sección [Firmware de Tiva](#-firmware-de-tiva).
* **_SDV_Control:_**  Realiza la cinematica inversa del robot, por medio de la transformación de las velocidades lineales y angulares a valores PWM para cada rueda. Para mayor información ir a 
### 3.4. ⚙️ Programación de la Cinemática del SDV

#### 3.4.1. 🧾 Pruebas iniciales

Para comprobar el correcto funcionamiento del robot, se verificó la cinematica implementada en la version inicial (Con ROS noetic) para ello se tomó un video enviadole una velocidad lineal de $0.1\tfrac{m}{s}$ y se procesó con Tracker

<div align="center">
<img width="700"  alt="Lineal_previo_Tracker" src="https://github.com/user-attachments/assets/fc5f697a-1fa2-445c-abf7-4f3b6a6b87b4" />
</div>

Al realizar el analisis con los datos recolectados con el programa dio un promedio de $0.06\tfrac{m}{s}$ dando un error absoluto en la velocidad lineal de aproximadamente 40%. Teniendo en cuenta esto, no se realizó la prueba angular y se siguió con la caracterización de los motores y la implementación de una nueva cinemática.

<!-- colocar el codigo o la ecuacion que habian empleado-->

#### 3.4.2. 📝 Caracterización de motores

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

#### 3.4.3. 🔧Cambio en programación de la cinemática

Con la regresion lineal hallada en la [Caracterización de motores](#-caracterización-de-motores) se implementó en el código teniendo las siguientes consideraciones:

- La velocidad enviada por el tópico será en metros por segundo $m/s$
- La conversion debe ser a PWM y se debe tener en cuenta que la regresion es PWM $vs$ RPM
- Al considerarse igualdad en la ganancia en ambos sentidos de giro se debe tomar el valor absoluto de la velocidad y solo cambia el signo para aplicar el cambio de giro

Por lo cual la ecuación cambia a

$$\text{PWM} = 0.8333(\tfrac{30}{\pi})|v_{\text{rueda}}| + 10$$

Dicha ecuación se implementó en el código como se observa acontinuación:

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

#### 3.4.4. ✅ Validación de la programación de la cinemática

Una vez cambiada la cinematica implementada, se verificó la velocidad lineal y angular, para ello se tomaron videos y se procesaron con el sofware Tracker

<div style="text-align: center;">
  <img src="https://github.com/user-attachments/assets/30ef1ca8-a4df-4e1f-a082-2f067486d7b9" 
       alt="Lineal_Tracker" width="45%" style="display: inline-block; margin-right: 10px;" />
  <img src="https://github.com/user-attachments/assets/1cafe035-001f-4ff1-9eef-9d4a9bfb6a33" 
       alt="Angular_Tracker" width="45%" style="display: inline-block;" />
</div>

Dando como resultado:

<div align ='center'>

|Velocidad|Enviada|Medida|Error|
|---|----|---|---|
|Lineal ($v$)|$0.1\tfrac{m}{s}$|$0.09\tfrac{m}{s}$|$7$%|
|Angular ($\omega$)|$0.5\tfrac{rad}{s}$|$0.43\tfrac{rad}{s}$|$13$%|

</div>

### 3.5. 🖥️ Simulación

Para la simulación se emplearon los archivos base de Gazebo desarrollados previamente, acotándolos específicamente para el SDV 1, ya que para cada SDV cambian ciertas características técnicas y físicas. Estos archivos de lanzamiento fueron actualizados a ROS 2, debido a que en esta versión ya no se utilizan archivos de tipo YML, sino que los parámetros deben declararse dentro de los propios archivos de lanzamiento.

En el desarrollo original existían diversos parámetros para cada SDV, tanto generales como específicos, por lo que fue necesario comprender la estructura de los archivos URDF para lograr el correcto ensamble del robot en Gazebo. Además, se actualizó la declaración correspondiente para emplear el mapa del laboratorio.

A continuación, se presenta el modelo CAD del robot y del entorno del laboratorio en Gazebo:

<div align="center">
<img width="882" height="643" alt="image" src="https://github.com/user-attachments/assets/e7bf0597-9bf4-4f9b-954e-439f32685df2" />
</div>

También se consideró el uso del software NVIDIA Isaac Sim para la simulación robótica; sin embargo, este producto requiere amplias capacidades de cómputo, por lo cual no ha sido posible su implementación.

Por otra parte, se crearon las dependencias necesarias para la transformación de marcos de referencia (tf) con el fin de visualizar el robot en RViz. A continuación, se muestra su visualización en dicho programa:

<div align="center">
<img width="882" height="643" alt="image" src="https://github.com/user-attachments/assets/27942275-2cc4-40c5-97e7-a8b4f41ea8b4" />
</div>

**Nota:** En RViz únicamente se muestra el robot, ya que las transformaciones tf solo se aplican al modelo del robot y no al mapa.

### 3.6. 📡 Lidar

Para conectar el lidar, se emplea el software oficial *SOPAS_ET*. En primer lugar se verifica la IP asignada al Lidar para su conexión, como se puede ver acontinuación:

<div align="center">
<img width="402" height="586" alt="SOPAS_IP" src="https://github.com/user-attachments/assets/1bb77ef5-e4bc-4f0c-9c7d-7a5f5b60fa0c" />

</div>

Quedando asignada la IP "169.254.7.16". Ya con esto se verifica el modo de operación del Lidar que debe ser Navegación para poder emplear su capacidad de mapeo y de odometria, en la siguiente imagen se puede observar el mapeo con el programa

<div align="center">
<img width="1366" height="730" alt="SOPAS_Nav" src="https://github.com/user-attachments/assets/bf68d156-fd40-4b27-8c28-6c06eb0a972a" />
</div>

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

dichos nodos permiten la comunicación con el Lidar y la habilitación del topico ```/scan``` el cual manda mensajes de tipo ```sensor_msgs/msg/LaserScan```  para visualizar el funcionamiento del Lidar una vez esté conectado a ROS2, se puede ejecutar RViz2 como se ve acontinuación:

<div align="center">
<img width="1919" height="1076" alt="Screenshot from 2025-10-30 10-21-09" src="https://github.com/user-attachments/assets/f12cae8f-1e4e-4693-9c63-7cb3fb104eda" />
</div>

Aca se puede ver el entorno que el lidar puede percibir

### 3.7. 🗺️ Mapa Global

El mapa global generado previamente por medio de SLAM con el SDV y el Lidar en la versión base. Para implementarlo en ROS2 se usa **map_server** de **Nav2**, este nodo toma el archivo **.yaml** y genera el tópico ```\map``` para poder consultar el mapa global cuando se requiera. En la siguiente imagen se puede ver el mapa global del laboratorio LabFabEx:

<div align="center">
      <img src="https://github.com/user-attachments/assets/83af0862-5127-4119-ad06-a2df389a701a"/>
</div>

### 3.8. 🌎 Odometría

Para la odometría, inicialmente se empleó una odometría teórica, es decir, a partir de los comandos de velocidad que recibiría el robot ($\dot{\theta}_r$ y $\dot{\theta}_l$) y una pose inicial, se calculaba en cada instante de tiempo la posición del robot mediante las fórmulas estándar de movimiento rectilíneo y curvilíneo.
Para implementarlo, se desarrolló un algoritmo basado en el siguiente pseudocódigo:

```cpp
función update_odometry():

    ahora = tiempo_actual()
    dt = ahora - tiempo_anterior

    si dt <= 0 o dt > 1:
        tiempo_anterior = ahora
        terminar función

    tiempo_anterior = ahora

    // 1. Calcular velocidades lineales de cada rueda
    v_r = R * dot_theta_r
    v_l = R * dot_theta_l

    // 2. Velocidad lineal y angular del robot
    v = (v_r + v_l) / 2
    w = (v_r - v_l) / L

    // 3. Integración de pose
    si |w| es muy pequeño:
        // Movimiento recto
        x  = x + v * cos(th) * dt
        y  = y + v * sin(th) * dt
    si no:
        // Movimiento curvo
        x = x + (v / w) * (sin(th + w*dt) - sin(th))
        y = y - (v / w) * (cos(th + w*dt) - cos(th))

    th = th + w * dt

    // Normalizar ángulo a [-π, π]
    mientras th >  π: th = th - 2π
    mientras th < -π: th = th + 2π

    pose = (x, y, th)
```

Que siguen las siguientes ecuaciones, velocidades:

$$v_r = R\,\dot{\theta}_r$$
$$v_l = R\,\dot{\theta}_l$$

Con $R$ como radio de la rueda. Movimiento rectilinio ($\omega<\epsilon$):

$$x_t = x_{t-1} + v \cos(\theta_{t-1})\,\Delta t$$
$$y_t = y_{t-1} + v \sin(\theta_{t-1})\,\Delta t$$

movimiento curvilineo ($\omega>\epsilon$):

$$x_t = x_{t-1} + \frac{v}{\omega}\left[\sin(\theta_{t-1}+\omega\Delta t) - \sin(\theta_{t-1})\right]
$$
$$y_t = y_{t-1} - \frac{v}{\omega}\left[\cos(\theta_{t-1}+\omega\Delta t) - \cos(\theta_{t-1})\right]$$

$$\theta_t = \theta_{t-1} + \Delta\theta$$

Es importante destacar que, al ser un modelo teórico, se asumen movimientos ideales: sin deslizamiento, sin pérdidas de potencia en los motores, sin inercia, con cambios instantáneos de velocidad (sin rampas de aceleración/desaceleración), entre otras simplificaciones. 

**Uso en el proyecto**

Para implementarlo dentro del proyecto, se colocó el código en el nodo [sdv_controller](src/sdv_controller/src/sdv_controller_node.cpp) que publica en el tópico ```/odom``` la odometria del robot y el TF correspondiente (Para mas información de esto ir a [TF]())

**Modificación para correcto funcionamiento**

Dado que la odometría teórica no ofrece la fiabilidad necesaria para garantizar una navegación precisa, se realizó un ajuste en el mensaje del tópico ```/odom``` de tipo ```nav_msgs/Odometry```.
En particular, dentro del submensaje ```pose``` (```geometry_msgs/PoseWithCovariance```), se incrementaron los valores de la covarianza con el fin de reducir la confianza del algoritmo de localización en la odometría.

Con esto, el sistema sigue utilizando la odometría como referencia básica para el funcionamiento general del robot, pero evita que el algoritmo de localización dependa excesivamente de ella.

### 3.9. 📍 Localización 

Para poder determinar '*¿dónde se encuentra el robot dentro del mapa?*' se debe implementar un algoritmo de localización, para ello se empleó el Algoritmo de Localización de Monte-Carlo (**AMCL** por sus siglas en inglés) 

**¿Cómo Funciona?**

AMCL es un método de localización basado en filtros de partículas. Mantiene un conjunto de hipótesis (partículas) sobre la posible posición del robot en el mapa. Cada vez que el robot se mueve, estas partículas se actualizan según el modelo de movimiento (odometría).
Al recibir mediciones del sensor láser, el algoritmo compara estas mediciones con el mapa y ajusta el peso de cada partícula según la coincidencia observada. Finalmente, emplea un proceso de resampling para concentrarse en las partículas más probables, logrando una estimación robusta incluso en presencia de ruido. A continuación se ve una breve muestra visual de como funciona:

<div align="center">
<img src="https://github.com/user-attachments/assets/897bfaea-438f-486e-be10-068130a8dee9" />
</div>

Como se puede ver, al inicio hay muchas particulas de supoción de donde se encuentra el robot, y con el laser se ubicado donde debe estar. En cada iteración de movimiento hay una nube de puntos de suposiciones de donde deberia estar el robot que converge con ayuda del laser.

**¿Cómo se implementa?**

Para implementar AMCL en ROS se utiliza principalmente la odometría del robot, el mapa estático, el LIDAR y una pose inicial. En este proyecto, la pose inicial se definió en un punto home ($x=0$, $y=0$) con una orientación predeterminada ($\omega=0$); sin embargo, dicha pose puede configurarse desde **RViz** o desde un nodo externo mediante el tópico ```\InitialPose```.

Se emplean los siguientes tópicos:
* ```\odom``` odometria del robot (Ver mas en [Odometría](#38--odometría))
* ```\scan``` datos del Lidar (Ver mas en [Lidar](#36--lidar))
* ```\map``` Mapa estático (Ver mas en [Mapa Global](#37-️-mapa-global))

**Uso en el proyecto**

En el proyecto, este algoritmo se empleó directamente del stack de NAV2 (se puede ver la declaración en [sdv_nav.launch.py](src/sdv_nav/launch/sdv_nav.launch.py)) con lo siguientes parámetros:


<div align="center">

|      Parámetro      |         Valor         |                  Rango típico                 |                       Descripción corta                       |
|:-------------------:|:---------------------:|:---------------------------------------------:|:-------------------------------------------------------------:|
| use_sim_time        | use_sim_time          | true/false                                    | Usa el reloj simulado en lugar del reloj del sistema.         |
| autostart           | False                 | true/false                                    | Determina si AMCL inicia automáticamente.                     |
| base_frame_id       | base_link             | string                                        | Frame base del robot.                                         |
| odom_frame_id       | odom                  | string                                        | Frame de odometría.                                           |
| global_frame_id     | map                   | string                                        | Frame global utilizado para localización.                     |
| laser_frame_id      | cloud                 | string                                        | Frame del sensor láser.                                       |
| scan_topic          | scan                  | string                                        | Tópico del escáner láser.                                     |
| update_min_d        | 0.5                   | 0.1 – 0.5                                     | Distancia mínima para actualizar partículas.                  |
| update_min_a        | 0.2                   | 0.05 – 0.2                                    | Rotación mínima para actualizar partículas.                   |
| recovery_alpha_slow | 0.005                 | 0.001 – 0.01                                  | Tasa lenta para detectar degeneración de partículas.          |
| recovery_alpha_fast | 0.05                  | 0.01 – 0.1                                    | Tasa rápida para acelerar recuperación.                       |
| odom_alpha1         | 0.6                   | 0.1 – 0.5                                     | Error de rotación causado por rotación.                       |
| odom_alpha2         | 0.6                   | 0.1 – 0.5                                     | Error de rotación causado por traslación.                     |
| odom_alpha3         | 0.8                   | 0.1 – 0.5                                     | Error de traslación causado por traslación.                   |
| odom_alpha4         | 0.4                   | 0.1 – 0.5                                     | Error de traslación causado por rotación.                     |
| laser_model_type    | likelihood_field_prob | beam, likelihood_field, likelihood_field_prob | Modelo de sensor empleado.                                    |
| laser_z_hit         | 0.85                  | 0.9 – 0.95                                    | Peso de lectura correcta; menor = más desconfianza del LiDAR. |

</div>

Para probar su correcto funcionamiento, en RViz2 se le colocaba una supoción inicial de la pose del robot, y el mismo algoritmo corregia con las mediciónes del Lidar para determinar la pose real, como se puede observar acontinuación:

<div align="center">
<video src="https://github.com/user-attachments/assets/1fc21658-fa80-4e16-9bbc-869c07fbd3f6">
</div>


### 3.10. 🗃️ Planeación

La planeación es la parte encargada de generar una ruta válida dentro de un mapa o escenaroio, uniendo una poosición inicial con un objetivo indicado dentro del mismo, evitando colisiones y procurando que la distancia entre el inicio y el objetivo sea la más corta.

#### 3.10.1. ⭐ A*
A continuación se presenta mediante una imagen el planteamiento lógico del algoritmo A*, el cual se emplea para generar una ruta desde la posición inicial a un objetivo asignado.

Funcionamiento:
El algoritmo inicia con la obtención de los valores del punto inicial y final dentro de un mapa al que lo descompone en celdas con posiciones libres, ocupadas o inalcanzables, estas celdas contienen cada una un nodo el cual hará parte de la explración en el mapa, a cada celda se le va asignando el valor correspondiente a su estado como se presenta a continuación:

<div align="center">
<img width="402" height="586" alt="OCCUPANCY GRID" src="https://github.com/user-attachments/assets/4400a19e-1cfd-4cc8-bbd9-6bb13bab8b16" />
</div>

Los valores asignados a la grilla son los que permiten al algoritmo hacer la exploración dentro de los espacios libres en busca del objetivo entregado, en el algoritmo se asignan pesos a las diferentes ubicaciones de la grilla además de un valor heuristico que se calcula automaticamente de acuerdo a la ubicación del punto con respecto al objetivo trazado. Se arranca desde la pose inicial hacia el objetivo buscando la ruta de menor peso, pretendiendo optimizar el recorrido realizado por el robot.

<div align="center">
<img width="402" height="586" alt="PATH EXPLORATION"src="https://github.com/user-attachments/assets/d2de1527-47a2-4f79-985b-308da84fd11a" />
</div>

Para el ejemplo la ruta más corta tiene un peso de 23 siendo el menor peso obtenido para los nodos que unen el inicio y el final. El proceso se basa en llegar al nodo siguiente con el menor peso, el nodo actual pasa a nodo visitado y el nodo nuevo pasa a la prioridad en la cola, según la cantidad de rutas existe el mismo valor de nodos en la cabeza de la cola, en la medida que se van visitando las zonas cercanas al nodo actual se van priorizando o decartando nodos dependiendo del peso o la cercania con el objetivo.

#### 3.10.2. 🖥️ Implementación

Dentro del bloque de programacíon el planeador A* es un nodo que lee el mapa global, que será el entorno en el que va a explorar, solicita la pose inicial y el objetivo a alcanzar, con esta información genera un PATH o camino el cual lleva al robot del inicio al objetivo señalado, el path es publicado para que el nodo de control se encargue de hacer que el robot siga la trayectoria. A continuación se presenta un esquema que visualiza la ubicación del algoritmo dentro de la lógica de programación.

<div align="center">
<img width="402" height="586" alt="PATH EXPLORATION"src="https://github.com/user-attachments/assets/96dca1db-f9a2-4df8-b701-d39b7a053fd9" />
</div>

### 3.11. 🕹️ Control

En el nodo de control se toma el vector Twist que contie los valores de velocidad lineal y angular para calcular el valor de PWM que se debe enviar a cada motor para el efectúe el movimiento que se ciña a la trayectoria entregada por el planeador, para la realización de estos cálculos toma como base la cinemática teórica del robot, ya que por su construcción no es posible obtener la lectura de los encoder, además de los valores de modulación en ancho de pulso, también calcula la odometría del robot de acuerdo a los valores medidos de la trocha y los radios de las ruedas.

#### 3.11.1. ⏭️ Pure Pursuit

El algoritmo seleccionado para el seguimiento de trayectoria es el Pure Pursuit, este algoritmo es conocido como el burro y la zanahoria. 
Funcionamineto: el nodo de seguimiento de trayectoria se suscribe al path generado por el planeador, también obtiene la pose del robot para saber dónde está ubicado, es necesario establecer parametros como la distancia frontal para el seguimiento, a continuación inicia con el cálculo de la distancia de la posición del robot con un punto en la ruta a seguir, calcula la distancia euclidina y si esta es mayor al punto de seguimiento genera los comandos de velocidad lineal y angular que se enviarán a los motores para seguir la trayectoria generada. Si la distancia es menor, entonces habrá alcanzado el objetivo. Tambien calcula la curvatura de giro del robot, esto con el fin de suavizar la trayactoria generada y evitar movimientos bruscos del robot, en la siguiente imagen se presenta el radio de curvatura generado para suavizar el movimiento del robot.

<div align="center">
<img width="402" height="586" alt="PATH EXPLORATION"src="https://github.com/user-attachments/assets/744d8c39-70ed-4416-b80a-38d57bb6db81" />
</div>


#### 3.11.2. 🖥️ Implementación

El seguidor es un paquete de ROS2 creado dentro del proyecto y con la implementación del algoritmo descrito, se suscribe a diferentes topicos que tienen la información básica como el path, la pose del robot y el escaneo del sensor. El nodo entrega los comandos de velocidad al nodo de control para su respectiva transformación.

<div align="center">
<img src="https://github.com/user-attachments/assets/c5cb308c-b3e2-4a9e-8980-5e1f13b93c7e"/>
</div>

Adicionalmente, se implementó con el escaner un control reactivo de parada que al presencial un objeto a $25cm$ el robot se detiene para evitar colisiones como se oberva en el siguiente video:

<div align="center">
<video src="https://github.com/user-attachments/assets/709b0390-fcae-4713-aa79-16064e6e9463">
</div>

## 4. 🧪 Resultados

### 4.1. Pipeline obtenido

Para cumplir con la finalidad del proyecto, se logró el siguiente pipeline de robot 

<div align="center">   
<img width="3465" height="1129" alt="Pipeline" src="https://github.com/user-attachments/assets/3000e395-c923-497c-903b-2ca8a2590f14" />
</div>

Aca se divide entre:

* Nivel Alto (Capa de Aplicación): Interfaz e interacción directa con el usuario. En este nivel se envían los comandos de objetivo (por ejemplo, la posición a la que debe llegar el robot).
* Nivel Medio (Capa de Control): Recibe los objetivos definidos en la capa de aplicación y los datos de la capa de hardware para ejecutar los algoritmos de localización, planificación y control necesarios para generar las velocidades de movimiento del robot.
* Nivel Bajo (Capa de Hardware): Ejecuta las velocidades generadas por el nivel de control y realiza la adquisición de datos desde los sensores del robot.

### 4.2. Arbol de TF

Se logró el siguiente arbol de TF:
<div align="center">
<img src="https://github.com/user-attachments/assets/95386f1a-7b97-45f7-8c9f-22ba554af42d" />
</div>

### 4.3. Arquitectura ROS2 Humble

Se logró armar una arquitectura de control en ROS2 Humble, como se puede observar en el siguiente grafo RQT:

<div align="center">
<img src="https://github.com/user-attachments/assets/f2def2b5-0d30-4847-9d71-a11d7e23b67a" />
</div>

En la siguiente tabla se puede ver los nodos diseñados y los tópicos que publica con una breve descripción


|         Nodo        |                     Descripción                    | Tópico [pub] |  Tópico [sub]  |
|:-------------------:|:--------------------------------------------------:|:------------:|:--------------:|
| sdv_controller_node | Calculo de la cinematica inversa y de la odometria | vel2cmd odom |     cmd_vel    |
|   sdv_serial_node   |              Comunicación con hardware             |      NA      |     vel2cmd    |
|     sdv_planner     |      Encontrar camino mas corto hasta objetivo     |     path     |  map pose goal |
|     sdv_tracking    |    lazo de control de sguimiento de trayectoria    | cmd_vel pose | path pose scan |
|      map_server     |     mantener disponible el mapa estático global    |      map     |       NA       |
|      Sick Scan      |                 Controlar el lidar                 |     scan     |       NA       |
|         amcl        |      localizar el robot dentro del mapa global     |     pose     |  map scan odom |

</div>

Por otro lado, acontinuación se puede ver los tópicos desarrollados y una breve descripción:

<div align="center">

| Tópico  | Descripción                                                   |
|---------|---------------------------------------------------------------|
| cmd_vel | velocidad lineal y angular del robot                          |
| vel2cmd | velocidad de cada rueda                                       |
| odom    | odometria teórica a partir de velocidades y medidas del robot |
| map     | mapa global estático                                          |
| pose    | pose del robot (posición + orientación)                       |
| goal    | objetivo del robot                                            |
| path    | trayectoria a seguir                                          |
| scan    | valores de mediciones con lidar                               |

</div>

### 4.4. Robot físico

Para las pruebas de funcionamiento del robot, en primer lugar se realizaron pruebas con una confianza alta en la odometria, logrando lo que se observa en el siguiente video:

Posteriormente, se realizaron cambios en la confianza de la odometria para que no se confie tanto en esta, reduciendo de 0.5 a 0.1 en la pose, con ello se notó una mejoria en lo que se observa en RViz y lo que sucede, como se pude observar acontinuación:

<div align="center">
   <video src='https://github.com/user-attachments/assets/53c12920-27c0-4d89-884f-c6d5e5da8f27'>
</div>

## 5. 🔚 Conclusiones

1. Se logró la operación del robot con ROS2 

## 6. 🔜 Trabajo a futuro

Para trabajo futuro se podría agregar las siguientes mejoras:

* Implementación de IMU para mejorar odometría
* Crear nodo con Hector Mapping para una mejor localización
* Lectura de encoders de motores
* Implementación de odometría nativa del Lidar
* Implementación de algoritmos de Task Planning con IA para toma de decisiones con varios objetivos a alcanzar
* Implementación de cámara esterográfica para localización (Trabajo de grado de maestría del [Ing. Juan Camilo Gomez Robayo](juagomezro@unal.edu.co))

## 7. 📖 Bibliografia

## 8. 📒 Contacto

Para obtener mas información se puede solicitar a:

* <labfabex_fibog@unal.edu.co>
* <antorresca@unal.edu.co>
* <juagomezro@unal.edu.co>
* <reramirezh@unal.edu.co>
* <pfcardenash@unal.edu.co>

