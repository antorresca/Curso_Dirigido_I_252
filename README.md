# 📡 Algoritmos de Navegación y Localización - 2025-2 <!-- omit from toc -->

## 🪶 Estudiantes: <!-- omit from toc -->
* Juan Camilo Gomez Robayo
* Andres Camilo Torres-Cajamarca

## 👨‍🏫 Profesores: <!-- omit from toc -->
* PhD. Ing. Ricardo Emiro Ramírez Heredia
* PhD. Ing. Pedro Fabian Cárdenas Herrera

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
  - [3.8. 🌎 Odometría y Localización](#38--odometría-y-localización)
    - [3.8.1. 👣 Odometría](#381--odometría)
    - [3.8.2. 📝 AMCL](#382--amcl)
      - [3.8.2.1. ¿Cómo Funciona?](#3821-cómo-funciona)
      - [3.8.2.2. ¿Cómo se implementa?](#3822-cómo-se-implementa)
    - [3.8.3. 🔥 Hector Mapping](#383--hector-mapping)
      - [3.8.3.1. ¿Cómo Funciona?](#3831-cómo-funciona)
      - [3.8.3.2. ¿Cómo se implementa?](#3832-cómo-se-implementa)
  - [3.9. 🗃️ Planeación](#39-️-planeación)
    - [3.9.1. ⭐ A\*](#391--a)
    - [3.9.2. 🖥️ Implementación](#392-️-implementación)
  - [3.10. 🕹️ Control](#310-️-control)
    - [3.10.1. ⏭️ Pure Pursuit](#3101-️-pure-pursuit)
    - [3.10.3. 🖥️ Implementación](#3103-️-implementación)
- [4. 🥼 Pruebas](#4--pruebas)
  - [Prueba con AMCL y odometria teórica](#prueba-con-amcl-y-odometria-teórica)
- [5. 🧪 Resultados](#5--resultados)
- [6. 🔚 Conclusiones](#6--conclusiones)
- [7. 👨🏼‍🏫 Proceso de aprendizaje](#7--proceso-de-aprendizaje)
- [8. 📖 Bibliografia](#8--bibliografia)

</details>

## 1. 🎯 Objetivos

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

El mapa global generado previamente por medio de SLAM con el SDV y el Lidar en la versión base se puede ver en [](). Para implementarlo en ROS2 se usa **map_server** de **Nav2**, este nodo toma el archivo **.yaml** y genera el tópico ```\map``` para poder consultar el mapa global cuando se requiera. En la siguiente imagen se puede ver el mapa global del laboratorio LabFabEx:

### 3.8. 🌎 Odometría y Localización

Para que el robot pueda navegar en un entorno, es necesario determinar de manera dinámica su posición dentro del mapa. Inicialmente se empleó la Localización Adaptativa de Monte Carlo (AMCL, por sus siglas en inglés). Sin embargo, debido a que el robot no cuenta con encoders disponibles para la lectura de odometría real, se decidió cambiar a Hector Mapping, que permite obtener tanto la odometría como la localización utilizando únicamente el LIDAR.
A continuación, se describen ambos enfoques.

#### 3.8.1. 👣 Odometría

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

#### 3.8.2. 📝 AMCL

##### 3.8.2.1. ¿Cómo Funciona?

AMCL es un método de localización basado en filtros de partículas. Mantiene un conjunto de hipótesis (partículas) sobre la posible posición del robot en el mapa. Cada vez que el robot se mueve, estas partículas se actualizan según el modelo de movimiento (odometría).
Al recibir mediciones del sensor láser, el algoritmo compara estas mediciones con el mapa y ajusta el peso de cada partícula según la coincidencia observada. Finalmente, emplea un proceso de resampling para concentrarse en las partículas más probables, logrando una estimación robusta incluso en presencia de ruido.

##### 3.8.2.2. ¿Cómo se implementa?

Para implementar AMCL en ROS se utiliza principalmente la odometría del robot, el mapa estático, el LIDAR y una pose inicial. En este proyecto, la pose inicial se definió en un punto home ($x=0$, $y=0$) con una orientación predeterminada ($\omega=0$); sin embargo, dicha pose puede configurarse desde **RViz** o desde un nodo externo mediante el tópico ```\InitialPose```.

Se emplean los siguientes tópicos:
* ```\odom``` odometria del robot (Ver mas en [Odometría](#381--odometría))
* ```\scan``` datos del Lidar (Ver mas en [Lidar](#36--lidar))
* ```\map``` Mapa estático (Ver mas en [Mapa Global](#37-️-mapa-global))

#### 3.8.3. 🔥 Hector Mapping

##### 3.8.3.1. ¿Cómo Funciona?

Hector Mapping es un algoritmo de SLAM 2D diseñado para operar únicamente con datos de un sensor láser, sin necesidad de odometría ni IMU. Utiliza un enfoque basado en scan matching, alineando cada escaneo del LIDAR con el mapa que se construye en tiempo real.

Para esto emplea un método de Gradient Descent sobre un mapa de ocupación tipo grid map, buscando la pose que produce la mejor coincidencia entre el escaneo actual y la estructura del entorno ya mapeada. Este proceso se realiza continuamente, lo que permite obtener localización precisa incluso sin encoders.

Gracias a esta estrategia, Hector Mapping resulta especialmente útil en robots con buenos sensores láser pero sin odometría confiable, entregando una estimación estable y fluida únicamente a partir de los datos del LIDAR.

##### 3.8.3.2. ¿Cómo se implementa?

Para implementar Hector Mapping en ROS se utiliza principalmente el tópico del LIDAR (```/scan```) para realizar el scan matching y construir el mapa dinámico.
A diferencia de AMCL, Hector Mapping no requiere odometría, aunque puede usarla opcionalmente si está disponible.

El algoritmo publica la pose estimada del robot y genera un mapa dinámico mientras opera, por lo que en este proyecto cumple simultáneamente el rol de odometría y localización, permitiendo la navegación sin encoders ni modelos de movimiento precisos.

### 3.9. 🗃️ Planeación

#### 3.9.1. ⭐ A*

#### 3.9.2. 🖥️ Implementación

### 3.10. 🕹️ Control

#### 3.10.1. ⏭️ Pure Pursuit

#### 3.10.3. 🖥️ Implementación

## 4. 🥼 Pruebas

### Prueba con AMCL y odometria teórica

Como prueba inicial, se evaluó el robot utilizando AMCL junto con una odometría teórica, tal como se describe en la sección [Odometría y Localización](#38--odometría-y-localización). Para ello, se configuró RViz2 para visualizar en tiempo real los principales tópicos de ROS (pose actual, goal, odometría, TFs, etc.) y compararlos con el comportamiento físico del robot. 

Durante las pruebas, se enviaron dos goals distintos con el fin de analizar la respuesta del sistema y observar si la localización se mantenía estable. Las ejecuciones pueden verse a continuación:

En la mayoría de los casos, el robot presentaba comportamientos incorrectos: ya fuera porque se perdía y comenzaba a realizar movimientos erráticos, o porque detenía su avance antes de tiempo, dado que el sistema estimaba de forma equivocada que ya había alcanzado el objetivo cuando en realidad todavía estaba lejos de él.

## 5. 🧪 Resultados

## 6. 🔚 Conclusiones

## 7. 👨🏼‍🏫 Proceso de aprendizaje

## 8. 📖 Bibliografia

