# simple_adaptive_cruise_control_robot_vehicle
This is an autonomous vehicle that operates using PID control feedback from an ultrasound sensor. Thanks to this, the vehicle can maintain a constant distance from an obstacle in front of it. It is constructed using 3D-printed PLA and the control is based on Arduino.

This is the second project (developed and constructed by Ssebastían Navia, Rodrigo Sepulveda, Benjamín Rojas, Fabián Zamorano) of the "ME4250 Mechatronics" course, taught by Carolina Silva and Leslie Cardenas at the University of Chile. 

<p align="center">
<img src=pics/mainpic.jpeg alt="Texto alternativo" width="25%" height="25%">
</p>

### Resumen

Este vehiculo pretende emular un control crusero adaptaivo mediante los datos obtenidos de un sensor de ultrasnodio. La idea es que el vehiculo pueda mantener una distancia constante con respecto a un objeto que tenga en frente, para esto debe ser capaz de moverse hacia atrás o adelante medainte un control de PID, 

### Materiales y componentes

Materiales chasis:
- Piezas impresas en 3D (3D_printed_parts folder).

Componentes:
- Sensor de ultrasonido.
- x2 motores DC con cajas reductoras.
- x2 Ruedas de 65 mm.
- Puente H L298D.
- Arduino UNO board.
- Módulo LCD I2C de 4 pines.
- x3 Pulsadores.
- x3 Resistencias.

### Diseño del esquema de conexiones

A continuación se muestra el diagrama de conexiones, se debe tener en cuenta que el módulo LCD I2C de 4 pines simplifica el esquema de conexiones de la pantalla que se muestra más abajo, pero esto cuenta para las conexiones de los pulsadores. Se debe considerar también que el módilo I2C solo necesita una conexioón a GND, otra a 5V y las dos que quedan: SCL -> pin A5 y SDA -> A4.

En el código subido a este repositorio, se tiene en cuenta la configuración para él módulo LCD de 4 pines. no se recomienta usar la pantalla sin módulo, puesto que requiere muchos pines dijitales y el proyecto ya contempla bastantes.

<p align="center">
<img src=pics/main_circuit.png alt="Texto alternativo" width="60%" height="60%">
</p>

<p align="center">
<img src=pics/main_lcd.png alt="Texto alternativo" width="60%" height="60%">
</p>
