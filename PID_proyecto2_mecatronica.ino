//Librerias para la pantalla LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Inicializa el objeto del LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Dirección del LCD y tamaño de pantalla (16 columnas y 2 filas)

//asignacion de pines de lops botones de control del LCD
#define menos 6
#define select 7
#define mas 8

//asignación de pines del puente H
const int pinE1 = 3;     // Conexión del pin Enable 1 del puente H
const int pinE2 = 5;     // Conexión del pin Enable 2 del puente H
const int pinI1 = 2;     // Conexión del pin Input 1 del puente H
const int pinI2 = 11;    // Conexión del pin Input 2 del puente H
const int pinI3 = 12;    // Conexión del pin Input 3 del puente H
const int pinI4 = 13;    // Conexión del pin Input 4 del puente H

//Asignación de pines del sensor ultrasónico
const int PinTrig = 10;  // Conexión del pin Trig del sensor ultrasónico
const int PinEcho = 9;   // Conexión del pin Echo del sensor ultrasónico

//definición de valores para el control PID
float setpoint; // Distancia deseada respecto a la pared

const float Kp = 20;     // Constante proporcional del PID
const float Ki = 0;      // Constante integral del PID
const float Kd = 45;     // Constante derivativo del PID

float lastError = 0;
float integral = 0;

//valores para el ultrasonico
const float VelSon=34000.0;
float distancia;

//variables de control manipulables desde el LCD
int actual = 0;
int objetivo = 0;


void setup() {
  
  //pines de botones de control LCD
  pinMode(menos, INPUT);
  pinMode(select, INPUT);
  pinMode(mas, INPUT);

  // Inicializa la comunicación I2C y la lantalla
  Wire.begin();
  lcd.begin(16, 2); 
  // Configura el retroiluminación del LCD (opcional)
  lcd.backlight();

  //iniciar comunicacion por USB
  Serial.begin(9600);

  //pines puente H
  pinMode(pinE1, OUTPUT);
  pinMode(pinE2, OUTPUT);
  pinMode(pinI1, OUTPUT);
  pinMode(pinI2, OUTPUT);
  pinMode(pinI3, OUTPUT);
  pinMode(pinI4, OUTPUT);

  //pines del sensor ultrasonico
  pinMode(PinTrig, OUTPUT);
  pinMode(PinEcho, INPUT);

  //Pre-asignar valores de entrada al control
  actual = 10;
  objetivo = 10;
  
}

void loop() {

  //para manipular los valores desde la pantalla
  pantalla(); //funcion definida más abajo.

  //se define como setpoint el valor actual actualizable desde el LCD
  setpoint = actual;


  // Medir la distancia actual
  distancia=iniciarTrigger();

  // Calcular el error
  float error = setpoint - distancia;


  // Calcular los términos PID
  float proportional = Kp * error;
  integral += Ki * error;
  float derivative = Kd * (error - lastError);

  // Calcular la señal de control
  float controlSignal = proportional + integral + derivative;
 
  //limite de variables
  controlSignal = constrain(controlSignal, -120, 120);
  objetivo = constrain(objetivo, 10, 200);
  actual = constrain(actual, 10, 200);
  

  // Ajustar la velocidad de los motores según la señal de control
  setMotorSpeed(controlSignal);

  // Actualizar el error anterior
  lastError = error;

  // Mostrar información por el puerto serie
  Serial.print("Distance: ");
  Serial.print(distancia);
  Serial.print(" cm   Setpoint: ");
  Serial.print(setpoint);
  Serial.print(" cm   Control: ");
  Serial.print(controlSignal);
  Serial.println();

  delay(50);  //periodo de retroalimentación
}


//función que permite ver y actualizar mediante botones el setpoint
void pantalla() {
  if (digitalRead(mas) == HIGH) {
    objetivo += 2;
  }
  else if (digitalRead(menos) == HIGH) {
    objetivo -= 2;  
  }
  else if (digitalRead(select) == HIGH) {
    actual = objetivo; 
    delay(1000); 
  }
  
  //Control de pantalla LCD.
  //estados
  lcd.setCursor(0,0);          
  lcd.print("Actual:"); 
  lcd.setCursor(0,1);           
  lcd.print(" Nuevo:"); 
  //numeros
  lcd.setCursor(8,0);          
  lcd.print(actual); 
  lcd.setCursor(8,1);           
  lcd.print(objetivo); 
  //unidades de medida
  lcd.setCursor(11,0);          
  lcd.print("[cm]"); 
  lcd.setCursor(11,1);          
  lcd.print("[cm]"); 
}


//medir distancia con el sensor ultrasonico.
float iniciarTrigger(){
	digitalWrite(PinTrig,LOW);
	delayMicroseconds(200);
	digitalWrite(PinTrig,HIGH);
	delayMicroseconds(100);
	digitalWrite(PinTrig,LOW);
 	unsigned long tiempo= pulseIn(PinEcho,HIGH);
	distancia=tiempo*0.000001*VelSon/2.0;
	return(distancia);
}

//control de velocidad de los motores en base al valor de control.
void setMotorSpeed(float controlSignal) {
  // Ajustar la velocidad de los motores
  //ir hacia adelante...
  if (controlSignal > 0) {
    analogWrite(pinE1, 0.8*controlSignal);
    analogWrite(pinE2, controlSignal);
    digitalWrite(pinI1, LOW);
    digitalWrite(pinI2, HIGH);
    digitalWrite(pinI3, LOW);
    digitalWrite(pinI4, HIGH);
    
  } 
  //ir hacia atrás...
  else {
    controlSignal = -controlSignal;
    analogWrite(pinE1, 0.8*controlSignal);
    analogWrite(pinE2, controlSignal);
    digitalWrite(pinI1, HIGH);
    digitalWrite(pinI2, LOW);
    digitalWrite(pinI3, HIGH);
    digitalWrite(pinI4, LOW);
  }
}
