#include <Adafruit_MPU6050.h>          // Incluir Libreria de sensores: acelerometro y giroscopio
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_GFX.h>              // Incluir Libreria Grafica de pantalla OLED
#include <Adafruit_SSD1306.h>          // Incluir Libreria del CHIP de pantalla OLED
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>           // Incluir Libreria de sensor barometrico
#include <PID.h>
#include <ESP32Servo.h>                // Incluye la libreria del servo para el ESP32

// Variables control RC //
#define numero_canales 8
uint64_t pulso_instante[numero_canales * 2 + 2];
uint16_t Mando_canal[numero_canales];
volatile uint8_t contador_flaco = 1;

//  Variables de modo de vuelo  //

int sw1 = 22;       // Usa pin 22 para switch 1 - AUTOMATICO/MANUAL
int sw2 = 23;       // Usa pin 23 para switch 2 - Paro de emergencia PROPELA modo AUTOMATICO
int sw3 = 24;       // Usa pin 24 para switch 3 - Paro de emergencia PROPELA modo MANUAL
int sw4 = 25;       // Usa pin 25 para switch 4 - Switch de cambio a MODO MANUAL al ACABAR EL MODO AUTOMATICO
int sw5 = 26;       // Usa pin 26 para switch 5 - Switch indicador de comunicacion controlador - RC
int pinLED_B = 8;  // LED AZUL indicador de modos switch
int pinLED_Y = 9;  // LED AMARILLO indicador de modos switch
int pinLED_R = 10; // LED ROJO indicador de comunicacion RC

// Variables de control manual (RC) //

int valueJx1 = 0;
int valueJy1 = 0;
int valueJx2 = 0;
int valueJy2 = 0;
int angle1 = 0;
int angle2 = 0;
int angle3 = 0;

//  Motor DC  //
int motorpin = 6;  // Pin digital al que está conectado el motor

//  Sensor Ultrasonico  //
int TRIG = 12;
int ECO = 11;
int duracion;
int distancia;

//  variables sensor barometrico  //
int altura=0; 
int alturaqro=1820;
int alturareal;

//  timer 1 de inicio automatizacion  //
unsigned long tiempoInicio = 0;  // Variable para almacenar el tiempo de inicio del temporizador
unsigned long duracionTemporizador = 2000;  // Duración deseada del temporizador en milisegundos


Servo alerones;
Servo estab;
Servo rudder;

Adafruit_MPU6050 srituhobby;
Adafruit_BMP085 bmp;                   // Configura sensor BMP

void setup(void) {

 //////////////////// CALIBRACION DEL Control RC y ESP32  ////////////////////////////

  pinMode(2, INPUT);    // Entrada señal del control
  attachInterrupt(digitalPinToInterrupt(2), button_ISR, CHANGE);
  pulso_instante[0] = micros();
  // alerones.attach(4);   // Usa el pin 3 //alerones
  // estab.attach(5);      // Usa el pin 4 //estab. horizontal
  // rudder.attach(18);     // Usa el pin 5 //rudder

  ///////////////// Servos  /////////////////

  Serial.begin(115200);

  alerones.attach(3);  // Usa el pin 3 //alerones
  estab.attach(4);     // Usa el pin 4 //estab. horizontal
  rudder.attach(5);    // Usa el pin 5 //rudder

  Wire.begin();
  srituhobby.begin();

  alerones.write(0);
  estab.write(0);
  rudder.write(0);
  
  //////////////// IMU  //////////////////////

  srituhobby.setAccelerometerRange(MPU6050_RANGE_8_G);//2_G,4_G,8_G,16_G
  srituhobby.setGyroRange(MPU6050_RANGE_500_DEG);//250,500,1000,2000
  srituhobby.setFilterBandwidth(MPU6050_BAND_21_HZ);

  /////////////// MODO DE VUELO (switches) //////////////

  pinMode(sw1, INPUT);  // Usa pin 22 para switch 1 - AUTOMATICO
  pinMode(sw2, INPUT);  // Usa pin 23 para switch 2 - Paro de emergencia modo AUTOMATICO
  pinMode(sw3, INPUT);  // Usa pin 24 para switch 3 - Paro de emergencia modo MANUAL
  pinMode(sw4, INPUT);  // Usa pin 25 para switch 4 - CAMBIO DE MODO MANUAL A AUTOMATICO
  pinMode(sw5, INPUT);  // Usa pin 26 para switch 5 - Indicador comunicacion Controlador - RC

  pinMode(pinLED_B, OUTPUT);  // 8 LED de modos switch 
  pinMode(pinLED_Y, OUTPUT);  // 9 LED de modos switch
  pinMode(pinLED_R, OUTPUT);  // 10 LED de modos switch 

  ////////////////  Motor DC   [Propela]  ////////////////////////////
  pinMode(motorpin, OUTPUT);  // Configurar el pin 6 para Motor DC como salida
  
  ////////////////  Sensor Ultrasonico  /////////////////////////
  pinMode(TRIG, OUTPUT);
  pinMode(ECO, INPUT);

  delay(100);
}

void loop() {

  int estadoSW1 = digitalRead(sw1); // Usa pin 22 para switch 1:  AUTOMATICO-ON/MANUAL-OFF
  int estadoSW2 = digitalRead(sw2); // Usa pin 23 para switch 2 - Paro de emergencia modo AUTOMATICO (propela)
  int estadoSW3 = digitalRead(sw3); // Usa pin 24 para switch 3 - Paro de emergencia modo MANUAL (propela)
  int estadoSW4 = digitalRead(sw4); //Usa pin 25 para switch 4 - Switch de CAMBIO MODO AUTOMATICO A MANUAL (acabado el proceso)
  int estadoSW5 = digitalRead(sw5); //Usa pin 26 para switch 5 - Switch de Comunicacion RC

  if (estadoSW1 == 1) {  // Switch 1 - ON:   AUTOMATICO
                                                                                            ///////  En catapulta: Listo para lanzar ////////
   /////////////////////   Automatica  ////////////////////////// 
   
   //Limpia Indicadores Anteriores
    digitalWrite(pinLED_R, LOW);        // Sin brillo rojo
    digitalWrite(pinLED_Y, LOW);        // Sin brillo amarillo 
    digitalWrite(pinLED_B, LOW);        // Sin brillo azul
   //Indicadores
    digitalWrite(pinLED_B, HIGH);      // LED Azul - pin 8 - MODO AUTOMATICO

   /* Obtener nuevos eventos del sensor con las lecturas */
    sensors_event_t a, g, temp;
    srituhobby.getEvent(&a, &g, &temp);

    int roll = a.acceleration.y;   //roll
    int pitch = a.acceleration.x;  //pitch
    int yaw = a.acceleration.z;  //yaw

    roll = map(roll,  -10, 10, 2, 178);  
    pitch = map(pitch, -10, 10, 2, 178); 
    yaw = map(yaw, -10, 10, 2, 178);

    alerones.write(roll);   //alerones
    estab.write(pitch); //estab. horizontal
    rudder.write(yaw); //rudder
    delay(10);
  
    //Serial.print("Servo1: ");
    //Serial.print(value);
    //Serial.print("Servo2: ");
    //Serial.println(value2);
    //Serial.print("Servo3: ");
    //Serial.println(value3);

    if (distancia <= 2.54) { // Si la distancia es menor o igual a 1 pulgada (2.54 cm)
      analogWrite(motorpin, 0); //  motor DC (pin 6)  OFF
      digitalWrite(pinLED_R, HIGH);   // Max brillo rojo - Para checar lo que detecta el sensor
    } 
    else {
      //  Inicio del motor de la propela
      analogWrite(motorpin, 255); // Encender el motor DC (pin 6)
      tiempoInicio = millis();    //Inicia temporizador de 2 segundos

      if (estadoSW2 == 1) {       // Paro de emergencia de la propela en MODO AUTOMATICO  
      analogWrite(motorpin, 0);   //  motor DC (pin 6)  OFF
      } 

      while (millis() - tiempoInicio < duracionTemporizador){           //  Estabilizacion hasta que el motor alcance su velocidad maxima (dandole 2 segundos de margen)
        
        /* Obtener nuevos eventos del sensor con las lecturas */
       sensors_event_t a, g, temp;
       srituhobby.getEvent(&a, &g, &temp);

       int pitch = a.acceleration.x;  //pitch
       int roll = a.acceleration.y;  //roll
       int yaw = a.acceleration.z;  //yaw
       
       //En esta fase se debe estabilizar a 0° en todos los ejes, de modo que el avion este horizontal y estable
       // Variables de control PID para el alerones
       double kpAleron = 1.0;
       double kiAleron = 1.0;
       double kdAleron = 1.0;
       double setpointAleron = 0.0;
       double prevErrorAleron = 0.0;
       double sumErrorAleron = 0.0;

        // Variables de control PID para el Estabilizador Horizontal
       double kpEstab = 1.0;
       double kiEstab = 1.0;
       double kdEstab = 1.0;
       double setpointEstab = 0.0;
       double prevErrorEstab = 0.0;
       double sumErrorEstab = 0.0;

        // Variables de control PID para el Rudder
       double kpRudder = 1.0;
       double kiRudder = 1.0;
       double kdRudder = 1.0;
       double setpointRudder = 0.0;
       double prevErrorRudder = 0.0;
       double sumErrorRudder = 0.0;

       // Operaciones de control:
       errorAleron= roll - setpointAleron;
       errorEstab= pitch - setpointEstab;
       errorRudder= yaw - setpointRudder;

       // Control PID y Salida para alerones //																
       controlPID_A(errorAleron, kpAleron, kiAleron, kdAleron, prevErrorAleron, sumErrorAleron);    

       // Control PID para estabilizador horizontal //
       controlPID_E(errorEstab, kpEstab, kiEstab, kdEstab, prevErrorEstab, sumErrorEstab);

       // Control PID para Rudder //
       controlPID_R(errorRudder, kpRudder, kiRudder, kdRudder, prevErrorRudder, sumErrorRudder);

      }

      while(alturareal < 25) {  // Ciclo encargado de verificar que el sistema repita el protocolo de ascenso hasta alcanzar una altura de 25 metros

       //En esta fase se debe inclinar el Pitch a 10° y estabilizar a 0° en Roll y Yaw [Y, Z], de modo que el avion este horizontal y estable mientras asciende
       // Variables de control PID para el alerones
       double kpAleron = 1.0;
       double kiAleron = 1.0;
       double kdAleron = 1.0;
       double setpointAleron = 0.0;
       double prevErrorAleron = 0.0;
       double sumErrorAleron = 0.0;

        // Variables de control PID para el Estabilizador Horizontal
       double kpEstab = 1.0;
       double kiEstab = 1.0;
       double kdEstab = 1.0;
       double setpointEstab = 10.0;
       double prevErrorEstab = 0.0;
       double sumErrorEstab = 0.0;

        // Variables de control PID para el Rudder
       double kpRudder = 1.0;
       double kiRudder = 1.0;
       double kdRudder = 1.0;
       double setpointRudder = 0.0;
       double prevErrorRudder = 0.0;
       double sumErrorRudder = 0.0;

       // Operaciones de control:
       errorAleron= roll - setpointAleron;
       errorEstab= pitch - setpointEstab;
       errorRudder= yaw - setpointRudder;

       // Control PID y Salida para alerones //																
       controlPID_A(errorAleron, kpAleron, kiAleron, kdAleron, prevErrorAleron, sumErrorAleron);    

       // Control PID para estabilizador horizontal //
       controlPID_E(errorEstab, kpEstab, kiEstab, kdEstab, prevErrorEstab, sumErrorEstab);

       // Control PID para Rudder //
       controlPID_R(errorRudder, kpRudder, kiRudder, kdRudder, prevErrorRudder, sumErrorRudder);

      }
      
      //  Una vez llega a esa altura se estabiliza nuevamente a 0°

      //En esta fase se debe estabilizar a 0° en todos los ejes, de modo que el avion este horizontal y estable. Mantiene vuelo recto y nivelado
      // Variables de control PID para el alerones
      double kpAleron = 1.0;
      double kiAleron = 1.0;
      double kdAleron = 1.0;
      double setpointAleron = 0.0;
      double prevErrorAleron = 0.0;
      double sumErrorAleron = 0.0;

      // Variables de control PID para el Estabilizador Horizontal
      double kpEstab = 1.0;
      double kiEstab = 1.0;
      double kdEstab = 1.0;
      double setpointEstab = 0.0;
      double prevErrorEstab = 0.0;
      double sumErrorEstab = 0.0;

      // Variables de control PID para el Rudder
      double kpRudder = 1.0;
      double kiRudder = 1.0;
      double kdRudder = 1.0;
      double setpointRudder = 0.0;
      double prevErrorRudder = 0.0;
      double sumErrorRudder = 0.0;

      // Operaciones de control:
      errorAleron= roll - setpointAleron;
      errorEstab= pitch - setpointEstab;
      errorRudder= yaw - setpointRudder;

      // Control PID y Salida para alerones //																
      controlPID_A(errorAleron, kpAleron, kiAleron, kdAleron, prevErrorAleron, sumErrorAleron);    

      // Control PID para estabilizador horizontal //
      controlPID_E(errorEstab, kpEstab, kiEstab, kdEstab, prevErrorEstab, sumErrorEstab);

      // Control PID para Rudder //
      controlPID_R(errorRudder, kpRudder, kiRudder, kdRudder, prevErrorRudder, sumErrorRudder);

     ////////////////   Cambio de modo AUTOMATICO a MANUAL  ///////////////////////

      if ( estadoSW4 == 1 ){
        //
        /////////////////////   Manual  //////////////////////////

        control_RC();

        //Limpia Indicadores Anteriores
        //digitalWrite(pinLED_R, LOW);        // Sin brillo rojo
        //digitalWrite(pinLED_Y, LOW);        // Sin brillo amarillo 
        //digitalWrite(pinLED_B, LOW);        // Sin brillo azul
        //  Indicadores  //
        //digitalWrite(pinLED_Y, HIGH);      // Max brillo Amarillo

        //int valueJx1 = analogRead(A0); //Alerones
        //int valueJy1 = analogRead(A1); //Estab. horizontal
        //int valueJx2 = analogRead(A2); //Rudder
        //int valueJy2 = analogRead(A3); // Velocidad Propela 

        //angle1 = map(valueJx1, 0, 1023, 2, 178);
        //angle2 = map(valueJy1, 0, 1023, 2, 178);
        //angle3 = map(valueJx2, 0, 1023, 2, 178);
        //int speed = map( valueJy2, 0, 1023, 0, 255);

        //alerones.write(angle1);  //Alerones
        //estab.write(angle2); //Estab. horizontal
        //rudder.write(angle3); //Rudder
        //analogWrite(motorpin, speed); // Velocidad Propela 

        if (estadoSW3 == 1) {       // Paro de emergencia de la propela en MODO MANUAL
         analogWrite(motorpin, 0); //  motor DC (pin 6) - Propela  OFF
        }
      }
    }
  }

  else if (estadoSW1 == 0) {  // Switch 1 - OFF = Manual

    /////////////////////   Manual  //////////////////////////

    control_RC();

    //Limpia Indicadores Anteriores
    //digitalWrite(pinLED_R, LOW);        // Sin brillo rojo
    //digitalWrite(pinLED_Y, LOW);        // Sin brillo amarillo 
    //digitalWrite(pinLED_B, LOW);        // Sin brillo azul
    //  Indicadores  //
    //digitalWrite(pinLED_Y, HIGH);      // Max brillo Amarillo

    //int valueJx1 = analogRead(A0); //Alerones
    //int valueJy1 = analogRead(A1); //Estab. horizontal
    //int valueJx2 = analogRead(A2); //Rudder
    //int valueJy2 = analogRead(A3); // Velocidad Propela 

    //angle1 = map(valueJx1, 0, 1023, 2, 178);
    //angle2 = map(valueJy1, 0, 1023, 2, 178);
    //angle3 = map(valueJx2, 0, 1023, 2, 178);
    //int speed = map( valueJy2, 0, 1023, 0, 255);

    //alerones.write(angle1);  //Alerones
    //estab.write(angle2); //Estab. horizontal
    //rudder.write(angle3); //Rudder
    //analogWrite(motorpin, speed); // Velocidad Propela 

    if (estadoSW3 == 1) {       // Paro de emergencia de la propela en MODO MANUAL
      analogWrite(motorpin, 0); //  motor DC (pin 6) - Propela  OFF
    }
  }

  else if (estadoSW5 == 0) {    //  posible Desconexion 
    //Limpia Indicadores Anteriores
    digitalWrite(pinLED_R, LOW);        // Sin brillo rojo
    digitalWrite(pinLED_Y, LOW);        // Sin brillo amarillo 
    digitalWrite(pinLED_B, LOW);        // Sin brillo azul

    // Indicadores -Parpadeo del LED rojo
    for (int i = 0; i < 5; i++) {
      digitalWrite(pinLED_R, HIGH);       // Max brillo rojo
      digitalWrite(pinLED_Y, LOW);        // Sin brillo amarillo 
      digitalWrite(pinLED_B, LOW);        // Sin brillo azul
      delay(500);                   

      digitalWrite(pinLED_R, LOW);        // Sin brillo rojo
      digitalWrite(pinLED_Y, LOW);        // Sin brillo amarillo 
      digitalWrite(pinLED_B, LOW);        // Sin brillo azul
      delay(500);                   
    }
  }
  
  else{
  }
}

//  Sensor Ultrasonico  //

void DETECCION() {              // Deteccion de presencia del sensor ultrasonico
  digitalWrite(TRIG, HIGH);
  delay(1);
  digitalWrite(TRIG,LOW);
  duracion= pulseIn(ECO,HIGH);
  distancia= duracion / 58.2;   //Obtencion distancia del objeto en cm
}

    //  Funcion para Deteccion y medicion de altura: sensor barometrico  //

void ALTURA(){
  altura=bmp.readAltitude(102960);
  alturareal=altura-alturaqro;
}

/////////////////////////////// 	Funciones para PID y Salidas a Superficies 	//////////////////////////////////////								

  //  Alerones (ROLL) //
void controlPID_A(double errorAleron, double kpAleron, double kiAleron, double kdAleron, double prevErrorAleron, double sumErrorAleron ) {
  
  double deltaTime = sampleTime / 1000.0;
  // Calcular las componentes del control PID
  double proportional =kpAleron * errorAleron;
  sumErrorAleron += errorAleron * deltaTime;
  double integral = kiAleron * sumErrorAleron;
  double derivative = kdAleron * (errorAleron - prevErrorAleron) / deltaTime;

  // Calcular la salida del control PID
  double outputAleron = proportional + integral + derivative;

  // Actualizar el error previo
  prevErrorAleron = errorAleron;

  alerones.write(outputAleron);

}

  //  Estabilizador Horizontal (Pitch) //
void controlPID_E(double errorEstab, double kpEstab, double kiEstab, double kdEstab, double prevErrorEstab, double sumErrorEstab ) {
  
  double deltaTime = sampleTime / 1000.0;
  // Calcular las componentes del control PID
  double proportional =kpEstab * errorEstab;
  sumErrorEstab += errorEstab * deltaTime;
  double integral = kiEstab * sumErrorEstab;
  double derivative = kdEstab * (errorEstab - prevErrorEstab) / deltaTime;

  // Calcular la salida del control PID
  double outputEstab = proportional + integral + derivative;

  // Actualizar el error previo
  prevErrorEstab = errorEstab;

  estab.write(outputEstab);

}

  //  Rudder (Yaw) //
void controlPID_Y(double errorYaw, double kpYaw, double kiYaw, double kdYaw, double prevErrorYaw, double sumErrorYaw ) {
  
  double deltaTime = sampleTime / 1000.0;
  // Calcular las componentes del control PID
  double proportional =kpYaw * errorYaw;
  sumErrorYaw += errorYaw * deltaTime;
  double integral = kiYaw * sumErrorYaw;
  double derivative = kdYaw * (errorYaw - prevErrorYaw) / deltaTime;

  // Calcular la salida del control PID
  double outputYaw = proportional + integral + derivative;

  // Actualizar el error previo
  prevErrorYaw = errorYaw;

  rudder.write(outputYaw);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////// Funciones para el RC  /////////////////////////

void control_RC(){
  if (contador_flaco == 18) {
    for (int i = 1; i <= numero_canales; i++) {
      Mando_canal[i] = pulso_instante[2 * i] - pulso_instante[2 * i - 1];
      Serial.print(Mando_canal[i]);
      //ele=Mando_canal(2); //VALUE 2 - ESTABILIZADOR HORIZONTAL
      //ale=Mando_canal(1); //VALUE 1 - ALERONES
      //mot=Mando_canal(3); //VALUE 3 - MOTOR DC (PROPELA)
      //rud=Mando_canal(4); //VALUE 4 - RUDDER
      //
      //                    VALUE 5 - switch 3  ([SWC]) Posicion ARIBA OFF, ENMEDIO - MEDIO (1099), ABAJO - HIGH (1599) 
      //                    VALUE 6 - POTENCIOMETRO [VRA]
      //                    VALUE 7 - Switch 1 ([SWA])  POSICION ARRIBA OFF, ABAJO ON
      //                    VALUE 8 - Switch 4  ([SWD]) POSICION ARRIBA OFF, ABAJO ON
      Serial.print("\t");
      int alerones1 = map(Mando_canal[1], 590, 1699, 2, 178);
      int elevador = map(Mando_canal[2], 590, 1699, 2, 178);
      int rudder1 = map(Mando_canal[4], 590, 1699, 2, 178);
      //int speed = map(Mando_canal[3], 605, 1598, 0, 255);  
      alerones.write(alerones1);
      estab.write(elevador);
      rudder.write(rudder1);
      //analogWrite(Pinmotor, speed); // Velocidad Propela 
      delay(10);
    }
    Serial.println();
  }
}

void button_ISR() {
  if (micros() - pulso_instante[contador_flaco - 1] > 2500) contador_flaco = 0;
  pulso_instante[contador_flaco] = micros();
  contador_flaco++;
}

