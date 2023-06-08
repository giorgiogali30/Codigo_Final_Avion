
#include <Adafruit_MPU6050.h>          // Incluir Libreria de sensores: acelerometro y giroscopio
#include <Wire.h>
#include <Adafruit_GFX.h>              // Incluir Libreria Grafica de pantalla OLED
#include <Adafruit_SSD1306.h>          // Incluir Libreria del CHIP de pantalla OLED
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>           // Incluir Libreria de sensor barometrico
#include <ESP32Servo.h>                // Incluye la libreria del servo para el ESP32
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
//#include <SPI.h>
//#include <SD.h>

//  Modulo MicroSD  //

//#define SD_CS_PIN 34
//File archivo;
//float pitch, yaw, roll, alturareal2;
//unsigned long tiempo;

// Variables control RC //

#define numero_canales 8
uint64_t pulso_instante[numero_canales * 2 + 2];
uint16_t Mando_canal[numero_canales];
volatile uint8_t contador_flaco = 1;

//  Variables de modo de vuelo  //

int estadosw1;
int estadosw2;
int pinLED_R = 27; // Pin 27 del ESP32 que es un LED ROJO indicador de sensor ultrasonico

// Variables de control manual (RC) (Joystick tipo POT) //

int valueJx1 = 0;
int valueJy1 = 0;
int valueJx2 = 0;
int valueJy2 = 0;
int angle1 = 0;
int angle2 = 0;
int angle3 = 0;
int angle4 = 0;

//  Motor DC  //
int Pinmotor = 19;  // Pin digital del ESP32 al que está conectado el motor

//  Sensor Ultrasonico  //
int TRIG = 23;      // Pin 23 del ESP32 al que esta conectado el TRIG del Sens. Ultra.
int ECO = 26;       // Pin 26 del ESP32 al que esta conectado el TRIG del Sens. Ultra.
int duracion;
int distancia;

//  timer 1 de inicio automatizacion  //
bool timerActive = false;                   // Indica si el temporizador está activo o no
unsigned long timerStartTime = 0;           // Tiempo de inicio del temporizador


//  variables sensor barometrico  //
int altura = 0;
int alturaqro = 1800;
int altura_i;
int alturareal;

Servo alerones; // Los alerones tienen una salida en Y, por lo que comparten la misma señal pero están colocados de forma invertida
Servo estab;
Servo rudder;

Adafruit_MPU6050 srituhobby;
Adafruit_BMP085 bmp;                   // Configura sensor BMP
Adafruit_SSD1306 display(128, 64);


void setup(void) {

 //////////////////// CALIBRACION DEL Control RC y ESP32  ////////////////////////////
  pinMode(2, INPUT);    // Entrada señal del control
  attachInterrupt(digitalPinToInterrupt(2), button_ISR, CHANGE);
  pulso_instante[0] = micros();
  
  /////////////////// Pantalla LED  /////////////////////////

  Conf_DISP();    // Configura e inicia la pantalla LED para imprimir en tiempo la altura real

  ///////////////// Servos  /////////////////

  Serial.begin(115200);
  alerones.attach(4);   // Usa el pin 4 del ESP32 para alerones
  estab.attach(5);      // Usa el pin 5 del ESP32 para estab. horizontal
  rudder.attach(18);    // Usa el pin 18 del ESP32 para rudder

  Wire.begin();
  srituhobby.begin();

  // Set de Inicio Servos
  alerones.write(0);
  estab.write(0);
  rudder.write(0);

  //int servoMin = 2;    // Rango mínimo del servo
  //int servoMax = 178;  // Rango máximo del servo
  //const int anguloObjetivoascenso = 10;


  //////////////// IMU  //////////////////////

  srituhobby.setAccelerometerRange(MPU6050_RANGE_8_G);//2_G,4_G,8_G,16_G
  srituhobby.setGyroRange(MPU6050_RANGE_500_DEG);//250,500,1000,2000
  srituhobby.setFilterBandwidth(MPU6050_BAND_21_HZ);

  ////////////////  Motor DC  ////////////////////////////
  pinMode(Pinmotor, OUTPUT);  // Configurar el pin 19 para Motor DC como salida
  
  ////////////////  Sensor Ultrasonico  /////////////////////////
  pinMode(TRIG, OUTPUT);  // Pin 23 Salida señal
  pinMode(ECO, INPUT);    //Pin 13  Recoge la señal
  pinMode(pinLED_R, OUTPUT);  // PIN 12 del ESP32 para LED de deteccion Sens. Ultra.


  ////////////////  Sensor Barometrico  /////////////////////////
  bmp.begin();  // inicializa sensor BMP Barometrico
  delay(300);  // tiempo mientras se inicializa

  ////////  Modulo MicroSD: Escritura variables /////////
  //Serial.println("Inicializando tarjeta SD...");
  //if (!SD.begin(SD_CS_PIN)) {
    //Serial.println("Fallo en inicializacion de la tarjeta SD");
    //return;
 //}
  //Serial.println("Inicializacion de la tarjeta SD correcta");
  //archivo = SD.open("datos.txt", FILE_WRITE);
  //if (!archivo) {
    //Serial.println("Error al abrir el archivo datos.txt");
    //return;
  //}

  delay(10);
}

void loop() {

  SWITCHES();   //  Funcion para determinar la posicion de los switches y determinar el modo de vuelo

  if (estadosw1 == 1) {  // Switch 1 (sw1 = 1) - ON:   AUTOMATICO [Valor del canal del control: sw1 == 1099]

  /////////////////////   Automatica  ////////////////////////// 
                                                                                     //  Estabilizacion del avion a 0° en CATAPULTA DESDE EL INICIO AUTOMATICO
   //Limpia Indicadores Anteriores
   //digitalWrite(pinLED_R, LOW);        // Sin brillo rojo
   //digitalWrite(pinLED_Y, LOW);        // Sin brillo amarillo 
   //digitalWrite(pinLED_B, LOW);        // Sin brillo azul
   //Indicadores
   //digitalWrite(pinLED_B, HIGH);      // LED Azul - pin 8 - MODO AUTOMATICO
  //////////////////////////////////////////////////////////////

   //  Inicia FASE 1 del modo automatico:  Estabilización a 0° desde el primer momento de activación de este modo  // 

   /* Obtener nuevos eventos del sensor con las lecturas */
   sensors_event_t a, g, temp;
   srituhobby.getEvent(&a, &g, &temp);

   float valorRoll = a.acceleration.y;   //roll
   float valorPitch = a.acceleration.x;  //pitch               // El programa inicia en modo automático estabilizando siempre a 0° para estar siempre horizontal, incluso en catapulta
   float valorYaw = a.acceleration.z;    //yaw

   adjustServos(valorPitch, valorRoll, valorYaw);      // Funcion que estabiliza el vuelo a 0° (estable horizontalmente)

  // 
    //alerones.write(roll);
    //estab.write(pitch);
    //rudder.write(yaw);
    //delay(10);

    //Serial.print("Alerones: ");
    //Serial.print(roll);
    //Serial.print("Estab H.: ");
    //Serial.println(estab);
    //Serial.print("Rudder: ");
    //Serial.println(rudder);
  //

   //  Inicia FASE 2 del modo automatico:  Espera hasta que se dispare la catapulta con el sensor ultrasonico como candado  //

   if (distancia <= 2.54) {      // Si la distancia es menor o igual a 1 pulgada (2.54 cm)
     analogWrite(Pinmotor, 0);       //  motor DC (pin 19 del ESP32)  OFF
     digitalWrite(pinLED_R, HIGH);   // Max brillo rojo - Para checar lo que detecta el sensor
     DISP(); //Despliega la altura en la pantalla LED

    } 
    //  Inicia FASE 3 del modo automatico:  Disparo del avion desde la catapulta  //

   else {
     //  Inicio del motor de la propela
     analogWrite(Pinmotor, 255);  // Encender el motor DC: PROPELA ON (pin 19 del ESP32)

     // INICIA TEMPORIZADOR para dar tiempo a la propela que alcance su max velocidad
     timerStartTime = millis();   // Guardar el tiempo de inicio actual como referencia

     ////////  Protocolo por si se activa el paro de emergencia:
     if (estadosw2 == 1) {             //switch 2 - Paro de emergencia ON (PROPELA OFF)
       analogWrite(Pinmotor, 0);       //  motor DC (pin 19 del ESP32)  OFF
     }
     
     //  Inicia FASE 4 del modo automatico:  el avion tiene el motor DC de la propela a toda potencia (salida de 255) y mientras estabiliza a 0° durante 2 segundos  //

     unsigned long elapsedTime = millis() - timerStartTime;  // Calcular el tiempo transcurrido desde el inicio del temporizador

     if (elapsedTime < 2100) {  // Si no han pasado los 2 segundos aún
       // Estabilizar a 0° durante 2 segundos
       //Estabilizacion del avion a 0° desde que sale de catapulta y hasta que acabe el timer

        /* Obtener nuevos eventos del sensor con las lecturas */
       sensors_event_t a, g, temp;
       srituhobby.getEvent(&a, &g, &temp);

       float valorRoll = a.acceleration.y;   //roll
       float valorPitch = a.acceleration.x;  //pitch               
       float valorYaw = a.acceleration.z;    //yaw
       
       analogWrite(Pinmotor, 255);  // Mantener el motor DC: PROPELA ON (pin 19 del ESP32)
       adjustServos(valorPitch, valorRoll, valorYaw);      // Funcion que estabiliza el vuelo a 0° (estable horizontalmente)
     
       DISP(); //Despliega la altura en la pantalla LED

       delay(10);
     }
     else{ 
       //
       //  Inicia FASE 5 del modo automatico:  transcurridos los 2 segundos con sus debidas tareas, AHORA el avion se inclina a 10° de Pitch hasta alcanzar una altura de 25 metros  //

       while (true){
         /////   Inclinacion del avion a 10° de pitch (encargado el ESTABILIZADOR HORIZONTAL)  ////////
         //  Lectura del sensor
         sensors_event_t a, g, temp;
         srituhobby.getEvent(&a, &g, &temp);

         int valorPitch = a.acceleration.x;  // Obtener el valor de inclinación en el eje X (pitch)
         int valorRoll = a.acceleration.y;  // Obtener el valor de inclinación en el eje Y (roll)
         int valorYaw = a.acceleration.z;

         ajusteASCENSO(valorPitch, valorRoll, valorYaw);

         DISP(); //Despliega la altura en la pantalla LED

         delay(10);

         if (alturareal >= 25) {
          // Salir del ciclo cuando se alcancen los 25 metros
          break;
         }
       }

       //  Inicia FASE 6 del modo automatico:  habiendo llegado y/o superado la altura de 25 metros, ahora el avion vuelve a estabilizar a 0°  //
       //En esta fase se debe estabilizar a 0° en todos los ejes, de modo que el avion este horizontal y estable. Mantiene vuelo recto y nivelado

       // Vuelve a leer los valores para ahora estabilizar todo
       sensors_event_t a, g, temp;
       srituhobby.getEvent(&a, &g, &temp);
       int valorPitch = a.acceleration.x;  // Obtener el valor de inclinación en el eje X (pitch)
       int valorRoll = a.acceleration.y;   // Obtener el valor de inclinación en el eje Y (roll)
       int valorYaw = a.acceleration.z;    // Obtener el valor de inclinación en el eje Z (Yaw)

       //ajustarAnguloServosEstabilizar(valorPitch, valorRoll, valorYaw, anguloObjetivoestable); // Experimental: Mantener vuelo recto y novelado en todos los ejes
       adjustServos(valorPitch, valorRoll, valorYaw);

       DISP(); //Despliega la altura en la pantalla LED

       delay(10);

       //  Inicia FASE 7 (ULTIMA FASE) del modo automatico:  El avión estabiliza a 0° hasta que el piloto haga el cambio a modo manual con el switch del control (sw1 [sw1 == 2]) //
       ////////////////   Cambio de modo AUTOMATICO a MANUAL  ///////////////////////

       if ( estadosw1 == 2 ){       // Switch de cambio a modo manual terminado el proceso automatico
         //
         /////////////////////   Manual  //////////////////////////

         control_RC(); //  Cambia el control completamente a manual
         DISP(); //Despliega la altura en la pantalla LED

         if (estadosw2 == 1) {       // Paro de emergencia de la propela en MODO MANUAL
           analogWrite(Pinmotor, 0); //  motor DC (pin 6) - Propela  OFF
         }
       }
       else{
         //Mantiene Recto y Nivelado indefinidamente
         sensors_event_t a, g, temp;
         srituhobby.getEvent(&a, &g, &temp);
         int valorPitch = a.acceleration.x;  // Obtener el valor de inclinación en el eje X (pitch)
         int valorRoll = a.acceleration.y;   // Obtener el valor de inclinación en el eje Y (roll)
         int valorYaw = a.acceleration.z;    // Obtener el valor de inclinación en el eje Z (Yaw)

         adjustServos(valorPitch, valorRoll, valorYaw);
       }
     }        
      //  //int valueJx1 = analogRead(A0); //Alerones
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
      //  //analogWrite(motorpin, speed); // Velocidad Propela 
    }
    
  }

  else if (estadosw1 == 2) {  // Switch 1 (sw1 = 2): Manual [Valor del canal: (sw1 >= 1590)]

    /////////////////////   Manual  //////////////////////////

    control_RC();

    //int valueJx1 = analogRead(A0); //Alerones
    //int valueJy1 = analogRead(A1); //Estab. horizontal
    //int valueJx2 = analogRead(A2); //Rudder
    //int valueJy2 = analogRead(A3); // Velocidad Propela 

    //angle1 = map(valueJx1, 0, 1023, 2, 178);
    //angle2 = map(valueJy1, 0, 1023, 2, 178);
    //angle3 = map(valueJx2, 0, 1023, 2, 178);
    //int speed = map(valueJy2, 0, 1023, 0, 255);

    //alerones.write(angle1);  //Alerones
    //estab.write(angle2); //Estab. horizontal
    //rudder.write(angle3); //Rudder
    //analogWrite(motorpin, speed); // Velocidad Propela 

  }

  else if (estadosw1 == 0) {    //  MODO INACTIVO
    //Limpia Indicadores Anteriores
    //digitalWrite(pinLED_R, LOW);        // Sin brillo rojo
    //digitalWrite(pinLED_Y, LOW);        // Sin brillo amarillo 
    //digitalWrite(pinLED_B, LOW);        // Sin brillo azul

    // Indicadores -Parpadeo del LED rojo
    //for (int i = 0; i < 5; i++) {
      //digitalWrite(pinLED_R, HIGH);       // Max brillo rojo
      //digitalWrite(pinLED_Y, LOW);        // Sin brillo amarillo 
      //digitalWrite(pinLED_B, LOW);        // Sin brillo azul
      //delay(500);                   

      //digitalWrite(pinLED_R, LOW);        // Sin brillo rojo
      //digitalWrite(pinLED_Y, LOW);        // Sin brillo amarillo 
      //digitalWrite(pinLED_B, LOW);        // Sin brillo azul
      //delay(500);                   
    //}
  }
}

//  Funciones del codigo:   //

    //   Funcion para configurar la pantalla  //
void Conf_DISP() {
  bmp.begin();
  delay(300);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setRotation(0);
  display.setTextWrap(false);
  display.dim(0);
  altura_i= bmp.readAltitude(102960);
}
    //   Funcion para desplegar la pantalla con el valor de la altura en tiempo real //
void DISP(){
  display.clearDisplay();
  display.setFont(&FreeMono9pt7b);
  display.setTextSize(0);
  display.setCursor(0, 10);
  display.println("Tem:");
  display.setCursor(0, 35);
  display.println("Alt:");

  display.setFont(&FreeMonoBold12pt7b);
  display.setCursor(45, 37);
  ALTURA();
  display.println(alturareal);

  display.setCursor(45, 14);
  display.println(bmp.readTemperature());

  display.display();
}

    //  Funcion para Deteccion y medicion de altura: sensor barometrico  //
void ALTURA () {
  altura = bmp.readAltitude(102960);
  alturareal = altura - altura_i; // Calcula la altura estableciendo su posicion inicial desde donde es lanzado
}

    //  Funcion para Deteccion de presencia del sensor ultrasonico  //
void DETECCION() {              
  digitalWrite(TRIG, HIGH);
  delay(1);
  digitalWrite(TRIG,LOW);
  duracion= pulseIn(ECO,HIGH);
  distancia= duracion / 58.2;   //Obtencion distancia del objeto en cm
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

          //   Funcion para fase de ASCENSO  -  Pitch a 10° //

    //  Funcion para mantener direccion en X a inclinacion de 10°  (Inlinacion del giro alrededor del eje X)  //
void ajusteASCENSO(float valorPitch, float valorRoll, float valorYaw) {
            //  PITCH //
  float pitchTarget = 10;  // Ángulo de pitch objetivo (10°)
  if (valorPitch != 0) {
    float pitchError = pitchTarget - valorPitch; // Cálculo del error de pitch
    float estabServoAngle = -pitchError; // Inversión del ángulo para compensar la inclinación
    estab.write(map(estabServoAngle, -10, 10, 2, 178)); // Mapeo del rango de ángulos del servo
  }
            // ROLL //
  if (valorRoll != 0) {
    float rollServoAngle = -valorRoll; // Inversión del ángulo para compensar la inclinación
    alerones.write(map(rollServoAngle, -10, 10, 2, 178)); // Mapeo del rango de ángulos del servo
  }
            // YAW  //
  if (valorYaw != 0) {
    float yawServoAngle = -valorYaw; // Inversión del ángulo para compensar la inclinación
    rudder.write(map(yawServoAngle, -10, 10, 2, 178)); // Mapeo del rango de ángulos del servo
  }
}

 //////////////////////////////////////////////////////////

    //  Funcion para recto y nivelado (TODOS LOS EJES EN 0°)  //

void adjustServos(float valorPitch, float valorRoll, float valorYaw) {
  // Ajuste del servo del estabilizador horizontal
  if (valorPitch != 0) {
    float pitchServoAngle = -valorPitch; // Inversión del ángulo para compensar la inclinación
    estab.write(map(pitchServoAngle, -10, 10, 2, 178)); // Mapeo del rango de ángulos del servo
  }
  // Ajuste del servos de alerones-
  if (valorRoll != 0) {
    float rollServoAngle = -valorRoll; // Inversión del ángulo para compensar la inclinación
    alerones.write(map(rollServoAngle, -10, 10, 2, 178)); // Mapeo del rango de ángulos del servo
  }
  // Ajuste del servo de rudder
  if (valorYaw != 0) {
    float yawServoAngle = -valorYaw; // Inversión del ángulo para compensar la inclinación
    rudder.write(map(yawServoAngle, -10, 10, 2, 178)); // Mapeo del rango de ángulos del servo
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////// Funciones para el RC  /////////////////////////

void SWITCHES(){
  if (contador_flaco == 18) {
    for (int i = 1; i <= numero_canales; i++) {
      Mando_canal[i] = pulso_instante[2 * i] - pulso_instante[2 * i - 1];
      Serial.print(Mando_canal[i]);
      //  Escala  switches y paros de emergencia  //
      int sw1= Mando_canal[5];  //switch 1 [SWC]- 3 posiciones: NADA, AUTOMATICO, MANUAL
      int sw2= Mando_canal[7];  //switch 2 [SWA] - Paro emergencia AMBOS MODOS       //////////////////    ANULA LA POSICION DE ENMEDIO (AUTOMATICA) DEL SWITCH 3 (desde el control prograamdo)
      int value8= Mando_canal[8];  //switch [SWD]: NADA
      
      //  Switch 1 [SWC] - 3 posiciones: MODOS DE VUELO //
      if (sw1 >= 1590 ){ // modo MANUAL
        //digitalWrite(PinLED2, HIGH);
        //digitalWrite(PinLED3, LOW);
        estadosw1=2;
      }
      if (sw1 == 1099 ){ // modo AUTOMATICO
        //digitalWrite(PinLED3, HIGH);
        //digitalWrite(PinLED2, LOW);
        estadosw1=1;
      }
      if(sw1 <= 650){    //modo INACTIVO
        //digitalWrite(PinLED2, LOW);
        //digitalWrite(PinLED3, LOW);
        estadosw1=0;
      }
      // Switch 2 [SWA] - Paro de emergencia AMBOS MODOS // 
      if (sw2 >= 1590 ){    // Paro  ACTIVO
        //digitalWrite(PinLED, HIGH);
        estadosw2=1;
      }
      if(sw2 < 650){        // Paro  INACTIVO
        //digitalWrite(PinLED, LOW);
        estadosw2=0;
      }
      // NO HACE NADA:   Canales 6 [SWC] y 8 [SWD]
      if (value8 >= 1590 ){
        //digitalWrite(PinLED4, HIGH);
      }
      if(value8 < 650){ 
        //digitalWrite(PinLED4, LOW);
      }
    }
  }
}

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

      //  Escala valores servos //
      int alerones1 = map(Mando_canal[1], 590, 1598, 2, 178);
      int elevador = map(Mando_canal[2], 590, 1598, 2, 178);
      int rudder1 = map(Mando_canal[4], 590, 1598, 2, 178);

      //  Escala valor velocidad propela  //
      int speed = map(Mando_canal[3], 605, 1598, 0, 255);

      alerones.write(alerones1);
      estab.write(elevador);
      rudder.write(rudder1);
      analogWrite(Pinmotor, speed); // Velocidad Propela 
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