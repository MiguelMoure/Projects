//-----------------------------Codigo de configuracion---------------------------------------------//
#pragma once

#define     LOOP_TIME             30000 //30s segundos tomando medidas
#define     TX_BUFFER_SIZE        21     //El paquete que se manda es de 20 bytes
#define     WAKE_TIME             300000 //5 min minutos en modo sleep


#define     BatteryPin            12      // Pin 12 para leer el voltaje
#define     VoltagePin            15      // Pin 15 para activar o desactivar la alimentacion
#define     BMEPowerPin           13      // Pin 13 para activar o desactivar el sensor BME280


#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>

Adafruit_BME280 bme;

#define     WindVanePin           4       //Entrada analogica del anemometro


long rotations = 0;                       //Inicializamos las rotaciones del anemometro a 0
long ContactBounceTime=0;                 //Tiempo de rebote del pluviometro a 0
#define     windSensorPin         34      //Entrada digital para contar los pulsos del anemometro
#define     rainSensorPin         36      //Entrada digital para contar los pulsos del pluviometro
#define     rainInterval          1000    //Tiempo de regreso de la cubeta del pluviometro para que no se lean revotes
volatile long tipTime = millis();         //Tiempo que tarda en desbordar la cubeta
volatile long rainTime = 1000000;         //Tiempo que ha tardado entre pulsos, el inicial lo inicializamos a un valor muy alto
RTC_DATA_ATTR volatile int OverflowCount = 0;           //Variable para contar si se han producido dos pulsos del pluviometro
volatile int LLUVIA = 0;  
volatile bool despertadoPorInterrupcion = false;                                       

#include <EEPROM.h>                       // Direcciones en la EEPROM para almacenar los valores
unsigned long TiempoInicial = 0;
unsigned long ContadorHora = 0; 

int count = 0;
int count2 = 0;


#define SEALEVELPRESSURE_HPA (1013.25)    //Definimos una presion a nivel del mar

const lmic_pinmap lmic_pins = {           //Pines utilizamos para la el modulo LoRa
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32} 
};
