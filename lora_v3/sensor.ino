//----------------------------Codigo de Sensores y creacion del paquete para mandar por LoRa---------//
//Inclusion de las librerias necesarias
#include "configuration.h"
#include "rom/rtc.h"
#include <EEPROM.h>
#include "esp_attr.h"
//Funcion para Leer la temperatura del sensor BME280
float bmeTemp() {
  // Obtener y mostrar la temperatura
  float temp = bme.readTemperature();
  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" °C");
  return temp;
}
//Funcion para leer la presion
float bmePres() {
  // Obtenemos y printeamos la presion en hpa
  float pressure = bme.readPressure();
  Serial.print("Pressure = ");
  Serial.print(pressure / 100.0F);
  Serial.println(" hPa");
  return pressure;
}
//Funcion para medir la altitud
float bmeAlt() {
  // Obtenemos y printeamos la altitud en metros
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float height = altitude + 196;
  Serial.print("Approx. Altitude = ");
  Serial.print(height);
  Serial.println(" m");
  return height;
}
//Funcion para leer la humedad
float bmeHum() {
  // Obtenemos y printeamos el porcentaje de humedad
  float hum = bme.readHumidity();
  Serial.print("Humidity = ");
  Serial.print(hum);
  Serial.println(" %");
  return hum;
}
//Funcion para Leer la direccion del viento
int windDirection() {
  int vaneValue = analogRead(WindVanePin); //Leemos el valor analogico que entra en el pin4
  int windDirection = map(vaneValue, 0, 3800, 0, 360); //Lo transformamos a un valor entre 0 y 360 grados
  int windCalDirection = windDirection;
  if (windCalDirection > 360) windCalDirection = windCalDirection - 360; //Correccion para valores mayores o menores de 360
  if (windCalDirection < 0) windCalDirection = windCalDirection + 360;

  //Creacion  del string para visualizar la abreviacion del la direccion del viento
  String windCompassDirection = " ";
  if (windCalDirection < 22) windCompassDirection = "N";
  else if (windCalDirection < 67) windCompassDirection = "NE";
  else if (windCalDirection < 112) windCompassDirection = "E";
  else if (windCalDirection < 157) windCompassDirection = "SE";
  else if (windCalDirection < 212) windCompassDirection = "S";
  else if (windCalDirection < 247) windCompassDirection = "SW";
  else if (windCalDirection < 292) windCompassDirection = "W";
  else if (windCalDirection < 337) windCompassDirection = "NW";
  else windCompassDirection = "N";


  Serial.print("Wind direction: ");
  Serial.print(windCalDirection);
  Serial.println(windCompassDirection);
  return windCalDirection;
}
//Funcion Velocidad del viento
float windSpeed(){
  //Calculamos la velocidad con la formula del fabricante
  float windSpeed = rotations*2.25/30;
  rotations = 0; //Reiniciamos rotaciones para futuras lecturas
  windSpeed = windSpeed * 1.61;  //Transformacion a Km/h
  Serial.print("Velocidad del viento =");
  Serial.print(windSpeed);
  Serial.println("km/h");
  return windSpeed;
}
//Funcion de interrupcion anemometro
void rotate() {
  if ((millis() - ContactBounceTime) > 15) { //Dejamos un tiempo de 15ms para no leer rebotes
    rotations++;                             //Aumentamos el nuemro de rotaciones
    ContactBounceTime = millis();            //Guardamos el tiempo para comprobar rebotes en la sigueinte lectura
  }
}

//Funcion para obtener la precipitacion
float rainRate() {
  
  float precipitacion = (0.2 * 3600000) / rainTime; //La precipitacion se calula como 0.2mm de agua de la cubeta *1h dividido entre el tiempo entre pulsos
   
  if (precipitacion < 1)  {
    precipitacion = 0;
  }
  Serial.print("Rainrate");
  Serial.println(precipitacion);
  return precipitacion;
}

//Funcion interrupcion del pluviometro
void rain() {
  LLUVIA = 1;
}

//Funcion para leer la bateria
float ReadBattery() {

  int analogBat = analogRead(BatteryPin);//Se lee el valor analogico en el pin 12 
  Serial.print(analogBat);
  float digitalBat = (analogBat - 0) * (2.1 - 0) / (2450 - 0); //Como el voltaje esta dividido por un divisor solo leemos la mitad de los valores
  float battery = digitalBat * 2; //Multiplicamos por 2 para obtener el valor real
  Serial.print(" Bateria Real ");
  Serial.print(battery);

  return battery;
}

//Funcion para crear paquete
void doSensor(uint8_t txBuffer[]) {
  // Llenar el búfer con caracteres nulos para borrar el contenido anterior
  memset(txBuffer, 0, TX_BUFFER_SIZE); //Creamos un paquete del numero de bytes elegido en configuracion

  float battery = ReadBattery();
  sprintf((char*)txBuffer, "B:%d", battery);
  
  //Para crear los bytes pasamos todos los valores a enteros y usamos la cantidad de bytes necesarios con cada medida 
  float t = bmeTemp();
  int shiftTemp = int(t * 100);
  txBuffer[0] = byte(shiftTemp);
  txBuffer[1] = shiftTemp >> 8;


  float p = bmePres();
  int shiftpresion = int(p * 100);
  txBuffer[2] = byte(shiftpresion);
  txBuffer[3] = shiftpresion >> 8;
  txBuffer[4] = shiftpresion >> 16;
  txBuffer[5] = shiftpresion >> 32;


  float a = bmeAlt();
  int shiftAltura = int(a * 100);
  txBuffer[6] = byte(shiftAltura);
  txBuffer[7] = shiftAltura >> 8;


  float h = bmeHum();
  int shifthumedad = int(h * 100);
  txBuffer[8] = byte(shifthumedad);
  txBuffer[9] = shifthumedad >> 8;

  float d = windDirection();
  int shiftdirviento = int(d * 100);
  txBuffer[10] = byte(shiftdirviento);
  txBuffer[11] = shiftdirviento >> 8;
  txBuffer[12] = shiftdirviento >> 16;

  float v = windSpeed();
  int shiftVel = int(v * 100);
  txBuffer[13] = byte(shiftVel);
  txBuffer[14] = shiftVel >> 8;


  float r = rainRate();
  int shiftprecipitacion = int(r * 100);
  txBuffer[15] = byte(shiftprecipitacion);
  txBuffer[16] = shiftprecipitacion >> 8;
  txBuffer[17] = shiftprecipitacion >> 16;
  

  int shiftVoltage = int(battery * 10);
  txBuffer[18] = byte(shiftVoltage);

  Serial.print("LLUVIA: ");
  Serial.println(LLUVIA);
  txBuffer[19] = byte(LLUVIA);
}
