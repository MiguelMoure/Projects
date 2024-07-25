
//---------------------Codigo principal-------------------------------//
#include <lmic.h>
#include <hal/hal.h>
#include "credentials.h"
#include "configuration.h"
#include <esp_sleep.h>
#include "rom/rtc.h"
#include "driver/rtc_io.h"
#include "esp_attr.h"



//Variable del numero de bytes que se mandan con LoRa
static uint8_t txBuffer[TX_BUFFER_SIZE]; 

int t1;
int t2;

//Funcion que muestra si el dispositivo se conecta al GateWay y envia el paquete de datos
void onEvent (ev_t ev) {
    switch(ev) {
        //Si no se encuentra el gateway
        case EV_SCAN_TIMEOUT:                         
            Serial.println(F("EV_SCAN_TIMEOUT"));     
            break;
        //Se encuentra el gateway
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));    
            break;
        //Se pierde el Gateway
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));    
            break;
        //Se rastrea el Gateway
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));   
            break;
        //Conexion con el gateway
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));          
            break;
        case EV_JOINED:
            //Conectado con el gateway
            Serial.println(F("EV_JOINED"));           
            {                     
              //Se Declaran variables para almacenar        
              // ID de la red           
              u4_t netid = 0;            
              //Direccion del dispositivo             
              devaddr_t devaddr = 0;    
              //Clave de sesion de Red              
              u1_t nwkKey[16];                  
              //Clave de la sesion de la aplicacion      
              u1_t artKey[16];                        
              //Obtenemos las claves desde el stack LoraWAN
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey); 
              //Se imprime el ID
              Serial.print("netid: ");                
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              //Se imprime la direccion del dispositivo
              Serial.println(devaddr, HEX);          
              //Se imprime la clave de sesion de la aplicacion AppKey 
              Serial.print("AppSKey: ");              
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
              //Hay que imprimir cada byte en formato hexadecimal
                printHex2(artKey[i]);                 
              }
              Serial.println(""); 
              //Se imprime la clave de sesion de Red NwkSKey
              Serial.print("NwkSKey: ");              
              for (size_t i=0; i<sizeof(nwkKey); ++i) { 
                      if (i != 0)
                              Serial.print("-");
                      //Cada byte en formato hexadecimal
                      printHex2(nwkKey[i]);           
              }
              Serial.println();
            }
          //Se establece el modo de verificacion (Desactivada)
            LMIC_setLinkCheckMode(0);
            break;
        //Si la union con el servidor LoRa falla
        case EV_JOIN_FAILED:                                             
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        //Si la reintegracion con el servidor LoRa falla
        case EV_REJOIN_FAILED:                        
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        //Transmision completada
        case EV_TXCOMPLETE:                           
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Recibido ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Recibido"));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes de payload"));
            }
            
            Serial.println("GO TO SLEEP");
            //Se manda a dormir el microcontrolador el tiempo especificado 
            sleep_millis(WAKE_TIME);                  
            break;

        //Si se pierde la sincronizacion de tiempo
        case EV_LOST_TSYNC:                           
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        //Evento de reseteo
        case EV_RESET:
            Serial.println(F("EV_RESET"));            
            break;
        //Si se completa la recepcion de datos
        case EV_RXCOMPLETE:                           
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        //Si se ha perdido la conexion
        case EV_LINK_DEAD:                            
            Serial.println(F("EV_LINK_DEAD"));
            break;
        //Si la conexion esta activa
        case EV_LINK_ALIVE:                           
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        //Comienza la transmision
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));          
            break;
        //Transimision cancelada
        case EV_TXCANCELED:                           
            Serial.println(F("EV_TXCANCELED"));
            break;
        // Comienza la recepcion de datos
        case EV_RXSTART:                              
            
            break;
        //Se completa la transimision de union sin conectarse
        case EV_JOIN_TXCOMPLETE:                      
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;
        //Event desconocido
        default:
            Serial.print(F("Unknown event: "));       
            Serial.println((unsigned) ev);
            break;
    }
}
//Funcion para imprimir un valor en formato decimal 
void printHex2(unsigned v) {
    //Limitamos el valor a un byte
    v &= 0xff;                             
    //Si el valor es menor que 16, imprimir un 0 para asegurar que no haya dos digitos en hexadecimal            
    if (v < 16)                                       
        Serial.print('0');
        Serial.print(v, HEX); 
    //Imprimir el valor en formato hexadecimal
}
                                 
//Funcion que manda el paquete de datos a traves de LoRa
void do_send(){
    //Verificar si no hay una transimision o recepcion en curso
    if (LMIC.opmode & OP_TXRXPEND) {                  
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        //Preparar la transmision de datos en proximo momento posible
        LMIC_setTxData2(1, txBuffer, sizeof(txBuffer)-1, 0); 
        Serial.println(F("Packet queued"));
    }
    // Siguiente transimision programada despues del evento TX_COMPLETE
}
//Inicializacion
void setup() {
    //Inicializacion de la comunicacion serial a 115200 baudios
    Serial.begin(115200);     
    Serial.println("Starting");
    TiempoInicial = millis(); 

    //Configuracion de las salidas del microcontrolador 

    pinMode(VoltagePin, OUTPUT);
    digitalWrite(VoltagePin, HIGH);
    
    pinMode(BMEPowerPin, OUTPUT);
    digitalWrite(BMEPowerPin, HIGH);

    delay(1000);
    //Mensaje para indicar que la alimentacion esta encendida
    Serial.println("Power On");   
    //Configuracion de la interrupcion digital del anemometro
    pinMode(windSensorPin, INPUT); 
    //Se asocia la interrupcion del anemometro a la funcion Rotate en el flanco de bajada
    attachInterrupt(windSensorPin, rotate, FALLING); 
    //Configuracion de la interrupcion digital del pluviometro con resistencia del pull-up
    pinMode(rainSensorPin, INPUT_PULLUP);
    //Se asocia la interrupcion del pliometro con la funcion Rain en el flanco de bajada
    attachInterrupt(digitalPinToInterrupt(rainSensorPin), rain, FALLING); 
    
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, 1);

    //Inicializacion de la Comunicacion Lora
    os_init();          
    LMIC_reset();

    //Inicializacion del sensor BME280   
    unsigned status;     
    status = bme.begin(0x76);

    //Se crea una variable que contiene el tiempo actual y el valor actual
    t1 = millis();  
    count2 = count;

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason) {

    case ESP_SLEEP_WAKEUP_EXT0:
    LLUVIA = 1;
    Serial.print("LLUVIA_INTR: ");
    Serial.println(LLUVIA);
    break;
    case ESP_SLEEP_WAKEUP_TIMER:
    Serial.print("Han pasado 5 minutos ");
    break;
    default:
    Serial.println("No sé porqué desperté");
    break;
  }
}
void loop() {
  
    os_runloop_once();//Ejecucion del procesador del modulo LoRa

    int t2 = millis();//Se crea una variable del tiempo actual 
    
    if((t2-t1)>LOOP_TIME){ //Si ha pasado el tiempo de ejecucion de loop entramos en la funcion

      t1 = millis(); //Guardartel tiempo actual en T1
      doSensor(txBuffer); // Llama a la función doSensor con txBuffer
      Serial.println("Sending"); // Agrega una nueva línea después de imprimir "Sending"
      do_send(); // Envía el paquete con la función do_send()
    }
  
}
