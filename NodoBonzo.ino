/******************************************************** 
*************** PROTOTIPO NODO BONZO *******************
********************************************************/

#include <Wire.h>
#include <Adafruit_BME280.h>
#include <DS3231_Simple.h>
#include "LowPower.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


#define PIN_ALARM 3       // Indicamos el pin de interrupción (INT1) que usaremos para despertar el Arduino mediante las alarmas del RTC
#define PERIFERICOS_ON 5  // Pin para activar la alimentación de los periféricos
#define CONVERSOR_ON 4    // Pin para activar el conversor de tensión y que se alimente el Arduino a 3,3 V
#define PIN_OLED A0       // Pin de la pantalla OLED
#define min_envio 2       // Variable que indica cada cuantos minutos queremos enviar datos
#define delta_temp 5      // Delta de Tª a partir del cual consideramos que hay un incendio
#define t_alerta 55       // Tª a partir de la cual se considera que hay fuego

// Claves del nodo creado en The Thing Network
//**** Hay que rellenar los campos con las claves obtenidas para el dispositivo ****
static const PROGMEM u1_t NWKSKEY[16] = {};
static const u1_t PROGMEM APPSKEY[16] = {};
static const u4_t DEVADDR = ;

// Para comunicación OTA. Aunque no se usen es necesario dejarlas.
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// Variables de envío de datos a TTN
static osjob_t sendjob;
static osjob_t initjob;

// Variables booleanas para controlar estados
bool joined = false;        // Indica si se ha establecido comunicación
bool sleeping = false;      // Indica en que estado se encuentra el Arduino
bool envio = false;         // Indica si hay que enviar datos o no
bool alarma_2 = true;       // Indica si el Arduino se ha despertado a causa de la alarma2 del reloj (alarma de envío de datos)
bool alarma_fuego = false;  // Indica si se ha detectado un delta de temperatura superior al establecido

//Creamos un objeto de tipo RTC y BME
DS3231_Simple Clock;
Adafruit_BME280 bme; // I2C

// Variables de los sensores para el envío de datos
uint16_t t_value, p_value, b_value; 
float h_value, t_value_anterior;
DateTime MyDateAndTime;

// Indicamos el canal y data rate a usar
int channel = 0;
int dr = DR_SF7;

// Pines a los que está conectada la antena
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
};


void setup() {
    //Serial.begin(115200); // Lo comentamos para que funcione con las baterías
    delay(250);
    Serial.println(F("Starting"));

    digitalWrite(PIN_OLED, LOW);  // Deshabilitamos la pantalla OLED para ahorrar batería
    // Encendemos el BME280 
    iniciar_bme();
    
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are to be provided.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

    // Configuración de los distintos canales de envío
    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    forceTxSingleChannelDr();

    // Iniciamos el reloj
    Clock.begin();
       
    //Configuramos la alarma
    config_alarma();
    delay(500);    
    
 }

void loop() {  

  // Los números indican el orden en el que se ejecuta el código una vez el Arduino ha despertado
    
  // 2) Desactivamos las interrupciones para que se hagan las operaciones necesarias sin que sean interrumpidas por posibles alarmas del reloj
  detachInterrupt(PIN_ALARM);

  // 3) Leemos los datos de los sensores y comprobamos el delta de temperatura respecto de la lectura anterior
  
  lectura_sensores();

  // 4) Comprobamos si hay que enviar datos: o bien ha saltado la alarma de envío de datos;
  // o se ha detectado un delta de temperatura mayor del límite establecido  
  if(envio == true){
    
    do_send(&sendjob);    // Sent sensor values
    
    while(sleeping == false)
        {
          os_runloop_once();
        }
    sleeping = false;
    envio = false;
  }
  
  // 5) Es necesario actualizar la alarma en cada loop
  config_alarma();

  // 6) Volvemos a activar las interrupciones  
  attachInterrupt(digitalPinToInterrupt(PIN_ALARM), despertar_arduino, FALLING);
  
  // 7) Dormimos el Arduino   
  // Cuando se detecte una interrupción HW provocada por la alarma del RTC el Arduino despertará.
  // Desactivamos la alimentación de los sensores y el conversor
                                         
  Serial.println(F("--DORMIMOS ARDUINO--"));
  delay(500);
  digitalWrite(PERIFERICOS_ON, LOW);   // Desactivamos la alimentación de los periféricos
  digitalWrite(CONVERSOR_ON, LOW);   // Desactivamos el conversor de tensión
                                       // Lo comentamos mientras se alimente mediante USB
                                             
  delay(500);  // Damos suficiente tiempo para que todo se haya configurado y enviado antes de dormirlo
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

  // 1) El Arduino se despierta, activa la alimentación de los periféricos
  // y comprueba qué alarma lo ha despertado
  
  // Encendemos el BME280 
  iniciar_bme();
  delay(500);      
  comprobar_alarmas();          
 }

// Función que salta cuando se produce la interrupción HW
void despertar_arduino(){
  Serial.println(F("--DESPERTAMOS ARDUINO--"));  
 }
 
void iniciar_bme(){
  //Comprobamos la correcta lectura del sensor BME280        
    bool status;
    
    digitalWrite(CONVERSOR_ON, HIGH);   // Activamos el conversor de tensión
                                        // Lo comentamos mientras se alimente mediante USB 
    // Activamos la alimentación de los periféricos
    digitalWrite(PERIFERICOS_ON, HIGH);
    // Dejamos tiempo para que se estabilice antes de iniciarlo
    delay(1000);  
          
    status = bme.begin();  
    if (!status) {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
        while (1);
    }
}
// Función que configura las alarmas  
void config_alarma(){

  // Al final del código, lista completa con todos los tipos de alarma: https://github.com/sleemanj/DS3231_Simple/blob/master/examples/z2_Alarms/Alarm/Alarm.ino
     
  // Timestamp para configurar cada alarma. Leemos la hora actual
  DateTime al_1 = Clock.read();
  DateTime al_2 = Clock.read();

  // Queremos que la alarma "1" se despierte cada segundo "30" de cada minuto
  al_1.Second = 30;       
  // Configuramos la alarma "1"  
  Clock.setAlarm(al_1, DS3231_Simple::ALARM_MATCH_SECOND);
  
  // Configuramos la alarma "2". Ocurre solo cuando el Arduino se ha despertado a causa de la alarma 2
  // y cuando se ejecuta el código por primera vez 
  if(alarma_2 == true){
    al_2.Minute += min_envio; // La configuramos para que salte dentro de x minutos

    // Si el resultado de la suma es mayor de 60 (cambio de hora) calculamos los minutos equivalentes en la siguiente hora
    if(al_2.Minute >= 60){
      al_2.Minute -= 60;     
      }      
    
    // Configuramos la alarma "2" y la ponemos a "false" para que no se vuelva a configurar hasta que vuelva a saltar
    Clock.setAlarm(al_2, DS3231_Simple::ALARM_MATCH_MINUTE); //Configuramos la alarma 2   
    alarma_2 = false;  
    }
   
 }

// Función que comprueba si se ha activado alguna alarma
void comprobar_alarmas(){
  
  uint8_t AlarmsFired = Clock.checkAlarms();
  
  // Comprobamos cuál de las dos alarmas ha despertado al Arduino
  if(AlarmsFired & 1)
  {
    Clock.printTo(Serial); Serial.println(": First alarm has fired!");            
  }
  // Si es la segunda alarma la que ha saltado (envío de datos) ponemos la variable "envio"
  // a true para que en la función de envío se manden los datos, y ponemos a true también la
  // variable "alarma_2" para que en la función "config_alarma" esta sea configurada de nuevo
  if(AlarmsFired & 2)
  {
    Clock.printTo(Serial); Serial.println(": Second alarm has fired!");
    envio = true;
    alarma_2 = true;    
  }
    
 }

// Función que lee los valores recogidos por los sensores y comprueba el
// delta de temperatura
void lectura_sensores(){
  
  // LECTURA Y COMPROBACIÓN DE DATOS TEMPERATURA 
  t_value = constrain(bme.readTemperature(),-24,80);    // Dado que no nos interesan los valores fuera de ese rango
                                                        // limitamos el rango de temperatura                                                                                          
  
  // Si hay un delta de "delta_temp" grados respecto a la lectura de hace un minuto probablemente haya fuego
  Serial.println(F("Comprobamos delta temperatura"));            
  if( (t_value >= t_value_anterior + delta_temp) || t_value >= t_alerta){
    envio = true;   // Si hay fuego enviamos los datos
    alarma_fuego = true;  // Este es el valor que se manda en el primer byte del payload
                          // Se usará para lanzar triggers de alerta en TTN
    Serial.println(F("*** FUEGO!! ***"));        
    }
  else{
    alarma_fuego = false;
    }
  // Almacenamos la temperatura leida para comprobarla la próxima vez que se despierte el Arduino  
  t_value_anterior = t_value;

  // Preparamos el dato de temperatura para su envío
  t_value = uint16_t(t_value * 10); //multiplicamos por 10 para conservar un decimal 
  
  // Leemos la presión atmosférica
  p_value = uint16_t(bme.readPressure()/10); // Lo enviamos en hPa conservando un decimal

  // Leemos la humedad relativa
  h_value = bme.readHumidity();                                                               
  h_value = uint8_t(round_number(h_value)) * 2; // Redondeamos el valor leído al entero o "0,5" más cercano
                                                // Multiplicamos por dos para que Cayenne pueda interpretar la
                                                // lectura (si Cayenne recibe un 1 lo interpretará como 0,5; 
                                                // un 2 -> interpretará un 1; 2 -> 1,5; 3 -> 2; etc)

  // Lectura del voltaje de la pila de alimentación
  b_value = uint16_t(lectura_voltaje() / 10);   // Devuelve el valor en mV (pasamos el valor en voltios con dos decimales)

  // Leemos la hora a la que se ha realizado la lectura de los datos
  MyDateAndTime = Clock.read();  
  
  }


// Función de envío de los datos
void do_send(osjob_t* j) {
  
  int bytes_buffer = 27;      // Número de bytes que tendrá el payload   
  
  byte buffer[bytes_buffer];  // Buffer con los datos que enviaremos al Gateway 
      // Referencias para codificar los datos y enviarlos a TTN
      // Working with Bytes: https://www.thethingsnetwork.org/docs/devices/bytes.html
      // Operaciones con bits en Arduino:  http://playground.arduino.cc/Code/BitMath#bit_pack
      // Formato Cayenne: https://mydevices.com/cayenne/docs/lora/#lora-cayenne-low-power-payload-ipso-smart-objects-reference-expansion-pack-data-types
    
     // CONSTRUCCIÓN DEL PAYLOAD
     
     // Almacenamos los datos codificados en el buffer en formato LPP (Low Power Payload) Cayenne
                
     // Bytes de temperatura
        buffer[0] = 0x02;                   // Canal 2 
        buffer[1] = 0x67;                   // "Sensor de temperatura"
        buffer[2] = highByte(t_value);      // Datos en ºC con un decimal 
        buffer[3] = lowByte(t_value);
    
      // Bytes de presión
        buffer[4] = 0x03;                   // Canal 3 
        buffer[5] = 0x73;                   // "Sensor de presión"
        buffer[6] = highByte(p_value);      // Datos enviados en hPa con un decimal
        buffer[7] = lowByte(p_value);
        
      // Bytes de humedad
        buffer[8] = 0x04;                   // Canal 4 
        buffer[9] = 0x68;                   // "Sensor de humedad"
        buffer[10] = h_value;                // Datos con precisión 0.5% 
          
       // Bytes de la hora
        buffer[11] = 0x05;                   // Canal 5 
        buffer[12] = 0x00;                   // "Digital input"
        buffer[13] = MyDateAndTime.Hour;     // Horas  
    
      // Bytes de los minutos
        buffer[14] = 0x06;                   // Canal 6 
        buffer[15] = 0x00;                   // "Digital input"
        buffer[16] = MyDateAndTime.Minute;   // Minutos
    
      // Bytes de los segundos
        buffer[17] = 0x07;                   // Canal 7 
        buffer[18] = 0x00;                   // "Digital input"
        buffer[19] = MyDateAndTime.Second;   // Segundos   
    
      // Bytes de nivel de batería      
        buffer[20] = 0x08;                  // Canal 8
        buffer[21] = 0x02;                  // "Analog input"
        buffer[22] = highByte(b_value);     // Datos en V con dos decimales
        buffer[23] = lowByte(b_value); 

      // Si se detecta fuego añadimos una señal indicadora al payload     
      // Bytes de alerta por fuego
        buffer[24] = 0x01;                   // Canal 1 
        buffer[25] = 0x00;                   // "Digital input"
        buffer[26] = alarma_fuego;           // Datos
                  
      // ENVÍO DE LOS DATOS
      
      // Check if there is not a current TX/RX job running
      if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
      } else {
        // Prepare upstream data transmission at the next possible time.
                     //(u1_t puerto, xref2u1_t datos, u1_t tamaño_payload, u1_t confirmed)
        LMIC_setTxData2(1, (uint8_t*) buffer, sizeof(buffer), 0);
        Serial.println(F("Sending: "));
      }        
}

// Función de lectura del voltaje de alimentación
long lectura_voltaje() { 
  
  long result; // Read 1.1V reference against AVcc
   
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); 
  delay(2);   // Wait for Vref to settle
   
  ADCSRA |= _BV(ADSC);   // Convert 
  while (bit_is_set(ADCSRA, ADSC)); 
  
  result = ADCL; // Low
  result |= ADCH << 8; //High
  result = 1126400L / result; // Back-calculate AVcc in mV 
  return result; 
}

// Función para redondear un float al entero o "0,5" más cercano (lo usaremos para la humedad)
float round_number(float n){
  float r;
  r = round(n * 2) / 2;
  return r;    
}  

 
// Función que indica el resultado de la operación de envío de datos
void onEvent (ev_t ev) {
  int i,j;
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      // after Joining a job with the values will be sent.
      joined = true;
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      // Re-init
      os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
        if (LMIC.dataLen) {
        // data received in rx slot after tx
        // if any data received, a LED will blink
        // this number of times, with a maximum of 10
        Serial.print(F("Data Received: "));
        Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
        i=(LMIC.frame[LMIC.dataBeg]);
        // i (0..255) can be used as data for any other application
        // like controlling a relay, showing a display message etc.        
      }
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      delay(50);  // delay to complete Serial Output before Sleeping
      // Schedule next transmission
      // next transmission will take place after next wake-up cycle in main loop
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      //Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

// Initial job
static void initfunc (osjob_t* j) {
    // reset MAC state
    LMIC_reset();
    // start joining
    LMIC_startJoining();
    // init done - onEvent() callback will be invoked...
}

// Función para restringir el envío a un solo canal
void forceTxSingleChannelDr() {
  for (int i = 0; i < 9; i++) { // For EU; for US use i<71
    if (i != channel) {
      LMIC_disableChannel(i);
    }
  }
  // Set data rate (SF) and transmit power for uplink
  LMIC_setDrTxpow(dr, 14);
}

/////////////////////////////////////////////////////////////////////////////
///////////////////////// CONFIGURACIÓN ALARMAS /////////////////////////////
/////////////////////////////////////////////////////////////////////////////

/* There are 2 different alarms possible in the chip, the first alarm has a
  * resolution of seconds, the second a resolution of minutes.
  * 
  * Which alarm gets modified by setAlarm() depends on the type of alarm
  * you want.
  * 
  * The following alarm types are available.
  * 
  *  (Alarm 1)
  *    ALARM_EVERY_SECOND  (Timestamp not required)                   
  *    ALARM_MATCH_SECOND                    
  *    ALARM_MATCH_SECOND_MINUTE             
  *    ALARM_MATCH_SECOND_MINUTE_HOUR        
  *    ALARM_MATCH_SECOND_MINUTE_HOUR_DATE   
  *    ALARM_MATCH_SECOND_MINUTE_HOUR_DOW    
  *    
  * (Alarm 2)
  *    ALARM_EVERY_MINUTE (Timestamp not required)                   
  *    ALARM_MATCH_MINUTE                    
  *    ALARM_MATCH_MINUTE_HOUR               
  *    ALARM_MATCH_MINUTE_HOUR_DATE          
  *    ALARM_MATCH_MINUTE_HOUR_DOW           
  *    
  * (Alarm 2)
  *    ALARM_HOURLY   (on the minute of the supplied timestamp *)
  *    ALARM_DAILY    (on the hour and minute *)            
  *    ALARM_WEEKLY   (on the hour and minute and day-of-week *)                             
  *    ALARM_MONTHLY  (on the hour and minute and day-of-month *)
  * 
  *  * If set without a timestamp, the current timestamp is used.
  */
