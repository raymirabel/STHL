
/*
  STHL.cpp - v1.00 - 01/08/2016:
  - Version inicial
   
  Sketch para el módulo sensor de temperatura ambiental STHL
  Copyright (c) 2016 Raimundo Alfonso
  Ray Ingeniería Electrónica, S.L.
  
  Este sketch está basado en software libre. Tu puedes redistribuir
  y/o modificarlo bajo los terminos de licencia GNU.

  Esta biblioteca se distribuye con la esperanza de que sea útil,
  pero SIN NINGUNA GARANTÍA, incluso sin la garantía implícita de
  COMERCIALIZACIÓN O PARA UN PROPÓSITO PARTICULAR.
  Consulte los terminos de licencia GNU para más detalles.
  
  * CARACTERISTICAS GENERALES
  - Medida de temperatura, humedad y punto de rocío
  - 6 interruptores dipswitch para direccionamiento modbus
  - 2 interruptores dipswitch para configuración de comunicaciones
  - Bus de comunicaciones RS485 con detección automática de dirección
  - Amplio rango de alimentación de 6.5 a 30VDC
  - Regulador conmutado de alta eficiencia
  - Bajo coste


  * CARACTERISTICAS SENSOR TEMPERATURA Y HUMEDAD
  - Tipo sensor: SHT21 (SENSIRION)
  - Resolución temperatura: 0.1ºC
  - Rango de medida: -40 ~ 85ºC
  - Precisión típica: +/- 0.3ºC
  - Precisión máxima: +/- 1ºC
  - Resolución humedad: 1%
  - Rango de medida: 0 ~ 100%RH
  - Precisión típica: +/- 2%RH
  - Precisión máxima: +/- 5%RH
 
  * MAPA MODBUS
    MODO R: FUNCION 3 - READ BLOCK HOLDING REGISTERS
    MODO W: FUNCION 6 - WRITE SINGLE HOLDING REGISTER
    
  DIRECCION   TIPO    MODO  FORMATO    MAXIMO      MINIMO    UNIDADES    DESCRIPCION
  ---------------------------------------------------------------------------------------------------------
  0x0000      int     R     0000.0    +0155.0     -0055.0    ºC          TEMPERATURA
  0x0001      uint    R     00000      00100       00000     %           HUMEDAD 
  0x0002      int     R     0000.0    +0155.0     -0055.0    ºC          TEMPERATURA DE ROCIO   
  0x0003      int     R     00000      00001       00000     ---         PARAMETROS SENSOR OK = 0 (solo SHT21)
  0x0004      int     R     00000      00000       00000     ---         RESERVADO
  0x0005      int     R     00000      00063       00000     ---         ESTADO DEL DIPSWITCH
*/


//  - DIP1: DIRECCION MODBUS BIT 0
//  - DIP2: DIRECCION MODBUS BIT 1
//  - DIP3: DIRECCION MODBUS BIT 2
//  - DIP4: DIRECCION MODBUS BIT 3
//  - DIP5: DIRECCION MODBUS BIT 4
//  - DIP6: DIRECCION MODBUS BIT 5
//  - DIP7: OFF(0): VELOCIDAD = 9600
//          ON(1) : VELOCIDAD = 19200
//  - DIP8: OFF(0): PARIDAD = NONE (NINGUNA)
//          ON(1) : PARIDAD = EVEN (PAR)


#include <Wire.h>
#include <ModbusSlave.h>
#include <avr/wdt.h> 
#include <SHT2x.h>



#define DIPSW1	5    // Dirección modbus 0
#define DIPSW2	6    // Dirección modbus 1
#define DIPSW3	7    // Dirección modbus 2
#define DIPSW4	8    // Dirección modbus 3
#define DIPSW5	9    // Dirección modbus 4
#define DIPSW6	10   // Dirección modbus 5
#define DIPSW7	A6   // Paridad none/par
#define DIPSW8	A7   // Velocidad 9600/19200

#define MAX_BUFFER_RX  15

// Variables globales...
char buffer_rx[MAX_BUFFER_RX];
SHT2xClass SHT2x;

// Mapa de registros modbus
enum {        
        MB_TEMPERATURE,  // Temperatura
        MB_HUMIDITY,     // Humedad del sensor (en caso del modelo con sensor de humedad)
        MB_DEW_POINT,    // Temperatura de rocio
        MB_SENS_FAIL,    // Estado del sensor
        MB_REV,          // Reservado
        MB_DIP,          // Estado dipswitch
        MB_REGS	 	       // Numero total de registros
};
int regs[MB_REGS];	

// Crea la clase para el modbus...
ModbusSlave modbus;


void setup()  { 
  wdt_disable();
  int velocidad;
  char paridad;

  // Configura puertos de Arduino  
  pinMode(DIPSW1,INPUT);
  pinMode(DIPSW2,INPUT);	
  pinMode(DIPSW3,INPUT);	
  pinMode(DIPSW4,INPUT);	
  pinMode(DIPSW5,INPUT);	
  pinMode(DIPSW6,INPUT);	

  // configura modbus...
  if(analogRead(DIPSW7) > 512) velocidad = 9600; else velocidad = 19200;  
  if(analogRead(DIPSW8) > 512) paridad = 'n'; else paridad = 'e';  
  modbus.config(velocidad,paridad);
  modbus.direccion = leeDIPSW() & 0x3F;
  
  // Activa WDT cada 4 segundos...   
  wdt_enable(WDTO_4S); 

  Wire.begin();
  // desactivate internal pullups for twi.
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);  

  // Asigna valores a la tabla modbus...
  regs[MB_TEMPERATURE] = 0;
  regs[MB_HUMIDITY]    = 0;
  regs[MB_DEW_POINT]   = 0;
  regs[MB_DIP]         = leeDIPSW();
  bitSet(regs[MB_SENS_FAIL],0);

} 



void loop()  { 
  // Modbus...
  modbus.actualiza(regs,MB_REGS);

  // Lectura de temperatura y humedad...
  lee_sht();

  // Control entradas digitales...
  controlIO();
  
  delay_modbus(100);
  wdt_reset(); 
}

void lee_sht(void){
  static float t = -999.0;
  static float h = -999.0;  
  static unsigned long timeOutSHT = millis();  
  static unsigned long tiempo_th = millis();  
  static boolean temperature_ok = false;
  static boolean humidity_ok = false;
  static boolean measActive = false;
  
  // Cada 3 segundos lee sensor....
  if (millis() - tiempo_th >= 3000L) {      
    if(measActive){
      SHT2x.PrepareTemperatureNoHold();
      measActive = false;
    }else{
      SHT2x.PrepareHumidityNoHold();
      measActive = true;
    }
    tiempo_th = millis();   
  }
 
  if(SHT2x.GetTemperatureNoHold(&t)){
    temperature_ok = true;
    timeOutSHT = millis();    
  }
  if(SHT2x.GetHumidityNoHold(&h)){
    humidity_ok = true;   
    timeOutSHT = millis();    
  }
  if(temperature_ok && humidity_ok){
    bitClear(regs[MB_SENS_FAIL],0);
    regs[MB_TEMPERATURE] = t * 10;
    regs[MB_HUMIDITY] = h;   
    regs[MB_DEW_POINT] = calcDewpoint(h, t) * 10;
  }

  // Time out a los 10 segundos...
  if((millis() - timeOutSHT) >= 10000){
    humidity_ok = false;
    temperature_ok = false;
    bitSet(regs[MB_SENS_FAIL],0);
    regs[MB_TEMPERATURE] = 0;
    regs[MB_HUMIDITY] = 0;
    regs[MB_DEW_POINT] = 0;
  }
}

void controlIO(void){
  regs[MB_DIP] = leeDIPSW();
}

// Rutina de espera que atiende la tarea modbus...
void delay_modbus(int t){
  int n,tt;
  tt = t/10;
  
  for(n=0;n<=tt;n++){
    modbus.actualiza(regs,MB_REGS);
    delay(10);
  }  
}

// Rutina para leer el dipswitch
byte leeDIPSW(void){
  boolean a0,a1,a2,a3,a4,a5,a6,a7;
  
  // Lee dipswitch...
  a0 = !digitalRead(DIPSW1);  
  a1 = !digitalRead(DIPSW2);
  a2 = !digitalRead(DIPSW3);
  a3 = !digitalRead(DIPSW4);
  a4 = !digitalRead(DIPSW5);  
  a5 = !digitalRead(DIPSW6);  
  if(analogRead(DIPSW7) > 512) a6 = 0; else a6 = 1;
  if(analogRead(DIPSW8) > 512) a7 = 0; else a7 = 1;

  // Calcula dirección...
  return(a0 + a1*2 + a2*4 + a3*8 + a4*16 + a5*32 + a6*64 + a7*128);
}


float calcDewpoint(float humi, float temp) {
  float k;
  k = log(humi/100) + (17.62 * temp) / (243.12 + temp);
  return 243.12 * k / (17.62 - k);
}

