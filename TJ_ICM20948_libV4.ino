#include <ICM_20948.h>
#include "AHRS_ICM20948.h"
#include <Wire.h>
#include <stdint.h>

#define ARDUINOJSON_DECODE_UNICODE 1
#define PROTOCOL_ITEM_NUMBER 6

#include "Protocol.h"

#define SERIAL_PORT Serial
#define WIRE_PORT Wire

ICM_20948_I2C myICM;
AHRS myAHRS(myICM);

const bool tuning = 1;
bool itint_print = 1;

void setup(){

  SERIAL_PORT.begin(115200);
  while(!SERIAL_PORT){};
  SERIAL_PORT.flush();
  
  WIRE_PORT.begin();
  WIRE_PORT.setClock(100000);
  
  bool initialized = false;
  while( !initialized ){
    myICM.begin( WIRE_PORT, 0x68 );
    //myICM.begin( WIRE_PORT, 0x0C );
    delay(500);
    if (itint_print){
      SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
      SERIAL_PORT.println( myICM.statusString() );
    }
    if( myICM.status != ICM_20948_Stat_Ok ){
      if (itint_print){
        SERIAL_PORT.println( "Trying again..." );
      }
      delay(500);
    }
    else{
      initialized = true;
    }
  }
  
  ARHS_start(&myICM, &myAHRS);
  if (tuning) {
    AHRS_init_tuning(&myICM, &myAHRS);
  }
  myAHRS.setAHRS_MagneticDeclination(0.44);
}

void loop () {

  if( myICM.dataReady() ){
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
    myAHRS.AHRS_update();
    //myAHRS.print_MotionCals(false,false,"all");//uni true or false, correction true or false, selection 
    
    myAHRS.print_AHRS(true,false);// bool all (true) or yaw only (false), rad (true) or degrees (false) //ACESS AHRS VALUES HERE (YAW AND THE REST)
  }
}
