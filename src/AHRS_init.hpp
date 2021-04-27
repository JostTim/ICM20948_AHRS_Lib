#pragma once

void ARHS_start(ICM_20948_I2C *myICM, AHRS *myAHRS){
  ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm2;        
  myFSS.g = dps250;                          
  myAHRS->setupscales(&myFSS); 
}

void AHRS_init_tuning(ICM_20948_I2C *myICM, AHRS *myAHRS){
//    SERIAL_PORT.println("Device connected!");
//    myICM.swReset( );
//    if(myICM.status != ICM_20948_Stat_Ok){
//      SERIAL_PORT.print(F("Software Reset returned: "));
//      SERIAL_PORT.println(myICM.statusString());
//    }
//    delay(250);

    myICM->sleep( false );
    myICM->lowPower( false );
    
    myICM->setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous );  // TODO Set sample rate to 1 kHz
    if( myICM->status != ICM_20948_Stat_Ok && itint_print){
      SERIAL_PORT.print(F("setSampleMode returned: "));
      SERIAL_PORT.println(myICM->statusString());
    }
  
    ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
    myFSS.a = gpm8;        
    myFSS.g = dps1000;                    
              
    myAHRS->setupscales(&myFSS);  // USELESS since the scale correction  
    myICM->setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );
    if( myICM->status != ICM_20948_Stat_Ok && itint_print){
      SERIAL_PORT.print(F("setFullScale returned: "));
      SERIAL_PORT.println(myICM->statusString());
    }
  
    
    ICM_20948_dlpcfg_t myDLPcfg; // Set up Digital Low-Pass Filter configuration  --   TODO Set low-pass filter to 188 Hz     
    myDLPcfg.a = acc_d111bw4_n136bw;         
    myDLPcfg.g = gyr_d151bw8_n187bw6;    
  
    myICM->setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
    if(myICM->status != ICM_20948_Stat_Ok && itint_print){
      SERIAL_PORT.print(F("setDLPcfg returned: "));
      SERIAL_PORT.println(myICM->statusString());
    }
    ICM_20948_Status_e accDLPEnableStat = myICM->enableDLPF( ICM_20948_Internal_Acc, true );
    ICM_20948_Status_e gyrDLPEnableStat = myICM->enableDLPF( ICM_20948_Internal_Gyr, true );

    if (itint_print){
      SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: ")); SERIAL_PORT.println(myICM->statusString(accDLPEnableStat));
      SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: ")); SERIAL_PORT.println(myICM->statusString(gyrDLPEnableStat));
      SERIAL_PORT.println();
      SERIAL_PORT.println(F("Configuration complete!")); 
    }
}
