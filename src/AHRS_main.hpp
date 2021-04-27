#pragma once

class AHRS {

  ICM_20948_I2C* _myICM;
  
  const uint8_t MPU9250_ADDRESS {0x69};  // Device address when ADO = 0
  const uint8_t AK8963_ADDRESS {0x0C};

 
  float gyroCalibration[3] = {-0.446390, 2.2258, -0.23901}; // calibrations offsets
  float accelCalibration[3] = {-1.276, -17.285, 37.229};
  float magCalibration[3] = {3.225, -2.324, 20.25}; // mag calibration (most important one)

  float gyroBias[3] = {0, 0, 0}; // biases corrections (UNUSED)
  float magBias[3] = {0, 0, 0};
  float accelBias[3] = {0, 0, 0};
  
  float magScale[3]  = {1.0, 1.0, 1.0}; // Scale correction in case the magnetometer 
  //produces "oval" results for any of the "three two axes relative projections" (UNUSED)
  
  float aRes;
  float gRes;
  float mRes; 

  float temperature;

  float magnetic_declination = -7.51; // Japan, 24th June - 
  // For real, we don't care about that since we don't want 
  // to fly an airplane but do a relative orientation job with a 0 deg set by user at start

  bool scalecorr = false;
  // if correction false : micro teslas,  milli g's, degrees per second straight from ICM_20948 library
  // DO not use the true mode wich was supposed to do the scale calculation in this class, this didn't worked and was not patched since the ICM class did it well
  
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
  float pitch, yaw, roll;
  float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
  float lin_ax, lin_ay, lin_az; 

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  QuaternionFilter qFilter;
  
  public:
  float a[3], g[3], m[3];
  
  void setAHRS_MagneticDeclination(const float d) { magnetic_declination = d; }
  
  float getAHRS_Roll() const { return roll; }
  float getAHRS_Pitch() const { return pitch; }
  float getAHRS_Yaw() const { return yaw; }

  AHRS(ICM_20948_I2C& myICM){
    _myICM = &myICM;
  }
  
  void setupscales(ICM_20948_fss_t *myFSS){
     switch (myFSS->a){
        case dps250: 
          aRes = 250.0 / 32768.0;
          break;
        case dps500:
          aRes = 500.0 / 32768.0;
          break;
        case dps1000:
          aRes = 1000.0 / 32768.0;
          break;
        case dps2000:
          aRes = 2000.0 / 32768.0;
          break;
     }
     
     switch (myFSS->g){
        case gpm2: 
          gRes = 2.0 / 32768.0;
          break;
        case gpm4:
          gRes = 4.0 / 32768.0;
          break;
        case gpm8:
          gRes = 8.0 / 32768.0;
          break;
        case gpm16:
          gRes = 16.0 / 32768.0;
          break;
     }
      mRes = 10. * 4912. / 32760.0; // Proper scale to return milliGauss
  }

  void AHRS_update(){
    updateAccelGyro();
    updateMag(); // TODO: set to 30fps?
    // required units : acc[mg],( milli gs ) gyro[deg/s],( degrees per second ),  mag [mG] ( 10 milligauss = 1 micro Tesla )
    // gyro will be convert from [deg/s] to [rad/s] inside of this function
    qFilter.update(-a[0], a[1], a[2], g[0], -g[1], -g[2], m[1], -m[0], m[2], q);
    //qFilter.MahonyQuaternionUpdate(-a[0], a[1], a[2], g[0], -g[1], -g[2], m[1], -m[0], m[2], q); // last time i checked, gave bad results when using this instead of the magdwick one
    updateRPY();
  }
  void updateAccelGyro() // old version with reference as arg to every function : ICM_20948_I2C *myICM
  {   
    if ( scalecorr ){
      a[0] = (float)_myICM->agmt.acc.axes.x * aRes - accelBias[0];  // DO NOT USE THIS : NOT WORKING 
      a[1] = (float)_myICM->agmt.acc.axes.y * aRes - accelBias[1];
      a[2] = (float)_myICM->agmt.acc.axes.z * aRes - accelBias[2];

      g[0] = (float)_myICM->agmt.gyr.axes.x * gRes - gyroBias[0];  // get actual gyro value from ICM library (internal scale calculations based on  FSS)
      g[1] = (float)_myICM->agmt.gyr.axes.y * gRes - gyroBias[1];
      g[2] = (float)_myICM->agmt.gyr.axes.z * gRes - gyroBias[2];
    }
    else {
      a[0] = (float)_myICM->accX() - accelCalibration[0];  // DO NOT USE THIS : NOT WORKING 
      a[1] = (float)_myICM->accY() - accelCalibration[1];
      a[2] = (float)_myICM->accZ() - accelCalibration[2];

      // Calculate the gyro value into actual degrees per second
      g[0] = (float)_myICM->gyrX() - gyroCalibration[0];  // get actual accel value from ICM library (internal scale calculations based on  FSS)
      g[1] = (float)_myICM->gyrY() - gyroCalibration[1];
      g[2] = (float)_myICM->gyrZ() - gyroCalibration[2];
    }
    // Now we'll calculate the accleration value into actual g's
  }
  void updateMag(){
    if ( scalecorr ){
      m[0] = (float)(_myICM->agmt.mag.axes.x * mRes * magCalibration[0] - magBias[0]) * magScale[0];  // DO NOT USE THIS : NOT WORKING 
      m[1] = (float)(_myICM->agmt.mag.axes.y * mRes * magCalibration[1] - magBias[1]) * magScale[1];
      m[2] = (float)(_myICM->agmt.mag.axes.z * mRes * magCalibration[2] - magBias[2]) * magScale[2];
    }
    else {
      m[0] = (float)((_myICM->magY() - magCalibration[1]) *-magScale[0] *10 )  ;  // get actual mag  value from ICM library (internal scale calculations based on  FSS)
      m[1] = (float)((_myICM->magX() - magCalibration[0]) *magScale[1] *10  )  ;  // times 10 because we want milligauss and not microteslas
      m[2] = (float)((_myICM->magZ() - magCalibration[2]) *magScale[2] *10  )  ;  // The values are swapped because 
      //MPU and ICM don't have same orientation for mag axes. See the technical doc named MORJ.md at section Accelerometer dev, Conversion MPU to ICM, for more info.
    }
  }

  float Uni_accX() const { return a[0]; }
  float Uni_accY() const { return a[1]; }
  float Uni_accZ() const { return a[2]; }
  
  float Uni_gyrX() const { return g[0]; }
  float Uni_gyrY() const { return g[1]; }
  float Uni_gyrZ() const { return g[2]; }

  float Uni_magX() const { return m[0]; }
  float Uni_magY() const { return m[1]; }
  float Uni_magZ() const { return m[2]; }
  
  void updateRPY(){
        
      a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
      a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
      a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
      a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
      a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
      pitch = -asinf(a32);
      roll  = atan2f(a31, a33);
      yaw   = atan2f(a12, a22);
      pitch *= 180.0f / PI;
      roll  *= 180.0f / PI;
      yaw   *= 180.0f / PI;
      yaw   += magnetic_declination;
      if      (yaw >= +180.f) yaw -= 360.f;
      else if (yaw <  -180.f) yaw += 360.f;

      lin_ax = a[0] + a31;
      lin_ay = a[1] + a32;
      lin_az = a[2] - a33;
  }
  
  void print_MotionCals(bool uni = true,bool correction = true, String selection = "all"){
    static unsigned long lastprint = 0;
    if ( (millis()-lastprint) > 50){
      if (!uni){
        Serial.print("Raw:");
        if (selection == "all" || selection == "accel"){
          Serial.print(_myICM->agmt.acc.axes.x);sep_(",");
          Serial.print(_myICM->agmt.acc.axes.y);sep_(",");
          Serial.print(_myICM->agmt.acc.axes.z);
          if (selection == "all"){
            ;sep_(",");
          }
        }
        if (selection == "all" || selection == "gyro"){
          Serial.print(_myICM->agmt.gyr.axes.x);sep_(",");
          Serial.print(_myICM->agmt.gyr.axes.y);sep_(",");
          Serial.print(_myICM->agmt.gyr.axes.z);
          if (selection == "all"){
            ;sep_(",");
          }
        }
        if (selection == "all" || selection == "mag"){
          Serial.print(_myICM->agmt.mag.axes.x);sep_(",");
          Serial.print(_myICM->agmt.mag.axes.y);sep_(",");
          Serial.print(_myICM->agmt.mag.axes.z);
        }
        sep_("\n");
      }
      if (uni){
        Serial.print("Uni:");
        if ( correction ){
          if (selection == "all" || selection == "accel"){
            Serial.print(Uni_accX());sep_(",");
            Serial.print(Uni_accY());sep_(",");
            Serial.print(Uni_accZ());
            if (selection == "all"){
              ;sep_(",");
            }
          }
          if (selection == "all" || selection == "gyro"){
            Serial.print(Uni_gyrX());sep_(",");
            Serial.print(Uni_gyrY());sep_(",");
            Serial.print(Uni_gyrZ());
            if (selection == "all"){
              ;sep_(",");
            }
          }
          if (selection == "all" || selection == "mag"){
            Serial.print(Uni_magX());sep_(",");
            Serial.print(Uni_magY());sep_(",");
            Serial.print(Uni_magZ());
          }
        }
        else{
          if (selection == "all" || selection == "accel"){
            Serial.print(_myICM->accX());sep_(",");
            Serial.print(_myICM->accY());sep_(",");
            Serial.print(_myICM->accZ());
            if (selection == "all"){
              ;sep_(",");
            }
          }
          if (selection == "all" || selection == "gyro"){
            Serial.print(_myICM->gyrX());sep_(",");
            Serial.print(_myICM->gyrY());sep_(",");
            Serial.print(_myICM->gyrZ());
            if (selection == "all"){
              ;sep_(",");
            }
          }
          if (selection == "all" || selection == "mag"){
            Serial.print(_myICM->magY());sep_(",");
            Serial.print(_myICM->magY());sep_(",");
            Serial.print(_myICM->magY());
          }
        }
        sep_("\n");
      }
      lastprint = millis();
    }
  }

  void print_Temperature(){
    Serial.println(_myICM->temp());
  }
  
//  void print_Vectors(){
//    Serial.print(m[0]);sep_();
//    Serial.print(m[1]);sep_();
//    Serial.print(m[2]);sep_("/n");
//  }
  
  void print_AHRS(bool all = true, bool rad_mode = false){
    float roll = getAHRS_Roll();
    float pitch = getAHRS_Pitch();
    float yaw = getAHRS_Yaw();
    //Serial.print(F("Orientation: "));
    if (rad_mode){
      roll = roll * (PI/180);
      pitch = pitch * (PI/180);
      yaw = yaw * (PI/180);
    }
    Serial.print(yaw);
    if (all) {
      sep_(",");
      Serial.print(pitch);sep_(",");
      Serial.print(roll);
    }
    sep_("\n");
  }
};
