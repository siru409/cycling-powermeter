
#include <Adafruit_AHRS_Madgwick.h>
#include <RAK12033-IIM42652.h>
#include <ArduinoBLE.h>
#include "esp_wifi.h"
#include "esp_pm.h"
#include "driver/rtc_io.h"
#include "esp_bt.h"
#include <Wire.h>  // Include the SoftwareWire library
#include "HX711.h"
#include <movingAvg.h>  
#include "Adafruit_HX711.h"
#include <Preferences.h>
#include <limits.h>





int rev_count = 2; //average power over n revolutions

float rotation_tracker=0, total_rotation_tracker=0;
float cadence = 0, velocity = 0;
long raw_force = 0;



#define LED 40
#define CLOCK 38
#define ADC_DATA 39
#define POWERMETER_SERVICE_UUID "00001818-0000-1000-8000-00805f9b34fb"
#define CALIBRATION_SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CALIBRATION_UUID "12345678-1234-1234-1234-1234567890ac"
#define CALIBRATION_UUID2 "12345678-1234-1234-1234-1234567890ad"
#define UUID1 "00002a66-0000-1000-8000-00805f9b34fb"
#define UUID2 "00002a63-0000-1000-8000-00805f9b34fb"
#define UUID3 "a026e005-0a7d-4ab3-97fa-f1500f9feb8b"



void sendPowerAndCadence(int16_t powerW, uint16_t cumulativeCrankRevs);
float read_gyro();
void saveSlope(int16_t v);
int16_t loadSlope();

float CRANK_LENGTH= 175;


Adafruit_Madgwick orientation_filter;
BLEService cadenceService("1816");
BLEService powerService(POWERMETER_SERVICE_UUID); 
BLEService cfgService(CALIBRATION_UUID);
BLECharacteristic cadenceChar(UUID1, BLERead | BLENotify, 5); 
BLECharacteristic powerChar(UUID2, BLERead | BLENotify, 20); 
BLECharacteristic util(UUID3, BLERead | BLENotify, 1); 
BLECharacteristic cfgRX(CALIBRATION_UUID, BLEWrite | BLEWriteWithoutResponse, 20);
BLECharacteristic cfgTX(CALIBRATION_UUID2, BLERead | BLENotify, 20);
Preferences prefs;
long long led_timer =0;


IIM42652 IMU;
float filter=0;
int ind =0;
float prev_pitch;
long offset;
float gyroX,gyroY, gyroZ,accelX_mss,accelY_mss,accelZ_mss, power;
int i = 0;
long long starttime = millis();
HX711 scale;



void setup() {
  delay(500); 
  pinMode(LED, OUTPUT);
  analogWrite(LED,10);

  pinMode(2, OUTPUT);
  digitalWrite(2, 1); //turns led on (low side switch)

  pinMode(ADC_DATA, INPUT);
  pinMode(CLOCK, OUTPUT);
  digitalWrite(CLOCK, LOW);
  scale.begin(ADC_DATA, CLOCK);
  Serial.begin(115200);
  Wire.begin(18,17);
  if (IMU.begin()==false)
      Serial.println("IIM-42652 is not connected.");
   
   while (!BLE.begin()){
      analogWrite(LED,100);
      delay(500);
      analogWrite(LED,0); 
      delay(500);
    }

  for(int i =0; i<50; i++){
    if (scale.is_ready()) {
        offset -= scale.read()/50;
        total_rotation_tracker+=read_gyro()/10;
        
      } else {
        Serial.println("HX711 not ready");
        i= i-1;
      }
      delay(100);
  }
  if(abs(total_rotation_tracker)<360){
      BLE.setLocalName("WUMMmE UHR");
      BLE.setAdvertisedService(powerService);
      powerService.addCharacteristic(powerChar);
      powerService.addCharacteristic(util);
      powerService.addCharacteristic(cadenceChar);
      BLE.addService(powerService);
      BLE.advertise();
    
  }else{ // calibration mode
      BLE.setLocalName("calibration_mode");
      BLE.setAdvertisedService(cfgService);
      cfgService.addCharacteristic(cfgRX);
      cfgService.addCharacteristic(cfgTX);
      BLE.addService(cfgService);
      BLE.advertise();
      while(1){
          BLEDevice central = BLE.central();
          if (!central) continue;
          while (central.connected()) {
            BLE.poll();
            if (scale.is_ready()) {
                raw_force = scale.read();
                Serial.println(raw_force);

                delay(100);
          
            } else {
                  Serial.println("HX711 not ready");
            }

            int16_t v16 = raw_force/10;
            uint8_t b2[2] = {(uint8_t)(v16 & 0xFF), (uint8_t)((v16 >> 8) & 0xFF) }; // LE
            cfgTX.writeValue(b2, 2);
            if (cfgRX.written()) {
              Serial.println("received data");
                uint8_t buf[20];
                int len = cfgRX.valueLength();
                len = min(len, (int)sizeof(buf));
                Serial.println(len);
                cfgRX.readValue(buf, len);
                if (len == 4) {
                  int16_t s1 = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
                  int16_t s2 = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
                  Serial.printf("int16 s1=%d  s2=%d\n", s1, s2);
                  int16_t calibration_value = (s2-s1);
                  saveSlope(calibration_value);
          }
      }
  }}}

orientation_filter.begin(1);
saveSlope((3.0f*5000)/(2.0f));

}
void loop() {
  BLEDevice central = BLE.central();
  analogWrite(LED,10);
  delay(20);
  analogWrite(LED,0);
  delay(200);
  
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    // Check the connection periodically
    while (central.connected()) {


      
      gyroX = read_gyro();
      
      if (abs(gyroX)>0.5){
        
        digitalWrite(2, 1); 
        if (scale.is_ready()) {
          raw_force = scale.read();
  
        } else {
          Serial.println("HX711 not ready");
        }
      }else{
        digitalWrite(2,0);
        
      }


      //orientation_filter.updateIMU(gyroX*3.1415/180 , gyroY*3.1415/180, gyroZ*3.1415/180, accelX_mss, accelY_mss, accelZ_mss);
      raw_force+= offset;
      raw_force = raw_force*0.1*5000/loadSlope();
      //float roll = orientation_filter.getRoll();
      //float pitch = orientation_filter.getPitch();

      ind++;

      filter *= ((float)ind-1)/(float)ind;
      filter += (float)(gyroX*3.1415/180*CRANK_LENGTH/1000*(float)raw_force/1000*9.81*2)/(float)ind;
      
      BLE.poll();
      rotation_tracker += gyroX*0.015;
      total_rotation_tracker += gyroX*0.015;




      //if (ind>500||((int)pitch/abs((int)pitch)!=(int)prev_pitch/abs((int)prev_pitch)&&ind>50&&roll<0&&abs(rotation_tracker)>719)){
      if (abs(rotation_tracker)>720||ind>300){
        Serial.print("bit changed!!");
        //prev_pitch = pitch;
        rotation_tracker=0;
        ind = 0;


          
        float voltage = analogRead(3)/1241.2*2;
        Serial.println(voltage);

        voltage = constrain(voltage, 3.3, 4.2);
        Serial.println(voltage);
        float soc = 1/(1+pow(2.71828, -8*(voltage-3.5)));
        power = abs(filter);
        soc = constrain(soc*100, 0.0, 100.0);
        Serial.println(soc);
        if (soc<20){
          analogWrite(LED,10);
          led_timer = millis(); 
        }
        power = constrain(power, 0.0, 3000.0);
        sendPowerAndCadence(power, abs(total_rotation_tracker/360));
        BLE.poll();
        
      }
      while(starttime+15>millis()){
        //Serial.println(led_timer);
        if (millis()-led_timer>100){analogWrite(LED,0); }


        BLE.poll();

        }; //loop timer

      starttime = millis();

    }
  }
}

void sendPowerAndCadence(int16_t powerW,uint16_t cumulativeCrankRevs){
uint8_t buf[8];
uint16_t flags = 0;
flags |= (1 << 5); // Crank Revolution Data Present

int16_t instPower = (int16_t)powerW;   // watts
uint16_t crankRevs = cumulativeCrankRevs; // ++ per revolution
uint16_t eventTime1024 = (millis() % 65536) * 1024 / 1000; // 1/1024 s units

buf[0] = flags & 0xFF;
buf[1] = (flags >> 8) & 0xFF;
buf[2] = instPower & 0xFF;
buf[3] = (instPower >> 8) & 0xFF;
buf[4] = crankRevs & 0xFF;
buf[5] = (crankRevs >> 8) & 0xFF;
buf[6] = eventTime1024 & 0xFF;
buf[7] = (eventTime1024 >> 8) & 0xFF;


powerChar.writeValue(buf, sizeof(buf));
}



float read_gyro(){
  
      IIM42652_axis_t gyro_data;
      IIM42652_axis_t accel_data;
      
      IMU.ex_idle();
      IMU.accelerometer_enable();
      IMU.gyroscope_enable();
      IMU.temperature_enable();
      IMU.get_gyro_data(&gyro_data);
      
      IMU.get_accel_data(&accel_data);
      accelX_mss = (float)accel_data.x / 16384.0f* 9.80665f;
      accelY_mss = (float)accel_data.y / 16384.0f* 9.80665f;
      accelZ_mss = (float)accel_data.z / 16384.0f* 9.80665f;
      gyroY = (float)gyro_data.y / 16.4f; 
      gyroZ = (float)gyro_data.z / 16.4f;
      gyroX = (float)gyro_data.z / 16.4;
      return gyroX;


}

void saveSlope(int16_t v) {
  prefs.begin("cal", false);          // namespace "cal", read/write
  prefs.putShort("slope", v);         // store signed 16-bit
  prefs.end();
}

int16_t loadSlope() {
  prefs.begin("cal", true);           // read-only is fine
  int16_t v = prefs.getShort("slope", 0);  // default=0 if not set yet
  prefs.end();
  return v;
}
