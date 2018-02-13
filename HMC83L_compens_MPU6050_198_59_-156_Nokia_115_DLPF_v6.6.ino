/*
  Tilt compensated HMC5883L + MPU6050 (GY-86 / GY-87). Output for HMC5883L_compensation_processing.pde
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
  GIT: https://github.com/jarzebski/Arduino-HMC5883L
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
 
Adafruit_PCD8544 display = Adafruit_PCD8544(2, 3, 4, 5, 6);

#include <Wire.h>
#include <HMC5883L.h>
#include <MPU6050.h>
#include <EEPROM.h>

HMC5883L compass;
MPU6050 mpu;

float roll;
float pitch;
float rollold=0;
float pitchold=0;
float Xh;
float Yh;
float Xhold=1;
float Yhold=1;
float heading1;
float heading2;
const float RTOG=180/M_PI; //convert Rad to Grad
  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / S-Pb declination angle is 10'38E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI)
const float declinationAngle = (10.0 + (38.0 / 60.0)) / (RTOG);
const float ALFA=0.15; //α = dt / (RC + dt) - coeff LOW pass filtr. dt = sampling period
// создаем глобальную переменную buttonState - состояние ртутного гравитационного замыкателя
boolean buttonState;
int flagPut = 0;
unsigned long curentTime;  // текущее значение времени
int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int minZ = 0;
int maxZ = 0;
int offX = 0;
int offY = 0;
int offZ = 0;

void setup()
{
   Serial.begin(9600);
   
  // определяем вывод 7 (ртутный замыкатель ) как вход 
  pinMode(7, INPUT_PULLUP);
  buttonState = digitalRead(7);  // считываем состояние 7 входа (ртутного замыкателя) и записываем в buttonState
  // Инициализируем наш дисплей
  display.begin();
  // Делаем его пустым
  display.clearDisplay();
  display.display();
  // Устанавливаем контраст
  display.setContrast(60);

   if (buttonState == HIGH) { goto initializeMPU;} 
   EEPROM.get(0, offX);
   EEPROM.get(4, offY);
   EEPROM.get(8, offZ);
   //Serial.println("Read offsets from EEPROM: ");
   //Serial.println(offX);
   //Serial.println(offY);
   //Serial.println(offZ);
   // Выводим текст на LCD5110
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("Offsets 5883L");
  display.setCursor(0, 10);
  display.setTextSize(1);
  display.print("offX=");
  display.setCursor(40, 10);
  display.setTextSize(1);
  display.print(offX);
  display.setCursor(0, 20);
  display.setTextSize(1);
  display.print("offY=");
  display.setCursor(40, 20);
  display.setTextSize(1);
  display.print(offY);
  display.setCursor(0, 30);
  display.setTextSize(1);
  display.print("offZ=");
  display.setCursor(40, 30);
  display.setTextSize(1);
  display.print(offZ);
  display.setCursor(0, 40);
  display.setTextSize(1);
  display.print("___v.6.6___");
  display.display();
  delay(3000);
  
initializeMPU:
  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }

  // Enable bypass mode
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);

  // Initialize Initialize HMC5883L
  while (!compass.begin())
  {
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_75HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);

  // Setting the DLPF to lowest Bandwidth
  mpu.setDLPFMode(MPU6050_DLPF_6);

  Vector mag = compass.readRaw();
  //Serial.print("mag.ZAxis (minZ) = ");
  //Serial.println(mag.ZAxis);
  minZ=mag.ZAxis;
  
  if (buttonState == HIGH) {delay(15000);}
  
}

// No tilt compensation
float noTiltCompensate(Vector mag)
{
  float heading = atan2((mag.YAxis-offY), (mag.XAxis-offX));
  return heading;
}
 


// Correct angle
float correctAngle(float heading)
{
  if (heading < 0) { heading += 2 * PI; }
  if (heading > 2 * PI) { heading -= 2 * PI; }

  return heading;
}

//-----------------------------Главный цикл вычислений--------------------------------------------
void loop()
{

softreset:
   // Read vectors
  
  Vector acc = mpu.readScaledAccel();

calibrcompass:

   Vector mag = compass.readRaw();
   curentTime= millis(); //Значение текущего времени времени переполняется через 1193 часов
   if ((buttonState == LOW)||(curentTime > 100000 )||(flagPut==1)) { goto readycompass;}
    
  // Determine Min / Max values
  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;
  maxZ=mag.ZAxis;

  // Calculate offsets
  offX = (maxX + minX)/2;
  offY = (maxY + minY)/2;
  offZ = (maxZ + minZ)/2;
  Serial.print(mag.XAxis);
  Serial.print(":");
  Serial.print(mag.YAxis);
  Serial.print(":");
  Serial.print(mag.ZAxis);
  Serial.print(":");
  Serial.print(minX);
  Serial.print(":");
  Serial.print(maxX);
  Serial.print(":");
  Serial.print(minY);
  Serial.print(":");
  Serial.print(maxY);
  Serial.print(":");
  Serial.print(offX);
  Serial.print(":");
  Serial.print(offY);
  Serial.print(":");
  Serial.print(offZ);
  Serial.print("\n");

  // Выводим текст на LCD5110
  display.clearDisplay();
  display.setCursor(0, 0);
  // Устанавливаем размер текста
  display.setTextSize(1);
  display.print("TimeCalib<100");
  display.setCursor(0, 10);
  display.setTextSize(4);
  display.print(curentTime/1000);
  display.display();

  if ((curentTime > 99000)&&(curentTime < 100000) ) {
    EEPROM.put(0, offX);
    EEPROM.put(4, offY);
    EEPROM.put(8, offZ);
    flagPut=1;
    Serial.print("99000 < curentTime < 100000 , offsets X-Y-Z =  ");
    Serial.print(offX);
    Serial.print(": ");
    Serial.print(offY);
    Serial.print(": ");
    Serial.println(offZ);
    goto readycompass;
    }
  
  goto calibrcompass;


 readycompass:
  
  // Calculate headings
  heading1 = noTiltCompensate(mag);
 
  // Tilt compensation
  // Pitch & Roll 
    
  pitch = asin(-acc.XAxis);
  float LCDpitch = ALFA*pitch + (1-ALFA)*pitchold;
  pitchold=LCDpitch;
  roll = asin(acc.YAxis/cos(pitch));
  float LCDroll = ALFA*roll + (1-ALFA)*rollold;
  rollold=LCDroll;
  

    // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);
  
  // Tilt compensation
  Xh = (mag.XAxis-offX) * cosPitch + (mag.ZAxis-offZ) * sinPitch;
  Xh=ALFA*Xh+(1-ALFA)*Xhold;
  Xhold=Xh;
  Yh = (mag.XAxis-offX) * sinRoll * sinPitch + (mag.YAxis-offY) * cosRoll - (mag.ZAxis-offZ) * sinRoll * cosPitch;
  Yh=ALFA*Yh+(1-ALFA)*Yhold;
  Yhold=Yh;
 
  float heading2 = atan2(Yh, Xh);
    
  heading1 += declinationAngle;
  heading2 += declinationAngle;
  
  // Correct for heading < 0deg and heading > 360deg
  heading1 = correctAngle(heading1);
  heading2 = correctAngle(heading2);

  // Convert to degrees
  heading1 = heading1 * RTOG; 
  heading2 = heading2 * RTOG; 

  // check if returns are valid, if they are NaN (not a number)
  if (isnan(roll) || isnan(pitch) || isnan(Xh) || isnan(Yh) || isnan(heading2)){
    roll=0;
    pitch=0;
    rollold=0;
    pitchold=0;
    Xh=1;
    Yh=1;
    Xhold=1;
    Yhold=1;
    goto softreset;
    }
  if (isnan(heading2)) {heading2=heading1;} 
 
  //Температура внутри MPU6050
  float temp = mpu.readTemperature();
  // Output COM-port
  Serial.print("$");
  Serial.print(Xh);
  Serial.print(",");
  Serial.print(Yh);
  Serial.print(",");
  Serial.println(floor(Xh)+floor(Yh));
  //Serial.print(heading1);
  //Serial.print(":");
  //Serial.println(heading2);
  //Serial.print(":");
  //Serial.println(buttonState);
  //Serial.print(mag.XAxis-offX);
  //Serial.print(":");
  //Serial.print(mag.YAxis-offY);
  //Serial.print(":");
  //Serial.print(mag.ZAxis-offZ);
  //Serial.print("::::::::::");
  //Serial.print(offX);
  //Serial.print(":");
  //Serial.print(offY);
  //Serial.print(":");
  //Serial.println(offZ);
  
  

   // Выводим текст на LCD5110
  display.clearDisplay();
  display.setCursor(0, 0);
  // Устанавливаем размер текста
  display.setTextSize(1);
  display.print(round(heading1));
  display.setCursor(40, 0);
  display.setTextSize(1);
  display.print(round(temp));
  display.setCursor(0, 10);
  display.setTextSize(4);
  display.print(round(heading2));
  display.setCursor(0, 41);
  display.setTextSize(1);
  display.print(LCDroll*RTOG);
  display.setCursor(42, 41);
  display.setTextSize(1);
  display.print(LCDpitch*RTOG);
  display.display();


 // delay(100);
}

