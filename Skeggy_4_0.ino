#include <GyverMotor.h>
#include <PPMReader.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Skeggy.h"
#include "Servo.h"
MPU6050 mpu;

//Константы
const int MIN_DUTY = 0;  //минимальное значение которое подается на моторы

//Пины
const byte overturn_Apin = 40;  //Пин направления А переворота
const byte overturn_Bpin = 42;  //Пин направления B переворота
const byte overturn_pwm = 44; //ШИМ пин
int echoPin = 26; // this pin recive echo, reflected audio signal
int trigPin = 24; // this pin generate audio signal
int Dat_L1 = 2; //Пин левого ИК датчика
int Dat_R1 = 3; //Пин правого ИК датчика

//Мотор 1
const int MOTOR_A1_PIN = 7;
const int MOTOR_B1_PIN = 8;

//Мотор 2
const int MOTOR_A2_PIN = 4;
const int MOTOR_B2_PIN = 9;

//ШИМ моторов
const int PWM_MOTOR_1 = 5;
const int PWM_MOTOR_2 = 6;

//Таймеры
int myTimer1 = millis();
int timerangle = millis();
int autotime = millis();
int datatime = millis();
int RightSpeedTime = millis();
int LeftSpeedTime = millis();
int LeftSpeedStopTime = millis();
int AllSpeedStopTime = millis();
int TimeToSpin = millis();
int timer1 = millis();
int timer2 = millis();
int timer3 = millis();
int timer4 = millis();
unsigned long timingAL = millis(); //поворот влево
unsigned long timingAR = millis(); //поворот вправо
unsigned long timingAB = millis(); //проезд через черную линию
unsigned long timingAW = millis(); //белый фон
uint32_t nowToward = millis();
uint32_t nowLeft = millis();
uint32_t nowRight = millis();

//MPU6050 переменные
float angleX = 0;
float angleY = 0;
#define M_PI 3.14159265358979323846
uint8_t fifoBuffer[45];  
static uint32_t tmr;
float ypr[3];
float constant_angle;

//Даты
int data1;  //Вправо влево
int data2;  //Вперед назад
int data3;  //Вперед назад
int data4;  //Влево вправо
int data5;  //Автолиния
int data6;  //Подъем и спуск по наклонной
int data7;  //Переворот
int data8;  //Лестница

//Всякие переменные
int speed1; 
int speed2;
int Axis_rotation_angle ;
bool check = 0;                     //флажок переворота
bool autoline_flag = 1;             //флажок автолинии
int angl = 180;
int num = 90;
int duration;//variable for delay
int cm;//variable for distance
bool flag = 0;
int Datinfo_AL1 = 0;  //Данные с левого ИК датчика
int Datinfo_AR1 = 0;  //Данные с правого ИК датчика
int autoflag;
int interruptPin = 19; //Пин PPMReader
int channelAmount = 8; //Ожидаемое число каналов

//Сервоприводы
Servo grab_servo;
Servo main_servo;
Servo servo1;
Servo servo2;

//Инициализация драйверов
GMotor overturn_motor(DRIVER3WIRE, overturn_Apin, overturn_Bpin, overturn_pwm, HIGH);
GMotor motorL(DRIVER3WIRE, MOTOR_A1_PIN, MOTOR_B1_PIN, PWM_MOTOR_1, HIGH);
GMotor motorR(DRIVER3WIRE, MOTOR_A2_PIN, MOTOR_B2_PIN, PWM_MOTOR_2, HIGH);

//Инициализируем PPMReader на 19 пин с 8 каналами
PPMReader ppm(interruptPin, channelAmount);


void setup() 
{
  //Подключение серв
  Serial.begin(57600);
  pinMode(32, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(33, OUTPUT);
  grab_servo.attach(32);
  main_servo.attach(30);
  grab_servo.write(angl);
  main_servo.write(65);
  servo1.attach(31);
  servo2.attach(33);
  servo1.write(30);
  servo2.write(60);
  
  //Режим пинов
  pinMode(overturn_Apin, OUTPUT);
  pinMode(overturn_Apin, OUTPUT);
  pinMode(overturn_pwm, OUTPUT);  
  pinMode(Dat_L1 , INPUT);
  pinMode(Dat_R1 , INPUT);
  pinMode(trigPin, OUTPUT);//configurate trigPin as output
  pinMode(echoPin, INPUT);//configurate echoPin as input

  //Режим определения направления мотора
  motorR.setMode(AUTO);
  motorL.setMode(AUTO);
  overturn_motor.setMode(AUTO);

  //Плавность скорости моторов
  motorR.setSmoothSpeed(100);
  motorL.setSmoothSpeed(100);
  overturn_motor.setSmoothSpeed(100);

  //Мин. сигнал вращения
  motorR.setMinDuty(MIN_DUTY);
  motorL.setMinDuty(MIN_DUTY);
  overturn_motor.setMinDuty(255);

  //Направление
  motorR.setDirection(NORMAL);
  motorL.setDirection(NORMAL);
  overturn_motor.setDirection(NORMAL);

  Serial.println("Begin motor control");

  Wire.begin();
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
}

void loop() {

  int serv1 = map (data3, 1000, 2000, 0, 180);
  servo1.write(serv1);

    //Вывод значений со всех каналов
  for (int channel = 1; channel <= channelAmount; ++channel) {
      unsigned long value = ppm.latestValidChannelValue(channel, 0);
      Serial.print(String(value) + " ");
    }
    Serial.println();

  //Считывание значений с каналов
  data1 = ppm.latestValidChannelValue(1, 0);
  data2 = ppm.latestValidChannelValue(2, 0);
  data3 = ppm.latestValidChannelValue(3, 0);
  data4 = ppm.latestValidChannelValue(4, 0);
  data5 = ppm.latestValidChannelValue(5, 0);
  data6 = ppm.latestValidChannelValue(6, 0);
  data7 = ppm.latestValidChannelValue(7, 0);
  data8 = ppm.latestValidChannelValue(8, 0);

  if (data2 > 1550 || data2 < 1450 || data1 > 1550 || data1 < 1450)  //движение
  {
    if (data2 > 1600 && data2 <= 2000 && data1 <= 1700 && data1 >= 1300)  //вперед
    {
      speed2 = map(data2, 1600, 2000, 0, -255);
      motorR.smoothTick(speed2);
      motorL.smoothTick(speed2);
      Serial.println("Вперед");
    }
    if (data2 >= 1000 && data2 < 1400 && data1 <= 1700 && data1 >= 1300)  //назад
    {
      speed2 = map(data2, 1000, 1400, 255, 0);
      motorR.smoothTick(speed2);
      motorL.smoothTick(speed2);
      Serial.println("Назад");
    }
    if (data1 > 1600 && data1 <= 2000 && data2 < 1700 && data2 >= 1300)  //вправо
    {
      speed1 = map(data1, 1600, 2000, 0, 255);
      motorR.smoothTick(-speed1);
      motorL.smoothTick(speed1);
      Serial.println("Вправо");
    }
    if (data1 >= 1000 && data1 < 1400 && data2 < 1700 && data2 >= 1300)  //влево
    {
      speed1 = map(data1, 1000, 1400, 255, 0);
      motorR.smoothTick(speed1);
      motorL.smoothTick(-speed1);
      Serial.println("Влево");
    }
  } 
  else 
  {
    if (data1 > 1450 && data1 < 1550) //стоп
    {
      //motorR.smoothTick(0);
      //motorL.smoothTick(0);
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, HIGH);
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, HIGH);
      analogWrite(PWM_MOTOR_1, 0);
      analogWrite(PWM_MOTOR_2, 0);
      //Serial.println("Стоп");
    }
  }

  if (data7 >= 1900 && check == 0)  //переворот
  {
    check = 1;
    overturn();
  } 
  else if (data7 <= 1800 && check == 1) 
  {
    check = 0;
    overturn_motor.smoothTick(0);
  }

  if (data6 >= 1400 && data6 <=1600)  //подъем по наклонной
  {
    mpuStart();
    if (degrees(ypr[1]) < -10)
     {
       if (millis() - myTimer1 >= 200) 
        { 
          ObliqueUp();
          data6 = ppm.latestValidChannelValue(7, 0);
          myTimer1 = millis();
        }
      }
  }

  if (data6 >= 1400 && data6 <=1600)  //спуск по наклонной
  {
    mpuStart();
  if (degrees(ypr[1]) > 10)
  {
      if (millis() - myTimer1 >= 200) 
      { 
        ObliqueDown();
        data6 = ppm.latestValidChannelValue(7, 0);
        myTimer1 = millis();
      }
    }
  }

  if (data6 >= 1900)  //быстрый подъем по наклонной
  {
    mpuStart();
  if (degrees(ypr[1]) < -15)
  {
      if (millis() - myTimer1 >= 200) 
      { 
        ObliqueUpFast();
        data6 = ppm.latestValidChannelValue(7, 0);
        myTimer1 = millis();
      }
    }
  }

  if (data5 == 2000 && autoline_flag == 0 && data5 != 1000)  //Автолиния
  {
    data5 = ppm.latestValidChannelValue(5, 0);
    Autoline();
  }

  if (data5 >= 1500)  //включение автолинии
  {
    autoline_flag = 0;
  } else  //отключение автолинии
  {
    autoline_flag = 1;
  }

  if(data8>1900)  //Запуск лестницы
{
  ladder();
}
else 
{
  servo1.write(0);
  servo2.write(60);
}

} //Конец лупа