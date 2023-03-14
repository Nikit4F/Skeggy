#include <GyverMotor.h>
#include <PPMReader.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Autoline.h"
#include "Servo.h"
MPU6050 mpu;

//Константы
#define MIN_DUTY 0  //минимальное значение которое подается на моторы

//Пины
const byte overturn_Apin = 40;  //Пин направления А переворота
const byte overturn_Bpin = 42;  //Пин направления B переворота
const byte overturn_pwm = 44; //ШИМ пин
int echoPin = 26;// this pin recive echo, reflected audio signal
int trigPin = 24;// this pin generate audio signal

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
//int data5;  //Объявлена в приложении
int data6;  //Подъем и спуск по наклонной
int data7;  //
int data8;  //Лестница
int data9;  //Переворот
int data10;
int data11;
int data12;




//Всякие переменные
int speed1; 
int speed2;
int Axis_rotation_angle ;
bool check = 0;                     //флажок переворота
bool autoline_flag = 1;                     //флажок автолинии
int angl = 180;
int num = 90;
int duration;//variable for delay
int cm;//variable for distance
bool flag = 0;

//Сервоприводы
Servo grab_servo;
Servo main_servo;
Servo servo1;
Servo servo2;

//Инициализация драйверов и серв
GMotor overturn_motor(DRIVER3WIRE, overturn_Apin, overturn_Bpin, overturn_pwm, HIGH);
//Servo servo1;
//Servo servo2;


void setup() {
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
  // data9 = ppm.latestValidChannelValue(9, 0);
  // data10 = ppm.latestValidChannelValue(10, 0);
  // data11 = ppm.latestValidChannelValue(11, 0);
  // data12 = ppm.latestValidChannelValue(12, 0);

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
  } else {
    if (data1 > 1450 && data1 < 1550) {  //стоп
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
    delay(100);
    overturn_motor.smoothTick(-255);
    delay(1500);
    overturn_motor.smoothTick(0);
    delay(300);
    overturn_motor.smoothTick(255);
    delay(600);
    overturn_motor.smoothTick(0);
    delay(100);

  } else if (data7 <= 1800 && check == 1) {
    check = 0;
    overturn_motor.smoothTick(0);
  }

  if (data6 >= 1400 && data6 <=1600)  //подъем по наклонной
    {
      if (millis() - tmr >= 11) 
      {  // таймер на 11 мс (на всякий случай)
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
          {
            // переменные для расчёта (ypr можно вынести в глобал)
          Quaternion q;
          VectorFloat gravity;

          // расчёты
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          // выводим результат в радианах (-3.14, 3.14)
          Serial.print(degrees(ypr[0])); // вокруг оси Z
          Serial.print('\t');
          Serial.print(degrees(ypr[1])); // вокруг оси Y
          Serial.print('\t');
          Serial.print(degrees(ypr[2])); // вокруг оси X
          Serial.println();
          // для градусов можно использовать degrees()
          tmr = millis();  // сброс таймера
        }
      }
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
    if (millis() - tmr >= 11) {  // таймер на 11 мс (на всякий случай)
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // переменные для расчёта (ypr можно вынести в глобал)
      Quaternion q;
      VectorFloat gravity;

      // расчёты
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // выводим результат в радианах (-3.14, 3.14)
      /*Serial.print(degrees(ypr[0])); // вокруг оси Z
      Serial.print('\t');
      Serial.print(degrees(ypr[1])); // вокруг оси Y
      Serial.print('\t');
      Serial.print(degrees(ypr[2])); // вокруг оси X
      Serial.println();*/
      // для градусов можно использовать degrees()
      tmr = millis();  // сброс таймера
    }
  }
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
    if (millis() - tmr >= 11) {  // таймер на 11 мс (на всякий случай)
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      // переменные для расчёта (ypr можно вынести в глобал)
      Quaternion q;
      VectorFloat gravity;

      // расчёты
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // выводим результат в радианах (-3.14, 3.14)
      Serial.print(degrees(ypr[0])); // вокруг оси Z
      Serial.print('\t');
      Serial.print(degrees(ypr[1])); // вокруг оси Y
      Serial.print('\t');
      Serial.print(degrees(ypr[2])); // вокруг оси X
      Serial.println();
      // для градусов можно использовать degrees()
      tmr = millis();  // сброс таймера
    }
  }
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
//управление манипулятором
if (data3 > 1900) {
      //if(millis() - timer3 > 200)
      //{
          num++;
          num = constrain(num, 10, 110);
          //timer3 = millis();
      //}
    }
    if (data3 < 1100) {
      //if(millis() - timer4 > 200)
      //{
          num--;
          num = constrain(num, 10, 110);
          //timer4 = millis();
      //}
    }
    main_servo.write(num);

if (data6 >= 1400 && data6 <=1600) {
  if (data4 >= 1600) {
    //if(millis() - timer1 > 1)
    //{
      angl--;
      angl = constrain(angl, 120, 175);
      //timer1 = millis();
      //}
    }
  if (data4 <= 1400) {
    //if(millis() - timer2 > 1)
      //{
        angl++;
        angl = constrain(angl, 120, 175);
       // timer2 = millis();
      //}
    }
  grab_servo.write(angl);
  }
  /*Serial.print(num);
  Serial.print('\t');
  Serial.println(angl);*/

  if(data8>1900)  //Запуск лестницы
{
  if (millis() - tmr >= 11) 
      {  // таймер на 11 мс (на всякий случай)
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
          {
            // переменные для расчёта (ypr можно вынести в глобал)
          Quaternion q;
          VectorFloat gravity;

          // расчёты
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          // выводим результат в радианах (-3.14, 3.14)
          Serial.print(degrees(ypr[0])); // вокруг оси Z
          Serial.print('\t');
          Serial.print(degrees(ypr[1])); // вокруг оси Y
          Serial.print('\t');
          Serial.println(degrees(ypr[2])); // вокруг оси X
          // для градусов можно использовать degrees()
          tmr = millis();  // сброс таймера
        }
      }
  digitalWrite(trigPin, LOW);//reset pin state. dont generate audio signal
  delayMicroseconds(2);// waiting  decay
  digitalWrite(trigPin, HIGH);//prepare to generate audio signal
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);//generate audio signal
  duration = pulseIn(echoPin, HIGH);//recive echo. calculate delay time
  cm = duration / 58;//calculate distance time. s=v*t v=340 m/s=1/29 m/mcs. andio signal travelled the distance to the site and back, dividing the result by 2.
  //Serial.print(cm);
  //Serial.println(" cm");
  if (cm > 1 && cm < 9)
  {
    flag = 1;
  }
    if(flag == 1){
      motorL.smoothTick(-115);
      motorR.smoothTick(-115);
      servo1.write(30);
      servo2.write(60);
    }

    if(degrees(ypr[1]) < -59)
    {
      flag = 0;
      motorL.smoothTick(0);
      motorR.smoothTick(0);
      servo1.write(180);
      servo2.write(75);
    }
}
else 
{
  servo1.write(0);
  servo2.write(60);
}

} //Конец лупа

void ObliqueUp() 
{
  motorR.smoothTick(-135);
  motorL.smoothTick(-135);
}

void ObliqueDown() 
{
  motorR.smoothTick(-30);
  motorL.smoothTick(-30);
}
void ObliqueUpFast()
{
  motorR.smoothTick(-200);
  motorL.smoothTick(-200);
}


