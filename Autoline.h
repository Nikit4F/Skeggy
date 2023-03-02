#include <Arduino.h>
#include <PPMReader.h>

//Подключаем пины
//MOTOR 1
#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8

//MOTOR 2
#define MOTOR_A2_PIN 4
#define MOTOR_B2_PIN 9

//ШИМ моторов
#define PWM_MOTOR_1 5
#define PWM_MOTOR_2 6


// a1-left, a2-right;


//Инициализируем моторы в библиотеке
GMotor motorL(DRIVER3WIRE, MOTOR_A1_PIN, MOTOR_B1_PIN, PWM_MOTOR_1, HIGH);
GMotor motorR(DRIVER3WIRE, MOTOR_A2_PIN, MOTOR_B2_PIN, PWM_MOTOR_2, HIGH);
//переменные скорости

//Таймеры
unsigned long timingAL = millis(); //поворот влево
unsigned long timingAR = millis(); //поворот вправо
unsigned long timingAB = millis(); //проезд через черную линию
unsigned long timingAW = millis(); //белый фон
uint32_t nowToward = millis();
uint32_t nowLeft = millis();
uint32_t nowRight = millis();

//Переменные
int Datinfo_AL1 = 0;
int Datinfo_AR1 = 0;
int autoflag;
int data5 = 1000;
int interruptPin = 19; //Пин PPMReader
int channelAmount = 8; //Ожидаемое число каналов

//Пины
int Dat_L1 = 2;
int Dat_R1 = 3;

//Инициализируем PPMReader на 19 пин с 8 каналами
PPMReader ppm(interruptPin, channelAmount);

void Autoline()
{
  // считывание информации с пинов
  Datinfo_AL1 = digitalRead(Dat_L1);
  Datinfo_AR1 = digitalRead(Dat_R1);
  autoflag = 0;
  if (data5 == 2000)
  {
    if (Datinfo_AL1 == Datinfo_AR1) // вперед
    {
      if (millis() - timingAW > 150)
      {
        nowToward = millis();
        while (millis() - nowToward < 50) {
          motorR.smoothTick(-70);
          motorL.smoothTick(-70);
          timingAW = millis();
        }
      }
    }
    else
    {
      if (Datinfo_AR1 == 0) // вправо
      {
        if (millis() - timingAR > 100)
        {
          nowRight = millis();
          while (millis() - nowRight < 50) {
            motorR.smoothTick(60);
            motorL.smoothTick(-60);
            timingAR = millis();
          }
        }
      }

      if (Datinfo_AL1 == 0) // влево
      {
        if (millis() - timingAL > 100)
        {
          nowLeft = millis();
          while (millis () - nowLeft < 50) {
            motorR.smoothTick(-60);
            motorL.smoothTick(60);
            timingAL = millis();
          }
        }
      }
    }
  }
}
