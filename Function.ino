void overturn()  //Функция переворота
{
  Serial.println("переворот");
  delay(100);
  overturn_motor.smoothTick(-255);
  delay(1500);
  overturn_motor.smoothTick(0);
  delay(300);
  overturn_motor.smoothTick(255);
  delay(600);
  overturn_motor.smoothTick(0);
  delay(100);
}

void mpuStart()  //Функция запуска гироскопа
{
  if (millis() - tmr >= 11)  // таймер на 11 мс (на всякий случай)
  {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))  // переменные для расчёта (ypr можно вынести в глобал)
    {
      Quaternion q;
      VectorFloat gravity;

      // расчёты
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // выводим результат в радианах (-3.14, 3.14)
      Serial.print(degrees(ypr[0]));  // вокруг оси Z
      Serial.print('\t');
      Serial.print(degrees(ypr[1]));  // вокруг оси Y
      Serial.print('\t');
      Serial.print(degrees(ypr[2]));  // вокруг оси X
      Serial.println();
      // для градусов можно использовать degrees()
      tmr = millis();  // сброс таймера
    }
  }
}

int ultrasonicSensorStart(int trigPin, int echoPin)  //Функция включения ультразвукового датчика и получение расстояния с него
{
  digitalWrite(trigPin, LOW);   //reset pin state. dont generate audio signal
  delayMicroseconds(2);         // waiting  delay
  digitalWrite(trigPin, HIGH);  //prepare to generate audio signal
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);         //generate audio signal
  duration = pulseIn(echoPin, HIGH);  //recive echo. calculate delay time
  cm = duration / 58;                 //calculate distance time. s=v*t v=340 m/s=1/29 m/mcs. andio signal travelled the distance to the site and back, dividing the result by 2.
  //Serial.print(cm);
  //Serial.println(" cm");
  return (cm);
}

void ladder()  //Функция подъема по лестнице
{
  mpuStart();
  int dist = ultrasonicSensorStart(trigPin, echoPin);
  if (dist > 1 && dist < 9) {
    flag = 1;
  }
  
  if (flag == 1) {
    goLadder();
  }

  if (degrees(ypr[2]) > 55) {
    flag = 0;
    upLadder();
  }
}

void goLadder()  //Функция езды вперед в режиме лестницы
{
  motorL.smoothTick(-130);
  motorR.smoothTick(-130);
  servo1.write(40);
  servo2.write(60);
}

void upLadder() //Функция распрямления хвоста в режиме лестницы
{
  motorL.smoothTick(0);
  motorR.smoothTick(0);
  servo1.write(180);
  servo2.write(75);
}

void ObliqueUp()  //Функция поднятия наверх по наклонной с обычной скоростью
{
  Serial.println("Наклонная");
  motorR.smoothTick(-150);
  motorL.smoothTick(-150);
}

void ObliqueDown()  //Функция спуска вниз по наклонной с обычной скоростью
{
  Serial.println("Наклоная вниз");
  motorR.smoothTick(-30);
  motorL.smoothTick(-30);
}
void ObliqueUpFast()  //Функция поднятия наверх по наклонной с быстрой скоростью
{
  Serial.println("Наклонная быстро");
  motorR.smoothTick(-200);
  motorL.smoothTick(-200);
}

void stopMotors()
{
  digitalWrite(MOTOR_A1_PIN, HIGH);
  digitalWrite(MOTOR_B1_PIN, HIGH);
  digitalWrite(MOTOR_A2_PIN, HIGH);
  digitalWrite(MOTOR_B2_PIN, HIGH);
  analogWrite(PWM_MOTOR_1, 0);
  analogWrite(PWM_MOTOR_2, 0);
}

void Autoline() //Функция запуска автолинии
{
  // считывание информации с пинов
  Datinfo_AL1 = digitalRead(Dat_L1);
  Datinfo_AR1 = digitalRead(Dat_R1);
  // Datinfo_AR2 = digitalRead(Dat_R2);
  // Datinfo_AL2 = digitalRead(Dat_L2);
  Serial.print(Datinfo_AL1);
  Serial.print('\t');
  Serial.print(Datinfo_AR1);
  // Serial.print('\t');
  // Serial.print(Datinfo_AR1);
  // Serial.print('\t');
  // Serial.println(Datinfo_AR2);
  autoflag = 0;
  if (data5 >= 1900) {
    if (Datinfo_AL1 == 0 && Datinfo_AR1 == 0)  // вперед
    {
      if (millis() - timingAW > 150) 
      {
        //nowToward = millis();
        //while (millis() - nowToward < 75) {
          motorR.smoothTick(-30);
          motorL.smoothTick(-30);
          timingAW = millis();
        //}
       }
    } 
     else
     {
       if (Datinfo_AR1 == 1)  // вправо
       {
        if (millis() - timingAR > 40) {
          nowRight = millis();
          while (millis() - nowRight < 100) {
            motorR.smoothTick(60);
            motorL.smoothTick(-60);
            timingAR = millis();
          }
        }
      }

      if (Datinfo_AL1 == 1)  // влево
      {
        if (millis() - timingAL > 40) {
          nowLeft = millis();
          while (millis() - nowLeft < 100) {          
            motorR.smoothTick(-60);
            motorL.smoothTick(60);
            timingAL = millis();
          }
        }
      }
    }
  }
}

void goForGrab()
{
  motorR.smoothTick(-30);
  motorL.smoothTick(-30);
}

void autoGrab() //функция включения автозахвата банки
{
  int distForGrab = ultrasonicSensorStart(trigPin, echoPin);
  if (distForGrab >=10 && grabFlag == 1) 
  {
    //Serial.println("Ehay");
    grab_servo.write(150);
    main_servo.write(10);
    goForGrab();
  }
  if(distForGrab < 10 && grabFlag == 1)
  {
    Serial.print("Hvatai");
    Serial.print('\t');
    Serial.println(data7);
    stopMotors();
    delay(500);
    main_servo.write(130);
    delay(1000);
    grab_servo.write(90);
    delay(500);
    main_servo.write(30);
    grabFlag = 0;
  }
}

