 void overturn()
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

void ladder()
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