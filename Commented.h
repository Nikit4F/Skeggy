//ПЕРЕМЕНННЫЕ

//const byte inApin = 40;            //Пин направления А мотора
//const byte inBpin = 42;            //Пин направления В мотора
//const int pwmpin[2] = {44, 46};    //Пины ШИМ

//--------------------------------------------------------------------------------------------
//ВЫВОДЫ В ПОРТ

  //Вывод значений со всех каналов
  /*for (int channel = 1; channel <= channelAmount; ++channel) {
      unsigned long value = ppm.latestValidChannelValue(channel, 0);
      Serial.print(String(value) + " ");
    }
    Serial.println();*/

  //Вывод значения с определенного канала
  //Serial.print(ppm.latestValidChannelValue(6, 0));
  //Serial.println();

  /*Datinfo_AL1 = digitalRead(Dat_L1);
  Datinfo_AR1 = digitalRead(Dat_R1);
  Serial.print(Datinfo_AL1);
  Serial.print('\t');
  Serial.println(Datinfo_AR1);*/

  //СЕРВОПРИВОДЫ МАНИПУЛЯТОРА
  /*Serial.print(num);
  Serial.print('\t');
  Serial.println(angl);*/

//--------------------------------------------------------------------------------------------
//ТЩЕТНЫЕ ПОПЫТКИ В ЭНКОДЕР

// коэффициенты ПИД регулятора
// пропорциональный - от него зависит агрессивность управления, нужно увеличивать kp
// при увеличении нагрузки на вал, чтобы регулятор подавал больший управляющий ШИМ сигнал
//float kp = 1.0;		// (знач. по умолчанию)

// интегральный - позволяет нивелировать ошибку со временем, имеет накопительный эффект
//float ki = 0.9;		// (знач. по умолчанию)

// дифференциальный. Позволяет чуть сгладить рывки, но при большом значении
// сам становится причиной рывков и раскачки системы!
//float kd = 0.1;		// (знач. по умолчанию)
// НУЖНЫЕ ПЕРЕМЕННЫЕ
/*const float toDeg = 180.0 / M_PI;
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer
Quaternion q;            // [w, x, y, z]         quaternion container
VectorFloat gravity;     // [x, y, z]            gravity vector
float ypr[3];            // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector*/

//GMotor LmotorSpin(DRIVER3WIRE, 33, inApin, pwmpin[0], HIGH);
//GMotor RmotorSpin(DRIVER3WIRE, 35, inBpin, pwmpin[1], HIGH);

  //overturn_motor.setTargetDeg(30);  // задаём новый целевой угол в градусах
  // обязательная функция. Делает все вычисления
  // принимает текущее значение с энкодера или потенциометра
  //overturn_motor.tick(encTick(4));

  /*static uint32_t tmr = 0;
  if (millis() - tmr > 100) {   // таймер на 100мс для графиков
    tmr += 100;
    // отладка позиции (открой плоттер)
    Serial.print(overturn_motor.getTargetDeg());
    Serial.print('\t');
    Serial.print(overturn_motor.getDuty());
    Serial.print('\t');
    Serial.print(overturn_motor.getCurrentDeg());
    Serial.print('\t');
    Serial.println (encTick(gpin));
  }*/

//--------------------------------------------------------------------------------------------
//СТАРЫЙ ПЕРЕВОРОТ

  //Старт переворота
  /*if (data6 >= 1900 && data6 <= 2100 && chek == 0)
    {
    chek = 1;
    data6 = ppm.latestValidChannelValue(6, 0);
    Serial.print("Start");
    LeftSpeedTime = millis();
    while (millis () - LeftSpeedTime < 1000)
    {
      LmotorSpin.smoothTick(200);
    }
    RightSpeedTime = millis();
    while (millis () - RightSpeedTime < 1000)
    {
      RmotorSpin.smoothTick(100);
    }
    AllSpeedStopTime = millis();
      while (millis() - AllSpeedStopTime < 200)
      {
      RmotorSpin.smoothTick(0);
      LmotorSpin.smoothTick(0);
      //LmotorSpin.setMode(STOP);
      //RmotorSpin.setMode(STOP);
      }

    Serial.print("End");
    //data6 = ppm.latestValidChannelValue(6, 0);
    }
    if (data6 < 1800 && chek == 1)
    {
      Serial.print("Stop");
      chek = 0;
      RmotorSpin.smoothTick(0);
      LmotorSpin.smoothTick(0);
    }*/

//--------------------------------------------------------------------------------------------
//НЕ ТРОГАТЬ ТЕСТОВЫЙ ПЕРЕВОРОТ

  /*spinLeft = map(data4, 1000, 1350, 230, 0);
  spinRight = map(data4, 1650, 2000, 0, 20);
  if (data4 > 1000 && data4 < 1350) {
    LmotorSpin.smoothTick(spinLeft);
  }
  if (data4 > 1650 && data4 < 2000) {
    RmotorSpin.smoothTick(spinRight);
  }
  if (data4 > 1350 && data4 < 1650) {
    LmotorSpin.smoothTick(0);
    RmotorSpin.smoothTick(0);
  }*/

//--------------------------------------------------------------------------------------------
//MPU6050

  /*Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  initDMP();*/

   /*if (millis() - tmr >= 11) {  // таймер на 11 мс (на всякий случай)
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
  }*/

  /*void initDMP() {
  devStatus = mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  packetSize = mpu.dmpGetFIFOPacketSize();
}*/
// получение углов в angleX,
/*void getAngles() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angleX = ypr[2] * toDeg;
    angleY = ypr[1] * toDeg;
  }
}*/

//--------------------------------------------------------------------------------------------
//ФУНКЦИИ МОТОРОВ
  
  /*pinMode(inApin, OUTPUT);     // выводы ключей A
  pinMode(inBpin, OUTPUT);     // выводы ключей B
  pinMode(pwmpin[1], OUTPUT);     // выводы ШИМ
  pinMode(pwmpin[2], OUTPUT);

  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);

  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);

  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);*/

  //функция запуска моторов езды
/*void motorgo(uint8_t motor, uint8_t direct, uint8_t pwm) {
  if (motor == MOTOR_1) {
    if (direct == CW) {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, HIGH);
    } else if (direct == CCW) {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);
    } else {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);
    }

    analogWrite(PWM_MOTOR_1, pwm);
  } else if (motor == MOTOR_2) {
    if (direct == CW) {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    } else if (direct == CCW) {
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);
    } else {
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);
    }

    analogWrite(PWM_MOTOR_2, pwm);
  }
}*/

// функция выключения моторов переворота:
/*void motorOff() {
  digitalWrite(inApin, LOW);
  digitalWrite(inBpin, LOW);
  analogWrite(pwmpin[0], 0);
  analogWrite(pwmpin[1], 0);
}

// функция включения моторов переворота:
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) {
  // если номер мотора правильный:
  if (motor <= 1) {
    // если направление совпадает со значениями направлений:
    if (direct <= 3) {
      // если направление мотора по часовой или плавный стоп,
      // устанавливаем соответствующие значения ключа А выбранного мотора:
      if (direct == 1) {
        digitalWrite(inApin, HIGH);
        //digitalWrite(inBpin, LOW);
      } else
        digitalWrite(inApin, LOW);

      // если направление мотора по часовой или резкий стоп,
      // устанавливаем соответствующие значения ключа B выбранного мотора:
      if ((direct == 0) || (direct == 2)) {
        digitalWrite(inBpin, HIGH);
        // digitalWrite(inApin, LOW);
      } else
        digitalWrite(inBpin, LOW);
      // устанавливаем ШИМ выбранного мотора
      analogWrite(pwmpin[motor], pwm);
    }
  }
}*/

//--------------------------------------------------------------------------------------------
//ДИАГОНАЛИ

   /*if (data2 > 1700 && data2 <= 2000 && data1 >= 1000 && data1 <= 1300)  //диагональ влево
    {
      speed3 = map(data1, 1000, 1300, 0, 150);
      //speed4 = map(data2, 1700, 2000, 180, 0);
      motorR.smoothTick(speed4);
      motorL.smoothTick(speed3);
      Serial.println("Диагональ влево");
    }
    if (data2 > 1700 && data2 <= 2000 && data1 >= 1700 && data1 <= 2000)  //диагональ вправо
    {
      speed3 = map(data1, 1700, 2000, 150, 0);
      //speed4 = map(data2, 1700, 2000, 0, 180);
      motorR.smoothTick(speed3);
      motorL.smoothTick(speed4);
      Serial.println("Диагональ вправо");
    }*/

//--------------------------------------------------------------------------------------------
//ПЕРЕКЛЮЧЕНИЕ РЕЖИМОВ ЗАДНЕЙ ОСИ

  /*if(data8 == 1000) //Переключение режимов задней оси
  {
    servo2.writeMicroseconds(0);
    servo1.writeMicroseconds(0);
  }
  if (data8 >= 1400 && data8 <=1600)
  {
    servo2.writeMicroseconds(1300);
    servo1.writeMicroseconds(1300);
    }
  if (data8 == 2000)
  {
    if(millis() - timer3 >100)
    {
    Axis_rotation_angle = map(data3, 1000, 2000, 1300, 0);
    servo1.writeMicroseconds(Axis_rotation_angle);
    servo2.writeMicroseconds(Axis_rotation_angle);
    timer3 = millis();
    }
  }*/
   