//------------------------------------------------------------------------------------------------------------------------------------------
//                                        П И Н Ы

#define Motor_L_A 4                     // Motor Left вывод A
#define Motor_L_B 9                     // Motor Left вывод B
#define Motor_L_PWM 6                   // Подача ШИМ на Motor Left
#define Encoder_L 32                    // Энкодер на Motor Left
#define Motor_R_A 7                     // Motor Right вывод A
#define Motor_R_B 8                     // Motor Right вывод B
#define Motor_R_PWM 5                   // Подача ШИМ на Motor Right
#define Encoder_R 34                    // Энкодер на Motor Right
#define TRIG_PIN 20                     // Trig pin УЗ датчика
#define ECHO_PIN 21                     // Echo pin УЗ датчика
#define LED_PIN 13                      // LED pin 

//------------------------------------------------------------------------------------------------------------------------------------------
//                             К О Н С Т А Н Т Ы         К О Э Ф Ф И Ц И Е Н Т Ы

#define speedMove 100                   // Скорость движения
#define speedRotate 80                  // Сниженная скорость поворота
#define OBSTACLE_DISTANCE 30            // Расстояние до препятствия (см)
#define CLEAR_DISTANCE 40               // Безопасное расстояние после объезда
#define TURN_CHECK_DELAY 100            // Интервал проверки при повороте (мс)

//------------------------------------------------------------------------------------------------------------------------------------------
//                                        П Е Р Е М Е Н Н Ы Е

volatile long counterL = 0;             // Счетчик энкодера левого мотора
volatile long counterR = 0;             // Счетчик энкодера правого мотора
bool avoidingObstacle = false;          // Флаг объезда препятствия

//------------------------------------------------------------------------------------------------------------------------------------------
//                                   Ф У Н К Ц И И

void cMove(int spdL, int spdR) {
  spdL = constrain(spdL, -255, 255);
  spdR = constrain(spdR, -255, 255);

  // Левый мотор (исправленная конфигурация)
  if(spdL > 0) {
    digitalWrite(Motor_L_A, HIGH);
    digitalWrite(Motor_L_B, LOW);
    analogWrite(Motor_L_PWM, spdL);
  } else {
    digitalWrite(Motor_L_A, LOW);
    digitalWrite(Motor_L_B, HIGH);
    analogWrite(Motor_L_PWM, -spdL);
  }

  // Правый мотор (исправленная конфигурация)
  if(spdR > 0) {
    digitalWrite(Motor_R_A, LOW);      // ИНВЕРТИРОВАНО
    digitalWrite(Motor_R_B, HIGH);     // ИНВЕРТИРОВАНО
    analogWrite(Motor_R_PWM, spdR);
  } else {
    digitalWrite(Motor_R_A, HIGH);     // ИНВЕРТИРОВАНО
    digitalWrite(Motor_R_B, LOW);       // ИНВЕРТИРОВАНО
    analogWrite(Motor_R_PWM, -spdR);
  }
}

float getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Таймаут 30 мс
  if (duration == 0) return 200; // Возвращаем большое значение при таймауте
  return duration * 0.034 / 2; // Расстояние в см
}

//------------------------------------------------------------------------------------------------------------------------------------------
//                                         S E T U P
void setup() {
  pinMode(Motor_L_A, OUTPUT);
  pinMode(Motor_L_B, OUTPUT);
  pinMode(Motor_L_PWM, OUTPUT);
  pinMode(Encoder_L, INPUT);
  pinMode(Motor_R_A, OUTPUT);
  pinMode(Motor_R_B, OUTPUT);
  pinMode(Motor_R_PWM, OUTPUT);
  pinMode(Encoder_R, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(Encoder_L), []{ counterL++; }, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_R), []{ counterR++; }, RISING);
  
  Serial.begin(9600);
}

//------------------------------------------------------------------------------------------------------------------------------------------
//                                            L O O P
void loop() {
  float distance = getUltrasonicDistance();
  
  if (!avoidingObstacle) {
    if (distance > 0 && distance < OBSTACLE_DISTANCE) {
      // Обнаружено препятствие - начинаем объезд
      avoidingObstacle = true;
      cMove(0, 0); // Короткая остановка
      delay(300);
      // Начинаем поворот
      cMove(-speedRotate, speedRotate); // Поворот налево
    } else {
      // Движение вперед без коррекции
      cMove(speedMove, speedMove);
    }
  } 
  else {
    // Режим объезда препятствия
    // Проверяем расстояние каждые TURN_CHECK_DELAY мс
    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > TURN_CHECK_DELAY) {
      lastCheck = millis();
      distance = getUltrasonicDistance();
      
      // Если путь свободен - завершаем объезд
      if (distance > CLEAR_DISTANCE || distance <= 0) {
        avoidingObstacle = false;
        cMove(0, 0); // Останавливаемся после поворота
        delay(300);
      }
    }
  }
}