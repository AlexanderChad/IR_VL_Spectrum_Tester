#include <avr/io.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>

#define IR_LED 3 //пин ИК диода
#define VL_LED 4 //пин светодиода

LiquidCrystal lcd(2, A4, A3, A2, A1, A0); //распиновка подключения дисплея

uint16_t EEMEM VL_PWM_addr;
uint16_t EEMEM IR_PWM_addr;

uint16_t VL_PWM = 65530; //значение ШИМ светодиода для достижения яркости фотодиода в значение sensor_max
uint16_t IR_PWM = 65530; //значение ШИМ ИК диода для достижения яркости фотодиода в значение sensor_max

const float sensor_max = 701;  // опытным путем было получено что до текущего значения график зависимости
const float sensor_min = 4;     // входа с АЦП фотодиода и яркоси свето-/ИК диода представляет собой прямую

const char Krt[4] = {124, 47, 45, 92};
uint8_t KrtCn = 0;

void setup() {
  RCal(); //достаем сохраненные ранее калибровки яркости диодов

  pinMode(A5, INPUT_PULLUP); //кнопка для калибровки
  pinMode(IR_LED, OUTPUT); //PWM выход на ИК диод
  pinMode(VL_LED, OUTPUT); //PWM выход на светодиод
  digitalWrite(IR_LED, 0);
  digitalWrite(VL_LED, 0);
  //привествие
  lcd.begin(16, 2);
  lcd.print("IR/VL Spect.Test");
  lcd.setCursor(1, 1);
  lcd.print("Alexander.Chad");
  delay(2000);
  PrintBg(); //рисуем на дисплее фон с названиеми значений

  // инициализация Timer1
  cli(); // отключить глобальные прерывания
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 65535; // установка регистра совпадения
  OCR1B = 65530; // установка регистра совпадения
  TCCR1B |= (1 << WGM12); // включение в CTC режим
  TCCR1B |= (1 << CS10); // Установка битов коэффициент деления
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B); // включение прерываний по совпадению
  sei(); // включить глобальные прерывания

  Serial.begin(9600);
  Serial.println("IR/VL Spect.Test is begin");
}

uint8_t L_Set = 0; //номер пина, который переводится в режим ШИМ (программной)
uint8_t Bt_cal = 0; //флаг удержания кнопки (для перехода в режим калибровки)
//uint8_t Bt_CT = 0; //счетчик, нужен для определения удержания кнопки
float sensor_IR; //значение на фотодиоде при облучении ИК
float sensor_VL; //значение на фотодиоде при облучении видимым спектром
uint8_t UglAnalysisCn = 0; //счетчик проходов углубленного анализа

void loop() {

  //опрос кнопки
  /*
    if (digitalRead(A5)) {
      Bt_CT = 0;
    } else {
      if (Bt_CT > 3) {
        Bt_cal = !Bt_cal;
      }
      Bt_CT++;
    }*/

  if (!digitalRead(A5)) {
    delay(150);
    if (!digitalRead(A5)) {
      Bt_cal = 1;
    } else {
      Bt_cal = 0;
    }
  }
  if (Bt_cal) {
    RCal(); //достаем сохраненные ранее калибровки яркости диодов
    Serial.println("UglAnalysis RUN!");
    UglAnalysis(); //углубленный анализ
    UglAnalysisCn = 1;
    Bt_cal = 0;
  }
  if (UglAnalysisCn) {
    if (UglAnalysisCn > 3) {
      UglAnalysisCn = 0;
      RCal(); //достаем сохраненные ранее калибровки яркости диодов
      Serial.println("UglAnalysis END!");
    } else {
      lcd.setCursor(0, 0);
      lcd.print(UglAnalysisCn);
      UglAnalysisCn++;
    }
  }


  //начинаем калибровку при поднятом флаге удержании кнопки или по принятому в Serial символу 'c'
  switch (Serial.read()) {
    case 'c':
      //рисуем на дисплее сообщение о процессе калибровки
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("IR/VL Spect.Test");
      lcd.setCursor(0, 1);
      lcd.print("Calibration...");
      CalibrationSensor(); //функция калибровки яркости диодов
      //Bt_cal = 0;
      PrintBg(); //рисуем на дисплее фон с названиеми значений
      break;
    case 'a':
      Bt_cal = 1;
      break;
  }

  /*
    //формируем данные для построения графика зависимости яркости излучающего диода и сигнала с фотодиода
    Serial.println("------------VL------------");
    L_Set = VL_LED;
    digitalWrite(IR_LED, 0);
    for (uint16_t w = 0; w < 10000; w += 128) {
    OCR1B = w;
    //delay(1);
    Serial.print(w);
    Serial.print(9);
    Serial.println(AR(A6));
    }
    Serial.println("------------IR------------");
    L_Set = IR_LED;
    digitalWrite(VL_LED, 0);
    for (uint16_t w = 0; w < 10000; w += 128) {
    OCR1B = w;
    //delay(1);
    Serial.print(w);
    Serial.print(9);
    Serial.println(AR(A6));
    }
    Serial.println("------------END------------");
    L_Set = 0;
    delay(65535);
  */

  //буферы для строкового представления на экране значений
  char PStr_AR[5];
  char PStr_PR_IR[4];
  char PStr_PR_VL[4];
  char PStr_Kf[10];

  L_Set = IR_LED;
  OCR1B = IR_PWM;
  digitalWrite(VL_LED, 0);
  sensor_IR = AR(A6);
  sprintf(PStr_PR_IR, "%03d", (int)((sensor_IR - sensor_min) * 100 / sensor_max));

  delay(100); //даем время ИК диоду разрядить свой конденсатор, необходимо для исключения влияния светодиода на фотодиод

  L_Set = VL_LED;
  OCR1B = VL_PWM;
  digitalWrite(IR_LED, 0);
  sensor_VL = AR(A6);
  sprintf(PStr_PR_VL, "%03d", (int)((sensor_VL - sensor_min) * 100 / sensor_max));

  delay(100); //даем время светодиоду разрядить свой конденсатор, необходимо для исключения влияния ИК диода на фотодиод

  lcd.setCursor(3, 1);
  lcd.print(PStr_PR_IR);
  lcd.setCursor(12, 1);
  lcd.print(PStr_PR_VL);
  lcd.setCursor(7, 0);
  dtostrf(sensor_IR / sensor_VL, 9, 3, PStr_Kf);
  //sprintf(PStr_Kf, "%011d", sensor_IR / sensor_VL);
  lcd.print(PStr_Kf);

  lcd.setCursor(0, 0); //Крутилка палочки слева вверху =)
  lcd.print(Krt[KrtCn]);
  if (KrtCn < 3) {
    KrtCn++;
  } else {
    KrtCn = 0;
  }

  Serial.print("IR_PWM:");
  Serial.print(IR_PWM);
  Serial.print(", IR:");
  Serial.print(sensor_IR);
  Serial.print(", IR:");
  Serial.print(PStr_PR_IR);
  Serial.print("%, VL_PWM:");
  Serial.print(VL_PWM);
  Serial.print(", VL:");
  Serial.print(sensor_VL);
  Serial.print(", VL:");
  Serial.print(PStr_PR_VL);
  Serial.print("%");
  Serial.print(", Koef:");
  Serial.println(sensor_IR / sensor_VL);
}
void RCal() { //достаем сохраненные ранее калибровки яркости диодов
  EEPROM.get((int)&VL_PWM_addr, VL_PWM); // для светодиода
  EEPROM.get((int)&IR_PWM_addr, IR_PWM); // для ИК диода
}

void UglAnalysis() {
  float Del_VL = (sensor_VL - sensor_min) / sensor_max;
  float Del_IR = (sensor_IR - sensor_min) / sensor_max;
  float VL_PWM_NF;
  float IR_PWM_NF;
  if (Del_VL > Del_IR) {
    VL_PWM_NF = (float)VL_PWM / Del_VL;
    IR_PWM_NF = (float)IR_PWM / Del_VL;
  } else {
    VL_PWM_NF = (float)VL_PWM / Del_IR;
    IR_PWM_NF = (float)IR_PWM / Del_IR;
  }
  if (VL_PWM_NF > IR_PWM_NF) {
    if (VL_PWM_NF > 65530) {
      IR_PWM = (uint16_t)(IR_PWM_NF * ((float)65530 / VL_PWM_NF));
      VL_PWM = 65530;
    } else {
      IR_PWM = (uint16_t)IR_PWM_NF;
      VL_PWM = (uint16_t)VL_PWM_NF;
    }
  } else {
    if (IR_PWM_NF > 65530) {
      VL_PWM = (uint16_t)(VL_PWM_NF * ((float)65530 / IR_PWM_NF));
      IR_PWM = 65530;
    } else {
      IR_PWM = (uint16_t)IR_PWM_NF;
      VL_PWM = (uint16_t)VL_PWM_NF;
    }
  }
}

void CalibrationSensor() {
  Serial.println("Start Calibration");
  Serial.print("VL_PWM: ");
  Serial.print(VL_PWM);
  Serial.print(", IR_PWM: ");
  Serial.println(IR_PWM);
  Calibration(VL_LED);
  Calibration(IR_LED);
  Serial.println();
  Serial.println("End Calibration");
  Serial.print("VL_PWM: ");
  Serial.print(VL_PWM);
  Serial.print(", IR_PWM: ");
  Serial.println(IR_PWM);
}

void Calibration(int Pin_L) { //калибровка яркости диода
  L_Set = Pin_L;
  switch (Pin_L) {
    case IR_LED:
      digitalWrite(VL_LED, 0);
      break;
    case VL_LED:
      digitalWrite(IR_LED, 0);
      break;
  }
  uint16_t step_pwm = 16384;
  OCR1B = 0;
  while (abs(AR(A6) - (sensor_max - 1)) > 1) {
    while (AR(A6) < sensor_max) {
      Serial.print("OCR1B: ");
      Serial.print(OCR1B);
      Serial.print(", step_pwm: ");
      Serial.println(step_pwm);
      if ((UINT16_MAX - OCR1B) > step_pwm) {
        OCR1B += step_pwm;
      } else {
        OCR1B = 65530;
      }
    }
    Serial.print("OCR1B: ");
    Serial.print(OCR1B);
    Serial.print(", step_pwm: ");
    Serial.println(step_pwm);
    OCR1B -= step_pwm;
    if (step_pwm > 1) {
      step_pwm = step_pwm / 2;
    }
  }
  OCR1B += step_pwm;
  switch (Pin_L) {
    case IR_LED:
      IR_PWM = OCR1B;
      EEPROM.put((int)&IR_PWM_addr, IR_PWM);
      break;
    case VL_LED:
      VL_PWM = OCR1B;
      EEPROM.put((int)&VL_PWM_addr, VL_PWM);
      break;
  }
  OCR1B = 0;
}

//АЦП сигнала фотодиода. Для компенсации ШИМ пульсаций излучающих диодов, помех, формирования задержки между обновлениями экрана
float AR(int Pin) { //формирования задержки между обновлениями экрана применено многократное преобразование с усреднением
  float val_ar = 0;
  switch (L_Set) {
    case IR_LED:
      val_ar = 1023 - sensor_IR;
      break;
    case VL_LED:
      val_ar = 1023 - sensor_VL;
      break;
  }
  for (uint16_t it = 0; it < 25000; it++) {
    if (it == 0) {
      val_ar = (float)analogRead(Pin);
    }{
      val_ar = (val_ar + (float)analogRead(Pin)) / 2;
    }
  }
  if (val_ar > (1023 - sensor_min)) {
    val_ar = 1023 - sensor_min;
  }
  return 1023 - val_ar;
}

void PrintBg() { //рисуем на дисплее фон с названиеми значений
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("Koef:");
  lcd.setCursor(0, 1);
  lcd.print("IR:   %");
  lcd.setCursor(9, 1);
  lcd.print("VL:   %");
}

//Ниже два прерывания таймера - формирование фронтов ШИМ сигнала на пине L_Set
ISR(TIMER1_COMPA_vect)
{
  digitalWrite(L_Set, 1);
}
ISR(TIMER1_COMPB_vect)
{
  digitalWrite(L_Set, 0);
}
