/*
  ================================================================
  Керування двигуном і розподілом навантаження за наявністю
  зовнішньої мережі 12В (ESP32)
  ================================================================

  Оптопара      -> GPIO33 (PIN_OPTOCOUPLER)   - детектує, чи є напруга
                                                 12В зовнішньої мережі.
  Кінцевий вимикач -> GPIO4 (PIN_LIMIT_SWITCH) - положення механізму.
  Реле 1 (двигун)              -> GPIO32 (PIN_RELAY_1_MOTOR)
  Реле 2 (розподіл навантаження) -> GPIO27 (PIN_RELAY_2_LOAD)
  Реле 3 (гальмо)              -> GPIO26 (PIN_RELAY_3_BRAKE)

  Гальмо: НормальноВідкритий (NO) контакт реле 3 замкнутий на GND.
  Коли котушка знеструмлена - контакт розімкнутий, гальмо відпущене.
  Тобто при втраті живлення (зникло 12В чи зникло живлення самого
  реле-модуля) гальмо ВІДПУСКАЄТЬСЯ автоматично, апаратно - це навмисний
  fail-safe.

  Блок реле живиться ОКРЕМО: 5В через DC-DC понижувач, який бере 12В
  тієї самої зовнішньої мережі. GND реле-модуля - спільний з ESP32,
  VCC реле-модуля - НЕ з ESP32.

  ⚠️ Піни реле перенесені з попередньої версії проєкту - перевір
  фізичну відповідність GPIO -> канал реле (тест почергового клацання)
  перед використанням з реальним двигуном під напругою.

  Датчик струму INA3221 (I2C, канал 1 - струм мотора). Другий (новий)
  модуль - VS і VPU об'єднані разом і підключені на 3.3V (VS - живлення
  чипа, VPU - підтяжка I2C, обидва в межах норми при 3.3V):
    VS+VPU -> 3.3V (ESP32)
    GND    -> спільний GND
    SDA    -> GPIO21 (ESP32)
    SCL    -> GPIO22 (ESP32)
    IN1-   -> "-" клема мотора
    IN1+   -> спільний GND (те, куди раніше йшов мінус мотора напряму)
  Якщо показання струму вийдуть від'ємні - поміняти місцями IN1+/IN1-.
  Адреса на шині - 0x40 (визначено сканером I2C при старті; стандартна
  за даташитом - відрізняється від попереднього модуля, в якого була
  0x50). Опір шунта (INA3221_SHUNT_OHM) - 0.1 Ом, визначено ЕМПІРИЧНО за
  трьома незалежними замірами (дивись коментар біля константи). Межа
  вимірювання каналу при такому опорі - ≈1.64 А (насичення 13-бітного
  АЦП шунта на ~163.8 мВ).

  ----------------------------------------------------------------
  АЛГОРИТМ
  ----------------------------------------------------------------
  Реле 1 (двигун):
    УВІМКНЕНО, коли є напруга 12В І кінцевик РОЗІМКНУТИЙ.
    В усіх інших випадках - ВИМКНЕНО.

  Реле 2 (розподіл навантаження):
    УВІМКНЕНО, коли є напруга 12В.
    ВИМКНЕНО, коли напруги немає.

  Реле 3 (гальмо):
    УВІМКНЕНО (з невеликою затримкою BRAKE_ENGAGE_DELAY_MS), коли є
    напруга 12В І кінцевик ЗАМКНУТИЙ (вантаж доїхав до верхньої точки).
    В усіх інших випадках - ВИМКНЕНО, одразу, без затримки.

  ================================================================
*/

#include <Arduino.h>
#include <Wire.h>

// ------------------- ДАТЧИК СТРУМУ INA3221 (I2C) -------------------
// Пряме читання регістрів, без зовнішньої бібліотеки.
// Канал 1 - струм мотора (IN1- на "-" мотора, IN1+ на спільний GND).
const int  PIN_I2C_SDA        = 21;
const int  PIN_I2C_SCL        = 22;
const uint8_t INA3221_ADDR    = 0x40;   // новий модуль - визначено сканером I2C (стандартна адреса)

// Опір шунта НОВОГО модуля (адреса 0x40) - визначено емпірично за
// трьома незалежними замірами (реальний струм з ЛБЖ / показання до
// уточнення), усі сходяться до ~0.1 Ом - підпис "R100" на цьому модулі
// виявився правильним:
//   536 мА / 562 мА -> R ≈ 0.1000 Ом
//   624 мА / 655 мА -> R ≈ 0.1001 Ом
//   700 мА / 732 мА -> R ≈ 0.0998 Ом
// Межа вимірювання при такому опорі: 163.8 мВ / 0.1 Ом ≈ 1.64 А.
const float INA3221_SHUNT_OHM = 0.1;
bool ina3221Found = false;

// ------------------- ПІНИ -------------------
const int PIN_OPTOCOUPLER   = 33;   // Оптопара - детектор напруги 12В (LOW = є напруга)
const int PIN_LIMIT_SWITCH  = 4;    // Кінцевий вимикач (LOW = замкнутий)
const int PIN_RELAY_1_MOTOR = 32;   // Реле 1 - керування двигуном
const int PIN_RELAY_2_LOAD  = 27;   // Реле 2 - керування розподілом навантаження
const int PIN_RELAY_3_BRAKE = 26;   // Реле 3 - гальмо (замикає "+" мотора на GND)
                                     // ⚠️ GPIO26 ще не перевірений фізичним тестом
                                     // клацання на цій платі - зроби це перед
                                     // використанням з реальним двигуном.

// Рівень на GPIO, який УВІМКНЕНОЇ котушку реле (COM з'єднано з NO).
// HIGH - звичайний модуль; LOW - модуль з інверсною логікою ("low level trigger").
// Реле 1 і 2 на цій платі мають інверсну логіку -> активний рівень LOW.
// (Реле 2 працювало навпаки; реле 1 на старті їхало ~0.5с, поки код
// підтверджував напругу 12В, бо "вимкнено" = LOW насправді вмикало котушку.)
// Реле 3 - той самий модуль/тип каналу, тому теж поставлено LOW за
// замовчуванням, але це ще НЕ перевірено окремо - перевір при першому
// ввімкненні (якщо гальмо тримається завжди увімкненим чи завжди
// вимкненим незалежно від умов - постав тут HIGH).
const int RELAY_1_ON_LEVEL = LOW;
const int RELAY_2_ON_LEVEL = LOW;
const int RELAY_3_ON_LEVEL = LOW;

// Невелика затримка перед тим, як гальмо реально увімкнеться (від моменту,
// коли обидві умови - є напруга і кінцевик замкнутий - стали істинними).
// Вимикається завжди одразу, без затримки.
const unsigned long BRAKE_ENGAGE_DELAY_MS = 300;

// ------------------- ОПТОПАРА: ЦИФРОВЕ ЗЧИТУВАННЯ -------------------
// ⚠️ НА GPIO33 НІКОЛИ НЕ ВИКЛИКАТИ analogRead/analogReadMilliVolts!
// В Arduino-ESP32 це перемикає пін в аналоговий режим і ВИМИКАЄ внутрішній
// pull-up: пін "висить у повітрі", у стані "напруги нема" повільно
// сповзає до ~140 мВ і виглядає як "напруга є". Саме це давало хибне
// "12В є" після перших перемикань. Тому тут тільки digitalRead + pull-up.
// LOW = транзистор оптопари відкритий = напруга 12В є.

// Скільки часу нове значення має протриматись стабільним, перш ніж
// прийметься - захист від короткочасних шумових викидів.
const unsigned long OPTO_CONFIRM_MS = 500;

// ------------------- КІНЦЕВИК: ДЕБАУНС -------------------
// Механічний контакт може "дребезжати" на межі спрацювання чи від
// вібрації під час руху - без дебаунсу це виглядає як хаотичне
// перемикання реле "по колу".
const unsigned long LIMIT_CONFIRM_MS = 50;

// ------------------- МІНІМАЛЬНИЙ ІНТЕРВАЛ ПЕРЕМИКАННЯ РЕЛЕ -------------------
// Апаратний запобіжник ресурсу контактів реле - не клацати частіше
// цього інтервалу, навіть якщо щось продовжує "тремтіти".
const unsigned long RELAY_MIN_SWITCH_INTERVAL = 300;
unsigned long lastRelay1SwitchTime = 0;
unsigned long lastRelay2SwitchTime = 0;
unsigned long lastRelay3SwitchTime = 0;

// ------------------- ТАЙМІНГ ДІАГНОСТИЧНОГО ВИВОДУ -------------------
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000;   // раз на секунду

// ------------------- ПРОТОТИПИ -------------------
bool isVoltagePresent();
bool isLimitOpen();
int16_t ina3221ReadRegister(uint8_t reg);
float ina3221GetShuntVoltage_mV(uint8_t channel);
float ina3221GetCurrent_mA(uint8_t channel);
void i2cReinit();


// ================================================================
// ФУНКЦІЯ: i2cReinit()
// Призначення: повний перезапуск I2C-шини (Wire.end() + Wire.begin()).
//              Клацання реле - джерело електричного шуму, після якого
//              внутрішній стан I2C-драйвера ESP32 іноді "заклякає" і
//              починає повертати застарілі/биті байти, хоча формально
//              читання "вдається" (2 байти отримано). Викликати одразу
//              після кожного фізичного перемикання реле - скидає це,
//              не чекаючи, поки воно саме собою "розсмокчеться".
// ================================================================
void i2cReinit() {
  Wire.end();
  delay(1);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(50000);
}


// ================================================================
// ФУНКЦІЯ: ina3221ReadRegister()
// Призначення: зчитує "сирий" 16-бітний регістр INA3221 по I2C.
// ================================================================
int16_t ina3221ReadRegister(uint8_t reg) {
  // ⚠️ НЕ endTransmission(false) (repeated start) - на Arduino-ESP32 після
  // першого такого читання шина "залипає" і всі наступні падають з
  // i2cWriteReadNonStop Error -1. STOP між записом і читанням стабільний.
  //
  // Одна спроба повтору: поодинокі збої (requestFrom не отримав 2 байти)
  // трапляються в момент клацання реле - електричний шум від котушок на
  // мить збиває I2C. Це не завжди вдається прибрати тільки програмно,
  // але повтор одразу зазвичай проходить, бо викид короткий.
  for (int attempt = 0; attempt < 2; attempt++) {
    Wire.beginTransmission(INA3221_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0) {
      continue;
    }
    Wire.requestFrom((int)INA3221_ADDR, 2);
    if (Wire.available() < 2) {
      continue;
    }
    uint16_t hi = Wire.read();
    uint16_t lo = Wire.read();
    ina3221Found = true;
    return (int16_t)((hi << 8) | lo);
  }

  ina3221Found = false;
  return 0;
}


// ================================================================
// ФУНКЦІЯ: ina3221GetShuntVoltage_mV()
// Призначення: напруга на шунті каналу (1..3), мВ. LSB = 40 мкВ.
//              Усереднення 5 відліків поспіль - поодинокий відлік може
//              зловити шумовий викид саме в момент клацання реле
//              (комунікація при цьому справна, шум - в самому вимірі).
// ================================================================
float ina3221GetShuntVoltage_mV(uint8_t channel) {
  uint8_t reg = 0x01 + (channel - 1) * 2;   // 0x01, 0x03, 0x05 для каналів 1, 2, 3

  const int SAMPLE_COUNT = 5;
  long sumRaw = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int16_t raw = ina3221ReadRegister(reg);
    raw >>= 3;   // значення 13-бітне, зсунуте вліво в регістрі
    sumRaw += raw;
  }

  return (sumRaw / (float)SAMPLE_COUNT) * 0.04f;
}


// ================================================================
// ФУНКЦІЯ: ina3221GetCurrent_mA()
// Призначення: струм через канал (1..3), мА - з напруги на шунті й
//              емпірично визначеного опору (INA3221_SHUNT_OHM).
// ================================================================
float ina3221GetCurrent_mA(uint8_t channel) {
  return ina3221GetShuntVoltage_mV(channel) / INA3221_SHUNT_OHM;
}


// ================================================================
// ФУНКЦІЯ: isVoltagePresent()
// Призначення: чи є напруга 12В (через оптопару) - digitalRead з
//              голосуванням більшості з 8 відліків + часове підтвердження
//              (OPTO_CONFIRM_MS) проти шуму. LOW = напруга є.
// ================================================================
bool isVoltagePresent() {
  static bool lastState = false;          // Прийняте (підтверджене) рішення
  static bool pendingState = false;       // Нове значення, що чекає підтвердження
  static unsigned long pendingSince = 0;  // Відколи pendingState тримається стабільним

  const int SAMPLE_COUNT = 8;
  int lowCount = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (digitalRead(PIN_OPTOCOUPLER) == LOW) lowCount++;
  }

  bool rawState = (lowCount > SAMPLE_COUNT / 2);   // більшість відліків LOW -> напруга є

  unsigned long now = millis();
  if (rawState != pendingState) {
    pendingState = rawState;
    pendingSince = now;
  } else if (rawState != lastState && now - pendingSince >= OPTO_CONFIRM_MS) {
    lastState = rawState;
  }

  return lastState;
}


// ================================================================
// ФУНКЦІЯ: isLimitOpen()
// Призначення: чи розімкнутий кінцевик, з дебаунсом механічного контакту.
// ================================================================
bool isLimitOpen() {
  static bool lastState = false;          // Безпечний старт: вважаємо замкнутим (двигун не поїде)
  static bool pendingState = false;
  static unsigned long pendingSince = 0;

  bool rawState = (digitalRead(PIN_LIMIT_SWITCH) == HIGH);   // HIGH = розімкнутий
  unsigned long now = millis();

  if (rawState != pendingState) {
    pendingState = rawState;
    pendingSince = now;
  } else if (rawState != lastState && now - pendingSince >= LIMIT_CONFIRM_MS) {
    lastState = rawState;
  }

  return lastState;
}


// ================================================================
// ФУНКЦІЯ: setup()
// ================================================================
void setup() {
  // ПЕРШЕ, що робимо - переводимо реле у "вимкнено", ДО будь-яких затримок
  // (Serial.begin, delay), щоб пін не "висів" у невизначеному стані довше
  // за необхідне і двигун не смикнувся при старті.
  // Рівень "вимкнено" - протилежний активному для кожного реле.
  digitalWrite(PIN_RELAY_1_MOTOR, RELAY_1_ON_LEVEL == HIGH ? LOW : HIGH);
  digitalWrite(PIN_RELAY_2_LOAD,  RELAY_2_ON_LEVEL == HIGH ? LOW : HIGH);
  digitalWrite(PIN_RELAY_3_BRAKE, RELAY_3_ON_LEVEL == HIGH ? LOW : HIGH);
  pinMode(PIN_RELAY_1_MOTOR, OUTPUT);
  pinMode(PIN_RELAY_2_LOAD, OUTPUT);
  pinMode(PIN_RELAY_3_BRAKE, OUTPUT);

  Serial.begin(115200);
  delay(500);

  pinMode(PIN_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(PIN_OPTOCOUPLER, INPUT_PULLUP);  // GPIO33 підтримує внутрішній pull-up

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(50000);   // знижена швидкість I2C - стійкість до шуму від щіток мотора/реле

  // Сканування шини I2C - показує, які адреси реально відповідають,
  // незалежно від того, яку адресу ми "здогадуємось" використати нижче.
  Serial.println("Сканування I2C...");
  int foundCount = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Знайдено пристрій на адресі 0x");
      Serial.println(addr, HEX);
      foundCount++;
    }
  }
  if (foundCount == 0) {
    Serial.println("  Жодного пристрою не знайдено - перевір VCC/GND/SDA/SCL і підтяжки на шині.");
  }

  ina3221ReadRegister(0x01);   // пробне зчитування на заданій адресі - виставить ina3221Found
  Serial.println(ina3221Found
    ? "Датчик струму INA3221 відповідає на I2C."
    : "УВАГА: датчик струму INA3221 не відповідає на адресі INA3221_ADDR - див. сканування вище.");

  Serial.println("Система готова.");
  Serial.println("---------------------------------------------");
}


// ================================================================
// ФУНКЦІЯ: loop()
// ================================================================
void loop() {
  unsigned long currentTime = millis();

  bool voltagePresent = isVoltagePresent();
  bool limitOpen       = isLimitOpen();

  // ---- Реле 1 (двигун): ON, коли є напруга І кінцевик розімкнутий ----
  static bool relay1Was = false;
  bool relay1ShouldBe = voltagePresent && limitOpen;

  if (relay1ShouldBe != relay1Was &&
      currentTime - lastRelay1SwitchTime >= RELAY_MIN_SWITCH_INTERVAL) {
    lastRelay1SwitchTime = currentTime;
    relay1Was = relay1ShouldBe;
    digitalWrite(PIN_RELAY_1_MOTOR,
                 relay1ShouldBe ? RELAY_1_ON_LEVEL : (RELAY_1_ON_LEVEL == HIGH ? LOW : HIGH));
    i2cReinit();   // скинути можливе заклякання I2C від шуму клацання
    Serial.println(relay1ShouldBe
      ? ">> РЕЛЕ 1 (двигун): УВІМКНЕНО"
      : ">> РЕЛЕ 1 (двигун): ВИМКНЕНО");
  }

  // ---- Реле 2 (розподіл навантаження): ON, коли є напруга ----
  static bool relay2Was = false;
  bool relay2ShouldBe = voltagePresent;

  if (relay2ShouldBe != relay2Was &&
      currentTime - lastRelay2SwitchTime >= RELAY_MIN_SWITCH_INTERVAL) {
    lastRelay2SwitchTime = currentTime;
    relay2Was = relay2ShouldBe;
    digitalWrite(PIN_RELAY_2_LOAD,
                 relay2ShouldBe ? RELAY_2_ON_LEVEL : (RELAY_2_ON_LEVEL == HIGH ? LOW : HIGH));
    i2cReinit();   // скинути можливе заклякання I2C від шуму клацання
    Serial.println(relay2ShouldBe
      ? ">> РЕЛЕ 2 (навантаження): УВІМКНЕНО"
      : ">> РЕЛЕ 2 (навантаження): ВИМКНЕНО");
  }

  // ---- Реле 3 (гальмо): ON з невеликою затримкою, коли є напруга
  //      І кінцевик ЗАМКНУТИЙ. OFF - завжди одразу, без затримки. ----
  static bool relay3Was = false;
  static bool brakeConditionWasTrue = false;
  static unsigned long brakeConditionSince = 0;

  bool brakeCondition = voltagePresent && !limitOpen;   // кінцевик замкнутий = !limitOpen

  if (brakeCondition && !brakeConditionWasTrue) {
    brakeConditionSince = currentTime;   // умова щойно стала істинною - почати відлік затримки
  }
  brakeConditionWasTrue = brakeCondition;

  bool relay3ShouldBe = brakeCondition &&
                         (currentTime - brakeConditionSince >= BRAKE_ENGAGE_DELAY_MS);

  if (relay3ShouldBe != relay3Was &&
      currentTime - lastRelay3SwitchTime >= RELAY_MIN_SWITCH_INTERVAL) {
    lastRelay3SwitchTime = currentTime;
    relay3Was = relay3ShouldBe;
    digitalWrite(PIN_RELAY_3_BRAKE,
                 relay3ShouldBe ? RELAY_3_ON_LEVEL : (RELAY_3_ON_LEVEL == HIGH ? LOW : HIGH));
    i2cReinit();   // скинути можливе заклякання I2C від шуму клацання
    Serial.println(relay3ShouldBe
      ? ">> РЕЛЕ 3 (гальмо): УВІМКНЕНО"
      : ">> РЕЛЕ 3 (гальмо): ВИМКНЕНО");
  }

  // ---- Діагностичний вивід раз на секунду ----
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime;

    Serial.print("12В: ");
    Serial.print(voltagePresent ? "1" : "0");
    Serial.print(" (пін=");
    Serial.print(digitalRead(PIN_OPTOCOUPLER));
    Serial.print(")  Кінцевик розімкнутий: ");
    Serial.print(limitOpen ? "1" : "0");
    Serial.print("  Реле1: ");
    Serial.print(relay1Was ? "1" : "0");
    Serial.print("  Реле2: ");
    Serial.print(relay2Was ? "1" : "0");
    Serial.print("  Реле3(гальмо): ");
    Serial.print(relay3Was ? "1" : "0");

    // Свіжий старт шини перед кожним читанням - не покладатись лише на
    // скидання після клацання реле (шина могла застрягнути й без нього).
    i2cReinit();
    float motorCurrentMa = ina3221GetCurrent_mA(1);
    Serial.print("  Струм мотора: ");
    if (ina3221Found) {
      Serial.print(motorCurrentMa, 1);
      Serial.println(" мА");
    } else {
      Serial.println("н/д (датчик не відповідає)");
    }
  }
}
