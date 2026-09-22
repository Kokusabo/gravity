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

  Блок реле живиться ОКРЕМО: 5В через DC-DC понижувач, який бере 12В
  тієї самої зовнішньої мережі. GND реле-модуля - спільний з ESP32,
  VCC реле-модуля - НЕ з ESP32.

  ⚠️ Піни реле перенесені з попередньої версії проєкту - перевір
  фізичну відповідність GPIO -> канал реле (тест почергового клацання)
  перед використанням з реальним двигуном під напругою.

  ----------------------------------------------------------------
  АЛГОРИТМ
  ----------------------------------------------------------------
  Реле 1 (двигун):
    УВІМКНЕНО, коли є напруга 12В І кінцевик РОЗІМКНУТИЙ.
    В усіх інших випадках - ВИМКНЕНО.

  Реле 2 (розподіл навантаження):
    УВІМКНЕНО, коли є напруга 12В.
    ВИМКНЕНО, коли напруги немає.

  ================================================================
*/

#include <Arduino.h>

// ------------------- ПІНИ -------------------
const int PIN_OPTOCOUPLER   = 33;   // Оптопара - детектор напруги 12В (LOW = є напруга)
const int PIN_LIMIT_SWITCH  = 4;    // Кінцевий вимикач (LOW = замкнутий)
const int PIN_RELAY_1_MOTOR = 32;   // Реле 1 - керування двигуном
const int PIN_RELAY_2_LOAD  = 27;   // Реле 2 - керування розподілом навантаження

// Рівень на GPIO, який УВІМКНЕНОЇ котушку реле (COM з'єднано з NO).
// HIGH - звичайний модуль; LOW - модуль з інверсною логікою ("low level trigger").
// Обидва канали на цій платі мають інверсну логіку -> активний рівень LOW.
// (Реле 2 працювало навпаки; реле 1 на старті їхало ~0.5с, поки код
// підтверджував напругу 12В, бо "вимкнено" = LOW насправді вмикало котушку.)
const int RELAY_1_ON_LEVEL = LOW;
const int RELAY_2_ON_LEVEL = LOW;

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

// ------------------- ТАЙМІНГ ДІАГНОСТИЧНОГО ВИВОДУ -------------------
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000;   // раз на секунду

// ------------------- ПРОТОТИПИ -------------------
bool isVoltagePresent();
bool isLimitOpen();


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
  pinMode(PIN_RELAY_1_MOTOR, OUTPUT);
  pinMode(PIN_RELAY_2_LOAD, OUTPUT);

  Serial.begin(115200);
  delay(500);

  pinMode(PIN_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(PIN_OPTOCOUPLER, INPUT_PULLUP);  // GPIO33 підтримує внутрішній pull-up

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
    Serial.println(relay2ShouldBe
      ? ">> РЕЛЕ 2 (навантаження): УВІМКНЕНО"
      : ">> РЕЛЕ 2 (навантаження): ВИМКНЕНО");
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
    Serial.println(relay2Was ? "1" : "0");
  }
}
