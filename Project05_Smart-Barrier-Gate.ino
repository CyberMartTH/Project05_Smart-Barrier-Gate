/* =========================================================================
   Mini Project : ไม้กั้นอัตโนมัติหน้าลานจอดรถ (Smart Parking Barrier Gate)
   บอร์ด        : ESP32 DevKit V1 / DevKitC  38 ขา
   ข้อกำหนด     : ใช้ขาฝั่งซ้ายของบอร์ดเพียงฝั่งเดียวเท่านั้น
   อุปกรณ์      : HC-SR04 x1, Servo 180 องศา, LCD 16x2 I2C,
                  LED ไฟจราจร 3 ดวง (เขียว / เหลือง / แดง)

   หลักการนับ   : เริ่มที่ 100 ช่อง  ทุกครั้งที่เซนเซอร์เจอรถ -> ลบ 1 แล้วยกคาน
                  เหลือ > 10  = ไฟเขียว  (ว่าง)
                  เหลือ 1-10 = ไฟเหลือง (ใกล้เต็ม)
                  เหลือ 0    = ไฟแดง    (เต็ม -> ไม่ยกคาน)

   ออปชัน       : ถ้าอยากให้ตัวเลขเพิ่มกลับได้ (ตอนนำเสนอจะสะดวก)
                  ให้ตั้ง USE_EXIT_BUTTON เป็น 1 แล้วต่อสวิตช์ปุ่มกด 1 ตัว
                  ระหว่าง GPIO33 กับ GND (ไม่ต้องใส่ตัวต้านทาน)
                  ถ้าไม่ใช้ ตัวเลขจะลดลงทางเดียวตามโจทย์ และกดปุ่ม EN
                  บนบอร์ดเพื่อรีเซ็ตกลับไปที่ 100

   ไลบรารีที่ต้องติดตั้งใน Arduino IDE (Library Manager)
     1) ESP32Servo            โดย Kevin Harrington
     2) LiquidCrystal I2C     โดย Frank de Brabander
   Board Manager URL:
     https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ========================================================================= */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

/* =========================================================================
   1) กำหนดขา  ---  ทุกขาอยู่ "คอลัมน์ซ้าย" ของ ESP32 38 pin ทั้งหมด
      ลำดับขาฝั่งซ้ายจากบนลงล่าง:
      3V3, EN, VP(36), VN(39), 34, 35, 32, 33, 25, 26, 27, 14, 12, GND,
      13, D2(9), D3(10), CMD(11), 5V

      ข้อควรรู้ของฝั่งซ้าย
      * 36/39/34/35 เป็น "อินพุตอย่างเดียว" สั่งเป็นเอาต์พุตไม่ได้
        และไม่มีตัวต้านทานพูลอัป/พูลดาวน์ในตัว
        -> ยกให้ขา Echo ของ HC-SR04 ซึ่งเป็นอินพุตอยู่แล้ว
      * D2/D3/CMD ต่อกับชิปแฟลชภายใน ห้ามใช้เด็ดขาด
      * EN เป็นขารีเซ็ต ห้ามใช้
      * 3V3 / 5V / GND อยู่ฝั่งนี้ครบ จ่ายไฟจอกับเซนเซอร์ได้จากบอร์ดเลย
   ========================================================================= */
#define ECHO_PIN      34   // HC-SR04 -> Echo   (input only, ผ่านตัวแบ่งแรงดัน)
#define TRIG_PIN      32   // HC-SR04 -> Trig
#define PIN_SDA       25   // LCD I2C -> SDA
#define PIN_SCL       26   // LCD I2C -> SCL
#define LED_RED       27   // ไฟแดง    (เต็ม)
#define LED_YELLOW    14   // ไฟเหลือง (ใกล้เต็ม)  * ขานี้มีพัลส์ตอนบูต LED อาจกะพริบครั้งเดียว
#define LED_GREEN     12   // ไฟเขียว  (ว่าง)      * strapping pin: ต่อ LED ลง GND ได้ ห้ามใส่ pull-up
#define PIN_SERVO     13   // สายสัญญาณ Servo (สีส้ม/เหลือง)

// ---- ออปชัน: ปุ่ม "รถออก" สำหรับคืนช่องว่าง (0 = ไม่ใช้, 1 = ใช้) ----
#ifndef USE_EXIT_BUTTON
#define USE_EXIT_BUTTON 0
#endif
#define BTN_EXIT      33   // ปุ่มกดลง GND · ขานี้มีพูลอัปในตัว ไม่ต้องใส่ตัวต้านทานเพิ่ม

// ขาฝั่งซ้ายที่ยังว่างอยู่หลังต่อครบ: 35, 36(VP), 39(VN) — ทั้งสามขาเป็นอินพุตเท่านั้น
// และ 33 ถ้าไม่ใช้ปุ่มรถออก (ขา 33 เป็น I/O เต็มรูปแบบ ต่อยอดอะไรก็ได้)

/* =========================================================================
   2) ค่าคงที่ปรับแต่งได้
   ========================================================================= */
const int      CAPACITY        = 20;   // จำนวนช่องจอดทั้งหมด (ตัวตั้งต้นของการนับถอยหลัง)

const int      NEAR_FULL       = 10;    // เหลือน้อยกว่าหรือเท่านี้ = ไฟเหลือง

const long     DETECT_CM       = 10;    // ระยะที่ถือว่า "มีรถ" (ซม.)
const long     CLEAR_CM        = 35;    // ระยะที่ถือว่า "รถผ่านไปแล้ว" (กันสั่น hysteresis)
const uint8_t  CONFIRM_N       = 3;     // ต้องอ่านค่าซ้ำกันกี่ครั้งจึงยืนยัน (กันสัญญาณรบกวน)

const int      ANGLE_CLOSED    = 180;     // องศาไม้กั้น "ปิด" (คานขนานพื้น)
const int      ANGLE_OPEN      = 140;    // องศาไม้กั้น "เปิด" (คานตั้งฉาก)
const int      SERVO_STEP      = 2;     // ก้าวละกี่องศา (ยิ่งน้อยยิ่งนุ่ม)
const uint16_t SERVO_STEP_MS   = 15;    // หน่วงต่อ 1 ก้าว (ms)

const uint32_t HOLD_OPEN_MS    = 3000;  // เปิดค้างอย่างน้อยกี่ ms หลังรถพ้นเซนเซอร์
const uint32_t MAX_OPEN_MS     = 15000; // กันค้าง: เปิดนานสุดเท่านี้แล้วปิดเอง
const uint16_t SENSOR_PERIOD   = 60;    // อ่านเซนเซอร์ทุกกี่ ms
const uint32_t MSG_HOLD_MS     = 2500;  // แสดงข้อความสถานะบน LCD นานเท่าไร
const uint16_t BTN_DEBOUNCE_MS = 50;    // หน่วงกันปุ่มเด้ง

/* =========================================================================
   3) อ็อบเจกต์และตัวแปรสถานะ
   ========================================================================= */
LiquidCrystal_I2C lcd(0x27, 16, 2);   // ถ้าจอไม่ติด ให้ลองเปลี่ยนเป็น 0x3F
Servo gateServo;

int  slotsFree = CAPACITY;            // จำนวนช่องว่างที่เหลือ (นับถอยหลังจาก 20)

// --- สถานะเซนเซอร์ ---
bool    carPresent = false;   // ตอนนี้มีรถอยู่หน้าเซนเซอร์หรือไม่
uint8_t confirmCnt = 0;       // ตัวนับยืนยันค่า
long    distanceCm = 999;     // ระยะล่าสุด (ซม.)

// --- สถานะไม้กั้น ---
enum GateState { G_CLOSED, G_OPENING, G_HOLD, G_CLOSING };
GateState gate = G_CLOSED;
bool carPassedThrough = false; // จำสถานะว่ารถเข้ามาบังเซนเซอร์แล้วหรือยัง
int      servoAngle   = ANGLE_CLOSED; // องศาปัจจุบันของ servo
uint32_t tServoStep   = 0;            // จับเวลาการขยับ servo
uint32_t tHoldStart   = 0;            // เวลาที่เริ่มเปิดค้าง
uint32_t tOpenStart   = 0;            // เวลาที่เริ่มเปิด (ใช้กับ MAX_OPEN_MS)
uint32_t tSensor      = 0;            // จับเวลาการอ่านเซนเซอร์

char     msgLine[17]  = "READY";      // ข้อความบรรทัดที่ 2
uint32_t tMsgUntil    = 0;            // แสดงข้อความถึงเมื่อไร

#if USE_EXIT_BUTTON
bool     btnLast      = true;         // สถานะปุ่มครั้งก่อน (true = ยังไม่กด)
uint32_t tBtn         = 0;            // จับเวลากันปุ่มเด้ง
#endif

/* =========================================================================
   4) ฟังก์ชันย่อย
   ========================================================================= */

// อ่านระยะจาก HC-SR04 หน่วยเซนติเมตร (คืน 999 ถ้าไม่มีสัญญาณกลับ)
long readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 25000UL);  // timeout 25 ms ≈ 4 เมตร
  if (dur == 0) return 999;
  return (long)(dur / 58);            // ซม. = เวลา(us) / 58
}

// อ่านและยืนยันสถานะเซนเซอร์  คืนค่า true เมื่อ "เพิ่งตรวจพบรถ" (ขอบขาเข้า)
bool updateSensor() {
  distanceCm = readDistanceCm();

  // ใช้ hysteresis: ถ้ายังไม่มีรถต้องใกล้กว่า DETECT_CM จึงนับว่ามี
  //                 ถ้ามีรถอยู่แล้วต้องไกลกว่า CLEAR_CM จึงนับว่าไปแล้ว
  bool raw = carPresent ? (distanceCm < CLEAR_CM) : (distanceCm < DETECT_CM);

  if (raw == carPresent) {            // ค่าไม่เปลี่ยน -> รีเซ็ตตัวนับ
    confirmCnt = 0;
    return false;
  }
  if (++confirmCnt >= CONFIRM_N) {    // เปลี่ยนค่าติดกันครบ -> ยืนยัน
    confirmCnt = 0;
    carPresent = raw;
    return carPresent;                // true เฉพาะตอนรถ "เข้ามา"
  }
  return false;
}

// ตั้งข้อความบรรทัดที่ 2 ของ LCD
void setMessage(const char *m, uint32_t holdMs = MSG_HOLD_MS) {
  strncpy(msgLine, m, sizeof(msgLine) - 1);
  msgLine[sizeof(msgLine) - 1] = '\0';
  tMsgUntil = millis() + holdMs;
}

// อัปเดตไฟจราจรตามจำนวนช่องว่าง
void updateTrafficLight() {
  bool full     = (slotsFree <= 0);
  bool nearFull = (!full && slotsFree <= NEAR_FULL);
  digitalWrite(LED_RED,    full     ? HIGH : LOW);
  digitalWrite(LED_YELLOW, nearFull ? HIGH : LOW);
  digitalWrite(LED_GREEN,  (!full && !nearFull) ? HIGH : LOW);
}

// วาดหน้าจอ LCD (เขียนเฉพาะตอนข้อความเปลี่ยน เพื่อไม่ให้จอกระพริบ)
void updateLcd() {
  static char prev1[17] = "";
  static char prev2[17] = "";
  char l1[17], l2[17];

  snprintf(l1, sizeof(l1), "FREE %3d/%3d", slotsFree, CAPACITY);
  snprintf(l2, sizeof(l2), "%-16s", (millis() < tMsgUntil) ? msgLine
                                   : (slotsFree <= 0        ? "  *** FULL ***"
                                   : (slotsFree <= NEAR_FULL ? "  NEARLY FULL"
                                                             : "    WELCOME")));

  if (strcmp(l1, prev1) != 0) {
    lcd.setCursor(0, 0);
    lcd.print("                ");   // ล้างบรรทัด
    lcd.setCursor(0, 0);
    lcd.print(l1);
    strcpy(prev1, l1);
  }
  if (strcmp(l2, prev2) != 0) {
    lcd.setCursor(0, 1);
    lcd.print(l2);
    strcpy(prev2, l2);
  }
}

// สั่งให้ไม้กั้นเริ่มเปิด
void openGate() {
  gate       = G_OPENING;
  tOpenStart = millis();
  tServoStep = millis();
}

/* =========================================================================
   5) setup()
   ========================================================================= */
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[Parking Gate] booting...");

  // --- ขา LED ---
  pinMode(LED_GREEN,  OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED,    OUTPUT);

  // --- ขาเซนเซอร์ ---
  pinMode(TRIG_PIN, OUTPUT);  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);   // GPIO34 เป็น input only ไม่มีพูลอัป ไม่ต้องตั้งอะไรเพิ่ม

#if USE_EXIT_BUTTON
  pinMode(BTN_EXIT, INPUT_PULLUP);   // GPIO33 มีพูลอัปในตัว ปุ่มต่อลง GND ได้เลย
#endif

  // --- จอ LCD I2C (ต้องระบุขา SDA/SCL เองเพราะเราย้ายมาฝั่งซ้าย) ---
  Wire.begin(PIN_SDA, PIN_SCL);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("SMART PARKING");
  lcd.setCursor(0, 1);
  lcd.print("CYBERMART v1.1");

  // --- Servo (ESP32Servo ต้องจอง timer ก่อน) ---
  ESP32PWM::allocateTimer(0);
  gateServo.setPeriodHertz(50);            // Servo อนาล็อกใช้ 50 Hz
  gateServo.attach(PIN_SERVO, 500, 2400);  // ความกว้างพัลส์ 0.5-2.4 ms (0-180°)
  servoAngle = ANGLE_CLOSED;
  gateServo.write(servoAngle);             // เริ่มต้นที่ตำแหน่งปิด

  // ทดสอบไฟจราจรตอนเปิดเครื่อง
  digitalWrite(LED_RED, HIGH);    delay(300);
  digitalWrite(LED_YELLOW, HIGH); delay(300);
  digitalWrite(LED_GREEN, HIGH);  delay(300);
  digitalWrite(LED_RED, LOW); digitalWrite(LED_YELLOW, LOW); digitalWrite(LED_GREEN, LOW);

  delay(1200);
  lcd.clear();
  updateTrafficLight();
  setMessage("SYSTEM READY");
  Serial.println("[Parking Gate] ready. slots = " + String(slotsFree));
}

/* =========================================================================
   6) loop()  --- ทำงานแบบ non-blocking ทั้งหมด (ไม่ใช้ delay ยาว ๆ)
   ========================================================================= */
void loop() {
  uint32_t now = millis();

  /* ---------- 6.1 อ่านเซนเซอร์ทุก 60 ms ---------- */
  if (now - tSensor >= SENSOR_PERIOD) {
    tSensor = now;

    if (updateSensor() && gate == G_CLOSED) {
      if (slotsFree > 0) {
        carPassedThrough = true; // ✅ บันทึกไว้ว่ามีรถเข้ามาจ่อแล้ว
        setMessage("CAR IN - OPEN");
        openGate();
        Serial.println("[IN] Car detected, opening gate...");
      } else {
        setMessage("FULL! NO ENTRY");
        Serial.println("[IN] rejected: lot is full");
      }
    }
  }

#if USE_EXIT_BUTTON
  /* ---------- 6.2 ปุ่มรถออก (ออปชัน) — คืนช่องว่าง 1 ช่อง ---------- */
  bool btnNow = digitalRead(BTN_EXIT);          // ปุ่มลง GND -> กดแล้วได้ LOW
  if (btnNow != btnLast && (now - tBtn) >= BTN_DEBOUNCE_MS) {
    tBtn    = now;
    btnLast = btnNow;
    if (btnNow == LOW && gate == G_CLOSED) {    // ขอบขาลง = เพิ่งกดปุ่ม
      if (slotsFree < CAPACITY) slotsFree++;
      updateTrafficLight();
      setMessage("CAR OUT - OPEN");
      openGate();
      Serial.println("[OUT] car exited. free = " + String(slotsFree));
    }
  }
#endif

  /* ---------- 6.3 สเตตแมชชีนควบคุมไม้กั้น ---------- */
  switch (gate) {

    case G_CLOSED:
      // ไม่ต้องทำอะไร รอเซนเซอร์ตรวจจับ
      break;

    case G_OPENING: 
      if (now - tServoStep >= SERVO_STEP_MS) {
        tServoStep = now;
        servoAngle += SERVO_STEP; // ❌ 180 + 2 = 182
        if (servoAngle >= ANGLE_OPEN) { // ❌ 182 >= 140 เลยเข้าเงื่อนไขนี้ทันที (ทำให้หมุนฟึบ)
          servoAngle = ANGLE_OPEN;
          gate       = G_HOLD;
          tHoldStart = now;
        }
        gateServo.write(servoAngle);
      }
      break;

    case G_HOLD: 
      if (carPresent) tHoldStart = now; 
        if ((now - tHoldStart >= HOLD_OPEN_MS) || (now - tOpenStart >= MAX_OPEN_MS)) {
          gate       = G_CLOSING;
          tServoStep = now;
          setMessage("GATE CLOSING");
        }
      break;

    case G_CLOSING: 
      if (now - tServoStep >= SERVO_STEP_MS) {
        tServoStep = now;
        servoAngle -= SERVO_STEP; // ❌ 140 - 2 = 138 
        if (servoAngle <= ANGLE_CLOSED) { // ❌ 138 <= 180 เข้าเงื่อนไขทันที
          servoAngle = ANGLE_CLOSED;
          gate       = G_CLOSED;
          setMessage("GATE CLOSED", 1500);
        }
        gateServo.write(servoAngle);
      }
      break;}

  /* ---------- 6.4 อัปเดตจอแสดงผล ---------- */
  updateLcd();
}