#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68

// =====================================================
// PIN TANIMLARI
// =====================================================
#define PPM_PIN      32

#define MOTOR_1_PIN  27   // M1 - Arka Sol
#define MOTOR_2_PIN  25   // M2 - Ön Sol
#define MOTOR_3_PIN  4    // M3 - Ön Sağ
#define MOTOR_4_PIN  14   // M4 - Arka Sağ

// =====================================================
// F450 AÇI PID KATSAYILARI
// =====================================================
float Kp_angle = 0.75;
float Ki_angle = 0.0005;
float Kd_angle = 4.0;

float Kp_yaw   = 0.5;

// =====================================================
// PPM ALICI DEĞİŞKENLERİ
// =====================================================
#define NUM_CHANNELS 6

volatile uint16_t rcValue[NUM_CHANNELS] = {
  1500,  // CH1: Roll
  1500,  // CH2: Pitch
  1000,  // CH3: Throttle
  1500,  // CH4: Yaw
  1000,  // CH5: Arm (SwA / SwD)
  1000   // CH6: Aux
};

volatile uint8_t channelIndex = 0;
volatile uint32_t lastPpmTime = 0;

// =====================================================
// MPU6050 & AÇI HESAPLAMA
// =====================================================
int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawGx, rawGy, rawGz;

float accAngleRoll, accAnglePitch;
float gyroRollCal = 0, gyroPitchCal = 0, gyroYawCal = 0;
float accRollCal = 0, accPitchCal = 0;

float angleRoll = 0, anglePitch = 0, gyroYawRate = 0;

// =====================================================
// PID DEĞİŞKENLERİ
// =====================================================
float errorRoll, errorPitch, errorYaw;
float prevErrorRoll = 0, prevErrorPitch = 0;
float iRoll = 0, iPitch = 0;

// =====================================================
// MOTORLAR & GÜVENLİK
// =====================================================
int m1 = 1000, m2 = 1000, m3 = 1000, m4 = 1000;
bool isArmed = false;
bool emergencyCutoff = false;

unsigned long lastPrintTime = 0;
uint32_t loopTimer = 0;

// =====================================================
// PPM KESME FONKSİYONU
// =====================================================
void IRAM_ATTR ppmInterrupt() {
  uint32_t now = micros();
  uint32_t pulseLength = now - lastPpmTime;
  lastPpmTime = now;

  if (pulseLength > 3000) {
    channelIndex = 0;
  } else if (channelIndex < NUM_CHANNELS) {
    if (pulseLength >= 900 && pulseLength <= 2100) {
      rcValue[channelIndex] = pulseLength;
    }
    channelIndex++;
  }
}

// =====================================================
// SENSÖR BAŞLATMA VE KALİBRASYON
// =====================================================
void initSensors() {
  Wire.begin(21, 22);
  Wire.setClock(400000);
  delay(100);

  // MPU6050 Uyandır
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Dahili Filtre (DLPF ~44Hz)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);
}

void calibrateSensors() {
  long sumGx = 0, sumGy = 0, sumGz = 0;
  double sumAccRoll = 0, sumAccPitch = 0;

  for (int i = 0; i < 1000; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);

    if (Wire.available() >= 14) {
      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read(); // Sıcaklık baytlarını atla
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();

      sumGx += gx; sumGy += gy; sumGz += gz;

      float aRoll  = atan2(ay, az) * 57.296;
      float aPitch = atan2(-ax, sqrt((float)ay * ay + (float)az * az)) * 57.296;

      sumAccRoll  += aRoll;
      sumAccPitch += aPitch;
    }
    delay(1);
  }

  gyroRollCal  = (sumGx / 1000.0) / 65.5;
  gyroPitchCal = (sumGy / 1000.0) / 65.5;
  gyroYawCal   = (sumGz / 1000.0) / 65.5;
  accRollCal   = sumAccRoll / 1000.0;
  accPitchCal  = sumAccPitch / 1000.0;
}

// =====================================================
// MPU AÇI OKUMA
// =====================================================
void readAngles() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);

  if (Wire.available() >= 14) {
    rawAccX = (Wire.read() << 8) | Wire.read();
    rawAccY = (Wire.read() << 8) | Wire.read();
    rawAccZ = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    rawGx   = (Wire.read() << 8) | Wire.read();
    rawGy   = (Wire.read() << 8) | Wire.read();
    rawGz   = (Wire.read() << 8) | Wire.read();

    accAngleRoll  = (atan2(rawAccY, rawAccZ) * 57.296) - accRollCal;
    accAnglePitch = (atan2(-rawAccX, sqrt((float)rawAccY * rawAccY + (float)rawAccZ * rawAccZ)) * 57.296) - accPitchCal;

    float gyroRollRate  = ((float)rawGx / 65.5) - gyroRollCal;
    float gyroPitchRate = ((float)rawGy / 65.5) - gyroPitchCal;
    gyroYawRate         = ((float)rawGz / 65.5) - gyroYawCal;

    // Tamamlayıcı Filtre (Complementary Filter - dt = 0.004s / 250Hz)
    angleRoll  = 0.98 * (angleRoll  + gyroRollRate * 0.004)  + 0.02 * accAngleRoll;
    anglePitch = 0.98 * (anglePitch + gyroPitchRate * 0.004) + 0.02 * accAnglePitch;
  }
}

// =====================================================
// MOTOR PWM ÇIKIŞI (50 Hz / 16-Bit LEDC)
// =====================================================
void writeMotors(int m1_us, int m2_us, int m3_us, int m4_us) {
  ledcWrite(0, (m1_us * 65535) / 20000);
  ledcWrite(1, (m2_us * 65535) / 20000);
  ledcWrite(2, (m3_us * 65535) / 20000);
  ledcWrite(3, (m4_us * 65535) / 20000);
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);

  initSensors();
  calibrateSensors();

  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmInterrupt, RISING);

  ledcSetup(0, 50, 16); ledcAttachPin(MOTOR_1_PIN, 0);
  ledcSetup(1, 50, 16); ledcAttachPin(MOTOR_2_PIN, 1);
  ledcSetup(2, 50, 16); ledcAttachPin(MOTOR_3_PIN, 2);
  ledcSetup(3, 50, 16); ledcAttachPin(MOTOR_4_PIN, 3);

  writeMotors(1000, 1000, 1000, 1000);
  delay(1000);
  loopTimer = micros();
}

// =====================================================
// LOOP (250 Hz Sabit Döngü)
// =====================================================
void loop() {
  while (micros() - loopTimer < 4000);
  loopTimer = micros();

  int userThrottle = constrain(rcValue[2], 1000, 2000);
  isArmed = (rcValue[4] > 1400);

  // Gaz sıfıra indirilince acil kesme kilidi sıfırlanır
  if (userThrottle < 1050) {
    emergencyCutoff = false;
  }

  readAngles();

  // Güvenlik Kesmesi (Arm Kapalı, Gaz Sıfır veya Acil Stop Kilidi)
  if (!isArmed || userThrottle < 1050 || emergencyCutoff) {
    writeMotors(1000, 1000, 1000, 1000);
    m1 = 1000; m2 = 1000; m3 = 1000; m4 = 1000;

    iRoll = 0; iPitch = 0;
    prevErrorRoll = 0; prevErrorPitch = 0;
  } else {
    // Aşırı Eğim Koruması (Gövde 45 dereceyi aşarsa anında motorları kapatır)
    if (abs(angleRoll) > 45.0 || abs(anglePitch) > 45.0) {
      emergencyCutoff = true;
      writeMotors(1000, 1000, 1000, 1000);
      m1 = 1000; m2 = 1000; m3 = 1000; m4 = 1000;
    } else {
      float pidRoll = 0, pidPitch = 0, pidYaw = 0;
      int finalThrottle = constrain(userThrottle, 1000, 1800);

      // Kumanda Joystick Açısal Hedefleri
      float targetAngleRoll  = map(rcValue[0], 1000, 2000, -20, 20);
      float targetAnglePitch = map(rcValue[1], 1000, 2000, -20, 20);
      float targetYawRate    = map(rcValue[3], 1000, 2000, -80, 80);

      errorRoll  = targetAngleRoll  - angleRoll;
      errorPitch = targetAnglePitch - anglePitch;
      errorYaw   = targetYawRate    - gyroYawRate;

      // Anti-Windup (Yerde veya düşük gazda integral birikmesini engeller)
      if (userThrottle < 1250) {
        iRoll  = 0;
        iPitch = 0;
      } else {
        iRoll  = constrain(iRoll + errorRoll * 0.004, -15, 15);
        iPitch = constrain(iPitch + errorPitch * 0.004, -15, 15);
      }

      float derivativeRoll  = (errorRoll  - prevErrorRoll);
      float derivativePitch = (errorPitch - prevErrorPitch);

      pidRoll  = (Kp_angle * errorRoll)  + (Ki_angle * iRoll)  + (Kd_angle * derivativeRoll);
      pidPitch = (Kp_angle * errorPitch) + (Ki_angle * iPitch) + (Kd_angle * derivativePitch);
      pidYaw   = (Kp_yaw * errorYaw);

      pidRoll  = constrain(pidRoll, -120, 120);
      pidPitch = constrain(pidPitch, -120, 120);
      pidYaw   = constrain(pidYaw, -50, 50);

      prevErrorRoll  = errorRoll;
      prevErrorPitch = errorPitch;

      // Motor Dağılımı (+ / X Konfigürasyonu)
      m2 = finalThrottle - pidPitch + pidRoll + pidYaw; // M2 Ön Sol  (GPIO 25)
      m3 = finalThrottle - pidPitch - pidRoll - pidYaw; // M3 Ön Sağ  (GPIO 4)
      m1 = finalThrottle + pidPitch + pidRoll - pidYaw; // M1 Arka Sol (GPIO 27)
      m4 = finalThrottle + pidPitch - pidRoll + pidYaw; // M4 Arka Sağ (GPIO 14)

      m1 = constrain(m1, 1000, 2000);
      m2 = constrain(m2, 1000, 2000);
      m3 = constrain(m3, 1000, 2000);
      m4 = constrain(m4, 1000, 2000);

      writeMotors(m1, m2, m3, m4);
    }
  }

  // =====================================================
  // SERIAL MONITOR TELEMETRİSİ (Her 100 ms)
  // =====================================================
  if (millis() - lastPrintTime >= 100) {
    lastPrintTime = millis();

    Serial.print("ARM:");
    Serial.print(isArmed ? "ON " : "OFF");
    Serial.print(" | THR:");
    Serial.print(userThrottle);

    Serial.print(" | R:");
    Serial.print(angleRoll, 1);
    Serial.print(" P:");
    Serial.print(anglePitch, 1);

    Serial.print(" | M1:");
    Serial.print(m1);
    Serial.print(" M2:");
    Serial.print(m2);
    Serial.print(" M3:");
    Serial.print(m3);
    Serial.print(" M4:");
    Serial.println(m4);
  }
}