#include <Arduino.h>

#define RPWM_1 PA0
#define LPWM_1 PA1
#define R_EN_1 PB0
#define L_EN_1 PB1

#define RPWM_2 PA8
#define LPWM_2 PA9
#define R_EN_2 PB4
#define L_EN_2 PB5

HardwareSerial Serial3(PB11, PB10);

// ===== Command protocol (khop voi motor_test.py) =====
// "M <pwm1> <pwm2>\n"  pwm1/pwm2 trong khoang [-255, 255]
// "Z\n"                reset encoder ve 0
#define CMD_TIMEOUT_MS 500   // khong nhan lenh moi trong 500ms -> tu dung (failsafe)

String rxBuffer = "";
unsigned long lastCmdTime = 0;
int currentPwm1 = 0;
int currentPwm2 = 0;

void setupEncoderTimer(TIM_TypeDef* tim) {
  tim->CR1 = 0;
  tim->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
  tim->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  tim->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  tim->ARR = 0xFFFF;
  tim->CNT = 0x8000;
  tim->CR1 |= TIM_CR1_CEN;
}

// Ap dung PWM cho 1 motor theo dau cua gia tri (BTS7960: RPWM/LPWM doi nhau theo chieu)
void applyMotor(int pwm, int rpwmPin, int lpwmPin) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) {
    analogWrite(rpwmPin, pwm);
    analogWrite(lpwmPin, 0);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -pwm);
  }
}

void setMotors(int pwm1, int pwm2) {
  currentPwm1 = pwm1;
  currentPwm2 = pwm2;
  applyMotor(pwm1, RPWM_1, LPWM_1);
  applyMotor(pwm2, RPWM_2, LPWM_2);
}

void stopMotors() {
  setMotors(0, 0);
}

void resetEncoders() {
  TIM3->CNT = 0x8000;
  TIM4->CNT = 0x8000;
}

// Xu ly 1 dong lenh da nhan day du (khong con '\n')
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  lastCmdTime = millis();

  if (line.charAt(0) == 'M') {
    // Format: "M <pwm1> <pwm2>"
    int firstSpace = line.indexOf(' ');
    int secondSpace = line.indexOf(' ', firstSpace + 1);
    if (firstSpace > 0 && secondSpace > firstSpace) {
      int pwm1 = line.substring(firstSpace + 1, secondSpace).toInt();
      int pwm2 = line.substring(secondSpace + 1).toInt();
      setMotors(pwm1, pwm2);
      Serial3.print("OK M "); Serial3.print(pwm1); Serial3.print(" "); Serial3.println(pwm2);
    } else {
      Serial3.println("ERR bad M format");
    }
  } else if (line.charAt(0) == 'Z') {
    resetEncoders();
    Serial3.println("OK encoder reset");
  } else {
    Serial3.println("ERR unknown cmd");
  }
}

void setup() {
  Serial3.begin(115200);
  pinMode(RPWM_1, OUTPUT); pinMode(LPWM_1, OUTPUT);
  pinMode(R_EN_1, OUTPUT); pinMode(L_EN_1, OUTPUT);
  pinMode(RPWM_2, OUTPUT); pinMode(LPWM_2, OUTPUT);
  pinMode(R_EN_2, OUTPUT); pinMode(L_EN_2, OUTPUT);
  digitalWrite(R_EN_1, HIGH); digitalWrite(L_EN_1, HIGH);
  digitalWrite(R_EN_2, HIGH); digitalWrite(L_EN_2, HIGH);

  // Mac dinh dung yen cho den khi co lenh - KHONG tu chay
  stopMotors();

  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
  GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 |
                   GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
  GPIOA->CRL |= GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_0;
  GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 |
                   GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
  GPIOB->CRL |= GPIO_CRL_CNF6_0 | GPIO_CRL_CNF7_0;
  setupEncoderTimer(TIM3);
  setupEncoderTimer(TIM4);

  lastCmdTime = millis();
  Serial3.println("READY");
}

void loop() {
  // ===== Doc lenh tu Pi (non-blocking) =====
  while (Serial3.available()) {
    char c = Serial3.read();
    if (c == '\n') {
      handleLine(rxBuffer);
      rxBuffer = "";
    } else if (c != '\r') {
      rxBuffer += c;
      if (rxBuffer.length() > 64) rxBuffer = ""; // chong tran buffer neu rac
    }
  }

  // ===== Failsafe: mat lien lac voi Pi qua lau -> tu dung =====
  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    if (currentPwm1 != 0 || currentPwm2 != 0) {
      stopMotors();
      Serial3.println("WARN timeout - auto stop");
    }
  }

  // ===== Gui feedback encoder dinh ky =====
  static unsigned long lastEncTime = 0;
  if (millis() - lastEncTime >= 200) {
    lastEncTime = millis();
    long count1 = (long)TIM3->CNT - 0x8000;
    long count2 = (long)TIM4->CNT - 0x8000;
    Serial3.print("ENC1: "); Serial3.print(count1);
    Serial3.print("  ENC2: "); Serial3.println(count2);
  }
}