#include <Arduino.h>
#include <math.h>
#include <string.h>

const uint8_t PIN_LED_R = 2;   // RED   (GP2, PWM)
const uint8_t PIN_LED_G = 3;   // GREEN (GP3, PWM)
const uint8_t PIN_LED_B = 4;   // BLUE  (GP4, PWM)

const bool    LED_COMMON_ANODE = false;  // true: コモンアノードLED（出力を論理反転）
const uint8_t PWM_MAX          = 255;    // 8bit PWM
const uint16_t PWM_FREQ_HZ     = 1000;   // PWM周波数（ちらつき防止）

const uint32_t FADE_PERIOD_MS  = 2000;   // 0.5Hz フェード明滅（STANDBY / STANDUP）
const uint32_t BLINK_PERIOD_MS = 1000;   // 1Hz 点滅（CALIBRATING / ERROR）
const uint32_t LED_UPDATE_MS   = 10;     // LED更新周期(100Hz)

const uint32_t COMMS_TIMEOUT_MS = 1000;  // 1秒 有効な状態名が来なければ ROS 未受信扱いに戻す。

enum class State {
  STANDBY,
  STANDUP,
  WALK,
  CALIBRATING,
  ERROR,
  EMERGENCY_STOP
};

struct Color { uint8_t r, g, b; };
const Color C_OFF    = {  0,   0,   0};
const Color C_YELLOW = {255, 100,   0};
const Color C_GREEN  = {  0, 255,   0};
const Color C_ORANGE = {255,  30,   0};
const Color C_RED    = {255,   0,   0};

struct CmdEntry { const char* name; State st; };
const CmdEntry CMD_TABLE[] = {
  {"STANDBY",        State::STANDBY},
  {"STANDUP",        State::STANDUP},
  {"WALK",           State::WALK},
  {"CALIBRATING",    State::CALIBRATING},
  {"ERROR",          State::ERROR},
  {"EMERGENCY_STOP", State::EMERGENCY_STOP}
};
const uint8_t CMD_COUNT = sizeof(CMD_TABLE) / sizeof(CMD_TABLE[0]);

State    state          = State::STANDBY;
uint32_t lastCommandMs  = 0;
uint32_t lastLedMs      = 0;
bool     hasCommand     = false;

const uint8_t CMD_BUF_SIZE = 24;
char     cmdBuf[CMD_BUF_SIZE];
uint8_t  cmdLen = 0;

uint8_t applyPolarity(uint8_t v) {
  return LED_COMMON_ANODE ? (uint8_t)(PWM_MAX - v) : v;
}
void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(PIN_LED_R, applyPolarity(r));
  analogWrite(PIN_LED_G, applyPolarity(g));
  analogWrite(PIN_LED_B, applyPolarity(b));
}
void setColor(Color c) {
  setRgb(c.r, c.g, c.b);
}

// 滑らかなフェード明滅: レイズドコサインで 0→1→0、さらに二乗(≈ガンマ2)で柔らかく。
void breathe(Color c, uint32_t nowMs, uint32_t periodMs) {
  float phase = (float)(nowMs % periodMs) / (float)periodMs;
  float b = 0.5f * (1.0f - cosf(2.0f * (float)PI * phase));
  b = b * b;
  setRgb((uint8_t)(c.r * b), (uint8_t)(c.g * b), (uint8_t)(c.b * b));
}

void blink(Color c, uint32_t nowMs, uint32_t periodMs) {
  setColor(((nowMs % periodMs) < (periodMs / 2)) ? c : C_OFF);
}

bool hostActive(uint32_t nowMs) {
  return hasCommand && (uint32_t)(nowMs - lastCommandMs) < COMMS_TIMEOUT_MS;
}

void renderLed() {
  uint32_t nowMs = millis();

  if (!hostActive(nowMs)) {
    breathe(C_YELLOW, nowMs, FADE_PERIOD_MS);
    return;
  }

  switch (state) {
    case State::STANDBY:
    case State::STANDUP:        breathe(C_GREEN, nowMs, FADE_PERIOD_MS);   break;
    case State::WALK:           setColor(C_GREEN);                         break;
    case State::CALIBRATING:    blink(C_ORANGE, nowMs, BLINK_PERIOD_MS);   break;
    case State::ERROR:          blink(C_RED, nowMs, BLINK_PERIOD_MS);      break;
    case State::EMERGENCY_STOP: setColor(C_RED);                          break;
  }
}

void applyCommand() {
  cmdBuf[cmdLen] = '\0';
  for (uint8_t i = 0; i < CMD_COUNT; i++) {
    if (strcmp(cmdBuf, CMD_TABLE[i].name) == 0) {
      state = CMD_TABLE[i].st;
      lastCommandMs = millis();
      hasCommand = true;
      break;
    }
  }
  cmdLen = 0;
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) applyCommand();
      continue;
    }
    if (cmdLen < CMD_BUF_SIZE - 1) cmdBuf[cmdLen++] = c;
  }
}

void setup() {
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  analogWriteResolution(8);
  analogWriteFreq(PWM_FREQ_HZ);
  Serial.begin(115200);

  lastLedMs      = millis();
  state          = State::STANDBY;
  renderLed();
}

void loop() {
  pollSerial();

  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastLedMs) >= LED_UPDATE_MS) {
    lastLedMs = nowMs;
    renderLed();
  }
}
