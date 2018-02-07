// Sensorless Blushless Motor Driver using Arduino nano

// ポートの定義
#define LED_MODE 5
#define SW1 3
#define SW2 4
#define ADVR 6
#define ADS 7
#define OPEN 12
#define BLAKE 13

// ゲートドライバの種類とArduinoのDigitalポートの対応
typedef enum
{
  UL = 8,  // U相ローサイド  PB0
  UU = 9,  // U相ハイサイド  PB1
  VL = 10, // PB2
  VU = 11, // PB3
  WL = 12, // PB4
  WU = 13  // PB5
} GATEDRIVER;

// AD変換するポートとUVW相の関係
typedef enum
{
  AD_U = 1,
  AD_V = 2,
  AD_W = 3
} ADPORT;

// ポートBの状態の定義
const unsigned short int portBStatus[14] = {
    0b00010011, // U -> V（U相High、V相Low、W相Open）
    0b00000111, // U -> W
    0b00001101, // V -> W
    0b00011100, // V -> U
    0b00110100, // W -> U
    0b00110001, // W -> V
    0b00010000, // U(Low) -> V (U相Low、V相Low、W相Open、PWM用)
    0b00000100, // U(Low) -> W
    0b00000001, // V(Low) -> W
    0b00010000, // V(Low) -> U
    0b00000100, // W(Low) -> U
    0b00000001, // W(Low) -> V
    0b00010101, // All off
    0b00000000  // Blake
};

// ある回転位置のときにどの相がオープン状態か
const unsigned short int lookADC[6] = {AD_W, AD_V, AD_U, AD_W, AD_V, AD_U};
unsigned short int curRotorStat = 0; // 現在の回転位置
const unsigned short int forwardTime = 0;

void setup()
{
  // タイマ割り込み設定
  TCCR1A = 0x00;
  TCCR1B = 0b00001001;
  OCR1A = 0x300; // PWM周波数は20kHz (16MHz/0x300)
  OCR1B = 0x0000;

  pinMode(UU, OUTPUT);
  pinMode(VU, OUTPUT);
  pinMode(WU, OUTPUT);
  pinMode(UL, OUTPUT);
  pinMode(VL, OUTPUT);
  pinMode(WL, OUTPUT);
  pinMode(LED_MODE, OUTPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  DDRB = 0b00111111;

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  // SW1かSW2を押すと駆動スタート
  while (digitalRead(SW1) == HIGH && digitalRead(SW2) == HIGH)
  {
    // 駆動開始まではブレーキ状態にしておく
    PORTB = portBStatus[BLAKE];
  }
  TIMSK1 = 0b00000110; // 割り込み有効にする
  if (digitalRead(SW2) == HIGH)
  {
    startup();
  }
}

// 割り込み処理（手動PWM）
ISR(TIMER1_COMPA_vect)
{
  // PWMのHigh状態
  PORTB = portBStatus[curRotorStat];
}
ISR(TIMER1_COMPB_vect)
{
  // PWMのLow状態
  PORTB = portBStatus[curRotorStat + 6];
}

// 次の回転状態を設定する
void portSwitch()
{
  curRotorStat++;
  if (curRotorStat >= 6)
  {
    curRotorStat = 0;
  }
}

// ビット数を犠牲にした高速AD変換
uint8_t myFastADC(uint8_t port)
{
  ADMUX = 0b01100000 | port;
  ADCSRA = 0b11000011; // AD Clockを2MHzにし、変換開始（7.5bit分解能）
  while (ADCSRA != 0b10010011)
    ; // 変換完了を待機
  return ADCH;
}

// PWM値をボリュームから読み込んで設定する
void setPWMfromVR()
{
  OCR1B = ((0xFF - (myFastADC(ADVR))) << 2) + 0x05;
}

// センサレス制御ループ
bool changeSwitchStatFlag = false; // 次の回転位置に進むフラグ
float rotatePeriod = 0;            // 60度回転する経過時間
unsigned long timeOld = 0;         // 60度回転する経過時間の測定用
int bemf = 0;                      // 誘起電圧

void loop()
{
  // 誘起電圧が中点電圧を横切るタイミングを検出する
  changeSwitchStatFlag = false;
  while (changeSwitchStatFlag == false)
  {
    bemf = (int)myFastADC(lookADC[curRotorStat]) - (int)myFastADC(ADS);
    switch (curRotorStat) // ロータ位置によって逆起電力の方向が異なる
    {
    case 1: // fall through
    case 3:
    case 5:
      if (bemf >= 0)
      {
        changeSwitchStatFlag = true;
      }
      break;
    case 0: // fall through
    case 2:
    case 4:
      if (bemf < 0)
      {
        changeSwitchStatFlag = true;
      }
      break;
    }
  }

  // 誘起電圧が中点電圧を横切ったタイミングで、60度回転するのに要した時間を計測する
  rotatePeriod = (rotatePeriod * 15.0 + micros() - timeOld) / 16; // 60度分の時間の測定

  delayMicroseconds(((int)rotatePeriod >> 1) - forwardTime); // 30度分待機する
  portSwitch();                                              // 磁界を60度回転させる
  timeOld = micros();
  setPWMfromVR();                            // PWMのデューティ比（出力パワー）を設定する
  delayMicroseconds((int)rotatePeriod >> 2); // 転流完了待ち
}

// 強制回転
void startup()
{
  float waitMicroSec = 0;
  float curRev = 0;
  const float maxRev = 15 * 60.0; // 最大の電気的回転数
  const float revAdd = 0.001;     // 回転数の増加スピード

  digitalWrite(LED_MODE, LOW);

  portSwitch();
  delay(500); // ある回転位置でしばらく待つ

  while (curRev < maxRev)
  {
    curRev = curRev + revAdd * waitMicroSec;
    waitMicroSec = 1.0 / (curRev / 60.0) / 6.0 / 3.0 * 100000.0;
    if (waitMicroSec >= 16000)
      waitMicroSec = 16000;
    if (waitMicroSec <= 1)
      waitMicroSec = 1;

    setPWMfromVR();
    portSwitch();

    // delayMicroseconds は 16000us までしか出力できないので、3つ繋げてより長い時間待てるようにする
    delayMicroseconds((int)waitMicroSec);
    delayMicroseconds((int)waitMicroSec);
    delayMicroseconds((int)waitMicroSec);
  }

  digitalWrite(LED_MODE, HIGH);
  rotatePeriod = waitMicroSec * 3;
  timeOld = micros();
}
