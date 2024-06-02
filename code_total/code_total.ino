#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Pixy2.h>
#include <RTClib.h>
#include <Wire.h>

// Définir la macro pour le débogage série
#define SERIAL_PRINT_ENABLED true

#if SERIAL_PRINT_ENABLED
#define SERIAL_PRINT(x) Serial.print(x)
#define SERIAL_PRINTLN(x) Serial.println(x)
#define SERIAL_BEGIN(x) Serial.begin(x)
#else
#define SERIAL_PRINT(x)
#define SERIAL_PRINTLN(x)
#define SERIAL_BEGIN(x)
#endif

// Définir les broches des composants
#define PIN_LED_MATRIX 7
#define PIN_HOME_1_MCC 2
#define PIN_HOME_2_MCC 3
#define PIN_HOME_MOR 4
#define PIN_IR { 18, 17, 16, 15, 14 }
#define SIGNAL_SIGNATURE_BALL 2
#define LED_BRIGHTNESS 10

// Définir les broches des moteurs
int MCC_RPWM_Output = 9;  // connect to IBT-2 pin 1 (RPWM)
int MCC_LPWM_Output = 10; // connect to IBT-2 pin 2 (LPWM)
int MOR_RPWM_Output = 5; // connect to IBT-2 pin 1 (RPWM)
int MOR_LPWM_Output = 6; // connect to IBT-2 pin 2 (LPWM)

int current_MCC_pos = 0;
int current_MOR_pos = 0;

// Initialiser les composants
Pixy2 pixy;
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(
    32, 8, PIN_LED_MATRIX,
    NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG,
    NEO_GRB + NEO_KHZ800);
RTC_DS1307 RTC;

// Définir les broches des détecteurs IR
int detectorPins[5] = PIN_IR;
int detections[5] = {0, 0, 0, 0, 0};
int detectionCounter = 0;

// Définir l'état des boutons
int buttonStateMCC1 = 0;
int buttonStateMCC2 = 0;
int buttonStateMOR = 0;

// Variables principales
int myVariable = 0;
unsigned long previousMillis10s = 0;
unsigned long previousMillis1s = 0;
const long interval10s = 10000;
const long interval1s = 1000;

void setup() {
  SERIAL_BEGIN(9600);

  // Configurer les broches des composants
  pinMode(PIN_HOME_1_MCC, OUTPUT);
  pinMode(PIN_HOME_2_MCC, OUTPUT);
  pinMode(PIN_HOME_MOR, OUTPUT);
  for (int i = 0; i < 5; i++) {
    pinMode(detectorPins[i], OUTPUT);
  }

  digitalWrite(PIN_HOME_1_MCC, LOW);
  digitalWrite(PIN_HOME_2_MCC, LOW);
  digitalWrite(PIN_HOME_MOR, LOW);
  for (int i = 0; i < 5; i++) {
    digitalWrite(detectorPins[i], LOW);
  }

  pinMode(PIN_HOME_1_MCC, INPUT);
  pinMode(PIN_HOME_2_MCC, INPUT);
  pinMode(PIN_HOME_MOR, INPUT);

  pinMode(MCC_RPWM_Output, OUTPUT);
  pinMode(MCC_LPWM_Output, OUTPUT);
  pinMode(MOR_RPWM_Output, OUTPUT);
  pinMode(MOR_LPWM_Output, OUTPUT);

  pinMode(PIN_LED_MATRIX, OUTPUT);
  for (int i = 0; i < 5; i++) {
    pinMode(detectorPins[i], INPUT);
  }

  // Initialiser les composants
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(LED_BRIGHTNESS);
  matrix.setTextColor(matrix.Color(255, 0, 0));

  Wire.begin();
  RTC.begin();
  pixy.init();
  Homing();
}

// Macros pour simplifier les commandes des moteurs
#define TURN_MCC(x) turn_driver_moteur(MCC_LPWM_Output, MCC_RPWM_Output, x)
#define TURN_MOR(x) turn_driver_moteur(MOR_LPWM_Output, MOR_RPWM_Output, x)

// Variables pour la boucle principale
int val = 0;
int score_team_R = 0;
int score_team_B = 0;

Block last_ball;


void loop() {
  SERIAL_PRINTLN("loop");

  read_button(PIN_HOME_1_MCC, &buttonStateMCC1);
  read_button(PIN_HOME_2_MCC, &buttonStateMCC2);
  read_button(PIN_HOME_MOR, &buttonStateMOR);

  LED_matrix_score(score_team_R, score_team_B);

  val = IR_sensor(val, detectorPins);
  SERIAL_PRINTLN(val);

  if(buttonStateMCC1 == HIGH && buttonStateMCC2 == HIGH && buttonStateMOR == HIGH) {
    // aucun bouton n'est pressé
    TURN_MCC(50);
    TURN_MOR(50);
  } else {
    // l'un des bouton a été pressé
    TURN_MCC(0);
    TURN_MOR(0);
  }
  
  delay(100);
}

void Homing() {
  read_button(PIN_HOME_1_MCC, &buttonStateMCC1);
  read_button(PIN_HOME_2_MCC, &buttonStateMCC2);
  read_button(PIN_HOME_MOR, &buttonStateMOR);
  while ((buttonStateMCC1==1 && buttonStateMCC1 ==1) || buttonStateMOR == 1) {
    read_button(PIN_HOME_1_MCC, &buttonStateMCC1);
    read_button(PIN_HOME_2_MCC, &buttonStateMCC2);
    read_button(PIN_HOME_MOR, &buttonStateMOR);

    if( buttonStateMCC1==1 && buttonStateMCC1 ==1 ) {
      TURN_MCC(50);
    } else {
      TURN_MCC(0);
    }
    if( buttonStateMOR == 1 ) {
      TURN_MOR(50);
    } else {
      TURN_MOR(0);
    }
  }
}

void LED_matrix_score(int score_B, int score_R) {
  matrix.fillScreen(0);
  matrix.setCursor(0, 0);

  if (score_B < 10) {
    matrix.print(" ");
  }

  matrix.setTextColor(matrix.Color(0, 0, 255));
  matrix.print(score_B);

  matrix.setTextColor(matrix.Color(255, 255, 255));
  matrix.print("/");

  matrix.setTextColor(matrix.Color(255, 0, 0));
  matrix.print(score_R);

  matrix.show();
}

int last_index = -1;
Block last_block;
Block Pixy_cam() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks > 0) {
    Block list_all[pixy.ccc.numBlocks];
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      list_all[i] = pixy.ccc.blocks[i];
    }

    int len = 0;
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (list_all[i].m_signature == SIGNAL_SIGNATURE_BALL) {
        len++;
      }
    }

    Block list_signal_ball[len];
    int index = 0;
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (list_all[i].m_signature == SIGNAL_SIGNATURE_BALL) {
        list_signal_ball[index++] = list_all[i];
      }
    }

    #define SCORE_TAILLE 10
    #define SCORE_INDEX 100
    #define SCORE_AGE 50

    int scores[len] = {0};
    int best = 0;

    for (int i = 0; i < len; i++) {
      if (list_signal_ball[best].m_width * list_signal_ball[best].m_height <
          list_signal_ball[i].m_width * list_signal_ball[i].m_height) {
        best = i;
      }
    }
    scores[best] += SCORE_TAILLE;

    for (int i = 0; i < len; i++) {
      if (list_signal_ball[i].m_index == last_index) {
        best = i;
      }
    }
    scores[best] += SCORE_INDEX;

    for (int i = 0; i < len; i++) {
      if (list_signal_ball[best].m_age < list_signal_ball[i].m_age) {
        best = i;
      }
    }
    scores[best] += SCORE_AGE;

    best = 0;
    for (int i = 0; i < len; i++) {
      if (scores[best] < scores[i]) {
        best = i;
      }
    }
    last_block = list_signal_ball[best];
  }
  return last_block;
}

void Pixy_cam_print() {
  pixy.ccc.getBlocks();
  if (pixy.ccc.numBlocks) {
    SERIAL_PRINT("Detected ");
    SERIAL_PRINTLN(pixy.ccc.numBlocks);
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      SERIAL_PRINT("  block ");
      SERIAL_PRINT(i);
      SERIAL_PRINT(": ");
      pixy.ccc.blocks[i].print();
    }
  }
}

int sensor_value = 200;
void loop_moteur() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis10s >= interval10s) {
    previousMillis10s = currentMillis;
    myVariable = 1;
    previousMillis1s = currentMillis;
    turn_driver_moteur(MCC_LPWM_Output, MCC_RPWM_Output, sensor_value);
  }

  if (myVariable == 1 && currentMillis - previousMillis1s >= interval1s) {
    myVariable = 2;
    previousMillis1s = currentMillis;
    turn_driver_moteur(MCC_LPWM_Output, MCC_RPWM_Output, 0);
  }

  if (myVariable == 2 && currentMillis - previousMillis1s >= interval1s) {
    myVariable = 3;
    previousMillis1s = currentMillis;
    turn_driver_moteur(MCC_LPWM_Output, MCC_RPWM_Output, sensor_value + 512);
  }

  if (myVariable == 3 && currentMillis - previousMillis1s >= interval1s) {
    myVariable = 0;
    previousMillis1s = currentMillis;
    turn_driver_moteur(MCC_LPWM_Output, MCC_RPWM_Output, 0);
  }
}

int IR_sensor(int stockage_detection, int list_IR[5]) {
  for (int i = 0; i < 5; i++) {
    list_IR[i] = analogRead(detectorPins[i]);
  }
  stockage_detection = 0;
  for (int i = 0; i < 5; i++) {
    if (list_IR[i] < 512) {
      stockage_detection += 1;
      SERIAL_PRINT("Sensor ");
      SERIAL_PRINT(i + 1);
      SERIAL_PRINT(": Object detected ");
      SERIAL_PRINTLN(list_IR[i]);
    }
  }
  return stockage_detection;
}

void read_button(int buttonPin, int *var_stockage) {
  *var_stockage = digitalRead(buttonPin);
  SERIAL_PRINTLN(*var_stockage);
}

void turn_driver_moteur(int pin_forward, int pin_reverse, int l_sensorValue) {
  if (l_sensorValue <= 512) {
    int reversePWM = (l_sensorValue - 511) / 2;
    SERIAL_PRINT("turning at ");
    SERIAL_PRINTLN(reversePWM);
    analogWrite(pin_forward, 0);
    analogWrite(pin_reverse, reversePWM);
  } else {
    int forwardPWM = (l_sensorValue - 512) / 2;
    SERIAL_PRINT("turning at ");
    SERIAL_PRINTLN(forwardPWM);
    analogWrite(pin_forward, forwardPWM);
    analogWrite(pin_reverse, 0);
  }
}