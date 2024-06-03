#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Pixy2.h> // Un-commented Pixy2 library inclusion
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
#define PIN_HOME_1_MCC 3
#define PIN_HOME_2_MCC 2
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
Pixy2 pixy; // Un-commented Pixy2 object initialization
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

int MCC_direction = 0;
int MOR_direction = 0;
unsigned long last_time_MCC = 0;
unsigned long last_time_MOR = 0;

// Macros pour simplifier les commandes des moteurs
#define TURN_MCC(x) turn_driver_moteur(MCC_LPWM_Output, MCC_RPWM_Output, x, &MCC_direction)
#define TURN_MOR(x) turn_driver_moteur(MOR_LPWM_Output, MOR_RPWM_Output, x, &MOR_direction)

#define SPEED_FACTOR_MCC 200
#define SPEED_FACTOR_MOR 100

unsigned long percent_100 = 10;

#define ACTUAL_POS_MCC actualisation_pos(&current_MCC_pos, &last_time_MCC, &MCC_direction)
#define ACTUAL_POS_MOR actualisation_pos(&current_MOR_pos, &last_time_MOR, &MOR_direction)


#define MCC_PRECISION 0.10
#define MOR_PRECISION 0.10
#define ESTIMATED_MOR_MAX 1000
#define MCC_FACTOR 1
#define GO_TO_MCC(x) MOTOR_go_to(x, MCC_PRECISION, percent_100,      MCC_LPWM_Output, MCC_RPWM_Output, &MCC_direction, MCC_FACTOR*SPEED_FACTOR_MCC, current_MCC_pos)
#define GO_TO_MOR(x) MOTOR_go_to(x, MOR_PRECISION, ESTIMATED_MOR_MAX,MOR_LPWM_Output, MOR_RPWM_Output, &MOR_direction, SPEED_FACTOR_MOR, current_MOR_pos)

void setup() {
  
  SERIAL_BEGIN(9600);
  int indzae = 0;
  SERIAL_PRINTLN(indzae);indzae++;

  // Configurer les broches des composants
  pinMode(PIN_HOME_1_MCC, OUTPUT);
  pinMode(PIN_HOME_2_MCC, OUTPUT);
  pinMode(PIN_HOME_MOR, OUTPUT);
  for (int i = 0; i < 5; i++) {
    pinMode(detectorPins[i], OUTPUT);
  }

  SERIAL_PRINTLN(indzae);indzae++;
  digitalWrite(PIN_HOME_1_MCC, LOW);
  digitalWrite(PIN_HOME_2_MCC, LOW);
  digitalWrite(PIN_HOME_MOR, LOW);
  for (int i = 0; i < 5; i++) {
    digitalWrite(detectorPins[i], LOW);
  }
  

  SERIAL_PRINTLN(indzae);indzae++;
  pinMode(PIN_HOME_1_MCC, INPUT);
  pinMode(PIN_HOME_2_MCC, INPUT);
  pinMode(PIN_HOME_MOR, INPUT);

  SERIAL_PRINTLN(indzae);indzae++;
  pinMode(MCC_RPWM_Output, OUTPUT);
  pinMode(MCC_LPWM_Output, OUTPUT);
  pinMode(MOR_RPWM_Output, OUTPUT);
  pinMode(MOR_LPWM_Output, OUTPUT);

  analogWrite(MCC_RPWM_Output, 0);
  analogWrite(MCC_LPWM_Output, 0);
  analogWrite(MOR_RPWM_Output, 0);
  analogWrite(MOR_LPWM_Output, 0);

  pinMode(PIN_LED_MATRIX, OUTPUT);
  for (int i = 0; i < 5; i++) {
    pinMode(detectorPins[i], INPUT);
  }
  SERIAL_PRINTLN(indzae);indzae++;
  
  // Initialiser les composants
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(LED_BRIGHTNESS);
  matrix.setTextColor(matrix.Color(255, 0, 0));
  
  SERIAL_PRINTLN(indzae);indzae++;
  Wire.begin();
  
  SERIAL_PRINTLN(indzae);indzae++;
  RTC.begin();
  SERIAL_PRINTLN(indzae);indzae++;
  pixy.init(); // Un-commented Pixy2 initialization
  
  SERIAL_PRINTLN(indzae);indzae++;
  
  Homing();
  SERIAL_PRINT("MCC ");
  ACTUAL_POS_MCC;
  SERIAL_PRINT("MOR ");
  ACTUAL_POS_MOR;
  calibrate_MCC();
  SERIAL_PRINT("MCC ");
  ACTUAL_POS_MCC;
  SERIAL_PRINT("MOR ");
  ACTUAL_POS_MOR;
  while(GO_TO_MCC(0.5)) { 
    ACTUAL_POS_MCC;
    delay(10);
  }
  
}

//assume that we are at HOME
void calibrate_MCC() {
  read_button(PIN_HOME_1_MCC, &buttonStateMCC1);
  read_button(PIN_HOME_2_MCC, &buttonStateMCC2);
  unsigned long starting_millis = millis();
  SERIAL_PRINT("MCC ");
  TURN_MCC(SPEED_FACTOR_MCC);
  delay(1000); // necessary 500
  read_button(PIN_HOME_1_MCC, &buttonStateMCC1);
  read_button(PIN_HOME_2_MCC, &buttonStateMCC2);
  while( buttonStateMCC2 ==1) { 
    TURN_MCC(SPEED_FACTOR_MCC);
    SERIAL_PRINT("MCC ");
    read_button(PIN_HOME_1_MCC, &buttonStateMCC1);
    read_button(PIN_HOME_2_MCC, &buttonStateMCC2);
    //delay(10);
  }
  SERIAL_PRINT("MCC ");
  TURN_MCC(0);
  unsigned long ending_millis = millis();
  percent_100 = ending_millis - starting_millis;
  SERIAL_PRINT("Total time=");
  SERIAL_PRINTLN(percent_100);
  current_MCC_pos = percent_100;
}

unsigned long MCC_to_percent(float percentage) {
  unsigned long target = percent_100*percentage;
  unsigned long diff_time;
  if (current_MCC_pos>target) {
    diff_time = current_MCC_pos - target;
  } else {
    diff_time = target - current_MCC_pos;
  }
  unsigned long time_to_stop = millis() + diff_time;
  return time_to_stop;
}


void actualisation_pos( int *l_current_MCC_pos, unsigned long *last_time, int *direction) {
  unsigned long current_time = millis();
  *l_current_MCC_pos =  *l_current_MCC_pos + (*direction)*(current_time-(*last_time));
  *last_time = current_time;
  SERIAL_PRINT("current pos:");
  SERIAL_PRINTLN(*l_current_MCC_pos);
}

// Variables pour la boucle principale
int val = 0;
int score_team_R = 0;
int score_team_B = 0;

Block last_ball; // Un-commented Pixy2 Block object

int direction_MCC = 0;

void loop() {

  SERIAL_PRINTLN("loop");
  //LED_matrix_score(0, 5);
  //delay(1000);
  
  read_button(PIN_HOME_1_MCC, &buttonStateMCC1);
  read_button(PIN_HOME_2_MCC, &buttonStateMCC2);
  read_button(PIN_HOME_MOR, &buttonStateMOR);
  
  LED_matrix_score(0, 5);
  

  val = IR_sensor(val, detectorPins);
  SERIAL_PRINTLN(val);
  
  LED_matrix_score(0, 5);
  
  Block ball = Pixy_cam();
  ball.print();
  float x = 1 - ((float)ball.m_x)/320;
  float y = 1 - ((float)ball.m_y)/200;
  SERIAL_PRINT("x=");
  SERIAL_PRINTLN(x);
  //reversed because vision is reversed
  int dx = last_ball.m_x - ball.m_x;
  int dy = last_ball.m_y - ball.m_y;
  
  LED_matrix_score(0, 5);

  SERIAL_PRINT("MCC ");
  ACTUAL_POS_MCC;
  /*if(buttonStateMCC1 == 1) {
    current_MCC_pos = 0;
  }
  if(buttonStateMCC2 == 1) {
    current_MCC_pos = percent_100;
  }*/
  if(buttonStateMCC1 == 1 && buttonStateMCC2 == 1) {
    // aucun bouton n'est pressé
    if (x == 1 && y == 1) {
      GO_TO_MCC(0.5);
    }else {
      GO_TO_MCC(max(min(x,0.95),0.05));
    }
  } else {
    // l'un des bouton a été pressé
    TURN_MCC(0);
    
  }
  /*
  SERIAL_PRINT("MOR ");
  ACTUAL_POS_MOR;
  if( buttonStateMOR == 1) {
    // aucun bouton n'est pressé
    if (zae == 1) {
      GO_TO_MOR(100*atan2(dy,dx));// 100 in a random factor currently
    } else {
      GO_TO_MOR(0);
    }
  } else {
    // l'un des bouton a été pressé
    TURN_MOR(0);
  }*/
  
  
}

void turn_driver_moteur(int pin_forward, int pin_reverse, int l_sensorValue, int *direction) {
  if (l_sensorValue <= 0) {
    *direction = -1;
    int reversePWM = abs(l_sensorValue);
    SERIAL_PRINT("turning at ");
    SERIAL_PRINTLN(reversePWM);
    analogWrite(pin_forward, 0);
    analogWrite(pin_reverse, reversePWM);
  } else {
    *direction = 1;
    int forwardPWM = abs(l_sensorValue);
    SERIAL_PRINT("turning at ");
    SERIAL_PRINTLN(forwardPWM);
    analogWrite(pin_forward, forwardPWM);
    analogWrite(pin_reverse, 0);
  }
  if (l_sensorValue==0) {
    *direction = 0;
  }
}

bool MOTOR_go_to(float percentage, int precision, unsigned long max_value, int pin_LPWM_Output, int pin_MCC_RPWM_Output, int *direction, int speed, unsigned long current_pos) {
  long wanted_value = percentage * max_value;
  long diff_minus = wanted_value - precision*max_value;
  long diff_plus = wanted_value + precision*max_value;
  SERIAL_PRINT("downer: ");
  SERIAL_PRINT(diff_minus);
  SERIAL_PRINT(" upper: ");
  SERIAL_PRINTLN(diff_plus);

  if (current_pos > diff_minus && current_pos < diff_plus) {
    SERIAL_PRINT("standing:");
    SERIAL_PRINTLN(pin_LPWM_Output);
    turn_driver_moteur(pin_LPWM_Output, pin_MCC_RPWM_Output, 0, direction);
    return true;
  } else {
    SERIAL_PRINT(pin_LPWM_Output);
    SERIAL_PRINT("wanting to go to:");
    SERIAL_PRINTLN(wanted_value);
    SERIAL_PRINT("diff");
    SERIAL_PRINTLN(wanted_value - (long)current_pos);
    if (wanted_value - (long)current_pos > 0) {
      turn_driver_moteur(pin_LPWM_Output, pin_MCC_RPWM_Output, speed, direction);
    } else {
      turn_driver_moteur(pin_LPWM_Output, pin_MCC_RPWM_Output, -speed, direction);
    }
    return false;
  }
}

void Homing() {
  read_button(PIN_HOME_1_MCC, &buttonStateMCC1);
  read_button(PIN_HOME_2_MCC, &buttonStateMCC2);
  read_button(PIN_HOME_MOR, &buttonStateMOR);
  while (buttonStateMCC1 == 1 && buttonStateMCC2 == 1) {
    SERIAL_PRINTLN("MCC");
    read_button(PIN_HOME_1_MCC, &buttonStateMCC1);
    SERIAL_PRINT(digitalRead(4));
    read_button(PIN_HOME_2_MCC, &buttonStateMCC2);
    read_button(PIN_HOME_MOR, &buttonStateMOR);

    if (buttonStateMCC1 == 1 && buttonStateMCC2 == 1) {
      TURN_MCC(-SPEED_FACTOR_MCC);
    } else {
      TURN_MCC(0);
    }
  }
  //delay(5000);
  TURN_MCC(0);
/*
  while (buttonStateMOR == 1) {
    SERIAL_PRINTLN("MOR");
    read_button(PIN_HOME_1_MCC, &buttonStateMCC1);
    read_button(PIN_HOME_2_MCC, &buttonStateMCC2);
    read_button(PIN_HOME_MOR, &buttonStateMOR);

    if (buttonStateMOR == 1) {
      TURN_MOR(-50);
    } else {
      TURN_MOR(0);
    }
  }
  TURN_MOR(0);
  */
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

Block last_block; // Un-commented Pixy2 variable
Block Pixy_cam() { // Un-commented Pixy2 function
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
    #define SCORE_GROWTH 55

    int scores[len] = {0};
    int best = 0;

    // adding score for being big
    for (int i = 0; i < len; i++) {
      if (list_signal_ball[best].m_width * list_signal_ball[best].m_height <
          list_signal_ball[i].m_width * list_signal_ball[i].m_height) {
        best = i;
      }
    }
    scores[best] += SCORE_TAILLE;

    // adding score for being the same block as the last guested ball
    for (int i = 0; i < len; i++) {
      if (list_signal_ball[i].m_index == last_block.m_index) {
        best = i;
        // add even more if the guested ball gets bigger 
        if (last_block.m_width * last_block.m_height <
          list_signal_ball[best].m_width * list_signal_ball[best].m_height) {
          scores[best] += SCORE_GROWTH;
        }
      }
    }
    scores[best] += SCORE_INDEX;

    
    // adding score for being growing (going towerd the camera)
    for (int i = 0; i < len; i++) {
      if (list_signal_ball[best].m_age < list_signal_ball[i].m_age) {
        best = i;
      }
    }
    scores[best] += SCORE_AGE;


    // sorting the best guess
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

void Pixy_cam_print() { // Un-commented Pixy2 function
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
  SERIAL_PRINT("Pin:");
  SERIAL_PRINT(buttonPin);
  SERIAL_PRINT("=");
  SERIAL_PRINTLN(*var_stockage);
}
