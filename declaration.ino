#include <Servo.h>
#include <EEPROM.h>

#define TRIG_PIN 9
#define ECHO_PIN 10
#define SERVO_PIN 6

int min_angle = 45;
int max_angle = 135;
int angle_step = 2;
int current_angle = 90;
int target_angle = 90;
int last_person_angle = 90;

float sensor_height = 122.0;
float current_distance = 0;
float previous_distance = 0;
float baseline_distance = 0;

float min_person_distance = 15.0;
float max_person_distance = 100.0;

float movement_threshold = 1.5;
float breathing_threshold = 0.3;
float major_movement_threshold = 5.0;
long calibration_time = 10000UL;
long stable_timeout = 30000UL;
long sleep_check_interval = 20000UL;

long last_movement_time = 0;
long last_major_movement = 0;
long state_change_time = 0;
long last_sleep_check = 0;
long session_start_time =0;

long total_movements = 0;
long major_movements = 0;
float average_distance = 0;
float movement_intensity = 0;



struct SleepProfile{
  float base_distance;
  float movement_variance;
  int preferred_angle;
  long last_major_movement ;
  float breathing_amplitude;
  bool is_calibrated;
};

enum SystemState{
  CALIBRATING,
  WAITING_FOR_PERSON,
  TRACKING_PERSON,
  PERSON_STABLE,
  SLEEP_MONITORING,
  ALERT_STATE,

};


Servo scannerServo;
SleepProfile sleep_profile;
SystemState current_state = CALIBRATING;

void setup(){
  Serial>begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  PinMode(BUZZER_PIN, OUTPUT);

  scannerServo.attach(SERVO_PIN);
  scannerServo.write(current_angle);
  delay(500);

  sleep_profile.is_calibrated = false;
  sleep_profile.base_distance = 0;
  sleep_profile.movement_variance = 0;
  sleep_profile.preferred_angle = 90;
  sleep_profile.breathing_amplitude = 0;

  for(int i =0; i<HISTORY_SIZE; i++){
    distance_history[i] = 0;
  }
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void loop() {
  if (current_state == CALIBRATING) {
    handleCalibration();
  } else if (current_state == WAITING_FOR_PERSON) {
    handleWaitingForPerson();
  } else if (current_state == TRACKING_PERSON) {
    handleTrackingPerson();
  } else if (current_state == PERSON_STABLE) {
    handlePersonStable();
  } else if (current_state == SLEEP_MONITORING) {
    handleSleepMonitoring();
  } else if (current_state == ALERT_STATE) {
    
  }
  
  delay(100); 
}





