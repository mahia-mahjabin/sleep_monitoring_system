#include <Servo.h>
#include <EEPROM.h>

#define TRIG_PIN 9
#define ECHO_PIN 10
#define SERVO_PIN 6
#define BUZZER_PIN 7
#define HISTORY_SIZE 20

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
long session_start_time = 0;

long total_movements = 0;
long major_movements = 0;
float average_distance = 0;
float movement_intensity = 0;

int history_count = 0;
float history_distance[HISTORY_SIZE];
int last_best_angle = 90;
float last_best_distance = 999.0;
bool is_stable = false;
long stable_start_time = 0;
float adaptive_threshold = 1.5;

struct SleepProfile {
  float base_distance;
  float movement_variance;
  int preferred_angle;
  long last_major_movement;
  float breathing_amplitude;
  bool is_calibrated;
};

enum SystemState {
  CALIBRATING,
  WAITING_FOR_PERSON,
  TRACKING_PERSON,
  PERSON_STABLE,
  SLEEP_MONITORING,
  ALERT_STATE
};

Servo scannerServo;
SleepProfile sleep_profile;
SystemState current_state = CALIBRATING;

class AdaptiveKalman {
public:
  AdaptiveKalman(float initial_q = 0.05, float initial_r = 0.8) {
    Q = initial_q;
    R = initial_r;
    P = 1.0;
    X = 0;
    adaptive_factor = 1.0;
  }

  float updateEstimate(float measurement, bool high_noise = false) {
    float current_Q = Q * adaptive_factor;
    if (high_noise) {
      current_Q = current_Q * 3.0;
    }

    P = P + current_Q;
    K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1 - K) * P;

    float innovation = abs(measurement - X);
    adaptive_factor = constrain(1.0 + innovation * 0.1, 0.5, 2.0);

    return X;
  }

  void reset() {
    P = 1.0;
    X = 0;
    adaptive_factor = 1.0;
  }

private:
  float Q, R, P, K, X;
  float adaptive_factor;
};

AdaptiveKalman distance_filter;
AdaptiveKalman movement_filter;

void updateHistory(float distance) {
  if (history_count == HISTORY_SIZE) {
    for (int i = 0; i < HISTORY_SIZE - 1; i++) {
      history_distance[i] = history_distance[i + 1];
    }
    history_distance[HISTORY_SIZE - 1] = distance;
  } else {
    history_distance[history_count] = distance;
    history_count++;
  }
}

float getAverageDistance() {
  if (history_count == 0) {
    return 0;
  }

  float sum = 0;
  for (int i = 0; i < history_count; i++) {
    sum = sum + history_distance[i];
  }
  return sum / history_count;
}

float getMovementVariance() {
  if (history_count == 0) {
    return 0;
  }
  float avg = getAverageDistance();
  float variance = 0;

  for (int i = 0; i < history_count; i++) {
    float diff = history_distance[i] - avg;
    variance = variance + diff * diff;
  }
  if (history_count > 1) {
    return variance / (history_count - 1);
  } else {
    return 0;
  }
}

float readDistance() {
  int num = 3;
  float dis_meas[num];
  int valid_dis = 0;

  for (int i = 0; i < num; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);

    if (duration > 0) {
      float distance = duration * 0.034 / 2.0;
      if (distance >= 5.0 && distance <= 150) {
        dis_meas[valid_dis++] = distance;
      }
    }
    delay(20);
  }

  if (valid_dis == 0) {
    return -1;
  }

  for (int i = 0; i < valid_dis - 1; i++) {
    for (int j = i + 1; j < valid_dis; j++) {
      if (dis_meas[i] > dis_meas[j]) {
        float temp = dis_meas[i];
        dis_meas[i] = dis_meas[j];
        dis_meas[j] = temp;
      }
    }
  }

  float median = dis_meas[valid_dis / 2];

  bool high_noise = false;
  if (getMovementVariance() > 4) {
    high_noise = true;
  }

  return distance_filter.updateEstimate(median, high_noise);
}

bool detectMovement(float current, float previous) {
  if (previous < 0 || current < 0) {
    return false;
  }

  float diff = abs(current - previous);
  float filtered_diff = movement_filter.updateEstimate(diff);

  if (sleep_profile.is_calibrated) {
    float profile_threshold = sleep_profile.movement_variance * 0.8;
    if (profile_threshold > movement_threshold) {
      adaptive_threshold = profile_threshold;
    } else {
      adaptive_threshold = movement_threshold;
    }
  }
  return filtered_diff > adaptive_threshold;
}

bool detectMajorMovement(float current, float baseline) {
  if (baseline < 0 || current < 0) {
    return false;
  }
  return abs(current - baseline) > major_movement_threshold;
}

void moveServoSmooth(int target) {
  int steps = abs(target - current_angle);
  int direction;
  if (target > current_angle) {
    direction = 1;
  } else {
    direction = -1;
  }
  for (int i = 0; i < steps; i++) {
    current_angle = current_angle + direction;
    scannerServo.write(current_angle);
    delay(20);
  }
}

float calculateConfidence(float distance) {
  if (distance < min_person_distance || distance > max_person_distance) {
    return 0;
  }

  float range = max_person_distance - min_person_distance;
  float distance_from_ideal = abs(distance - sleep_profile.base_distance);
  float max_allowed_deviation = range / 2;

  return 1.0 - (distance_from_ideal / max_allowed_deviation);
}

void searchAngleRange(int start_angle, int end_angle, int step_size, int delay_ms, int &best_angle, float &best_distance, float &best_confidence) {
  for (int angle = start_angle; angle <= end_angle; angle += step_size) {
    scannerServo.write(angle);
    delay(delay_ms);

    float distance = readDistance();
    if (distance < 0) continue;

    float confidence = calculateConfidence(distance);
    if (confidence > best_confidence || (confidence > 0.7 && distance < best_distance)) {
      best_angle = angle;
      best_distance = distance;
      best_confidence = confidence;
    }
  }
}

int findPersonPosition() {
  int best_angle = current_angle;
  float best_distance = 999.0;
  float best_confidence = 0;

  int search_range = 20;
  int start_angle = constrain(last_person_angle - search_range, min_angle, max_angle);
  int end_angle = constrain(last_person_angle + search_range, min_angle, max_angle);
  searchAngleRange(start_angle, end_angle, angle_step, 100, best_angle, best_distance, best_confidence);

  if (best_confidence > 0.3) {
    last_person_angle = best_angle;
    return best_angle;
  }

  return -1;
}

void printSleepStats() {
  Serial.print("Sleep Stats - Total Movements: ");
  Serial.print(total_movements);
  Serial.print(", Major Movements: ");
  Serial.print(major_movements);
  Serial.print(", Session Time: ");
  Serial.print((millis() - session_start_time) / 1000);
  Serial.println(" seconds");
}

void handleCalibration() {
  static long calibration_start = 0;
  static int calibration_readings = 0;
  static float calibration_sum = 0;
  static int cal_angle = 45;

  if (calibration_start == 0) {
    calibration_start = millis();
    Serial.println("CALIBRATION MODE");
  }

  if (millis() - calibration_start < calibration_time) {
    scannerServo.write(cal_angle);
    delay(200);

    float distance = readDistance();
    if (distance > 0) {
      calibration_sum += distance;
      calibration_readings++;
      updateHistory(distance);
    }

    cal_angle += 10;
    if (cal_angle > max_angle) {
      cal_angle = min_angle;
    }
  } else {
    if (calibration_readings > 0) {
      baseline_distance = calibration_sum / calibration_readings;
      sleep_profile.base_distance = baseline_distance;
      sleep_profile.movement_variance = getMovementVariance();
      sleep_profile.is_calibrated = true;
      Serial.println("CALIBRATION COMPLETE");

      current_state = WAITING_FOR_PERSON;
      state_change_time = millis();
    } else {
      calibration_start = 0;
      calibration_readings = 0;
      calibration_sum = 0;
    }
  }
}

void handleWaitingForPerson() {
  current_distance = readDistance();
  if (current_distance < 0) {
    return;
  }
  updateHistory(current_distance);

  if (current_distance < baseline_distance - 5.0) {
    if (detectMovement(current_distance, previous_distance)) {
      Serial.println("PERSON DETECTED");
      current_state = TRACKING_PERSON;
      state_change_time = millis();
      session_start_time = millis();
      last_movement_time = millis();
      int found_angle = findPersonPosition();
      if (found_angle >= 0) {
        current_angle = found_angle;
        last_person_angle = found_angle;
      }
    }
  }
  previous_distance = current_distance;
}

void handleTrackingPerson() {
  current_distance = readDistance();
  if (current_distance < 0) {
    return;
  }

  updateHistory(current_distance);
  bool is_moving = detectMovement(current_distance, previous_distance);
  bool major_movement = detectMajorMovement(current_distance, sleep_profile.base_distance);

  if (is_moving) {
    last_movement_time = millis();
    total_movements++;
    Serial.println("Movement detected!");

    int new_angle = findPersonPosition();
    if (new_angle >= 0) {
      moveServoSmooth(new_angle);
      current_angle = new_angle;
      last_person_angle = new_angle;
    }
  } else {
    if (current_distance < previous_distance - 1.0) {
      int new_angle = constrain(current_angle + 2, min_angle, max_angle);
      moveServoSmooth(new_angle);
      current_angle = new_angle;
    } else if (current_distance > previous_distance + 1.0) {
      int new_angle = constrain(current_angle - 2, min_angle, max_angle);
      moveServoSmooth(new_angle);
      current_angle = new_angle;
    }
  }

  if (millis() - last_movement_time > stable_timeout) {
    Serial.println("PERSON IS STABLE");
    current_state = PERSON_STABLE;
    state_change_time = millis();
    sleep_profile.base_distance = getAverageDistance();
  }

  previous_distance = current_distance;
}

void handlePersonStable() {
  current_distance = readDistance();
  if (current_distance < 0) {
    return;
  }

  updateHistory(current_distance);
  bool is_moving = detectMovement(current_distance, previous_distance);
  if (is_moving) {
    last_movement_time = millis();
    total_movements++;

    float movement_intensity = abs(current_distance - previous_distance);
    if (movement_intensity > major_movement_threshold) {
      current_state = TRACKING_PERSON;
      state_change_time = millis();
    }
  } else {
    if (millis() - last_movement_time > stable_timeout * 2) {
      current_state = SLEEP_MONITORING;
      state_change_time = millis();
      sleep_profile.breathing_amplitude = getMovementVariance();
    }
  }
  previous_distance = current_distance;
}

void handleSleepMonitoring() {
  static long last_report = 0;

  if (millis() - last_sleep_check > sleep_check_interval) {
    current_distance = readDistance();
    if (current_distance < 0) {
      return;
    }
    updateHistory(current_distance);
    bool is_moving = detectMovement(current_distance, previous_distance);
    bool major_movement = detectMajorMovement(current_distance, sleep_profile.base_distance);

    if (major_movement) {
      Serial.println("major movement returning to tracking");
      current_state = TRACKING_PERSON;
      state_change_time = millis();
      last_major_movement = millis();
      major_movements++;
    } else if (is_moving) {
      total_movements++;
      last_movement_time = millis();

      if (current_distance < previous_distance - 0.5) {
        int new_angle = constrain(current_angle + 1, min_angle, max_angle);
        scannerServo.write(new_angle);
        current_angle = new_angle;
      } else if (current_distance > previous_distance + 0.5) {
        int new_angle = constrain(current_angle - 1, min_angle, max_angle);
        scannerServo.write(new_angle);
        current_angle = new_angle;
      }
    }

    if (millis() - last_report > 30000) {
      printSleepStats();
      last_report = millis();
    }
  }
  last_sleep_check = millis();
  previous_distance = current_distance;
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  scannerServo.attach(SERVO_PIN);
  scannerServo.write(current_angle);
  delay(500);

  sleep_profile.is_calibrated = false;
  sleep_profile.base_distance = 0;
  sleep_profile.movement_variance = 0;
  sleep_profile.preferred_angle = 90;
  sleep_profile.breathing_amplitude = 0;

  for (int i = 0; i < HISTORY_SIZE; i++) {
    history_distance[i] = 0;
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
  }

  delay(100);
}