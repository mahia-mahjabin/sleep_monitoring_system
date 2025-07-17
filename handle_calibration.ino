void handle_calibration(){
  long calibration_start = 0;
  int calibration_readings = 0;
  float calibration_sum = 0;
  int cal_angle = MIN_ANGLE;
  if(calibration_start == 0){
    calibration_start = millis();
    serial.println("CALIBRATION MODE");

  }

  if(millis() - calibration_start < CALIBRATION_TIME){
    scannerServo.write(cal_angle);
    delay(200);

    float distance = readDistance();
    if(distance > 0){
      calibration_sum += distance;
      calibrsation_readings++;
      updateHistory(distance);
    }

    cal_angle +=10;
    if(cal_angle > MAX_ANGLE){
      cal_angle = MIN_ANGLE;
    }
  }

  else{
    if(calibration_readings > 0){

      baseline_distance = calibration_sum / calibration_readings;
      sleep_profile.base_distance = baseline_distance;
      sleep_profile.movement_variance = getMovementVariance();
      sleep_profile.is_calibrated = true;
      Serial.println("CALIBRATION COMPLETE");

      current_state = WAITING_FOR_PERSON;
      state_change_time = milllis();


    }
    else{
      calibration_start = 0;
      calibration_readings = 0;
      calibration_sum = 0 ;
    }
  }
}

void handleWaitingForPerson(){
  current_distance = readDistance();
  if(current<0){
    return 0;
  }
  updateHistory(current_distance);

  if(current_distance < baseline_distance - 5.0){
    if(detectMovement(current_distance, previous_distance)){
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


void handleTrackingPerson(){

  current_distance = readDistance();
   if(current_distance < 0){
    return 0;
   }

   updateHistory(current_distance);
   bool is_moving = detectMovements(current_distance, previous_distance);
   bool major_movement = detectMajorMovement(current_distance, sleep_profile.base_distance);

   if(is_moving){
    last_movement_time = millis();
    total_movements++;
    Serial.println("Major movement detected!");

    int new_angle = findPersonPosition();
    if(new_angle >= 0){
      moveServoSmooth(new_angle);
      current_angle = new_angle;
      last_person_angle = new_angle;

    }
   }

   else{
    if(current_distance < previous_distance -1.0 ){
      int new_angle = constrain(current_angle + 2, MIN_ANGLE, MAX_ANGLE);
      moveServoSmooth(new_angle);
      current_angle = new_angle;
      
    }
    else if (current_distance > previous_distance +1.0){
      int new_angle = constrain(current_angle+2, MIN_ANGLE, MAX_ANGLE);
      moveServoSmooth(new_angle);
      current_angle = new_angle;
    }
   }

   if(millis() - last_movement_time > STABLE_TIMEOUT){
    Serial.println("PERSON IS STABLE");
    state_change_time = millis();
    sleep_profile.base_distance = getAverageDistance();
   }

   previous_distance = current_distance;

}

void handlePersonStable(){

  current_distance = readDistance();
  if(current_distance < 0){
    return 0;
  }

  updateHistory(current_distence);
  bool is_moving = detectMovement(current_distance, previous_distance);
  if(is_moving){
    last_movement_time = millis();
    total_movements++;

    float movement_intensity = abs(current_distance - previous_distance);
    if(movement_intensity > major_movement_threshold){
      current_state = TRACLING_PERSON;
      state_change_time = millis();
    }
  }
  else{
    if(millis() - last_movement_time > STABLE_TIMEOUT*2){
       current_state = SLEEP_MONITORING;
       state_change_time = millis();
       sleep_profile.breathing_amplitude = getMovementVariance();

    }
  }
  previous_distance = current_distance;
}

void handleSleepMonitoring(){
  long last_report = 0;

  if(millis() - last_sleep_check > SLEEP_CHECK_INTERVAL){
    current_distance = readDistance();
    if(current_distance < 0){
      return ;
    }
    updateHistory(current_distance);
    bool is_moving = detectMovement(current_distance , previous_distance);
    bool major_movement = detectMajorMovement(current_distance , sleep_profile.base_distance);
  
    if(major_movement){
      Serial.println("major movement returning to tracking");
      current_state = TRACKING_PERSON;
      state_change_time = millis();
      last_major_movement = millis();
      major_movements++;
    }
    else if (is_moving){
      total_movements++;
      last_movement_time = millis();

      if(current_distance < previous_distance - 0.5){

        int new_angle = constrain(current_angle + 1, MIN_ANGLE, MAX_ANGLE);
        scannerServo.write(new_angle);
        current_angle = new_angle;
      }
      else if(current_distance > previous_distance + 0.5){
        int new_angle = constrain(current_angle -1, MIN_ANGLE , MAX_ANGLE);
        scannerServo.write(new_angle);
        current_angle = new_angle;
      }

    }

    if(millis() - last_report > 30000){
      printSleepStats();
      last_report = millis();
    }
  }
  last_sleep_check = millis();
  previous_distance = current_distance;
}


