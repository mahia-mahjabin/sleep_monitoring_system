void moveServoSmooth(int target){
  int stepts = abs(target - current_angle);
  int direction;
  if(target > current_angle){
    direction = 1;
  }
  else{
    direction  = -1;
  }
  for(int i =0; i<steps; i++){
    current_angle = current_angle + direction;
    scannerServo.write(current_angle);
    delay(20);
  }
} 

float calculateConfidence(float distance){
  if(distance < MIN_PERSON_DISTANCE || distance> MAX_PERSON_DISTANCE){
    return 0;
  }

 float range = max_person_distance - min_person_distance;
 float distance_from_ideal = abs(distance - sleep_profile.base_distance)
 float max_allowed_deviation = range/2;
 
 return 1.0 - (distance_from_ideal/max_allowed_deviation);
 
 }

}

int search_angle_range(int start_angle, int end_angle, int step_size, int delay_ms, int &best_angle, float &best_distance, float &best_confidence  ){
  for(int angle = start_angle; angle <= end_angle; angle +=step_size ){
    scannerServo.write(angle);
    delay(delay_ms);

    float distance = read_distance();
    if(distance < 0) continue;

    float confidence = calculateConfidence(distance);
    if(confidence > best_confidence || confidence > 0.7 && distance < best_distance){
      best_angle = angle;
      best_distance = distance;
      best_confidence = confidence;
    }
  }
  return 0;
}

int findPersonPosition(){
  int best_angle = current_angle;
  float best_distance = 999.0;
  float best_confidence = 0;

  int search_range = 20;
  int start_angle = constrain(last_person_angle-search_range, MIN_ANGLE, MAX_ANGLE);
  int end_angle = constrain(last_person_angle+search_range, MIN_ANGLE, MAX_ANGLE );
  searchAngle(start_angle, end_angle, AGLE_STEP, 100, best_angle, best_distance, best_confidence);
}

if(best_confidence > 0.3){
  last_person_angle = best_angle;
  return best_angle;
}

return -1; //when person is not found