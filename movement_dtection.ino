bool movement_detect(float current, float previous){
  if(previous<0 || current<0){
    return false;
  }

  float diff = abs(current - previous);
  float filtered_diff =  float filtered_diff = movement_filter.updateEstimate(diff);
  
}

if(sleep_profile.is_calibrated){
      float profile_threshold = sleep_profile.movement_variance * 0.8;
      if(profile_threshold>movement_threshold){
        adaptive_threshold = profile_threshold;
      }
      else{
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
