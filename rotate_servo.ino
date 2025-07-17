void rotate_servo(float filtered_distance, bool moving_object) {
   if (filtered_distance <sensor_height  && moving_object) {
    int best_angle = current_angle;
    float best_distance = 999;

    for (int angle = min_angle ; angle <= max_angle; angle += 5) {
      scannerServo.write(angle);
      delay(100);

      float total = 0;
      int valid_samples = 0;

      for (int i = 0; i < 3; i++) {
        float d = read_distance();
        if (d > movement_threshold && d < sensor_height) {
          total += d;
          valid_samples++;
        }
        delay(50);
      }

      if (valid_samples > 0) {
        float avg_distance = total / valid_samples;
        if (avg_distance < best_distance) {
          best_distance = avg_distance;
          best_angle = angle;
        }
      }
    }

    if (abs(best_angle - last_best_angle) <= 1 && abs(best_distance - last_best_distance) <= 1) {
      if (!is_stable) {
        stable_start_time = millis();
        is_stable = true;
      } else if (millis() - stable_start_time >= STABLE_TIMEOUT) {
        return; // Do not rotate anymore
      }
    } else {
      is_stable = false;
    }

    current_angle = best_angle;
    scannerServo.write(current_angle);

    last_best_angle = best_angle;
    last_best_distance = best_distance;
  }
}w