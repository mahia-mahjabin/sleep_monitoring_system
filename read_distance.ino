float average_distance(){
  if(history_count==0){
    return 0;
  }

  float sum =0;
  for(int i =0; i<history_count; i++){
    sum = sum+History_distance[i];
  }
  return sum/history_count;
}

float movement_variance(){
  if(history_count==0){
    return 0;
  }
  float avg = average_distance();
  float variance = 0;

  for(int i =0 ; i<history_count; i++){
    float dif = History_distance[i] - avg;
    variance  = variance + dif*dif;
  }
if(history_count > 1){
  return variance/(history_count-1);
}
else{ 
  return 0;
    }
}

float read_distance(){
int num = 3;
float dis_meas[num];
int valid_dis = 0;

for(int i =0; i<num ; i++){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if(duration>0){

    float distance = duration*.034 / 2.0;
    if(distance>=5.0 && distance <=150){
      dis_meas[valid_dis++] = distance;
    }
  }

    delay(20);

}
if(valid_dis == 0){
  return 0;
}

for(int i = 0; i<valid_dis -1  ; i++){
  for(int j = i+1 ; j < valid_dis; j++){
    if(dis_meas[i]> dis_meas[j]){
      float temp = dis_meas[i];
      dis_meas[i] = dis_meas[j];
      dis_meas[j] = temp;
    }
  }

}

float median = dis_meas[valid_dis/2];

bool high_noise = false;

if(movement_variance() > 4)
{
  high_noise = true;

}

return distance_filter.updateEstimate(median, high_noise);


}



