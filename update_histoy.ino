update_history(float distance){
  int history_count = 0;
  int History_size = 20;
  float History_distance[History_size];
  if(histoy_coun t== History_size){
    for(i =0; i<History_size-1;i++){
      History_distance[i] = History_distance[i+1];
    }
    History_distance[History_size-1] = distance;

  }
  else{
    History_distance[history_count] = distance;
    history_count++;
  }
}

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
if(history_couny > 1){
  return variance/history_count-1;
}
else{ 
  return 0;
    }
}




