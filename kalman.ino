class_Adaptivekalman{
  public:
  Adaptivekalman(float initial_q = 0.05, float initial_r = 0.8){
  Q = initial_q;
  R = initial_r;
  X =0;
  adaptive_factor = 1.0;
}

float updateEstimate(float measurement, bool high_noise = false){
  float current_Q = Q*adaptive_factor;
  if(high_noise){
    current_Q = current_Q*3.0;

  }

  P = P+current_Q;
  K = p/(p+R);
  X = X + K*(measurement -X );
  P = (1-K)*P;

  float innovation = abs(measurement - X);
  adaptive_factor = constrain(1.0+ innovation* 0.1, .05, 2.0);

return X;

}

void reset(){
   P = 1.0;
   X = 0;
   adaptive_factor = 1.0;
   }

  private:
  float Q,R,P,K,X;
  float adaptive_factor;
  };

  Adaptivekalman distance_filter ;
  Adaptivekalman movement_filter;