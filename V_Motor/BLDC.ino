void BLDC_filterOutput() {
  short V_BLDC_OUT[2] = {1500 + depth_Out - pitch_Out, 1500 + depth_Out + pitch_Out};
  
  for (byte i = 0; i < 2; i++) {
    if ( V_BLDC_OUT[i] - V_BLDC[i] >= 5 ) {
      V_BLDC[i] += 5;
    } else if (V_BLDC_OUT[i] - V_BLDC[i] <= -5) {
      V_BLDC[i] -= 5;
    }
  }
}

