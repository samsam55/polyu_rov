void BLDC_filterOutput() {
  short V_BLDC_OUT[4] = {1500 - yaw_Out, 1500 - yaw_Out, 1500 + yaw_Out, 1500 + yaw_Out};
  short V_COM_OUT[4], angle;

  if (com_angle >= -45 && com_angle <= 45) {
    angle = com_angle;
    V_COM_OUT[0] =   45 - angle;
    V_COM_OUT[1] =   45 + angle;
    V_COM_OUT[2] = - 45 + angle;
    V_COM_OUT[3] = - 45 - angle;
  } else if (com_angle >= 45 && com_angle <= 135) {
    angle = com_angle - 90;
    V_COM_OUT[0] = - 45 - angle;
    V_COM_OUT[1] =   45 - angle;
    V_COM_OUT[2] =   45 + angle;
    V_COM_OUT[3] = - 45 + angle;    
  } else if (com_angle >= -135 && com_angle <= -45) {
    angle = com_angle + 90;
    V_COM_OUT[0] =   45 + angle;
    V_COM_OUT[1] = - 45 + angle;
    V_COM_OUT[2] = - 45 - angle;
    V_COM_OUT[3] =   45 - angle;      
  } else if (com_angle >= 135 && com_angle <= 180) {
    angle = com_angle - 180;
    V_COM_OUT[0] = - 45 + angle;
    V_COM_OUT[1] = - 45 - angle;
    V_COM_OUT[2] =   45 - angle;
    V_COM_OUT[3] =   45 + angle;  
  } else if (com_angle >= -180 && com_angle <= -135) {
    angle = com_angle + 180;
    V_COM_OUT[0] = - 45 + angle;
    V_COM_OUT[1] = - 45 - angle;
    V_COM_OUT[2] =   45 - angle;
    V_COM_OUT[3] =   45 + angle;  
  }

  V_BLDC_OUT[0] += ((float)V_COM_OUT[0]/45.0 * com_mag);
  V_BLDC_OUT[1] -= ((float)V_COM_OUT[1]/45.0 * com_mag);
  V_BLDC_OUT[2] -= ((float)V_COM_OUT[2]/45.0 * com_mag);
  V_BLDC_OUT[3] += ((float)V_COM_OUT[3]/45.0 * com_mag);
  
  for (byte i = 0; i < 4; i++) {
    if ( V_BLDC_OUT[i] - V_BLDC[i] >= 5 ) {
      V_BLDC[i] += 5;
    } else if (V_BLDC_OUT[i] - V_BLDC[i] <= -5) {
      V_BLDC[i] -= 5;
    }
  }
}

