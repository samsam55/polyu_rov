void KalmanUpdate(float b) {
  p = p + q;
  k = p/(p + r);
  p = (1-k)*p;
  f_depth_cm = k*(b-f_depth_cm)+f_depth_cm;
}

