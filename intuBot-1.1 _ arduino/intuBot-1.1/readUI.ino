void readUI()
{
  //Set motor 5 position to 0 if hall effect sensor triggered
  SLED_ZERO = digitalRead(home_pos_pin);
  if(!SLED_ZERO){motor5_position = 0;}
  
  RIGHT_BUTT = digitalRead(right_push_pin);
  LEFT_BUTT = digitalRead(left_push_pin);

  AUTO_MODE = digitalRead(auto_mode_pin);

  left_horz = analogRead(left_horz_pin);
  left_vert = analogRead(left_vert_pin);

  right_horz = analogRead(right_horz_pin);
  right_vert = analogRead(right_vert_pin);

  analog_rocker = analogRead(rocker_pin);

  // Read the digital buttons for alternative rocker
  ROCKER_ALT1 = digitalRead(push_1_pin);
  ROCKER_ALT2 = digitalRead(push_2_pin);
  

  power_switch = analogRead(power_sw_pin);
}
