void driveMotors()
{   
  // Cycle through all motors
  for(int motor = motor1; motor <= motor5; motor++)
  {
//    if(DEBUG)
//    {
//      Serial.print("Motor: ");
//      Serial.print(motor);
//      Serial.print("\tPWM: ");
//      Serial.println(motor_controls[motor]
//    }
    
    // Choose PWM channel based on motor and control orientation
        if(motor_controls[motor] >= 0)
        {
          if(motor_class[motor][motorDriverNumber] == 1)
          {
            motor_driver_1.pwm(motor_class[motor][PWM_channel_R], byte(abs(motor_controls[motor])));
            motor_driver_1.pwm(motor_class[motor][PWM_channel_F], 0x00);    
          }
          else if(motor_class[motor][motorDriverNumber] == 2)
          {
            motor_driver_2.pwm(motor_class[motor][PWM_channel_R], byte(abs(motor_controls[motor])));
            motor_driver_2.pwm(motor_class[motor][PWM_channel_F], 0x00);      
          }
        }
        else
        {
          if(motor_class[motor][motorDriverNumber] == 1)
          {
            motor_driver_1.pwm(motor_class[motor][PWM_channel_R], 0x00);
            motor_driver_1.pwm(motor_class[motor][PWM_channel_F], byte(abs(motor_controls[motor])));
          }
          else if(motor_class[motor][motorDriverNumber] == 2)
          {
            motor_driver_2.pwm(motor_class[motor][PWM_channel_R], 0x00);
            motor_driver_2.pwm(motor_class[motor][PWM_channel_F], byte(abs(motor_controls[motor])));
          }
        }
    
  }
}

// Cap control, returns 1 if capped, 0 if not
boolean capControls(uint8_t motor)
{
  if(abs(motor_controls[motor]) > control_cap)
  {
    if(motor_controls[motor] > 0) {motor_controls[motor] = control_cap;}
    else                          {motor_controls[motor] = -control_cap;}
    return 1;
  }
  else
  {
    return 0;
  }
}
