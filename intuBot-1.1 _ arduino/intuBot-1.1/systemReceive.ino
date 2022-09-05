/*Serial Communication from CPU
 * e.g. b'M3R-123M5R22Z' --> Drive motor 3 to -123 PWM, and motor 5 to 22 PWM
 * e.g. b'SZ'            --> STOP!
 * e.g. b'LA123LD222LL69Z'   --> Drive button LED to 132, drive display backlight to 222
 */  
void systemReceive()
{
  input_string = "";
  uint8_t i = 0;
  uint8_t motor = 255; //Initiate arbitrary value outside of motor scope
  
  while(Serial.available()) 
  {
    char in_char = (char)Serial.read();
    if(DEBUG){input_string += in_char; i = i + 1;}

    switch(in_char)
    {
      //Motor Select (1 - 5)
      case 'm':
        motor = Serial.parseInt() - 1;
        if(DEBUG){input_string += String(motor+1);}

        //if bad motor value, set to arbitrary value outside of motor scope
        if(motor < motor1 || motor > motor5){motor = 255;}
        break;

      //Parse Raw command (-255 - 255)
      case 'r':
        if (motor == 255)
        {
          STOP = true;
          break;
        }
        else
        {
          STOP = false;
          motor_controls[motor] = Serial.parseInt();//*motor_polarity[motor];
          
          if(DEBUG){input_string += String(motor_controls[motor]);}
          capControls(motor);
          break;
        }

      // LED Brightness (0 - 255)
      case 'l':
        switch((char)Serial.read())
        {
          // Auto Mode Button LED
          case 'a':
            auto_LED = Serial.parseInt();
            break;
          
          // Display backlight LED
          case 'd':
            display_LED = Serial.parseInt();
            break;
          
          // Display backlight LED
          case 'l':
            laryngo_LED = Serial.parseInt();
            break;
        }
        
      // STOP value (0,1)
      case 's':
        STOP = bool(Serial.parseInt());
        break;

      //End transmission flag, not strictly necessary...
      case 'z':
      if(DEBUG){Serial.println(input_string);}
        break;
    }
  } 
}  
