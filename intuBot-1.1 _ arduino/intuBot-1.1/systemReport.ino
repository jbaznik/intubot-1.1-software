/*
 * Machine Interface
 * Output raw data, all data big endian
 * Bytes 0-3:   motor 1 pos
 * Bytes 4-7:   motor 2 pos
 * Bytes 8-11:  motor 3 pos    
 * Bytes 12-15: motor 4 pos    
 * Bytes 16-19: motor 5 pos    
 * Byte 20:     automated button    
 * Byte 21:     hall sensor    
 * Byte 22:     joystick left button    
 * Byte 23:     joystick right button    
 * Byte 24-25:  power switch    
 * Bytes 26-27: joystick left X    
 * Bytes 28-29: joystick left Y    
 * Bytes 30-31: joystick right X    
 * Bytes 32-33: joystick right Y    
 * Bytes 34-35: analog rocker    
 * Bytes 36-38: end flag (UUU)
 * (e.g.)     
 * b'\x01\x04\xfc\x00\x00C\x00\x00\x00\x00\x00\x00\x00\x01\x55\x55\x55'
 */
void systemReport()
{
  if(millis() - report_time > reportDelay && !PLOT && !DEBUG && !MOTOR_ZERO)
    {
      //Positions 
      Serial.write((byte*) &motor_positions_out, 16);
      Serial.write((byte*) &motor5_position, 4);

      //Auto Mode Buttons
      Serial.write((byte*) &AUTO_MODE, 1);

      //Translational Sled Zeroing
      Serial.write((byte*) &SLED_ZERO, 1);

      //Joystick Buttons
      Serial.write((byte*) &LEFT_BUTT, 1);
      Serial.write((byte*) &RIGHT_BUTT, 1);

      //Analog Variables
      Serial.write((byte*) &power_switch, 2);
      
      Serial.write((byte*) &left_horz, 2);
      Serial.write((byte*) &left_vert, 2);

      Serial.write((byte*) &right_horz, 2);
      Serial.write((byte*) &right_vert, 2);

      Serial.write((byte*) &analog_rocker, 2);

      // Finish byte array with b'UUU' = 01010101 01010101 01010101
      Serial.write(0x55);Serial.write(0x55);Serial.write(0x55);
      //Serial.write('\n');
      
      report_time = millis();
    }
  if(PLOT)
  {
    
    
    //Serial.print(dt);Serial.print("\t");
    /*
    Serial.print(motor_rotations[motor1]);Serial.print("\t");
    Serial.print(motor_rotations[motor2]);Serial.print("\t");
    Serial.print(motor_rotations[motor3]);Serial.print("\t");
    Serial.print(motor_rotations[motor4]);Serial.print("\t");


    Serial.print(SLED_ZERO);Serial.print("\t");
    */
    
    Serial.print(left_horz);Serial.print("\t");
    Serial.print(left_vert);Serial.print("\t");
    Serial.print(right_horz);Serial.print("\t");
    Serial.print(right_vert);Serial.print("\t");
    
    /*
    Serial.print(left_horz/3.0);Serial.print("\t");
    Serial.print(left_vert/3.0);Serial.print("\t");
    Serial.print(right_horz/3.0);Serial.print("\t");
    Serial.print(right_vert/3.0);Serial.print("\t");
    Serial.print(analog_rocker/3.0);Serial.print("\t");
    Serial.print(motor_positions_out[motor1]*360.0/fourteenBit);Serial.print("\t");
    Serial.print(motor_positions_out[motor2]*360.0/fourteenBit);Serial.print("\t");
    Serial.print(motor_positions_out[motor3]*360.0/fourteenBit);Serial.print("\t");
    Serial.print(motor_positions_out[motor4]*360.0/fourteenBit);Serial.print("\t");
    Serial.print(motor_positions_out[motor5]);Serial.print("\t");
    */

    Serial.print("\n");
  }
  if(MOTOR_ZERO)
  {
    Serial.print("{");
    Serial.print(motor_positions[motor1]);Serial.print(", ");
    Serial.print(motor_positions[motor2]);Serial.print(", ");
    Serial.print(motor_positions[motor3]);Serial.print(", ");
    Serial.print(motor_positions[motor4]);Serial.print("};");Serial.print("\n");
    
  }
}
