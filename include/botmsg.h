struct ctrlmsg {
  float x;
  float z;
  char  debug[8];
} ctrlmsg;

struct statmsg { // A status msg passed back and forthe from an I2C connected board.
  float x; // Confirm current requested X value. Feedback 
  float z; // Confirm current requested Z value. Feedback 
  int   mtr_pos_right; // right motor encoder position
  int   mtr_pos_left; // left motor encoder position
  int   mtr_speed_right;  // motor a speed
  int   mtr_speed_left;  // motor b speed
  int   sen_sonar_fwd; // forward sonar value
  int   sen_sonar_rear; // rear sonar value
  int   sen_ir_right; // right IR value
  int   sen_ir_left; // left IR value
  char  debug[8]; // short logging message
} statmsg; // create a status message struct
