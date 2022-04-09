#include <Wire.h>

const float pi = 3.14159265359;
const int MPU6050_Addr = 0x68;
// const float gravitational_acceleration = 9.80665;

boolean set_gyro_angles = 0;

unsigned long loop_period = 4000; // in microseconds
unsigned long last_loop_period;

long raw_acc_x, raw_acc_y, raw_acc_z;
long raw_gyro_x, raw_gyro_y, raw_gyro_z;
long raw_temperature;

float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;
float temperature;

float acc_total_vector;
float acc_pitch_angle, acc_roll_angle;

long raw_gyro_x_cal, raw_gyro_y_cal, raw_gyro_z_cal;

float angle_pitch, angle_roll;
float angle_pitch_output, angle_roll_output;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  setup_mpu_6050_registers();
  get_gyro_calibration_data();
  last_loop_period = micros();
}

void loop() {
  read_mpu_6050_data();
  process_raw_data();

  Serial.print("angle_pitch = ");
  Serial.print(angle_pitch);
  Serial.print(" angle_roll = ");
  Serial.print(angle_roll);
  Serial.print(" acc_x = ");
  Serial.print(acc_x);
  Serial.print(" acc_y = ");
  Serial.print(acc_y);
  Serial.print(" acc_z = ");
  Serial.println(acc_z);

  // ROBOT ONE DOGRU EGILDIGINDE ACC_Z < 0
  // ROBOT ARKAYA DOGRU EGILDIGINDE ACC_Z > 0
  // Serial.printler hariç imu hesaplamaları ortalama 1600 microsecond süre alıyor.
  // Diğer işlemler için 2400 microsecond sürem var.
  
  while(micros()-last_loop_period < loop_period) { // 3.3 VOLT'LUK 8 MHZ'LIK ARDUINO PRO MINI'DE LOOP_PERIOD'U BURADA 2 İLE ÇARP (IDEDE SIKINTI VAR 16 MHZ GİBİ DÜŞÜNÜYOR)
    //do nothing while we wait for the fixed period to expire
  }
  last_loop_period = micros();
}

void process_raw_data(){
  raw_gyro_x -= raw_gyro_x_cal;
  raw_gyro_y -= raw_gyro_y_cal;
  raw_gyro_z -= raw_gyro_z_cal;
  
  temperature = raw_temperature/340.0 + 36.53;
  
  acc_x = raw_acc_x/4096.0; // acceleration about x-axis in g
  acc_y = raw_acc_y/4096.0; // acceleration about y-axis in g
  acc_z = raw_acc_z/4096.0; // acceleration about z-axis in g
  
  gyro_x = raw_gyro_x/65.5; // rotation about x-axis in degree/second
  gyro_y = raw_gyro_y/65.5; // rotation about y-axis in degree/second
  gyro_z = raw_gyro_z/65.5; // rotation about z-axis in degree/second

  angle_pitch += gyro_x * 0.004; // Main loop iterates every 4 milliseconds so in order to get rotation in degrees we multiply by 0.004 seconds
  angle_roll += gyro_y * 0.004; // Main loop iterates every 4 milliseconds so in order to get rotation in degrees we multiply by 0.004 seconds
  
  // The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.004 * (pi / 180));               //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_roll -= angle_pitch * sin(gyro_z * 0.004 * (pi / 180));               //If the IMU has yawed transfer the pitch angle to the roll angle
  
  // The Arduino asin function is in radians
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  acc_pitch_angle = asin((float)acc_y/acc_total_vector)* (180/pi);
  acc_roll_angle = asin((float)acc_x/acc_total_vector)* -(180/pi);

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  acc_pitch_angle -= 0;                                              //Accelerometer calibration value for pitch
  acc_roll_angle -= 0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles){                                                 //If the IMU is already started
    angle_pitch = angle_pitch * 0.98 + acc_pitch_angle * 0.02;        //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.98 + acc_roll_angle * 0.02;           //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{                                                                //At first start
    angle_pitch = acc_pitch_angle;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = acc_roll_angle;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }

/*
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
*/
}

void get_gyro_calibration_data(){
  Serial.println("\n\nCalibrating gyro");                              //Print text to screen
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    raw_gyro_x_cal += raw_gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    raw_gyro_y_cal += raw_gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    raw_gyro_z_cal += raw_gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  raw_gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  raw_gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  raw_gyro_z_cal /= 2000; 
}

void read_mpu_6050_data(){
  Wire.beginTransmission(0x68);                                      //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  raw_acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  // Read 1 byte and shift it 8 position to the left and then read another 1 byte, to concatanate these two bytes, bitwise OR them
  raw_acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  raw_acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  raw_temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  raw_gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  raw_gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  raw_gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(MPU6050_Addr);   //Start communicating with the MPU-6050
  Wire.write(0x6B);                       //Send the requested starting register (PWR_MGMT_1)
  Wire.write(0x00);                       //Set the register (PWR_MGMT_1) as b'00000000'
  Wire.endTransmission();                 //End the transmission

    //Configure the accelerometer (+/-8g) SET ACCELEROMETER'S FULL SCALE as (+/-8g) --> LSB SENSITIVITY = 4096 LSB/g
  Wire.beginTransmission(MPU6050_Addr);                                //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register (ACCEL_CONFIG)
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  
  //Configure the gyro (500dps full scale) SET GYROSCOPE'S FULL SCALE as (500dps) --> LSB SENSITIVITY = 65.5 LSB/(degree/second)
  Wire.beginTransmission(MPU6050_Addr);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register (GYRO_CONFIG)
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();

  //Configure the MPU_6050 to work at gyroscope 8 kHz and accelerometer at 1 kHz
  Wire.beginTransmission(MPU6050_Addr);
  Wire.write(0x19);                              // Send the requested starting register (SMPRT_DIV)
  Wire.write(0x00);                              // Write 0x00 in that register to obtain maximum sample rate  // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_Addr);
  Wire.write(0x1A);                             // Send the requested starting register (CONFIG)
  Wire.write(0x00);                             // Write 0x00 in that register to set DLPF_CFG = 0
  Wire.endTransmission();
  
  delay(1500);
  Serial.println("Setup of MPU-6050 registers are done.");
}
