#include "Gyems_Teensy41.h"
#include <FlexCAN_T4.h>

/*Canbus Setup*/
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;
CAN_message_t msgR;
/*Canbus Setup*/

/*Motor Setup*/
uint32_t Motor_ID1 = 1; // Motor Can Bus ID, left leg, loadcell: port 1, positive current = flexion
uint32_t ID_offset = 0x140;
double Gear_ratio = 6;
int CAN_ID = 3;
double Pgain = 7.5;    //P gain of torque control
double Igain = 0.7;    //I gain of torque control
double Dgain = 0;      //D gain of torque control
double MaxPIDout = 10; //Max torque control PID output (Unit Current A, inner loop is current controller)
double Cur_command_L = 0;
Gyems_Teensy41 m1(Motor_ID1, CAN_ID, Gear_ratio);
/*Motor Setup*/

/*Time Setup*/
double Fsample = 50;
unsigned long current_time = 0;
unsigned long previous_time = 0;
unsigned long Tinterval_microsecond = (unsigned long)(20000);
/*Time Setup*/

void setup() {
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  delay(1000);
  Serial.begin(115200);
  initial_CAN();
  m1.init_motor();
  delay(100);
  reset_motor_angle();
}

void loop() {

  current_time = micros();

  Cur_command_L = 0;//.75 * sin(current_time / 1000000.0 / 2.0);

  if (current_time - previous_time > Tinterval_microsecond)
  {
    //m1.send_current_command(Cur_command_L);
    m1.send_current_command_for36(Cur_command_L);
    receive_CAN_data();
    delay(2);
    m1.read_multi_turns_angle_for36();
    receive_CAN_data();
    delay(2);
    m1.read_motor_current_for36();
    receive_CAN_data();
    Serial.print(" ");
    Serial.print(2);
    Serial.print(" ");
    Serial.print(-2);
    Serial.print(" ");
    Serial.print(Cur_command_L);
    Serial.print(" ");
    Serial.print(m1.iq_A);
    Serial.print(" ");
    Serial.print(m1.motorAngle);
    Serial.println(" ");
    delay(1);
    previous_time = current_time;
  }

}

void reset_motor_angle()
{
  for (int i = 0; i < 20; i++)
  {
    m1.read_multi_turns_angle();
    delay(10);
    receive_CAN_data();
    m1.motorAngle_offset = m1.motorAngle_raw;
    delay(10);
  }
}

void receive_CAN_data()
{
  //  if (Can3.read(msgR))
  //  {
  Can3.read(msgR);
  if (msgR.id == (ID_offset + Motor_ID1))
  {
    m1.DataExplanation(msgR);
  }
  //  }
}

void initial_CAN()
{
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
  Serial.println("Can bus setup done...");
  delay(200);
}
