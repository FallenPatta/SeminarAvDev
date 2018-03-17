#include <ESP8266WiFi.h>
#include <Arduino.h>
#include "scheduling.h"

#define TICKTIMESSIZE 10

//Alle Abmessungen in cm
#define WHEEL_DIAMETER 7.1f
#define WHEEL_RADIUS WHEEL_DIAMETER/2.0f
//#define WHEEL_CIRC  WHEEL_DIAMETER * PI
#define WHEEL_CIRC  21.4f
#define TICK_LEN  WHEEL_CIRC/96.0f

#define ROBOT_WIDTH 13.5f
#define ROBOT_WIDTH_2 ROBOT_WIDTH/2.0f

#define TICK_TRUST 0.5f

#define RINGBUFFERSIZE 1024

//ESP.getFreeHeap() // freier Arbeitsspeicher ##########>>> NICHT löschen, das brauchst du wieder! <<<########## 

Scheduler task_scheduler;
bool lrTaskStarted = false;

IPAddress localip(192, 168, 178, 55);
IPAddress gateway(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);
int port = 5000;

WiFiServer server(port);

bool serial_output_enabled = false;

//Status Variables
int left_ticks = 0;
int left_direction = 1;
long left_ticks_times[TICKTIMESSIZE];
int left_times_index = 0;

int right_ticks = 0;
int right_direction = 1;
long right_ticks_times[TICKTIMESSIZE];
int right_times_index = 0;

float wheel_speeds[2];


//Control Variables
int tick_difference = 0;
float integral_tickcontrol = 0;

long stoptime = 0;
long stoptime2 = 0;
bool driving_status = true;

int motor_speeds[2];
int motor_setSpeeds[2];

//Predeclared Functions:
int motorCommandToSpeed();
int motorControlTask();

void tick_left_ISR() {
  long tickTime = millis();
  if(tickTime != left_ticks_times[(left_times_index-1)%TICKTIMESSIZE] && tickTime != left_ticks_times[(left_times_index-1)%TICKTIMESSIZE]+1){
    left_ticks += left_direction;
    left_ticks_times[left_times_index] = tickTime;
    left_times_index = (left_times_index + 1) % TICKTIMESSIZE;
  }
}

void tick_right_ISR() {
  long tickTime = millis();
  if(tickTime != right_ticks_times[(right_times_index-1)%TICKTIMESSIZE] && tickTime != right_ticks_times[(right_times_index-1)%TICKTIMESSIZE]+1){
    right_ticks += right_direction;
    right_ticks_times[right_times_index] = tickTime;
    right_times_index = (right_times_index + 1) % TICKTIMESSIZE;
  }
}

int clamp(int a, int minval, int maxval){
  if(a<minval) return minval;
  if(a>maxval) return maxval;
  return a;
}

float signedMax(float val1, float val2){
  if(val1 < 0){
    if(val2<val1) return val2;
    return val1;
  }
  if(val2>val1) return val2;
  return val1;
}

int sign(int num){
  return num>=0?1:-1;
}

float sign(float num){
  return num>=0?1:-1;
}

void setMotor(char m, int value) {
  unsigned int abs_value = abs(value);
  switch (m) {
    case 'A':
    case 'a':
      if (value > 0)
      {
        left_direction = 1;
        digitalWrite(D3, HIGH);
      }
      else if (value < 0)
      {
        left_direction = -1;
        digitalWrite(D3, LOW);
      }
      else
      {

      }
      analogWrite(D1, abs_value);
      motor_speeds[0] = value;
      break;
    case 'B':
    case 'b':
      if (value > 0)
      {
        right_direction = 1;
        digitalWrite(D4, LOW);
      }
      else if (value < 0)
      {
        right_direction = -1;
        digitalWrite(D4, HIGH);
      }
      else
      {

      }
      analogWrite(D2, abs_value);
      motor_speeds[1] = value;
      break;
    default:
      break;
  }
}

void setMotors(int val1, int val2) {
  setMotor('A', val1);
  setMotor('B', val2);
}

//Ab hier: Tasks
WiFiClient * working_client = new WiFiClient();
float kurvenradius = 30.0f;
float geschwindigkeit = 10.0f;
bool forward = true;

uint8_t * data_ring_buffer = new uint8_t[RINGBUFFERSIZE]();
int ring_buffer_write_position = 0;
int ring_buffer_read_position = 0;

void clearBuffer(){
  ring_buffer_write_position = 0;
  ring_buffer_read_position = 0;
}

int bufferFramePosition(){
  int start_index = -1;
  int end_index = -1;
  for(int i = ring_buffer_read_position; i!=ring_buffer_write_position; i=(i+1)%RINGBUFFERSIZE){
    if((char)data_ring_buffer[i] == '{'){
      start_index = i;
    }
    if((char)data_ring_buffer[i] == '}'){
      end_index = i;
      if(start_index >= 0){
        return start_index;
      }
    }
  }
  return -1;
}

bool bufferGetFrame(uint8_t ** frame, int ** frame_size){
  int start_index = bufferFramePosition();
  if(start_index >= 0){
    ring_buffer_read_position = start_index%RINGBUFFERSIZE;
  } else {
    return false;
  }
  int end_index = -1;
  for(int i = ring_buffer_read_position; i!=ring_buffer_write_position; i=(i+1)%RINGBUFFERSIZE){
    if((char)data_ring_buffer[i] == '}'){
      end_index = i;
      break;
    }
  }
  int frame_len = end_index-start_index+1;
  if(frame_len < 0){
    frame_len = RINGBUFFERSIZE - start_index + end_index +1;
  }
  *frame = (uint8_t*) malloc((frame_len)*sizeof(uint8_t));
  *frame_size = (int*) malloc(sizeof(int));
  **frame_size = frame_len;
  int write_pos = 0;
  for(int i = start_index; i!=(end_index+1)%RINGBUFFERSIZE; i=(i+1)%RINGBUFFERSIZE){
    (*frame)[write_pos] = data_ring_buffer[i];
    write_pos++;
  }
  ring_buffer_read_position = (end_index+1)%RINGBUFFERSIZE;
  return true;
}

bool writeToBuffer(uint8_t* input, int input_size){
  int pos = 0;
  while(pos < input_size){
    data_ring_buffer[ring_buffer_write_position] = input[pos];
    if(ring_buffer_write_position == ring_buffer_read_position){
      ring_buffer_read_position = (ring_buffer_read_position+1)%RINGBUFFERSIZE;
    }
    ring_buffer_write_position = (ring_buffer_write_position+1)%RINGBUFFERSIZE;
    pos += 1;
  }
  if(ring_buffer_write_position == ring_buffer_read_position){
    ring_buffer_read_position += 1;
    ring_buffer_read_position = ring_buffer_read_position%RINGBUFFERSIZE;
  }
}

void frameToInstructions(uint8_t * frame, int frame_size){
  String * instructions = new String[2]();
  instructions[0] = String("");
  instructions[1] = String("");
  int pos = 0;
  while(frame[pos] != '{' && pos < frame_size){
    pos+=1;
  }
  pos+=1;

  while(frame[pos] != ',' && pos < frame_size){
    instructions[0] += (char)frame[pos];
    pos+=1;
  }
  pos+=1;
  while(frame[pos] != '}' && pos < frame_size){
    instructions[1] += (char)frame[pos];
    pos+=1;
  }
  if(!lrTaskStarted && instructions[1].indexOf("LR:") >= 0){
      String motorctrlTaskName = "motor-ctrl";
      task_scheduler.deleteTask(motorctrlTaskName);
      task_scheduler.addFunction(&motorCommandToSpeed, motorctrlTaskName, 150, millis(), 10);
      lrTaskStarted = true;
      delete [] instructions;
      geschwindigkeit = 0.0;
      kurvenradius = 0.0;
      return;
  }
  else if(lrTaskStarted && instructions[1].indexOf("Rad:") >= 0){
      String motorctrlTaskName = "motor-ctrl";
      task_scheduler.deleteTask(motorctrlTaskName);
      task_scheduler.addFunction(&motorControlTask, motorctrlTaskName, 150, millis(), 10);
      lrTaskStarted = false;
      delete [] instructions;
      geschwindigkeit = 0.0;
      kurvenradius = 400.0;
      return;
  }
  instructions[0] = instructions[0].substring(4);
  geschwindigkeit = instructions[0].toFloat();
  if(lrTaskStarted){
    instructions[1] = instructions[1].substring(3);
  } else {
    instructions[1] = instructions[1].substring(4);
  }
  kurvenradius = instructions[1].toFloat();
  if(!serial_output_enabled){
    if(Serial.available() > 0){
      serial_output_enabled = true;
    }
  }
  if(serial_output_enabled){
    Serial.print("Geschwindigkeit: ");
    Serial.print(instructions[0]);
    Serial.print(", Radius: ");
    Serial.print(instructions[1]);
    Serial.println("\n");
  }
  delete [] instructions;
}

//Feedback von den Motoren
int calcSpeeds(){
  double dT = 30.0;
  double dT_sec = (dT)/1000.0;
  static int last_ticks_l = 0;
  static int last_ticks_r = 0;
  float dTicks_l = (left_ticks - last_ticks_l) * sign(motor_setSpeeds[0]);
  float dTicks_r = (right_ticks - last_ticks_r) * sign(motor_setSpeeds[1]);
  wheel_speeds[0] = (1.0f-TICK_TRUST) * wheel_speeds[0] 
                    + TICK_TRUST * (TICK_LEN*dTicks_l/dT_sec) * sign(motor_setSpeeds[0]);
  last_ticks_l = left_ticks;
  if(fabs(wheel_speeds[0]) < 0.1) wheel_speeds[0] = 0;
    
  wheel_speeds[1] = (1.0f-TICK_TRUST) * wheel_speeds[1] 
                    + TICK_TRUST * (TICK_LEN*dTicks_r/dT_sec) * sign(motor_setSpeeds[1]);
  last_ticks_r = right_ticks;
  if(fabs(wheel_speeds[1]) < 0.1) wheel_speeds[1] = 0;

  return 0;
}

//Regelung
void speedControl(float dT, float vSoll, float rSoll, int* output) {
  static float integral_part_l = 0;
  static float integral_part_r = 0;
  static float last_speed_l = 0;
  static float last_speed_r = 0;

  float k_p = 20.0; //20.0
  float k_d = 5.0;  //0.0
  float k_i = 2.0;  //1.0

  bool switch_sides = rSoll < 0;

  float radius = fabs(rSoll);
  
  float rSollMax = radius+ROBOT_WIDTH_2;
  float rSollMin = radius-ROBOT_WIDTH_2;
  float vInner = vSoll * (rSollMin / radius);
  float vAusser = vSoll * (rSollMax / radius);

  if(abs(rSoll) - ROBOT_WIDTH_2 <= 0.1 ){
    vInner = 0;
    vAusser = vSoll;
  }
  
  if(abs(rSoll) >= 300){
    vInner = vSoll;
    vAusser = vSoll;
  }
  
  float vLinks = vAusser;
  float vRechts = vInner;
  
  if(switch_sides){
    vLinks = vInner;
    vRechts = vAusser;
  }

  float error_l = vLinks - wheel_speeds[0];
  float dError_l = wheel_speeds[0] - last_speed_l;
  float error_r = vRechts - wheel_speeds[1];
  float dError_r = wheel_speeds[1] - last_speed_r;

  float pVal_l = vLinks * k_p;
  float dVal_l = dError_l * k_d;
  integral_part_l += signedMax(error_l, sign(error_l)) * k_i;
  last_speed_l = wheel_speeds[0];

  
  float pVal_r = vRechts * k_p;
  float dVal_r = dError_r * k_d;
  integral_part_r += signedMax(error_r, sign(error_r)) * k_i;
  last_speed_r = wheel_speeds[1];

  if(vSoll == 0){
    integral_part_r *= 0.5;
    integral_part_l *= 0.5;
  }
  
  output[0] = (int)(pVal_l - dVal_l + integral_part_l);
  output[1] = (int)(pVal_r - dVal_r + integral_part_r);
}

int motorCommandToSpeed() {

    float kurvenfaktor = 1.0f;

    motor_setSpeeds[0] = 0;
    motor_setSpeeds[1] = 0;
    motor_setSpeeds[0] += kurvenradius * 300.0f * kurvenfaktor;
    motor_setSpeeds[1] -= kurvenradius * 300.0f * kurvenfaktor;
    motor_setSpeeds[0] += geschwindigkeit * 650.0f;
    motor_setSpeeds[1] += geschwindigkeit * 650.0f;

    motor_setSpeeds[0] = clamp(motor_setSpeeds[0], -1024, 1024);
    motor_setSpeeds[1] = clamp(motor_setSpeeds[1], -1024, 1024);

    if(geschwindigkeit == 0 && kurvenradius == 0){
      motor_setSpeeds[0] = 0;
      motor_setSpeeds[1] = 0;
    }
    setMotors(motor_setSpeeds[0], motor_setSpeeds[1]);
    return 0;

}

int comReadoutTask(){
  if(working_client && working_client->connected()){
    uint8_t * frame = NULL;
    int * frame_size = NULL;
    if(bufferGetFrame(&frame, &frame_size)){
      frameToInstructions(frame, *frame_size);
      if (frame) free(frame);
      if (frame_size) free(frame_size);
    }
  }
  else{
    if(!working_client || !working_client->connected()){
      kurvenradius = 0;
      geschwindigkeit = 0;
      setMotors(0,0);
    }
  }
  return 0;
}

int comTask(){
  if(!working_client || !working_client->connected()){
    *working_client = server.available();
  }
  if(working_client->connected()){
    if(working_client->available()){
      int available_bytes = working_client->available();
      uint8_t * raw = (uint8_t*)malloc((available_bytes)*sizeof(uint8_t));
      working_client->read(raw, available_bytes);
      writeToBuffer(raw, available_bytes);
      free(raw);
      working_client->println(".");
    }
  }
  return 0;
}

int motorControlTask(){
    double dT = 150.0f;
    double dT_sec = (dT)/1000.0;
  
    static int * cVal = new int[2];
    static int * cVal_last = new int[2];
    cVal_last[0] = cVal[0];
    cVal_last[1] = cVal[1];
    speedControl(dT_sec, geschwindigkeit, kurvenradius, cVal);

    //Restrict acceleration
    int * d_cVal = new int[2];
    d_cVal[0] = cVal[0] - cVal_last[0];
    d_cVal[1] = cVal[1] - cVal_last[1];
    
    cVal[0] = cVal_last[0] + d_cVal[0];
    cVal[1] = cVal_last[1] + d_cVal[1];
    motor_setSpeeds[0] = clamp(cVal[0], -1024, 1024); //motor_setSpeeds[0]+
    motor_setSpeeds[1] = clamp(cVal[1], -1024, 1024); //motor_setSpeeds[1]+
    delete [] d_cVal;

    if(geschwindigkeit == 0){
      motor_setSpeeds[0] = 0;
      motor_setSpeeds[1] = 0;
    }

    //During Start
    if(fabs(wheel_speeds[0]) < 0.1 && fabs(motor_setSpeeds[0]) > 110){
      motor_setSpeeds[0] = sign(motor_setSpeeds[0]) * 400;
    }
    if(fabs(wheel_speeds[1]) < 0.1 && fabs(motor_setSpeeds[1]) > 110){
      motor_setSpeeds[1] = sign(motor_setSpeeds[1]) * 400;
    }
    
    setMotors(motor_setSpeeds[0], motor_setSpeeds[1]);
    return 0;
}

//Ab hier: WIFI
void wifiSetup() {
  Serial.println("Setting up WiFI");
  WiFi.mode(WIFI_STA);  
  WiFi.config(localip, gateway, subnet);
  WiFi.begin("Ultron", "Zu120%SICHERundMITsicherMEINichSICHER");
  delay(1000);

  Serial.println("Connected.");
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("IP:  ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS: ");
  Serial.println(WiFi.dnsIP());
  Serial.print("Channel: ");
  Serial.println(WiFi.channel());
  Serial.print("Status: ");
  Serial.println(WiFi.status());
  server.begin();
}

int eichungNegativ(){
  static int leftStart = 0;
  static int rightStart = 0;
  static bool geeicht_l = false;
  static bool geeicht_r = false;
  
  if(!geeicht_l && fabs(wheel_speeds[0]) < 1.0)
  {
    setMotor('A', leftStart);
    leftStart--;
  } else if (!geeicht_l) 
  {
    setMotor('A', 0);
    geeicht_l = true;
    Serial.print("Links: ");
    Serial.println(leftStart, DEC);
  }

   if(!geeicht_r && fabs(wheel_speeds[1]) < 1.0){
    setMotor('B', rightStart);
    rightStart--;
  } else if (!geeicht_r)
  {
    setMotor('B', 0);
    geeicht_r = true;
    Serial.print("Rechts: ");
    Serial.println(rightStart, DEC);
  }

  if(geeicht_r && geeicht_l){
    String eichungTaskName = "eichung";
    task_scheduler.deleteTask(eichungTaskName);
    
    String motorctrlTaskName = "motor-ctrl";
    task_scheduler.addFunction(&motorControlTask, motorctrlTaskName, 150, millis(), 10);
  }
  
  return 0;
}

int eichungPositiv(){
  static int leftStart = 0;
  static int rightStart = 0;
  static bool geeicht_l = false;
  static bool geeicht_r = false;
  
  if(!geeicht_l && fabs(wheel_speeds[0]) < 1.0)
  {
    setMotor('A', leftStart);
    leftStart++;
  } else if (!geeicht_l) 
  {
    setMotor('A', 0);
    geeicht_l = true;
    Serial.print("Links: ");
    Serial.println(leftStart, DEC);
  }

   if(!geeicht_r && fabs(wheel_speeds[1]) < 1.0){
    setMotor('B', rightStart);
    rightStart++;
  } else if (!geeicht_r)
  {
    setMotor('B', 0);
    geeicht_r = true;
    Serial.print("Rechts: ");
    Serial.println(rightStart, DEC);
  }

  if(geeicht_r && geeicht_l){
    String eichungTaskName = "eichung";
    task_scheduler.deleteTask(eichungTaskName);
    
    task_scheduler.addFunction(&eichungNegativ, eichungTaskName, 50, millis()+500, 100);
  }
  
  return 0;
}

//Ab hier: Setup und main Loop

void setup() {
  //PIN SETUP
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D3, HIGH);
  digitalWrite(D4, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  attachInterrupt(D5, tick_left_ISR, CHANGE);
  attachInterrupt(D6, tick_right_ISR, CHANGE);

  //COM SETUP
  Serial.begin(115200);

  //DEFAULTS EINSTELLEN
  setMotors(0, 0);
  motor_speeds[0] = 0;
  motor_speeds[1] = 0;
  motor_setSpeeds[0] = 0;
  motor_setSpeeds[1] = 0;
  
  //WIFI
  wifiSetup();

  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

//  int r = 300;
//  setMotor('B', r);
//  delay(500);
//  calcSpeeds();
//  r = 100;
//  while(r < 1023){
//    calcSpeeds();
//    setMotor('B', r);
//    r++;
//    Serial.println(wheel_speeds[1], DEC);
//    Serial.println(r, DEC);
//    delay(30);
//  }
//  setMotor('B', 0);
//  delay(2000);

  //TASKS SETUP
  String comTaskName = "communication";
  task_scheduler.addFunction(&comTask, comTaskName, 10, millis(), 5);
  
  String comReadoutTaskName = "comReadout";
  task_scheduler.addFunction(&comReadoutTask, comReadoutTaskName, 5, millis(), 7);

  String speedUpdateTaskName = "motor-speed";
  task_scheduler.addFunction(&calcSpeeds, speedUpdateTaskName, 30, millis(), 50);

  //String eichungTaskName = "eichung";
  //task_scheduler.addFunction(&eichungPositiv, eichungTaskName, 50, millis()+500, 100);
  
  String motorctrlTaskName = "motor-ctrl";
  task_scheduler.addFunction(&motorControlTask, motorctrlTaskName, 150, millis(), 10);
}

void loop() {
  task_scheduler.execute();
}
