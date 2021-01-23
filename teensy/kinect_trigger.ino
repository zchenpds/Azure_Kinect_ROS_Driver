const int SYNC_OUT = 5;
const int RISE_THRESHOLD = 80;
const double MOCAP_FREQ = 210.0;
const double KINECT_FREQ = 30.0;
const int MOCAP_PERIOD_US = 1e6 / MOCAP_FREQ;
const int KINECT_PERIOD_US = 1e6 / KINECT_FREQ;
const int KINECT_TIMEOUT_US = 150;
const int KINECT_TRIGGER_OFFSET_US = 0;
const int EDGE_CNT_TARGET = (int)(MOCAP_FREQ / KINECT_FREQ + 0.5);

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL)


// the setup routine runs once when you press reset:
void setup() {
  pinMode(SYNC_OUT, OUTPUT);
  digitalWrite(SYNC_OUT, LOW);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  while (!Serial.available() || Serial.read() != 'B') delay(1);
}

int capture_trigger_cnt = 0;

void triggerCapture() {
  digitalWrite(SYNC_OUT, HIGH);
  delayMicroseconds(150); // must be larger than 8 us
  digitalWrite(SYNC_OUT, LOW);
  capture_trigger_cnt++;
  
}

// the loop routine runs over and over again forever:
void loop() {
  int val = analogRead(A4);
  int t = micros(); 
  
  static int t_prev_freq = t;
  static int t_prev_rising = t;
  static int t_prev_sync = t;
  static int freq = 0;
  static int val_prev = 0;
  static int edge_cnt = 0;
  static bool trigger_after_offset = false;
  static int next_trigger_time_us = t;
  static int kinect_period_us = KINECT_PERIOD_US;
  
  int delta_val = val - val_prev;
  if (delta_val > RISE_THRESHOLD && t - t_prev_rising > MOCAP_PERIOD_US * 0.9) {
    edge_cnt++;
    freq++;
    t_prev_rising = t;
  }

  if (edge_cnt >= EDGE_CNT_TARGET) {
    kinect_period_us = (t - t_prev_sync) * 0.1 + kinect_period_us * 0.9;
    t_prev_sync = t;
    next_trigger_time_us = t + KINECT_TRIGGER_OFFSET_US;
    trigger_after_offset = true;
    edge_cnt = 0;
  }
  else if (t - t_prev_sync >= KINECT_PERIOD_US + KINECT_TIMEOUT_US) {
    t_prev_sync += kinect_period_us;
    next_trigger_time_us = t_prev_sync + KINECT_TRIGGER_OFFSET_US;
    trigger_after_offset = true;
    edge_cnt = 0;
  }

  if (trigger_after_offset && t >= next_trigger_time_us) {
    trigger_after_offset = false;
    triggerCapture();
  }
  
  if (t - t_prev_freq >= 1e6) {
    Serial.print(1e6 / (t - t_prev_freq) * freq);
    Serial.print(',');
    Serial.print(kinect_period_us);
    Serial.print(',');
    Serial.print(KINECT_PERIOD_US);
    Serial.print(',');
    Serial.println(capture_trigger_cnt);
    t_prev_freq = t;
    freq = 0;
  }
//  if (delta_val > 0) Serial.println(delta_val);

  if (Serial.available() && Serial.read() == 'S') CPU_RESTART; 
  
  val_prev = val;
}
