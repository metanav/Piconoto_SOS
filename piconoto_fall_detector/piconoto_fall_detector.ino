/* Includes ---------------------------------------------------------------- */
#include <Notecard.h>
#include <Wire.h>
#include <RingBuf.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Fall_Detection_inferencing.h>

#define serialDebugOut Serial
#define I2C_SDA_PIN  2
#define I2C_SCL_PIN  3
#define ATTN_PIN 7
#define EN_PIN  22
#define LED_PIN 25
#define BTN_PIN 28
#define BTN_LONG_PRESS_MS 3000
#define MY_PRODUCT_ID       "com.xxxxx.yyy:your_project_id"
#define FROM_PHONE_NUMBER   "+16xxxxxxxxx"
#define TO_PHONE_NUMBER     "+8xxxxxxxxxxx"
#define N_LOC 5

void btnISR(void);
void attnISR(void);

volatile bool btnInterruptOccurred = false;
volatile bool notecardAttnFired = false;

typedef struct  {
  double lat;
  double lon;
  unsigned long timestamp;
} location_t;

RingBuf<location_t, N_LOC> locations;

// Accelerometer data queue
queue_t sample_queue;
// Init Accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
// Notecard instance
Notecard notecard;

// This buffer is filled by the accelerometer data
float signal_buf[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr)
{
  memcpy(out_ptr, signal_buf + offset, length * sizeof(float));
  return 0;
}

// Interrupt Service Routine for BTN_PIN transitions rising from LOW to HIGH
void btnISR()
{
  btnInterruptOccurred = true;
}

void  attnISR() {
  notecardAttnFired = true;
}

void restore_notecard()
{
  J *req = notecard.newRequest("card.restore");
  if (req) {
    JAddBoolToObject(req, "delete", true);
    if (!notecard.sendRequest(req)) {
      notecard.logDebug("ERROR: restore card request\n");
    }
  } else {
    notecard.logDebug("ERROR: Failed to restore card!\n");
  }
}

void setup_notehub()
{
  // Setup Notehub
  J *req = notecard.newRequest("hub.set");
  if (req) {
    JAddStringToObject(req, "product", MY_PRODUCT_ID);
    JAddBoolToObject(req, "sync", true);
    JAddStringToObject(req, "mode", "periodic");
    JAddNumberToObject(req, "outbound", 15); // 15 mins
    JAddNumberToObject(req, "inbound", 60); // 60 mins
    if (!notecard.sendRequest(req)) {
      notecard.logDebug("ERROR: Setup Notehub request\n");
    }
  } else {
    notecard.logDebug("ERROR: Failed to set notehub!\n");
  }
}

void enable_tracking_notecard()
{
  J *req = NoteNewRequest("card.location.mode");
  if (req) {
    JAddStringToObject(req, "mode", "periodic");
    JAddNumberToObject(req, "seconds", 300);
    if (!notecard.sendRequest(req)) {
      notecard.logDebug("ERROR: card.location.mode request\n");
    }

    J *req = notecard.newRequest("card.location.track");
    if (req) {
      JAddBoolToObject(req, "start", true);
      JAddBoolToObject(req, "sync", true);
      JAddBoolToObject(req, "heartbeat", true);
      JAddNumberToObject(req, "hours", 1);
      if (!notecard.sendRequest(req)) {
        notecard.logDebug("ERROR: card.location.track request\n");
      }

      J *req = NoteNewRequest("card.motion.mode");
      if (req) {
        JAddBoolToObject(req, "start", true);
        if (!notecard.sendRequest(req)) {
          notecard.logDebug("ERROR: card.motion.mode request\n");
        }
      }  else {
        notecard.logDebug("ERROR: Failed to set card motion mode!\n");
      }
    } else {
      notecard.logDebug("ERROR: Failed to set location track!\n");
    }
  } else {
    notecard.logDebug("ERROR: Failed to set  location mode!\n");
  }
}

void register_location()
{
  J *rsp = notecard.requestAndResponse(notecard.newRequest("card.location"));

  if (rsp != NULL) {
    location_t location;
    location.lat = JGetNumber(rsp, "lat");
    location.lon = JGetNumber(rsp, "lon");
    location.timestamp = JGetNumber(rsp, "time");
    
    notecard.deleteResponse(rsp);
    notecard.logDebugf("lat=%f,  lon=%f\n", location.lat, location.lon);
    
    if (locations.isFull()) {
      location_t loc;
      locations.pop(loc);
    }
    
    locations.push(location);
  }
}

void arm_attn()
{
  // Arm ATTN Interrupt
  J *req = NoteNewRequest("card.attn");
  if (req) {
    // arm ATTN if not already armed and fire whenever the Notecard GPS module makes a position fix.
    JAddStringToObject(req, "mode", "rearm,location");
    
   // JAddStringToObject(req, "mode", "sleep");
   // JAddNumberToObject(req, "seconds", 120);

    if (notecard.sendRequest(req)) {
      notecard.logDebug("Arm ATTN interrupt enabled!\n");
    } else {
      notecard.logDebug("ERROR: Failed to arm ATTN interrupt!\n");
    }
  }
}

void send_notification(char *event)
{
  // Add a note
  J *req = notecard.newRequest("note.add");
  if (req != NULL) {
    // send immediately
    JAddBoolToObject(req, "sync", true);
    JAddStringToObject(req, "file", "twilio.qo");
    J *body = JCreateObject();
    if (body != NULL) {
      JAddStringToObject(body, "event", event);
      J *arr = JAddArrayToObject(body, "locations");

      for (uint8_t i = 0; i < locations.size(); i++) {
        J *location = JCreateObject();
        if (location != NULL) {
          JAddNumberToObject(location, "lat", locations[i].lat);
          JAddNumberToObject(location, "lon", locations[i].lon);
          JAddNumberToObject(location, "time", locations[i].timestamp);
          JAddItemToObject(arr, "", location);
        }
      }
      JAddStringToObject(body, "from", FROM_PHONE_NUMBER);
      JAddStringToObject(body, "to", TO_PHONE_NUMBER);
      JAddItemToObject(req, "body", body);
    }

    if (!notecard.sendRequest(req)) {
      notecard.logDebug("ERROR: add note request\n");
    }
  }
}


// Running on core0
void setup()
{
  serialDebugOut.begin(115200);
  pinMode(LED_PIN, OUTPUT);

//  while (!serialDebugOut) {
//    delay(250);
//  }

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  // Wait 2.5 seconds until Notecard is ready
  sleep_ms(10000);

  digitalWrite(LED_PIN, LOW);

  // Notecard I2C SDA/SCL is attached to RPi Pico GPIO 2/3 which uses i2c1/Wire1 instead of default i2c0/Wire
  Wire1.setSDA(I2C_SDA_PIN);
  Wire1.setSCL(I2C_SCL_PIN);
  Wire1.begin();

  // Attach Button Interrupt
  pinMode(BTN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), btnISR, RISING);

  // Attach Notecard Interrupt
  pinMode(ATTN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ATTN_PIN), attnISR, RISING);

  // Initialize Notecard with I2C communication
  notecard.begin(NOTE_I2C_ADDR_DEFAULT, NOTE_I2C_MAX_DEFAULT, Wire1);
  notecard.setDebugOutputStream(serialDebugOut);

  // Restore Notecard 
  //restore_notecard();
  //sleep_ms(100);
  setup_notehub();
  sleep_ms(100);
  // Configure location tracking
  enable_tracking_notecard();
  sleep_ms(100);
  // Arm ATTN
  arm_attn();
  sleep_ms(1000);
}

int sample_start = 0;
int sample_end = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / 4;
int continous_fall_detected = 0;
uint64_t last_fall_detected_time = 0;

// Running on core0
void loop()
{

  if (notecardAttnFired) {
    notecardAttnFired = false;
    notecard.logDebug("ATTN fired\n");
    // Save location data
    register_location();
    // Re-arm ATTN
    arm_attn();
  }

  if (btnInterruptOccurred) {
    btnInterruptOccurred = false;
    unsigned long int start_time = millis();

    while (digitalRead(BTN_PIN) == HIGH) {
      if (millis() - start_time > BTN_LONG_PRESS_MS) {
        send_notification("BUTTON_PRESSED");
        break;
      }
    }
  }


  ei_impulse_result_t result = { 0 };
  signal_t features_signal;
  features_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
  features_signal.get_data = &raw_feature_get_data;

  // get data from the queue
  for (int i = sample_start; i < sample_end; i++) {
    queue_remove_blocking(&sample_queue, &signal_buf[i]);
  }

  if (sample_end ==  EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    sample_start = 0;
    sample_end = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / 4;
  } else {
    sample_start = sample_end;
    sample_end += EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / 4;
  }

  // invoke the impulse
  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
  if (res == 0) {
    // above 80% confidence score
    if (result.classification[1].value > 0.8f) {
      ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("\t%s: %.5f\n", result.classification[ix].label,
                  result.classification[ix].value);
      }

      continous_fall_detected += 1;

      if (continous_fall_detected > 2) {
        send_notification("FALL_DETECTED");
        continous_fall_detected = 0;
        digitalWrite(LED_PIN, HIGH);
      }
    } else {
      continous_fall_detected = 0;

      // turn off the led after 5s since last fall detected
      if (ei_read_timer_ms() - last_fall_detected_time >= 5000) {
        digitalWrite(LED_PIN, LOW);
      }
    }
  }
}

// Running on core1
void setup1()
{
  if (!accel.begin()) {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  accel.setDataRate(ADXL345_DATARATE_400_HZ);

  // add space for 4 additional samples to avoid blocking main thread
  queue_init(&sample_queue, sizeof(float), EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE + (4 * sizeof(float)));

  // wait 10s until core0 finishes setup and starts fetching accelerometer data
  sleep_ms(10000);
}

uint64_t last_sample_time = 0;

// Running on core1
void loop1()
{
  sensors_event_t event;
  accel.getEvent(&event);
  float acceleration[EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME];

  // read sample every 5000 us = 5ms (= 200 Hz)
  if (ei_read_timer_us() - last_sample_time >= 5000) {
    acceleration[0] = event.acceleration.x;
    acceleration[1] = event.acceleration.y;
    acceleration[2] = event.acceleration.z;

    //ei_printf("%.1f, %.1f, %.1f\n", acceleration[0], acceleration[1], acceleration[2]);

    for (int i = 0; i < EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME; i++) {
      if (queue_try_add(&sample_queue, &acceleration[i]) == false) {
        //ei_printf("Data queue full!\n");
        sleep_ms(100);
        break;
      }
    }
    last_sample_time = ei_read_timer_us();
  }
}


void ei_printf(const char *format, ...) {
  static char print_buf[1024] = { 0 };

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0) {
    Serial.write(print_buf);
  }
}
