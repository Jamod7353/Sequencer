#include <Arduino.h>
#include <LedControl.h>
#include <Bounce2.h>
#include "config.h"
#include "patterns.h"


// play and pick contol
#define AVG_LENGTH 8
byte sequenceLength = 15;
unsigned int seqLengthAVG[AVG_LENGTH];
byte seqLengthAVG_pointer = 0;
byte playPointer = 15;
byte pickPointer = 0;
byte pickPointer_old = 0;
byte sequencePointer = 0;
boolean patternMode = false;

unsigned int pickSequence = patterns[4];
unsigned int randomSequence;

unsigned int sequence0 = patterns[2];
unsigned int sequence1 = patterns[1];
unsigned int sequence2 = patterns[0];

// buttons
#define RESET 0
#define PATTERN_MODE 1
#define PICK_DOT 2
#define CLK_IN 3
#define RESET_CLK 4

#define NUM_OF_BUTTONS 5

Bounce * b = new Bounce[NUM_OF_BUTTONS];


// Matrix
LedControl lc = LedControl(PIN_LED_DIN, PIN_LED_CLK, PIN_LED_CS, 0);
byte matrix[8];
boolean sequenceBlinker = false;
byte pickBlinker = 0;
boolean pickBlinker2 = false;
byte animationCounter = 0;
byte ANIMATION_NUM = 3;
byte pickCounter = 0;
byte PICK_COUNTER_NUM = 50;

// clock mode
#define POTI_MODE 0
#define CLK_MODE 1
byte timer_mode = POTI_MODE;
#define CLK_OFF 0
#define CLK_LISTEN 1
#define CLK_RESET 2
#define CLK_RUN 3
byte clk_state = 0;
boolean clk_blocked = false;

// bpm and timer
unsigned int bpm = 310;
unsigned int bpmAVG[AVG_LENGTH];
byte bpmAVG_pointer = 0;
unsigned int bpm_old = 119;
unsigned int countsToInterrupt = 0;
unsigned int countsToRelease = 65536 - (62.5*MS_TO_RELEASE); // care for changes in timer-settings!
long lastBeat = 0;
byte divider = 1; 
unsigned int dividerAVG[AVG_LENGTH];
byte dividerAVG_pointer = 0;

boolean flagInterrupt = true;

void generateRandomPattern(){
  randomSequence = random(0, 65535);
}

unsigned int avg(unsigned int array[]){
  // save some values in an array and calculate avarage to flatten values
  unsigned int sum = 0;
  for(int i=0; i<AVG_LENGTH; i++){
    sum += array[i];
  }
  double value = sum * 1.0 / AVG_LENGTH;
  return round(value);
}

void updateBPM_poti(){
  bpmAVG[bpmAVG_pointer] = (unsigned int) map(analogRead(PIN_BPM_POTI), 0, 1023, MIN_BPM, MAX_BPM);
  bpmAVG_pointer = ++bpmAVG_pointer % AVG_LENGTH;
  bpm = avg(bpmAVG);

  if(abs(bpm_old-bpm)>1){
    // Timer-interrupt in ms: 65535-(16*10^6*60/bpm/256)
    float tmp = (65536 - 3750000.0/bpm);
    countsToInterrupt = (int) tmp;
    bpm_old = bpm;
  }
  clk_state = CLK_OFF;
}

// Input signal interrupt method
void update_BPM_CLK(){
  if(timer_mode == CLK_MODE && !clk_blocked){ 
    if(b[CLK_IN].read() == HIGH){
      if(clk_state == CLK_OFF){
        // 
        clk_state++;
      } else if (clk_state == CLK_LISTEN){
        //
        clk_state++;
      } else { // state == run
        long beat_time = millis();
        long diff = beat_time - lastBeat;
        lastBeat = beat_time;

        diff = max(diff, 60000/MAX_BPM);
        diff = min(diff, 60000/MIN_BPM);

        // 65536-(16*10^6* diff /256)
        countsToInterrupt = 65536 - (62.5*diff/divider);
        // TODO: divider testen

        bpm = (int) (60000/diff);

        if(clk_state == CLK_RESET){ // reset for sync (only 1 time)  // TODO: eventuell immer beim 1er reset machen!
          TCNT1 = countsToRelease;
          flagInterrupt = false;
          clk_state++;
        }
      }
    }
  }
}

void updatePick(){

    if(patternMode){ // pick pattern
      //int arraysize = sizeof(patterns);
      int value = map(analogRead(PIN_PICK), 0, 1023, 0, sizeof(patterns)/2);
      // TODO auch Wert glätten
      
      if(value == sizeof(patterns)/2){
        pickSequence = randomSequence;
      } else {
        pickSequence = patterns[value];
      }
     
    } else { // pick dot
      int value = map(analogRead(PIN_PICK), 0, 1023, 0, 47);
      pickPointer = value % 16;
      sequencePointer = value / 16;
        // counter for animation
      if(pickPointer != pickPointer_old){
      pickCounter = 0;
    }
  }
}

void setDot(){
  if(patternMode){
    // switch pattern
    if(sequencePointer == 0){
      sequence0 = pickSequence;
    } else if(sequencePointer == 1){
      sequence1 = pickSequence;
    } else {
      sequence2 = pickSequence;
    }
  } else {
    // switch value on pointer position
    if(sequencePointer == 0){
      sequence0 ^= 32768 >> pickPointer;
    } else if (sequencePointer == 1){
      sequence1 ^= 32768 >> pickPointer;
    } else {
      sequence2 ^= 32768 >> pickPointer;
    }
  }
}

void updateControls(){
  for(int i=0; i<NUM_OF_BUTTONS; i++){
    b[i].update();
  }
  if(b[PICK_DOT].fell()){
    setDot();
  }
  if(b[RESET].read() == HIGH){
    if(clk_state == 4){
      clk_state = 3;
    }
  }
  if(b[RESET_CLK].fell()){
    clk_state = 0;
  }
  //sequenceLength = map(analogRead(PIN_SEQUENCE_LENGTH), 0, 1023, 1, 16);
  seqLengthAVG[seqLengthAVG_pointer] = (unsigned int) map(analogRead(PIN_SEQUENCE_LENGTH), 0, 1023, 1, 16);
  seqLengthAVG_pointer = ++seqLengthAVG_pointer % AVG_LENGTH;
  sequenceLength = avg(seqLengthAVG);
  // TODO: flag (alter/neuer Wert) -> Anzeigen (Zwischenpunkte) oder Matrix + timer(millis) setzen

  //divider = map(analogRead(PIN_DIVIDER), 0, 1023, 1, 8);
  dividerAVG[dividerAVG_pointer] = (unsigned int) map(analogRead(PIN_DIVIDER), 0, 1023, 1, 8);
  dividerAVG_pointer = ++dividerAVG_pointer % AVG_LENGTH;
  divider = avg(dividerAVG);
  // TODO: flag (alter/neuer Wert) -> Anzeigen (Zwischenpunkte) oder Matrix

  patternMode = b[PATTERN_MODE].read() == LOW;
  if(b[PATTERN_MODE].fell()){
    generateRandomPattern();
  }

  // TODO: Switch- 16 to 32 mode (combine seq 1 and 2)
}

void buildMatrix(){
  matrix[0] = (byte) (sequence0 >> 8);
  matrix[1] = (byte) sequence0;
  matrix[2] = 0;
  matrix[3] = (byte) (sequence1 >> 8);
  matrix[4] = (byte) sequence1;
  matrix[5] = 0;
  matrix[6] = (byte) (sequence2 >> 8);
  matrix[7] = (byte) sequence2;
}

void printMatrix(){
  for(int i=0; i<8; i++){
    lc.setRow(0, i, matrix[i]);
  }
}

void animateMatrix(){
  // TODO: animate settings (if-else oder zwischenpunkte)

  // animate sequence
  matrix[playPointer / 8]     ^= (128*sequenceBlinker) >> (playPointer % 8);
  matrix[playPointer / 8 + 3] ^= (128*sequenceBlinker) >> (playPointer % 8);
  matrix[playPointer / 8 + 6] ^= (128*sequenceBlinker) >> (playPointer % 8);
  sequenceBlinker ^= 1;

  // aminate pattern pick
  if(patternMode){
    matrix[3*sequencePointer]   = (byte) (pickSequence >> 8);
    matrix[3*sequencePointer+1] = (byte) pickSequence;

    pickCounter = 0; //reset animation
  }
  // animate dot pick
  else {
    if(pickBlinker2){
      if(pickCounter++ < PICK_COUNTER_NUM){
        matrix[pickPointer/8 + sequencePointer*3] ^= (pickBlinker*128)>>(pickPointer%8);
        pickBlinker ^= 1;
        pickBlinker2 = 0;
      }
    } else {
      pickBlinker2 = 1;
    }
  }
}


void setup() {
  Serial.begin(9600);
  Serial.println("Start...let's go!");

  // setup digital pins
  pinMode(PIN_TIMER_MODE, INPUT);
  pinMode(PIN_OUT1, OUTPUT);
  pinMode(PIN_OUT2, OUTPUT);
  pinMode(PIN_OUT3, OUTPUT);
  pinMode(PIN_LED_DIN, OUTPUT);
  pinMode(PIN_LED_CLK, OUTPUT);
  pinMode(PIN_LED_CS, OUTPUT);
  pinMode(PIN_CLK_OUT, OUTPUT);
  pinMode(PIN_32MODE, INPUT); //TODO: Pullup?

  b[RESET_CLK].attach(PIN_RESET_CLK, INPUT_PULLUP);
  b[CLK_IN].attach(PIN_CLK_IN, INPUT);
  b[RESET].attach(PIN_RESET, INPUT);
  b[PATTERN_MODE].attach(PIN_PATTERN, INPUT_PULLUP);
  b[PICK_DOT].attach(PIN_PICK_DOT, INPUT_PULLUP);
  for(int i=0; i<NUM_OF_BUTTONS; i++){
    b[i].interval(25);
    Serial.println(i);
  }

  for(int i=0; i<AVG_LENGTH; i++){
    bpmAVG[i] = map(analogRead(PIN_BPM_POTI), 0, 1023, MIN_BPM, MAX_BPM);
    dividerAVG[i] = map(analogRead(PIN_DIVIDER), 0, 1023, 1, 8);
    seqLengthAVG[i] = map(analogRead(PIN_SEQUENCE_LENGTH), 0, 1023, 1, 16);
  }

  // setup LCD-Matrix
  lc.shutdown(0, false);
  lc.setIntensity(0,1);
  lc.clearDisplay(0);

  Serial.println("Matrix setup done");

  timer_mode = digitalRead(PIN_TIMER_MODE);

  // set interrupt for clk
  attachInterrupt(digitalPinToInterrupt(PIN_CLK_IN), update_BPM_CLK, CHANGE);
  // set Timer for bpm
  if(timer_mode == POTI_MODE){
    updateBPM_poti();
  }
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = countsToRelease;
  TCCR1B |= (1 << CS12);  //prescale 256
  TIMSK1 |= (1 << TOIE1);
  interrupts();

  Serial.println("Timer setup done");
}

void loop() {
  timer_mode = digitalRead(PIN_TIMER_MODE);
  if(timer_mode == POTI_MODE){
    updateBPM_poti();
  } // bpm in CLK_Mode is set at interrupts
  
  updatePick();
  updateControls();

  buildMatrix();
  animateMatrix();
  printMatrix();

  Serial.print(bpm);
  Serial.print(" :bpm - length: ");
  Serial.print(sequenceLength);
  Serial.print(" - divider: ");
  Serial.println(divider);

  delay(10);
}

// Timer-interrupt method
ISR(TIMER1_OVF_vect){
  if(!flagInterrupt){
    TCNT1 = countsToRelease;
    flagInterrupt = true;

    playPointer = (playPointer+1) % sequenceLength;
    
    // trigger clk and signal if in sequence
    digitalWrite(PIN_CLK_OUT, HIGH);
    if(sequence0 & (32768>>playPointer)){digitalWrite(PIN_OUT1, HIGH);}
    // TODO: 32Mode
    if(sequence1 & (32768>>playPointer)){digitalWrite(PIN_OUT2, HIGH);}
    if(sequence2 & (32768>>playPointer)){digitalWrite(PIN_OUT3, HIGH);}
  } else {
      flagInterrupt = false;
      TCNT1 = countsToInterrupt + countsToRelease;

      // end trigger signal
      digitalWrite(PIN_CLK_OUT, LOW);
      digitalWrite(PIN_OUT1, LOW);
      digitalWrite(PIN_OUT2, LOW);
      digitalWrite(PIN_OUT3, LOW);
  }
}