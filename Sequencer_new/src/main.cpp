#include <Arduino.h>
#include <LedControl.h>
#include <Bounce2.h>
#include "config.h"
#include "patterns.h"

// play and pick contol
#define AVG_LENGTH 12
byte sequenceLength[4] = {16, 16, 16, 16};
int seqLengthAVG[AVG_LENGTH];
byte seqLengthAVG_pointer = 0;
byte playPointer[3] = {15, 15, 15};
byte pickPointer = 0;
int pickPointerAVG[AVG_LENGTH];
byte pickPointerAVG_pointer = 0;
byte sequencePointer = 0;
boolean patternMode = false;
byte mode32_counter = 0;
boolean mode32 = false;

int pickSequence = patterns[4];
int randomSequence;

int sequence0 = patterns[2];
int sequence1 = patterns[1];
int sequence2 = patterns[3];

// buttons
#define SYNC 0
#define PATTERN_MODE 1
#define PICK_DOT 2
#define PICK_SEQUENCE 3 

#define NUM_OF_BUTTONS 4

Bounce *b = new Bounce[NUM_OF_BUTTONS];


// Matrix
LedControl lc = LedControl(PIN_LED_DIN, PIN_LED_CLK, PIN_LED_CS, 0);
byte matrix[8];
boolean sequenceBlinker = false;
byte pickBlinker = 0;
boolean pickBlinker2 = false;
byte animationCounter = 0;
byte ANIMATION_NUM = 3;
#define NORMAL_ANIMATION 0
#define BPM_ANIMATION 1
#define SEQ_LENGTH_ANIMATION 2
#define DIVIDER_ANIMATION 3
#define SEQ_PICK_ANIMATION 4
byte flagInfo = 0;
long infoTime;

// clock mode
#define POTI_MODE 0
#define CLK_MODE 1
byte timer_mode = POTI_MODE;
boolean clk_blocked = false;

// bpm and timer
int bpm = 310;
int bpmAVG[AVG_LENGTH];
byte bpmAVG_pointer = 0;
int bpm_old = 119;
int countsToInterrupt = 0;
int countsToRelease =(int) (62.5*MS_TO_RELEASE);
int diffToRelease = 65536 - countsToRelease; // care for changes in timer-settings!
long lastBeat = 0;
byte divider = 1; 
byte divider_counter = 0;
int dividerAVG[AVG_LENGTH];
byte dividerAVG_pointer = 0;
long divider_time = 0;
long divider_next_step = 0;
long divider_diff;

boolean flagInterrupt = true;

void generateRandomPattern(){
  randomSequence = random(0, 65535);
}

int avg(int array[]){
  // save some values in an array and calculate avarage to flatten values
  int sum = 0;
  for(int i=0; i<AVG_LENGTH; i++){
    sum += array[i];
  }
  double value = sum * 1.0 / AVG_LENGTH;
  return round(value);
}

void updateBPM_poti(){
  bpmAVG[bpmAVG_pointer] = (int) map(analogRead(PIN_BPM_POTI), 0, 1023, MIN_BPM, MAX_BPM);
  bpmAVG_pointer = ++bpmAVG_pointer % AVG_LENGTH;
  int bpmRedAVG = avg(bpmAVG);
  if((bpm>bpmRedAVG && bpm-bpmRedAVG > 2) || (bpm<bpmRedAVG && bpmRedAVG-bpm > 2)){
  //if(abs(bpm - avg(bpmAVG)) > 3){
    infoTime = (long) (millis() + ANIMATION_TIME);
    flagInfo = BPM_ANIMATION;
  }
  bpm = bpmRedAVG;

  if(abs(bpm_old-bpm)>1){
    // Timer-interrupt in ms: 65535-(16*10^6*60/bpm/256)
    float tmp = (65536 - 3750000.0/bpm);
    countsToInterrupt = (int) tmp;
    bpm_old = bpm;
  }
}

void trigger(){
  TCNT1 = diffToRelease;
  flagInterrupt = true;
  if(timer_mode == POTI_MODE || !clk_blocked){
    for(int i=0; i<3; i++){
      playPointer[i] = (playPointer[i]+1) % sequenceLength[i];
    }  
  if(playPointer == 0){
    mode32_counter = (mode32_counter+1) % 2;
  }
    // trigger clk and signal if in sequence
    digitalWrite(PIN_CLK_OUT, HIGH);
    if(sequence0 & (32768>>playPointer[0])){digitalWrite(PIN_OUT1, HIGH);}

    // in mode32 play seq1 and seq2 on both outputs
    if(mode32){
      if(mode32_counter == 0){
        if(sequence1 & (32768>>playPointer[1])){digitalWrite(PIN_OUT2, HIGH);}
        if(sequence1 & (32768>>playPointer[1])){digitalWrite(PIN_OUT3, HIGH);}
      } else {
        if(sequence2 & (32768>>playPointer[2])){digitalWrite(PIN_OUT2, HIGH);}
        if(sequence2 & (32768>>playPointer[2])){digitalWrite(PIN_OUT3, HIGH);}
      }
    } else {
      if(sequence1 & (32768>>playPointer[1])){digitalWrite(PIN_OUT2, HIGH);}
      if(sequence2 & (32768>>playPointer[2])){digitalWrite(PIN_OUT3, HIGH);}
    }
  }
}

// Input signal interrupt method
void update_BPM_CLK(){
  if(timer_mode == CLK_MODE){
    if(digitalRead(PIN_CLK_IN)){
      if(divider < 6){
        countsToInterrupt = 0;
        flagInterrupt = false;
        clk_blocked = false;
        if(divider_counter == 0){
          trigger();
        }
        clk_blocked = true;
        divider_counter = (byte) ((divider_counter+1) % divider);
      } else { // divider >= 6
        long actTime = (long) millis();
        clk_blocked = false;
        trigger();
        clk_blocked = true;
        byte mult = (byte) (divider - 4);
        divider_diff = (long) ((actTime - divider_time) / mult);
        divider_next_step = actTime + divider_diff;
        divider_time = actTime;
        divider_counter = (byte) (mult - 1);
      }
    }
  }
}

void triggerMult(){
  if(divider >= 6 && timer_mode == CLK_MODE && divider_counter > 0){
    Serial.println(divider_counter);
    long actTime = (long) millis();
    if(actTime >= divider_next_step){
      clk_blocked = false;
      trigger();
      clk_blocked = true;
      divider_counter = (byte) (divider_counter - 1);
      divider_next_step = (long) (divider_next_step + divider_diff);
    }
  }
}

void updatePick(){
  if(patternMode){ // pick pattern
    
    int value = map(analogRead(PIN_PICK), 0, 1023, 0, sizeof(patterns)/2);
    
    if(value == sizeof(patterns)/2){
      pickSequence = randomSequence;
    } else {
      pickSequence = patterns[value];
    }
    
  } else { // pick dot
    pickPointerAVG[pickPointerAVG_pointer] = (int) map(analogRead(PIN_PICK), 0, 1023, 0, 15);
    pickPointerAVG_pointer = ++pickPointerAVG_pointer % AVG_LENGTH;
    pickPointer = avg(pickPointerAVG);
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
  timer_mode = digitalRead(PIN_TIMER_MODE)^1;
  if(timer_mode == POTI_MODE){
    updateBPM_poti();
  } // bpm in CLK_Mode is set at interrupts

  for(int i=0; i<NUM_OF_BUTTONS; i++){
    b[i].update();
  }
  if(b[PICK_DOT].fell()){
    setDot();
  }
  if(b[SYNC].fell()){
    for(int i=0; i<3; i++){
      playPointer[i] = 0;
    }
  }
  if(b[PICK_SEQUENCE].fell()){
    sequencePointer = ++sequencePointer % 4;
    infoTime = (long) (millis() + ANIMATION_TIME);
    flagInfo = SEQ_PICK_ANIMATION; 
  }
  seqLengthAVG[seqLengthAVG_pointer] = (int) map(analogRead(PIN_SEQUENCE_LENGTH), 0, 1023, 1, 16);
  seqLengthAVG_pointer = ++seqLengthAVG_pointer % AVG_LENGTH;
  byte tmp_seqLen = (byte) avg(seqLengthAVG);

  if(sequenceLength[sequencePointer] != tmp_seqLen){
    infoTime = (long) (millis() + ANIMATION_TIME);
    flagInfo = SEQ_LENGTH_ANIMATION;
    if(sequencePointer == 3){ // if all seq selected -> change all
      for(int i=0; i<4; i++){
        sequenceLength[i] = tmp_seqLen;
      }
    } else { // change only selected
      sequenceLength[sequencePointer] = tmp_seqLen;
    }
  }

  dividerAVG[dividerAVG_pointer] = (int) map(analogRead(PIN_DIVIDER), 0, 1023, 1, 8);
  dividerAVG_pointer = ++dividerAVG_pointer % AVG_LENGTH;
  byte tmp_divider = (byte) avg(dividerAVG); 
  if(divider != tmp_divider){
    infoTime = (long) (millis() + ANIMATION_TIME);
    flagInfo = DIVIDER_ANIMATION;
  }  
  divider = (byte) tmp_divider;

  patternMode = b[PATTERN_MODE].read() == LOW;

  if(b[PATTERN_MODE].fell()){
    generateRandomPattern();
  }

  mode32 = digitalRead(PIN_32MODE);
}

void buildMatrix(){
  if(flagInfo == NORMAL_ANIMATION){
    int number = 0;
    matrix[7] = (byte) (sequence0 >> 8);
    matrix[6] = (byte) sequence0;
    matrix[5] = 0;
    matrix[4] = (byte) (sequence1 >> 8);
    matrix[3] = (byte) sequence1;
    matrix[2] = 0;
    matrix[1] = (byte) (sequence2 >> 8);
    matrix[0] = (byte) sequence2;

  } else {
    int number;
    byte divider_operator = 0;
    if(flagInfo == BPM_ANIMATION){
      number = bpm;
    } else if(flagInfo == DIVIDER_ANIMATION){
      if(divider == 1){
        number = divider;
      }
      else if(divider <= 5){
        number = divider;
        divider_operator = 1;
      } else {
        number = divider-4;
        divider_operator = 2;
      }
    } else if(flagInfo == SEQ_LENGTH_ANIMATION){
      number = sequenceLength[sequencePointer];
    } else if(flagInfo == SEQ_PICK_ANIMATION){
      if(sequencePointer == 3){
        for(int i=0; i<8; i++){
          matrix[i] = 255;
        }
        matrix[2] = 0;
        matrix[5] = 0;
      } else {
        for(int i=0; i<8; i++){
          matrix[i] = 0;
        }
        matrix[7-sequencePointer*3] = 255;
        matrix[6-sequencePointer*3] = 255;
      }
      return;
    }

    for(int i=0; i<8; i++){
      matrix[i] = 0;
    }
    int hundreds = number / 100;
    switch (hundreds) {
      case 6:
        matrix[2] = 128;
      case 5:
        matrix[3] = 128;
      case 4:
        matrix[4] = 128;
      case 3:
        matrix[5] = 128;
      case 2:
        matrix[6] = 128;
      case 1:
        matrix[7] = 128;
        break;
      default:
        matrix[7] = 0;
        break;
    }
    int tens = (number % 100) / 10; 
    if(flagInfo != DIVIDER_ANIMATION){
      switch (tens) {
        case 0:
          matrix[6] = 32+16+8+4+2 | matrix[6];
          matrix[5] = 32+2 | matrix[5];
          matrix[4] = 32+16+8+4+2 | matrix[4];
          break;
        case 1:
          matrix[6] = 8 | matrix[6];
          matrix[5] = 16 | matrix[4];
          matrix[4] = 32+16+8+4+2 | matrix[4];
          break;
        case 2:
          matrix[6] = 32+8+4+2 | matrix[6];
          matrix[5] = 32+8+2 | matrix[5];
          matrix[4] = 32+16+8+2 | matrix[4];
          break;
        case 3:
          matrix[6] = 32+8+2 | matrix[6];
          matrix[5] = 32+8+2 | matrix[5];
          matrix[4] = 32+16+8+4+2 | matrix[4];
          break;
        case 4:
          matrix[6] = 32+16+8 | matrix[6];
          matrix[5] = 8 | matrix[5];
          matrix[4] = 32+16+8+4+2 | matrix[4];
          break;
        case 5:
          matrix[6] = 32+16+8+2 | matrix[6];
          matrix[5] = 32+8+2 | matrix[5];
          matrix[4] = 32+8+4+2 | matrix[4];
          break;
        case 6:
          matrix[6] = 32+16+8+4+2 | matrix[6];
          matrix[5] = 32+8+2 | matrix[5];
          matrix[4] = 32+8+4+2 | matrix[4];
          break;
        case 7:
          matrix[6] = 32 | matrix[6];
          matrix[5] = 32 | matrix[4];
          matrix[4] = 32+16+8+4+2 | matrix[4];
        case 8:
          matrix[6] = 32+16+8+4+2 | matrix[6];
          matrix[5] = 32+8+2 | matrix[5];
          matrix[4] = 32+16+8+4+2 | matrix[4];
        case 9:
          matrix[6] = 32+16+8+2 | matrix[6];
          matrix[5] = 32+8+2 | matrix[5];
          matrix[4] = 32+16+8+4+2 | matrix[4];
        default:
          break;
      }
    } else {
      if(divider_operator == 1){
        matrix[4] = 16+4 | matrix[4];
      } else if(divider_operator == 2){
        matrix[4] = 8 | matrix[4];
      }
    }

    int num = (number % 10); 

    switch (num) {
      case 0:
        matrix[2] = 32+16+8+4+2 | matrix[2];
        matrix[1] = 32+2 | matrix[1];
        matrix[0] = 32+16+8+4+2 | matrix[0];
        break;
      case 1:
        matrix[2] = 8 | matrix[2];
        matrix[1] = 16 | matrix[1];
        matrix[0] = 32+16+8+4+2 | matrix[0];
        break;
      case 2:
        matrix[2] = 32+8+4+2 | matrix[2];
        matrix[1] = 32+8+2 | matrix[1];
        matrix[0] = 32+16+8+2 | matrix[0];
        break;
      case 3:
        matrix[2] = 32+8+2 | matrix[2];
        matrix[1] = 32+8+2 | matrix[1];
        matrix[0] = 32+16+8+4+2 | matrix[0];
        break;
      case 4:
        matrix[2] = 32+16+8 | matrix[2];
        matrix[1] = 8 | matrix[1];
        matrix[0] = 32+16+8+4+2 | matrix[0];
        break;
      case 5:
        matrix[2] = 32+16+8+2 | matrix[2];
        matrix[1] = 32+8+2 | matrix[1];
        matrix[0] = 32+8+4+2 | matrix[0];
        break;
      case 6:
        matrix[2] = 32+16+8+4+2 | matrix[2];
        matrix[1] = 32+8+2 | matrix[1];
        matrix[0] = 32+8+4+2 | matrix[0];
        break;
      case 7:
        matrix[2] = 32 | matrix[2];
        matrix[1] = 32 | matrix[1];
        matrix[0] = 32+16+8+4+2 | matrix[0];
      case 8:
        matrix[2] = 32+16+8+4+2 | matrix[2];
        matrix[1] = 32+8+2 | matrix[1];
        matrix[0] = 32+16+8+4+2 | matrix[0];
      case 9:
        matrix[2] = 32+16+8+2 | matrix[2];
        matrix[1] = 32+8+2 | matrix[1];
        matrix[0] = 32+16+8+4+2 | matrix[0];
      default:
        break;
    }
  } 
}

void printMatrix(){
  for(int i=0; i<8; i++){
    lc.setRow(0, i, matrix[i]);
  }
}

void animateMatrix(){
  if(flagInfo == NORMAL_ANIMATION){
    // animate sequence
    matrix[7-(playPointer[0] / 8)] ^= (128*sequenceBlinker) >> (playPointer[0] % 8);
    
    if(mode32){
      if(mode32_counter == 0){
        matrix[7-(playPointer[1] / 8 + 3)] ^= (128*sequenceBlinker) >> (playPointer[1] % 8);
      } else {
        matrix[7-(playPointer[2] / 8 + 6)] ^= (128*sequenceBlinker) >> (playPointer[2] % 8);
      }
    } else {
      matrix[7-(playPointer[1] / 8 + 3)] ^= (128*sequenceBlinker) >> (playPointer[1] % 8);
      matrix[7-(playPointer[2] / 8 + 6)] ^= (128*sequenceBlinker) >> (playPointer[2] % 8);
    }
    sequenceBlinker ^= 1;

    // aminate pattern pick
    if(patternMode){
      matrix[7-3*sequencePointer]   = (byte) (pickSequence >> 8);
      matrix[7-(3*sequencePointer+1)] = (byte) pickSequence;
    }
    // animate dot pick
    else {
      if(pickBlinker2){
        matrix[7-(pickPointer/8 + sequencePointer*3)] ^= (byte) ((pickBlinker*128)>>(pickPointer%8));
        pickBlinker ^= 1;
        pickBlinker2 = 0;
      } else {
        pickBlinker2 = 1;
      }
    }
  } else if(infoTime < millis()){
    flagInfo = NORMAL_ANIMATION;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Start...let's go!");

  // setup digital pins
  pinMode(PIN_TIMER_MODE, INPUT_PULLUP);
  pinMode(PIN_CLK_IN, INPUT);
  pinMode(PIN_OUT1, OUTPUT);
  pinMode(PIN_OUT2, OUTPUT);
  pinMode(PIN_OUT3, OUTPUT);
  pinMode(PIN_LED_DIN, OUTPUT);
  pinMode(PIN_LED_CLK, OUTPUT);
  pinMode(PIN_LED_CS, OUTPUT);
  pinMode(PIN_CLK_OUT, OUTPUT);
  pinMode(PIN_32MODE, INPUT_PULLUP);

  b[PICK_SEQUENCE].attach(PIN_PICK_SEQUENCE, INPUT_PULLUP);
  b[SYNC].attach(PIN_SYNC, INPUT_PULLUP);
  b[PATTERN_MODE].attach(PIN_PATTERN, INPUT_PULLUP);
  b[PICK_DOT].attach(PIN_PICK_DOT, INPUT_PULLUP);
  for(int i=0; i<NUM_OF_BUTTONS; i++){
    b[i].interval(25);
  }

  for(int i=0; i<AVG_LENGTH; i++){
    bpmAVG[i] = (int) map(analogRead(PIN_BPM_POTI), 0, 1023, MIN_BPM, MAX_BPM);
    dividerAVG[i] = (int) map(analogRead(PIN_DIVIDER), 0, 1023, 1, 8);
    seqLengthAVG[i] = (int) map(analogRead(PIN_SEQUENCE_LENGTH), 0, 1023, 1, 16);
    pickPointerAVG[i] = (int) map(analogRead(PIN_PICK), 0, 1023, 0, 47);
  }

  // setup LCD-Matrix
  lc.shutdown(0, false);
  lc.setIntensity(0,1);
  lc.clearDisplay(0);

  Serial.println("Matrix setup done");

  timer_mode = digitalRead(PIN_TIMER_MODE)^1;

  // set interrupt for clk
  attachInterrupt(digitalPinToInterrupt(PIN_CLK_IN), update_BPM_CLK, CHANGE);
  // set Timer for bpm
  if(timer_mode == POTI_MODE){
    updateBPM_poti();
  }
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1B |= (1 << CS12);  //prescale 256
  TIMSK1 |= (1 << TOIE1);
  interrupts();

  Serial.println("Timer setup done");
}

void loop() {
  updatePick();
  updateControls();

  buildMatrix();
  animateMatrix();
  printMatrix();

  triggerMult();

  delay(10);
}

// Timer-interrupt method
ISR(TIMER1_OVF_vect){
  if(!flagInterrupt){
    trigger();
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