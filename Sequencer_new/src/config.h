// Led Matrix 8x8
#define PIN_LED_DIN 10
#define PIN_LED_CS 9
#define PIN_LED_CLK 8

// Audio out
#define PIN_CLK_OUT 13       // Jack
#define PIN_OUT1 5          // Jack
#define PIN_OUT2 6          // Jack
#define PIN_OUT3 7          // Jack

// Control Inputs
#define PIN_BPM_POTI A0         // Poti
#define PIN_SEQUENCE_LENGTH A1  // Poti
#define PIN_DIVIDER A3          // Poti
#define PIN_PICK A5             // Poti

#define PIN_32MODE A4        // Switch - Pullup
#define PIN_CLK_IN 2        // Jack - Widerstand 100k->GND
#define PIN_PATTERN 3       // Button - Pullup
#define PIN_TIMER_MODE 4    // Switch
#define PIN_PICK_DOT 11     // Button - Pullup
#define PIN_SYNC 12        // Button & Bus
#define PIN_RESET_CLK 1    // Button - Pullup

// Values
int MIN_BPM = 58;
int MAX_BPM = 570;
int MS_TO_RELEASE = 3;



// Hardware
// Switch -> clk bus input or output