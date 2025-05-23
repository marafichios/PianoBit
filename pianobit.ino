#define LCD_ADDR (0x27 << 1)

#define LCD_BACKLIGHT 0x08
#define ENABLE        0x04
#define READ_WRITE    0x02
#define REGISTER_SELECT 0x01

const int rowPins[4] = {2, 3, 4, 5};
const int colPins[4] = {6, 7, 8, 9};
const int buzzerPin = 10;
const int dataPin = 11;
const int clockPin = 13;
const int latchPin = 12;

const int tones[16] = {
  262, 294, 330, 349,
  392, 440, 494, 523,
  587, 659, 698, 784,
  880, 988, 1047, 1175
};

const char* noteNames[16] = {
  "Do", "Re", "Mi", "Fa",
  "Sol", "La", "Si", "Do2",
  "Re2", "Mi2", "Fa2", "Sol2",
  "La2", "Si2", "Do3", "Re3"
};

volatile bool keypadChanged = false;

int stableButton = -1;
int lastButtonReading = -1;
unsigned long buttonLastChangeTime = 0;
const unsigned long buttonStableDelay = 50;

int lastNoteIndex = -1;
bool isPlaying = false;

unsigned long lastScanTime = 0;
const unsigned long scanInterval = 50;

void setup() {
  Serial.begin(9600);
  Serial.println("Start program");


  i2c_init();          //init regs
  lcd_init_custom();   //init manual

  lcd_clear_custom();
  lcd_setCursor_custom(0, 0);
  lcd_print_custom("Apasa o nota: ");

  for (int i = 0; i < 4; i++) {
    pinMode(rowPins[i], OUTPUT);
    digitalWrite(rowPins[i], HIGH); //deactivate rows
  }

  for (int i = 0; i < 4; i++) {
    pinMode(colPins[i], INPUT_PULLUP);
  }

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, HIGH);

  //setup timer1
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 3999;
  TCCR1B |= (1 << WGM12); //CTC
  TCCR1B |= (1 << CS11);  //presc 8
  TIMSK1 |= (1 << OCIE1A); //interrupt

  // port d 0-7 si portb 8-13
  PCICR |= (1 << PCIE2) | (1 << PCIE0);

  PCMSK2 |= (1 << 6) | (1 << 7);  // pini 6,7 (PORTD)
  PCMSK0 |= (1 << 0) | (1 << 1);  // pini 8,9 (PORTB)

  sei();

  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);

  clearAllLEDs();
  updateLED(-1);
}


ISR(TIMER1_COMPA_vect) {
  static int activeRow = 0;
  digitalWrite(rowPins[activeRow], HIGH);
  activeRow = (activeRow + 1) % 4;
  digitalWrite(rowPins[activeRow], LOW);
}

ISR(PCINT2_vect) {
  keypadChanged = true;
}

ISR(PCINT0_vect) {
  keypadChanged = true;
}

//so that the lcd cand read the sent data
void lcd_write4bits(uint8_t data) {
  i2c_start(LCD_ADDR);
  i2c_write(data | LCD_BACKLIGHT);
  i2c_write(data | ENABLE | LCD_BACKLIGHT);
  delayMicroseconds(1);
  i2c_write((data & ~ENABLE) | LCD_BACKLIGHT);
  i2c_stop();
}

void lcd_clear_custom() {
  lcd_command(0x01);   // Comanda clear display LCD
  delay(2);
}

void lcd_setCursor_custom(int col, int row) {
  //lcd addresses for each new line
  const uint8_t row_offsets[] = {0x00, 0x40};
  lcd_command(0x80 | (col + row_offsets[row]));
}

//send each character to the lcd
void lcd_print_custom(const char* str) {
  while (*str) {
    lcd_data(*str++);
  }
}


void i2c_init() {
  //presc 1
  TWSR = 0x00;
  //bitrate
  TWBR = 72;
  //activate i2c
  TWCR = (1 << TWEN);
}

uint8_t i2c_start(uint8_t address) {
  //generate start condition
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));

  //send device address
  TWDR = address;

  //start sending
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));

  //check ack
  uint8_t status = TWSR & 0xF8;
  if ((status != 0x18) && (status != 0x40)) {
    return 0;
  }
  return 1;
}

void i2c_stop() {
  //send stop
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

uint8_t i2c_write(uint8_t data) {
  TWDR = data;

  //initiate transmission
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));

  //check ack
  uint8_t status = TWSR & 0xF8;
  return (status == 0x28);
}

//send 2 halves of the specificied byte,
//0 command, 1 data
void lcd_send(uint8_t value, uint8_t mode) {
  uint8_t highNib = value & 0xF0;
  uint8_t lowNib  = (value << 4) & 0xF0;
  
  lcd_write4bits(highNib | mode);
  lcd_write4bits(lowNib | mode);
}

void lcd_init_custom() {
  i2c_init();
  delay(50);
  
  lcd_command(0x33);  //init
  lcd_command(0x32);  //4bit mode
  lcd_command(0x28);  //2 lines, 5x8 font
  lcd_command(0x0C);  //display on, cursor off
  lcd_command(0x06);  //inc cursor
  lcd_command(0x01);  //clear display
  delay(5);
}

//send a command
void lcd_command(uint8_t cmd) {
  lcd_send(cmd, 0);
}

//send a character
void lcd_data(uint8_t data) {
  lcd_send(data, REGISTER_SELECT);
}

void loop() {
  //if pcint interrupt detected a change
  if (keypadChanged) {
    scanKeypad();  //scan piano 
    keypadChanged = false; //reset isr
    lastScanTime = millis();
  }
  //if no ISR, scan periodically
  else if (millis() - lastScanTime > scanInterval) {
    scanKeypad();
    lastScanTime = millis();
  }
}

void scanKeypad() {
  bool buttonPressed = false;
  int pressedRow = -1;
  int pressedCol = -1;

  //scan button matrix
  for (int row = 0; row < 4; row++) {
    for (int r = 0; r < 4; r++) {
      digitalWrite(rowPins[r], (r == row) ? LOW : HIGH);
    }

    delayMicroseconds(100);

    //read portstate with col pins
    uint8_t portDState = PIND;
    uint8_t portBState = PINB;

    //check first 2 cols on portD, pins 6, 7
    for (int col = 0; col < 2; col++) {
      if ((portDState & (1 << (6 + col))) == 0) {
        pressedRow = row;
        pressedCol = col;
        buttonPressed = true;
        break;
      }
    }
    if (buttonPressed) break;

    //check last 2 cols on portB, pins 0, 1
    for (int col = 2; col < 4; col++) {
      if ((portBState & (1 << (col - 2))) == 0) {
        pressedRow = row;
        pressedCol = col;
        buttonPressed = true;
        break;
      }
    }
    if (buttonPressed) break;
  }

  //check pressed button
  int currentButton = buttonPressed ? pressedRow * 4 + pressedCol : -1;

  //check if button is changed than last one
  if (currentButton != lastButtonReading) {
    buttonLastChangeTime = millis(); //reset timer
    lastButtonReading = currentButton;
  }

  //if more than 50ms form last change and button is changed
  if ((millis() - buttonLastChangeTime) > buttonStableDelay) {
    if (currentButton != stableButton) {
      stableButton = currentButton; //confirm new pressed button

      //if button is pressed
      if (stableButton >= 0) {
        if (stableButton != lastNoteIndex || !isPlaying) {
          lastNoteIndex = stableButton;
          isPlaying = true;

          //print message
          lcd_clear_custom();
          lcd_setCursor_custom(0, 0);
          lcd_print_custom("Nota: ");
          lcd_print_custom(noteNames[stableButton]);

          Serial.print("Buton apasat: ");
          Serial.print(stableButton);
          Serial.print(" - Frecventa: ");
          Serial.println(tones[stableButton]);

          //turn on sound and led
          tone(buzzerPin, tones[stableButton]);
          updateLED(stableButton);
        }
      } else {
        if (isPlaying) {

          //stop sound if no button pressed
          noTone(buzzerPin);
          digitalWrite(buzzerPin, HIGH);
          isPlaying = false;
          lastNoteIndex = -1;
          updateLED(-1);

          //reset lcd
          lcd_clear_custom();
          lcd_setCursor_custom(0, 0);
          lcd_print_custom("Apasa o nota: ");
        }
      }
    }
  }
}

void updateLED(int ledIndex) {
  uint16_t ledPattern = 0;
  if (ledIndex >= 0 && ledIndex < 16) {
    ledPattern = (uint16_t)1 << ledIndex;
  }

  //clear interrupts to not distrub the led output
  cli();
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, (ledPattern >> 8) & 0xFF);
  shiftOut(dataPin, clockPin, MSBFIRST, ledPattern & 0xFF);
  digitalWrite(latchPin, HIGH);
  sei();

  Serial.print("LED pattern: ");
  Serial.println(ledPattern, BIN);
}

void clearAllLEDs() {
  cli();
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, 0x00);
  shiftOut(dataPin, clockPin, MSBFIRST, 0x00);
  digitalWrite(latchPin, HIGH);
  sei();
}
