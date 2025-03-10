#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <bitset>

//Constants
  const uint32_t interval = 100; //Display update interval
  const uint32_t stepSizes[12] = {51076057, 54113197, 57330935, 60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346, 91007187, 96418756};  //440Hz

//Global variables
  volatile uint32_t currentStepSize = 0;
  HardwareTimer sampleTimer(TIM1);

  struct {
    std::bitset<32> inputs;  
    SemaphoreHandle_t mutex;  
  } sysState; //store system state

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

//interrupt service routine
void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  int32_t Vout = (phaseAcc >> 24) - 128;

  Vout = (int)(Vout * 0.1); //volume reduction
  analogWrite(OUTR_PIN, Vout + 128);
}

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN,LOW); //disable row select enable before row address pins changed, prevents glitches
  delayMicroseconds(3);

  uint8_t rowCopy = rowIdx; //decimal -> binary conversion below
  if(rowCopy >= 4){ //b100 or higher
    digitalWrite(RA2_PIN, HIGH);
    rowCopy = rowCopy - 4;
  }
  else{
    digitalWrite(RA2_PIN, LOW);
  }
  if(rowCopy >= 2){ //b010 or higher
    digitalWrite(RA1_PIN, HIGH);
    rowCopy = rowCopy - 2;
  }
  else{
    digitalWrite(RA1_PIN, LOW);
  }
  if(rowCopy >= 1){ //b001
    digitalWrite(RA0_PIN, HIGH);
  }
  else{
    digitalWrite(RA0_PIN, LOW);
  }

  digitalWrite(REN_PIN,HIGH); //re-enable row select enable
}

std::bitset<4> readCols(){
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);

  return result;
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;  //50ms thread initiation interval (20Hz)
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    //checking key input
    sysState.inputs = 0;
    for(int i=0; i<3; i++){
      setRow(i); //scanning key loop rows 0-2
      delayMicroseconds(3);
      std::bitset<32> rowInput = readCols().to_ulong(); //conversion to 32 bit
      sysState.inputs |= (rowInput  << (i*4)); 
    }

    //key input -> step size conversion
    int localCurrentStepSize = 0;
    for(int i=0; i<12; i++){
      if(sysState.inputs[i] == 0){  //inputs are active low
        localCurrentStepSize = stepSizes[i];
      } 
    }
    xSemaphoreGive(sysState.mutex);
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);  //single access to global variable currentStepSize
  }
}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;  //100ms thread initiation interval (10Hz)
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  while(1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency);
    //static uint32_t count = 0;
  
    //update display
    std::string noteStr;
    switch(__atomic_load_n(&currentStepSize, __ATOMIC_RELAXED)) {
      case 51076057:
        noteStr = "C";
        break;
      case 54113197:
        noteStr = "C#/Db";
        break;
      case 57330935:
        noteStr = "D";
        break;
      case 60740010:
        noteStr = "D#/Eb";
        break;
      case 64351799:
        noteStr = "E";
        break;
      case 68178356:
        noteStr = "F";
        break;
      case 72232452:
        noteStr = "F#/Gb";
        break;
      case 76527617:
        noteStr = "G";
        break;
      case 81078186:
        noteStr = "G#/Ab";
        break;
      case 85899346:
        noteStr = "A";
        break;
      case 91007187:
        noteStr = "A#/Bb";
        break;
      case 96418756:
        noteStr = "B";
        break;
      default:
        noteStr = "No notes pressed!";
    }
  
    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
    u8g2.setCursor(2,20);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY); //take sysState mutex
    u8g2.print(sysState.inputs.to_ulong(),HEX); 
    xSemaphoreGive(sysState.mutex);
    u8g2.drawStr(2,30, noteStr.c_str());
    //u8g2.print(count++);
    u8g2.sendBuffer();          // transfer internal memory to the display
  
    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void setup() {
  //setup hardware timer for 22kHz sample rate
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR); //sawtooth waveform generation
  sampleTimer.resume();

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  //start key input thread (scanKeysTask())
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  2,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,
  "displayUpdate",
  256,
  NULL,
  1,  //lower priority than key handling
  &displayUpdateHandle );
  
  //mutex handle for system state
  sysState.mutex = xSemaphoreCreateMutex();

  //start real time operating system (RTOS) scheduler
  vTaskStartScheduler();
}

void loop() {

}