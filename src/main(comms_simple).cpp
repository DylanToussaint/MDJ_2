#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <bitset>
#include <ES_CAN.h>

//Constants
  const uint32_t interval = 100; //Display update interval
  const uint32_t stepSizes[12] = {51076057, 54113197, 57330935, 60740010, 64351799, 68178356, 72232452, 76527617, 81078186, 85899346, 91007187, 96418756};  //440Hz

//Global variables
  volatile uint32_t currentStepSize = 0;
  QueueHandle_t msgInQ; //receive queue handle
  QueueHandle_t msgOutQ; //transmit queue handle
  SemaphoreHandle_t CAN_TX_Semaphore; //transmit semaphore handle
  HardwareTimer sampleTimer(TIM1);  //sampleISR timer

  struct {
    std::bitset<32> inputs;  
    int knob3Rot = 0;
    uint8_t RX_Message[8]={0};
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

//sample interrupt service routine
void sampleISR (void) { 
  static uint32_t phaseAcc = 0;
  phaseAcc += __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  int32_t Vout = (phaseAcc >> 24) - 128; //set sample offset to 0
  Vout = Vout >> (8 - __atomic_load_n(&sysState.knob3Rot, __ATOMIC_RELAXED));  //volume control 
  analogWrite(OUTR_PIN, Vout + 128);  //TODO: if volume = 0 Set Vout to 128
}

//CANbus receive message ISR
void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);  //place message in queue
}

//CANbus transmit message ISR
void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);  //release transmit mailbox semaphore
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
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;  //20ms thread initiation interval (20Hz)
  TickType_t xLastWakeTime = xTaskGetTickCount();
  std::bitset<2> knob3State = 0; 
  int knob3Legal = 0;
  std::bitset<32> sysStatePrev = 0;  

  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    //checking key input
    std::bitset<32> sysStateTemp = 0;  
    for(int i=0; i<4; i++){
      setRow(i); //scanning key loop rows 0-3
      delayMicroseconds(3);
      std::bitset<32> rowInput = readCols().to_ulong(); //conversion to 32 bit
      sysStateTemp |= (rowInput  << (i*4)); 
    }

    //key input -> step size conversion
    uint32_t localCurrentStepSize;
    for(int i=0; i<12; i++){
      if(sysStatePrev[i] != sysStateTemp[i]){
        if(sysStateTemp[i] == 0){  //key inputs are active low
          localCurrentStepSize = stepSizes[i];
        } 
        else{ //key released
          localCurrentStepSize = 0;
        }
        __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);  //single access to global variable currentStepSize
      }
    }

    //key input -> CANbus TX message conversion
    uint8_t TX_Message[8] = {0};
    for (int i=0; i<12; i++){
      if(sysStatePrev[i] != sysStateTemp[i]){
        if(sysStateTemp[i] == 0) { //key pressed
          TX_Message[0] = 'P';
        }
        else {  //key released
          TX_Message[0] = 'R';
        }
        TX_Message[1] = 4;  //octave number
        TX_Message[2] = i;  //note number

        xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
      }
    }
    sysStatePrev = sysStateTemp;  //update previous system state for notes

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    int knob3RotTemp = sysState.knob3Rot;
    xSemaphoreGive(sysState.mutex);

    //knob 3 A & B -> rotation variable conversion
    std::bitset<2> knob3StateCurrent = std::bitset<2> ( (sysStateTemp >> 12).to_ulong() );
    //state transition table
    if((knob3State==0b00) && (knob3StateCurrent==0b01)){
      knob3RotTemp++;
      knob3Legal = 1;
    }
    else if ((knob3State==0b01) && (knob3StateCurrent==0b00)){
      knob3RotTemp--;
      knob3Legal = -1;
    }
    else if ((knob3State==0b10) && (knob3StateCurrent==0b11)){
      knob3RotTemp--;
      knob3Legal = -1;
    }
    else if ((knob3State==0b11) && (knob3StateCurrent==0b10)){
      knob3RotTemp++;
      knob3Legal = 1;
    }
    else if (((knob3State==0b00) && (knob3StateCurrent==0b11)) || 
    ((knob3State==0b01) && (knob3StateCurrent==0b10)) || 
    ((knob3State==0b10) && (knob3StateCurrent==0b01)) || 
    ((knob3State==0b11) && (knob3StateCurrent==0b00))) {  //impossible state transition handling
      knob3RotTemp = knob3RotTemp + knob3Legal;
    }

    if (knob3RotTemp > 8){ //bound knob 3 rotation variable
      knob3RotTemp = 8;
    } 
    else if (knob3RotTemp < 0){
      knob3RotTemp = 0;
    }
    knob3State = knob3StateCurrent; //update previous knob state
    
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    //Single operation to copy sysStateTemp to sysState global variable
    sysState.inputs = sysStateTemp;
    sysState.knob3Rot = knob3RotTemp;
    xSemaphoreGive(sysState.mutex);
  }
}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;  //100ms thread initiation interval (10Hz)
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t localRX_Message[8]={0};
  
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
        noteStr = "None!";
    }

    //Update display
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    //u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
    xSemaphoreTake(sysState.mutex, portMAX_DELAY); //take sysState mutex
    u8g2.setCursor(2,10);
    u8g2.print(sysState.inputs.to_ulong(),HEX); //display system state
    u8g2.setCursor(2,20);
    u8g2.print(sysState.knob3Rot,HEX);  //display knob 3 rotation variable
    xSemaphoreGive(sysState.mutex);
    u8g2.drawStr(2,30, noteStr.c_str());  //display current note played

    u8g2.setCursor(66,30);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    memcpy(localRX_Message, &sysState.RX_Message, sizeof(localRX_Message)); 
    xSemaphoreGive(sysState.mutex);
    u8g2.print((char) localRX_Message[0]); //display CANbus TX message
    u8g2.print(localRX_Message[1]);
    u8g2.print(localRX_Message[2]);
    
    //u8g2.print(count++);
    u8g2.sendBuffer();          // transfer internal memory to the display
  
    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void decodeTask(void * pvParameters) {
  uint8_t localRX_Message[8]={0};

  while(1) {  //loop until all RX messages in queue cleared
    xQueueReceive(msgInQ, localRX_Message, portMAX_DELAY);

    uint32_t localCurrentStepSize = 0;  //default 0 step size for invalid messages
    if (localRX_Message[0] == 'P'){ //key pressed
      uint8_t note = localRX_Message[2];
      if((note<0) || (note>11)){
        note = 0; //default C for invalid note
      }
      uint8_t octave = localRX_Message[1];
      if(octave > 4){
        localCurrentStepSize = (stepSizes[note] << (octave - 4));
      }
      else if ((octave > 0) && (octave < 4)){
        localCurrentStepSize = (stepSizes[note] >> (4 - octave));
      } 
      else { //default no multiply for invalid octave
        localCurrentStepSize = stepSizes[note];
      }
    }
    else if (localRX_Message[0] == 'R'){  //key released
      localCurrentStepSize = 0;
    }

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    memcpy(sysState.RX_Message, &localRX_Message, sizeof(localRX_Message)); 
    xSemaphoreGive(sysState.mutex);
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED); 
  }
}

void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];

	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);  //block until message to transmit available

		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);  //block until CAN mailbox available
		CAN_TX(0x123, msgOut);
	}
}

void setup() {
  //setup hardware timer for 22kHz sample rate
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR); //sawtooth waveform generation
  sampleTimer.resume();

  //initialise queues 
  msgInQ = xQueueCreate(36,8); //receive (queue length, CAN frame size)
  msgOutQ = xQueueCreate(36,8); //transmit

  //initalise mutex for system state
  sysState.mutex = xSemaphoreCreateMutex();
  //intialise semaphore for transmit 
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

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

  //Initialise  CANbus
  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR); //CANbus receive message ISR
  CAN_RegisterTX_ISR(CAN_TX_ISR); //CANbus transmit message ISR
  CAN_Start();

  //start key input thread (scanKeysTask())
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		/* Function that implements the task */
  "scanKeys",		/* Text name for the task */
  64,      		/* Stack size in words, not bytes */
  NULL,			/* Parameter passed into the task */
  4,			/* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,
  "displayUpdate",
  256,
  NULL,
  1,  //lowest priority UI display 
  &displayUpdateHandle );

  TaskHandle_t decodeTaskHandle = NULL;
  xTaskCreate(
  decodeTask,
  "Decode_RX",
  256,
  NULL,
  3,  //lower priority than key handling
  &decodeTaskHandle );

  TaskHandle_t CAN_TX_TaskHandle = NULL;
  xTaskCreate(
  CAN_TX_Task,
  "CAN_TX",
  256,
  NULL,
  2,  //lower priority than decode task
  &CAN_TX_TaskHandle );

  //start real time operating system (RTOS) scheduler
  vTaskStartScheduler();
}

void loop() {

}