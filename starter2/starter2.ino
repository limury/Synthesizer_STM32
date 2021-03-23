
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include <STM32FreeRTOS.h>
// #include <stm32l4xx_hal.h>
// #include <stm32l4xx_hal_dac.h>
// #include <stm32l4xx_hal_dma.h>
// #include <stm32l4xx_hal_dma_ex.h>
// #include <stm32l4xx_hal_tim.h>

// DAC_HandleTypeDef hdac1;
// DMA_HandleTypeDef s_DMAHandle;;
// TIM_HandleTypeDef htim2;

// void HAL_MspInit(void){
//     __HAL_RCC_SYSCFG_CLK_ENABLE();
//     __HAL_RCC_PWR_CLK_ENABLE();
// }

#include "pins.h"
// Other constants
const uint32_t STEP_SIZES [] = {/*C*/ 51076057, /*C#*/ 54113197, /*D */ 57330935, 
                               /*D#*/ 60740010, /*E */ 64351798, /*F */ 68178356,
                               /*F#*/ 72232452, /*G */ 76527617, /*G#*/ 81078186, 
                               /*A */ 85899346, /*A#*/ 91007187, /*B */ 96418756};
const char NOTE_NAMES[12][3] = { "C ", "C#", "D ", "D#", "E ", "F ", "F#", "G ", "G#", "A ", "A#", "B "};
const char INT_TO_HEX[] = "0123456789ABCDEF";

const uint64_t INCREMENTS[] = {/*C */ 10215211334, /*C#*/ 10822639409, /*D */ 11466187038, 
                               /*D#*/ 12148001999, /*E */ 12870359789, /*F */ 13635671207, 
                               /*F#*/ 14446490411, /*G */ 15305523433, /*G#*/ 16215637218, 
                               /*A */ 0x400000000, /*A#*/ 18201437366, /*B */ 19283751153};

const uint32_t SINE_WAVE_110_HZ [200] = { 2147483647, 2214937738, 2282325260, 2349579710, 2416634716, 2483424102, 2549881955, 2615942690, 2681541112, 2746612484, 2811092589, 2874917791, 2938025103, 3000352247, 3061837712, 3122420821, 3182041784, 3240641762, 3298162926, 3354548509, 3409742864, 3463691521, 3516341241, 3567640063, 3617537362, 3665983896, 3712931853, 3758334902, 3802148235, 3844328614, 3884834412, 3923625655, 3960664059, 3995913074, 4029337912, 4060905587, 4090584945, 4118346697, 4144163445, 4168009712, 4189861963, 4209698633, 4227500146, 4243248934, 4256929454, 4268528206, 4278033744, 4285436685, 4290729725, 4293907640, 4294967294, 4293907640, 4290729725, 4285436685, 4278033744, 4268528206, 4256929454, 4243248934, 4227500146, 4209698633, 4189861963, 4168009712, 4144163445, 4118346697, 4090584945, 4060905587, 4029337912, 3995913074, 3960664059, 3923625655, 3884834412, 3844328614, 3802148235, 3758334902, 3712931853, 3665983896, 3617537362, 3567640063, 3516341241, 3463691521, 3409742864, 3354548509, 3298162926, 3240641762, 3182041784, 3122420821, 3061837712, 3000352247, 2938025103, 2874917791, 2811092589, 2746612484, 2681541112, 2615942690, 2549881955, 2483424102, 2416634716, 2349579710, 2282325260, 2214937738, 2147483647, 2080029555, 2012642033, 1945387583, 1878332577, 1811543191, 1745085338, 1679024603, 1613426181, 1548354809, 1483874704, 1420049502, 1356942190, 1294615046, 1233129581, 1172546472, 1112925509, 1054325531, 996804367, 940418784, 885224429, 831275772, 778626052, 727327230, 677429931, 628983397, 582035440, 536632391, 492819058, 450638679, 410132881, 371341638, 334303234, 299054219, 265629381, 234061706, 204382348, 176620596, 150803848, 126957581, 105105330, 85268660, 67467147, 51718359, 38037839, 26439087, 16933549, 9530608, 4237568, 1059653, 0, 1059653, 4237568, 9530608, 16933549, 26439087, 38037839, 51718359, 67467147, 85268660, 105105330, 126957581, 150803848, 176620596, 204382348, 234061706, 265629381, 299054219, 334303234, 371341638, 410132881, 450638679, 492819058, 536632391, 582035440, 628983397, 677429931, 727327230, 778626052, 831275772, 885224429, 940418784, 996804367, 1054325531, 1112925509, 1172546472, 1233129581, 1294615046, 1356942190, 1420049502, 1483874704, 1548354809, 1613426181, 1679024603, 1745085338, 1811543191, 1878332577, 1945387583, 2012642033, 2080029555 };
volatile uint64_t increment = 0;
// volatiles
volatile uint8_t knob_3_pos = 0;
volatile uint32_t current_step_size = 0;
volatile SemaphoreHandle_t key_array_mutex;
volatile bool mute = false;

volatile char note_message[] = "xxx";
QueueHandle_t msg_out_q;

// one-hot encoded value
volatile uint32_t pressed_keys = 0;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs via matrix
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

// interrupt to send output sound to the speaker
void sampleISR(){
    static uint64_t buff_ptr = 0;
    buff_ptr += increment;
    while (buff_ptr >= 0xC800000000){ // check if >= 200 <<32
        buff_ptr -= 0xC800000000;
    }
    bool muteL = __atomic_load_n( &mute, __ATOMIC_RELAXED);
    if( !muteL ){
        analogWrite(OUTR_PIN, (SINE_WAVE_110_HZ[ buff_ptr >> 32 ] >> 24) >> (8 - knob_3_pos/4));
    }
    
}

// reads all bits in a certain row of the pin matrix
uint8_t readCols(){
    uint8_t out = (digitalRead(C3_PIN) << 3) + (digitalRead(C2_PIN) << 2) + (digitalRead(C1_PIN) << 1) + (digitalRead(C0_PIN));
    return out; }

// Selects which row to read from
void setRow(const uint8_t& row){
    digitalWrite(REN_PIN, 0);
    static const int pins [] = {RA0_PIN, RA1_PIN, RA2_PIN};
    for (uint8_t i = 0; i < 3; i++)
        digitalWrite( pins[i], (row >> i) & 1 );
    digitalWrite(REN_PIN, 1); }

// scan keys being pressed
void scanKeysTask(void * pvParameters){
    uint8_t knob_3_internal      = 0;
    uint8_t knob_3_last          = 0;
    uint8_t knob_3_change        = 0;
    uint8_t local_knob_3_pos     = 0;
    char local_note_message [] = "xxx";
    uint32_t reading             = 0;
    uint8_t last_note            = -1;

    // setup interval timer
    const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // gets autoupdated by xTaskDelayUntil
    uint8_t pressed_key_index_last = -1;
    uint8_t prev_knob3_press=0;
    while(1){   vTaskDelayUntil( &xLastWakeTime, xFrequency );

        // read rows ----------------------------------------------------------
        setRow(6); delayMicroseconds(3);
        reading = readCols();
        /*
        setRow(2); delayMicroseconds(3);
        reading = (reading << 4) + readCols();
        setRow(1); delayMicroseconds(3);
        reading = (reading << 4) + readCols();
        setRow(0); delayMicroseconds(3);
        reading = (reading << 4) + readCols();
        */
        for(int i = 5; i >=0  ; i--){
            setRow(i); 
            delayMicroseconds(3);
            reading = (reading << 4) + readCols();
        }
        reading = ~reading;
        __atomic_store_n( &pressed_keys, reading, __ATOMIC_RELAXED);
        // reading = onehot-encoded value with active keys = 1  ---------------

        //update mute. knob 3 press is mute and unmute
        uint8_t knob3_press = (reading >> 21) & 0b1;
        if(prev_knob3_press==1){
            if(knob3_press==0){
                bool local_mute = __atomic_load_n( &mute, __ATOMIC_RELAXED);
                __atomic_store_n( &mute, !local_mute, __ATOMIC_RELAXED);
            }
        }
        prev_knob3_press = knob3_press;

        // update knob 3 readings----------------------------------------------
        knob_3_internal = (reading >> 12) & 0b11;
        knob_3_change = decodeKnobChange( (knob_3_last << 2) | knob_3_internal , knob_3_change);
        knob_3_last = knob_3_internal;
        local_knob_3_pos += knob_3_change;
        if (local_knob_3_pos > 136)         local_knob_3_pos = 0;
        else if (local_knob_3_pos > 32)     local_knob_3_pos = 32;
        __atomic_store_n( &knob_3_pos, local_knob_3_pos, __ATOMIC_RELAXED);
        // --------------------------------------------------------------------

        // set pointer increment size and update message-----------------------
        for (uint8_t i = 0; i < 12; i++){
            // if key at index i is pressed
            if ( (reading >> i) & 0b1 ){
                if (i != last_note){
                    increment = INCREMENTS[ i ];
                    last_note = i;
                }
                local_note_message[0] = 'P'; local_note_message[1] = '4'; local_note_message[2] = INT_TO_HEX[ i ];
                goto keySelectDone;
            }
        }
        local_note_message[0] = 'R'; local_note_message[1] = '4'; local_note_message[2] = 'x';
        if ( last_note != -1 )
            increment = 0;
        last_note = -1;
        keySelectDone:;
        // ---------------------------------------------------------------------

        // place readings in global messagge
        xSemaphoreTake(key_array_mutex, portMAX_DELAY);
        memcpy( (uint8_t*) note_message, local_note_message, 3 );
        xSemaphoreGive(key_array_mutex);

        xQueueSend( msg_out_q, (char*) local_note_message, portMAX_DELAY );
    }
}

void displayUpdateTask(void * pvParameters){
    uint32_t local_pressed_keys = 0;
    uint8_t local_knob_3_pos;
    char local_note_message[3];

    // setup interval timer
    const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // gets autoupdated by xTaskDelayUntil
    while(1){   vTaskDelayUntil( &xLastWakeTime, xFrequency );
        // copy key array 
        local_pressed_keys = __atomic_load_n( &pressed_keys, __ATOMIC_RELAXED);
        xSemaphoreTake(key_array_mutex, portMAX_DELAY);
        memcpy( local_note_message, (const uint8_t*) note_message, 3);
        local_knob_3_pos = knob_3_pos;
        xSemaphoreGive(key_array_mutex);

        // operate on display
        u8g2.clearBuffer();         // clear the internal memory
        // read key and print
        // -----------------------------------------------------------------
        for (uint8_t i = 0; i < 12; i++){
            if ( (local_pressed_keys >> i) & 0b1 ){
                u8g2.drawStr(2,30, NOTE_NAMES[ i ]);
                u8g2.setCursor(22,30);
                u8g2.print(i);
                break;
            }
        }

        //display mute
        bool local_mute = __atomic_load_n( &mute, __ATOMIC_RELAXED);
        if(local_mute){
            u8g2.drawStr(20,20,"MUTE");
        }else{
            u8g2.drawStr(20,20,"UNMUTE");
        }

        // -----------------------------------------------------------------

        
        u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
        
        //u8g2.drawStr(2,10,"S");  // write something to the internal memory
        u8g2.setCursor(5, 20);
        u8g2.print(local_knob_3_pos>>1);
        u8g2.setCursor(2,10);
        u8g2.print(local_pressed_keys,BIN);
        u8g2.setCursor(64, 30);
        u8g2.print((char*) note_message);
        u8g2.sendBuffer();          // transfer internal memory to the display

        //Toggle LED
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    }
}

void msgOutTask(void * pvParameters){
    char out_msg[4];
    while(1){ 
        xQueueReceive(msg_out_q, out_msg, portMAX_DELAY);
        Serial.println(out_msg);
    }
}

void serialDecoderTask(void * pvParameters){
    uint8_t it = 0;

    // setup interval timer
    const TickType_t xFrequency = 5/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // gets autoupdated by xTaskDelayUntil
    while(1){   vTaskDelayUntil( &xLastWakeTime, xFrequency );
        char in_msg[] = "xxx";
        while(Serial.available() > 0){
            while (it < 3){
                in_msg[it] = Serial.read();
                it++;
            }
            if ( Serial.read() == '\n' && it >= 3) {
                it = 0;
                break;
            }
        }
        // process input
        if (1 || in_msg[0] == 'P'){
            if (in_msg[2] != 'x'){
                uint8_t note = strtol( in_msg + 1, NULL, 16);
                uint32_t step = STEP_SIZES[ note & 0xF ];
                uint8_t octave = ((note >> 4) & 0xF) - 4;
                step = octave < 0 ? step >> octave : step << octave;
                __atomic_store_n( &current_step_size, step, __ATOMIC_RELAXED );
            }
        }
        else {
            __atomic_store_n( &current_step_size, 0, __ATOMIC_RELAXED);
            break;
        }

    
    }
}

// decodes the state change of the knob into a reading for the value change
inline uint8_t decodeKnobChange(const uint8_t& state, const uint8_t& past_change){
    switch(state){
        case 0: case 5: case 10: case 15:
            return 0;
        case 1: case 7: case 8: case 14:
            return 1;
        case 2: case 4: case 11: case 13:
            return -1;
        default:
            return past_change * 2;
    }
}


void setup() {
    // put your setup code here, to run once:


    //Set pin directions
    pinMode(RA0_PIN, OUTPUT); pinMode(RA1_PIN, OUTPUT); pinMode(RA2_PIN, OUTPUT); pinMode(REN_PIN, OUTPUT); pinMode(OUT_PIN, OUTPUT); pinMode(OUTL_PIN, OUTPUT); pinMode(OUTR_PIN, OUTPUT); pinMode(LED_BUILTIN, OUTPUT);

    pinMode(C0_PIN, INPUT); pinMode(C1_PIN, INPUT); pinMode(C2_PIN, INPUT); pinMode(C3_PIN, INPUT); pinMode(JOYX_PIN, INPUT); pinMode(JOYY_PIN, INPUT);

    //Initialise display
    setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
    delayMicroseconds(2);
    setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
    u8g2.begin();
    setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

    //Initialise UART
    Serial.begin(115200);
    Serial.println("Hello World");

    // my setups
    // interrupt service routines 
    TIM_TypeDef * Instance = TIM1;
    HardwareTimer *sampleTimer = new HardwareTimer(Instance);

    // interrupts
    sampleTimer->setOverflow(22000, HERTZ_FORMAT);
    sampleTimer->attachInterrupt(sampleISR);

    // threads
    TaskHandle_t displayUpdateHandle = NULL;
    xTaskCreate( displayUpdateTask, "displayUpdate", 256, NULL, 1, &displayUpdateHandle );
    
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate( scanKeysTask, "scanKeys", 64, NULL, 2, &scanKeysHandle );

    TaskHandle_t msgOutHandle = NULL;
    xTaskCreate( msgOutTask, "msgOut", 32, NULL, 2, &msgOutHandle );

    TaskHandle_t serialDecoderHandle = NULL;
    xTaskCreate( serialDecoderTask, "serialDecoder", 64, NULL, 3, &serialDecoderHandle );

    // declare mutexes and other structures
    key_array_mutex = xSemaphoreCreateMutex(); // could use xSemaphoreCreateMutexStatic for it to be faster
    msg_out_q = xQueueCreate( 8, 4 );

    // start interrupts and scheduler
    sampleTimer->resume();
    vTaskStartScheduler();
    Serial.println("ERROR: Insufficient RAM");

}


void loop() {
    // put your main code here, to run repeatedly:

    //Update display

}
