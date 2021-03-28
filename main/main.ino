#include <Arduino.h> 
#include "pins.h"

#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include <STM32FreeRTOS.h>
#include "src/main.h"
//#include <CircularBuffer.h>

//#include <stm32l4xx_hal.h>



//tmp lib
//#include <stdint.h>

using namespace std;

#define SOUND false
#define DMA_REQUEST_DAC1_CH1 6U
#define BUFFER_SIZE 500
#define HALF_BUFFER_SIZE BUFFER_SIZE/2 


//classes 
//typedef unsigned int uint32_t;
///fixed sized buffer
#define buffMaxSize 1000
class buffer{

private:
  int maxSize = buffMaxSize;
  uint32_t data[buffMaxSize];
  uint32_t saved_data[buffMaxSize];
  int saved_data_size = 0;
  uint32_t currentPointer =0;
  bool empty = true;

public:
  
  
  ///push data in. If successfully stored then return true otherwise if full then dont store and return false.
  bool push(uint32_t value){
    if(currentPointer  < maxSize  ){
      currentPointer++;
      data[currentPointer] = value;
      if(empty){
        empty = false;
      }
      return true;
    }else{
      return false;
    }
  };

  ///returns data from head. If empty then return 0.
  uint32_t pop(){
    if(!empty){
      if(currentPointer == 0){
        empty = true;
        return data[currentPointer];
      }else{
        currentPointer--;
        return data[currentPointer + 1];
      }
    }else{
      return 0;
    }
  }

  
  bool isEmpty(){
    return empty;
  }

  uint32_t sizeOfbuffer(){
    return currentPointer+1;
  }

  uint32_t sizeOfSaved(){
    return saved_data_size;
  }

  uint32_t readBuffer(uint32_t i){
    return data[i];
  }

  uint32_t read_saved_Buffer(uint32_t i){
    return saved_data[i];
  }

  void save(){
    memcpy ( &saved_data, &data, (currentPointer+1) * 4 );
    saved_data_size = currentPointer+1;
  }

  void clear_buffer(){
    empty = true;
    currentPointer = 0;
  }
};

class replayBuffer : public buffer
{
public:
  ///input a 12 bit key presses and input time in millisecond. Time can be maximum of 20 bit number. If time is greater than 20bit then the last 12 msb gets cut off. If save not successful then return false otherwise true.
  bool SaveKeyPress(uint32_t keys , uint32_t time ){

    uint32_t value = (time << 12) + (keys & 0b111111111111);
    push(value);
  }

  ///return a tuple type. First tuple is key press (12bit) and 2nd is time(20bit) in milliseconds.
  tuple<uint32_t, uint32_t> readKeypressAndTime(int i){
    uint32_t value = readBuffer(i);
    return make_tuple( (value>>12) , (value  & 0b111111111111) );
  }

  ///return a tuple type. First tuple is saved key press (12bit) and 2nd is time(20bit) in milliseconds.
  tuple<uint32_t, uint32_t> read_Saved_KeypressAndTime(int i){
    uint32_t value = read_saved_Buffer(i);
    return make_tuple( (value>>12) , (value  & 0b111111111111 ) );
  }

};

typedef struct displayInfoStruct{
  uint8_t x;
  uint8_t y;
  uint32_t data;
  uint8_t base;
}displayInfo_t;


/*
//global helper functions
///return pointer to a struct
displayInfo_t encodeDisplayInfo(uint8_t X, uint8_t Y, uint32_t Data, uint8_t Base){
  
  displayInfo_t* pntr = new  displayInfo_t;
  pntr->x = X;
  pntr->y = Y;
  pntr->data = Data;
  pntr->base = Base;
  return pntr;
};

void decodeDisplayInfo (displayInfo_t* pntr){
  
  //delete pntr;
};
*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);

void DMA1_Channel3_IRQHandler(void);
void DMA_Buffer_End_ISR( DMA_HandleTypeDef* hdma );
uint8_t decodeKnobChange(const uint8_t& state, const uint8_t& past_change);



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

uint32_t SINE_WAVE_110_HZ [200] = { 2147483647, 2214937738, 2282325260, 2349579710, 2416634716, 2483424102, 2549881955, 2615942690, 2681541112, 2746612484, 2811092589, 2874917791, 2938025103, 3000352247, 3061837712, 3122420821, 3182041784, 3240641762, 3298162926, 3354548509, 3409742864, 3463691521, 3516341241, 3567640063, 3617537362, 3665983896, 3712931853, 3758334902, 3802148235, 3844328614, 3884834412, 3923625655, 3960664059, 3995913074, 4029337912, 4060905587, 4090584945, 4118346697, 4144163445, 4168009712, 4189861963, 4209698633, 4227500146, 4243248934, 4256929454, 4268528206, 4278033744, 4285436685, 4290729725, 4293907640, 4294967294, 4293907640, 4290729725, 4285436685, 4278033744, 4268528206, 4256929454, 4243248934, 4227500146, 4209698633, 4189861963, 4168009712, 4144163445, 4118346697, 4090584945, 4060905587, 4029337912, 3995913074, 3960664059, 3923625655, 3884834412, 3844328614, 3802148235, 3758334902, 3712931853, 3665983896, 3617537362, 3567640063, 3516341241, 3463691521, 3409742864, 3354548509, 3298162926, 3240641762, 3182041784, 3122420821, 3061837712, 3000352247, 2938025103, 2874917791, 2811092589, 2746612484, 2681541112, 2615942690, 2549881955, 2483424102, 2416634716, 2349579710, 2282325260, 2214937738, 2147483647, 2080029555, 2012642033, 1945387583, 1878332577, 1811543191, 1745085338, 1679024603, 1613426181, 1548354809, 1483874704, 1420049502, 1356942190, 1294615046, 1233129581, 1172546472, 1112925509, 1054325531, 996804367, 940418784, 885224429, 831275772, 778626052, 727327230, 677429931, 628983397, 582035440, 536632391, 492819058, 450638679, 410132881, 371341638, 334303234, 299054219, 265629381, 234061706, 204382348, 176620596, 150803848, 126957581, 105105330, 85268660, 67467147, 51718359, 38037839, 26439087, 16933549, 9530608, 4237568, 1059653, 0, 1059653, 4237568, 9530608, 16933549, 26439087, 38037839, 51718359, 67467147, 85268660, 105105330, 126957581, 150803848, 176620596, 204382348, 234061706, 265629381, 299054219, 334303234, 371341638, 410132881, 450638679, 492819058, 536632391, 582035440, 628983397, 677429931, 727327230, 778626052, 831275772, 885224429, 940418784, 996804367, 1054325531, 1112925509, 1172546472, 1233129581, 1294615046, 1356942190, 1420049502, 1483874704, 1548354809, 1613426181, 1679024603, 1745085338, 1811543191, 1878332577, 1945387583, 2012642033, 2080029555 };

volatile uint64_t increment = 0;
// volatiles
volatile uint8_t knob_3_pos = 0;
volatile uint32_t current_step_size = 0;
volatile uint32_t tmp_var = 0;
volatile bool mute = false;
volatile bool display_countdown=false;
volatile bool record = false;
volatile bool record_play =false;
volatile uint16_t  max_record_time_seconds = 3;


volatile bool start_replay;

//circular buffer 5000ms not protected by mutex currently
//CircularBuffer<uint32_t,1000 > recorded_buffer;
//uint32_t MaxBuffSize = 1000;
replayBuffer recorded_buffer;

volatile SemaphoreHandle_t key_array_mutex;
namespace DMA {
    uint32_t* DMABuffer;
    uint32_t* DMAHalfBuffPtr;
    uint32_t* NextDMABuffer; 
    volatile uint32_t* DMACurrBuffPtr;
    TaskHandle_t xDMAHandle = NULL;
    const UBaseType_t xArrayIndex = 1;

    volatile uint32_t* DMATMP;

    void sampleGeneratorTask( void* pvParameters ){

        static uint64_t buff_ptr = 0;
        static uint32_t phaseAcc = 0;
        uint32_t ulInterruptStatus;
        while(1){
            //generate 100 samples in DMA::NextDMABuffer
            // DMA::NextDMABuffer uses 12 bits.
            for (uint8_t i = 0; i < HALF_BUFFER_SIZE; i++){
                phaseAcc += increment;
                // DMA::NextDMABuffer[i] = (phaseAcc >> 24) >> (8 - knob_3_pos/2);
                // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

                if(!mute){
                DMA::DMACurrBuffPtr[i] = (phaseAcc >> 20) >> (8 - knob_3_pos/2);
                }
                else{
                   DMA::DMACurrBuffPtr[i] = 0;
                }
            }
            if (DMA::DMATMP == DMA::DMACurrBuffPtr){
                //Serial.println("Jerrror");
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            }
            DMA::DMATMP = DMA::DMACurrBuffPtr;
            // memcpy( (void*) DMA::DMACurrBuffPtr, (void*) DMA::NextDMABuffer, HALF_BUFFER_SIZE*4 );

            // ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
            
            // works
            xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulInterruptStatus, portMAX_DELAY);

            // vTaskSuspend( DMA::xDMAHandle );

            // if (increment != 0)
            //     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));


            // ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
            // take semaphore
            // transfer data


        }
    }
    void DMA_Buffer_End_Callback( DMA_HandleTypeDef* hdma ){
        __atomic_store_n( &DMA::DMACurrBuffPtr, (uint32_t*) DMA::DMAHalfBuffPtr, __ATOMIC_RELAXED );

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // vTaskNotifyFromISR( DMA::xDMAHandle, &xHigherPriorityTaskWoken );

        // works
        uint32_t ulStatusRegister;
        xTaskNotifyFromISR( DMA::xDMAHandle, 0, eNoAction, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        
        // Serial.println("a");
        // set next data transfer to go to half buffer
        // vTaskNotifyGiveFromISR( DMA::xDMAHandle, &xHigherPriorityTaskWoken );
            // tmp_var++;
            // if (tmp_var == 440)
            //   Serial.println("10 seconds");

        // if ( xTaskResumeFromISR( DMA::xDMAHandle ) == pdTRUE){
        //     portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        // }
        // else {
        //     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        // }

    }

    void DMA_Buffer_Half_Callback( DMA_HandleTypeDef* hdma ){
        __atomic_store_n( &DMA::DMACurrBuffPtr, (uint32_t*) DMA::DMABuffer, __ATOMIC_RELAXED );

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // vTaskNotifyGiveFromISR( DMA::xDMAHandle, &xHigherPriorityTaskWoken );

        // works
        uint32_t ulStatusRegister;
        xTaskNotifyFromISR( DMA::xDMAHandle, 0, eNoAction, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

        // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        // set next data transfer to go to half buffer
        // vTaskNotifyGiveFromISR( DMA::xDMAHandle, &xHigherPriorityTaskWoken );

        // if ( xTaskResumeFromISR( DMA::xDMAHandle ) == pdTRUE){
        //     portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
        // }
        // else {
        //     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        // }

        // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        // // Allow S
        // xSemaphoreGiveFromISR( DMA::xDMAHandle, &xHigherPriorityTaskWoken );
        // if (xSemaphoreGiveFromISR( DMA::xDMAHandle, NULL ) == errQUEUE_FULL){
        //     // error, last sample generation didn't finish yet
        //     //Serial.println("ERROR: Data generation not finished in time for buffer cycle.");
        // }
    }
}

volatile char note_message[] = "xxx";
QueueHandle_t msg_out_q;
QueueHandle_t display_q;

// one-hot encoded value
volatile uint32_t pressed_keys = 0;
volatile uint32_t execute_keys = 0;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

// //Function to set outputs via matrix
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
    analogWrite(OUTR_PIN, (SINE_WAVE_110_HZ[ buff_ptr >> 32 ] >> 24) >> (8 - knob_3_pos/2));
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
    uint8_t prev_knob2_press=0;
    uint16_t prev_keys = 0;
    uint16_t key = 0;
    uint32_t last_key_change_time = 0;
    bool record_setup = true;
    
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
        bool local_display_countdown = __atomic_load_n( &display_countdown, __ATOMIC_RELAXED);
        bool local_record_play = __atomic_load_n( &record_play, __ATOMIC_RELAXED);
        if(local_display_countdown || local_record_play){
          reading = __atomic_load_n( &pressed_keys, __ATOMIC_RELAXED);
        }else{
          __atomic_store_n( &pressed_keys, reading, __ATOMIC_RELAXED);
        }
        //make sure reading is 28 bit long
        reading = reading << 4;
        reading = reading >> 4;

        //recording code
        bool local_record = __atomic_load_n( &record, __ATOMIC_RELAXED);
        key = reading&0b111111111111;
        
        

        if(local_record){
          if(record_setup){
            last_key_change_time = millis();
            prev_keys = 0b1 << 13;
            record_setup = false;
            delay(2);
          }
          if(prev_keys != key ){
            uint32_t time = millis() - last_key_change_time;
            uint16_t local_max_time =  __atomic_load_n( &max_record_time_seconds, __ATOMIC_RELAXED);
            if(time > (local_max_time*1000) ){
              time = 0;
            }
            last_key_change_time = millis();
            Serial.println("hi");
            Serial.println(prev_keys);
            Serial.println(time);
            recorded_buffer.push( (prev_keys&0b111111111111) + (time << 12) );
            
          }
        }else{
          if(!record_setup){
            uint32_t time = millis() - last_key_change_time;
            uint16_t local_max_time =  __atomic_load_n( &max_record_time_seconds, __ATOMIC_RELAXED);
            if(time > (local_max_time*1000) ){
              time = 0;
            }
            last_key_change_time = millis();
            Serial.println("last");
            Serial.println(prev_keys);
            Serial.println(time);
            recorded_buffer.push( (prev_keys&0b111111111111) + (time << 12) );
            record_setup = true;
          }
        }
        prev_keys = key;
        
        // reading = onehot-encoded value with active keys = 1  ---------------

        // update knob 3 readings----------------------------------------------
        knob_3_internal = (reading >> 12) & 0b11;
        knob_3_change = decodeKnobChange( (knob_3_last << 2) | knob_3_internal , knob_3_change);
        knob_3_last = knob_3_internal;
        local_knob_3_pos += knob_3_change;
        if (local_knob_3_pos > 136)         local_knob_3_pos = 0;
        else if (local_knob_3_pos > 16)     local_knob_3_pos = 16;
        __atomic_store_n( &knob_3_pos, local_knob_3_pos, __ATOMIC_RELAXED);
        // --------------------------------------------------------------------

        //update mute (knob 3 press)-------------------------------------------
        uint8_t knob3_press = (reading >> 21) & 0b1;
        if(prev_knob3_press==1){
            if(knob3_press==0){
                bool local_mute = __atomic_load_n( &mute, __ATOMIC_RELAXED);
                __atomic_store_n( &mute, !local_mute, __ATOMIC_RELAXED);
            }
        }
        prev_knob3_press = knob3_press;
        //---------------------------------------------------------------------


        //update display timer (knob 2 press)-------------------------------------------
        uint8_t knob2_press = (reading >> 20) & 0b1;
        if(prev_knob2_press==1){
            if(knob2_press==0){
                bool local_displaycountdown =true;
                __atomic_store_n( &display_countdown, local_displaycountdown, __ATOMIC_RELAXED);
            }
        }
        prev_knob2_press = knob2_press;
        //---------------------------------------------------------------------

        // set pointer increment size and update message-----------------------
        for (uint8_t i = 0; i < 12; i++){
            // if key at index i is pressed
            if ( (reading >> i) & 0b1 ){
                if (i != last_note){
                    increment = STEP_SIZES[ i ];
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

        // xQueueSend( msg_out_q, (char*) local_note_message, portMAX_DELAY );
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
                break;
            }
        }
        // -----------------------------------------------------------------

         // choose a suitable font

        bool local_display_countdown = __atomic_load_n( &display_countdown, __ATOMIC_RELAXED);
        if(local_display_countdown){
          u8g2.clearBuffer(); 
          u8g2.setFont(u8g2_font_ncenB08_tr);
          u8g2.drawStr(2,10,"3");
          u8g2.sendBuffer();
          __atomic_store_n( &pressed_keys, (1<<7), __ATOMIC_RELAXED);
          delay(300);
          __atomic_store_n( &pressed_keys, 0, __ATOMIC_RELAXED);
          delay(700);
          u8g2.drawStr(22,10,"2");
          u8g2.sendBuffer();
          __atomic_store_n( &pressed_keys, (1<<7), __ATOMIC_RELAXED);
          delay(300);
          __atomic_store_n( &pressed_keys, 0, __ATOMIC_RELAXED);
          delay(700);
          u8g2.drawStr(42,10,"1");
          u8g2.sendBuffer();
          __atomic_store_n( &pressed_keys, (1<<9), __ATOMIC_RELAXED);
          delay(1000);
          __atomic_store_n( &pressed_keys, 0, __ATOMIC_RELAXED);

          local_display_countdown = false;

          __atomic_store_n( &display_countdown, false, __ATOMIC_RELAXED);
          __atomic_store_n( &record,true, __ATOMIC_RELAXED);
          u8g2.clearBuffer(); 
        }

        //record display
        bool local_record = __atomic_load_n( &record, __ATOMIC_RELAXED);
        if(local_record){
          uint16_t local_max_time =  __atomic_load_n( &max_record_time_seconds, __ATOMIC_RELAXED);
          u8g2.clearBuffer();
          for(int i = 1 ; i <= local_max_time ; i++){
             u8g2.setFont(u8g2_font_ncenB08_tr);
             u8g2.setCursor(50,20);
             u8g2.print(i);
             u8g2.sendBuffer();
             delay(1000);
             u8g2.clearBuffer(); 
          }
          __atomic_store_n( &record,false, __ATOMIC_RELAXED);
          __atomic_store_n( &record_play,true, __ATOMIC_RELAXED);
        }

        //play record
        bool local_record_play = __atomic_load_n( &record_play, __ATOMIC_RELAXED);
        if(local_record_play){
          uint16_t local_max_time =  __atomic_load_n( &max_record_time_seconds, __ATOMIC_RELAXED);


          for(int i = 0 ; i < recorded_buffer.sizeOfbuffer() ; i++){
            uint32_t buffdata = recorded_buffer.readBuffer(i);
            uint16_t keys = buffdata & 0b111111111111;
            u8g2.setCursor(2,10);
            u8g2.print((buffdata >> 12) , DEC);
            u8g2.setCursor(2,20);
            u8g2.print(keys , BIN);
            u8g2.sendBuffer();
            __atomic_store_n( &pressed_keys,keys, __ATOMIC_RELAXED);
            delay((buffdata >> 12));           
            u8g2.clearBuffer();
          }
          recorded_buffer.clear_buffer();
          __atomic_store_n( &record_play,false, __ATOMIC_RELAXED);
        }

        //dev -shaf
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.setCursor(2,10);
        u8g2.print(local_pressed_keys,BIN);
        bool local_mute = __atomic_load_n( &mute, __ATOMIC_RELAXED);
        u8g2.setCursor(2,20);
        if(local_mute){
          u8g2.print("MUTE");
        }else{
           u8g2.print("UNMUTE");
        }

        //u8g2.drawStr(2,10,"Hello World!");  // write something to the internal memory
        u8g2.setCursor(60, 20);
        u8g2.print(local_knob_3_pos);
        u8g2.setCursor(64, 30);
        u8g2.print((char*) note_message);
        u8g2.sendBuffer();          // transfer internal memory to the display

        //Toggle LED
        // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    }
}

void msgOutTask(void * pvParameters){

    const TickType_t xFrequency = 1000/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // gets autoupdated by xTaskDelayUntil
    while(1){   vTaskDelayUntil( &xLastWakeTime, xFrequency );
        // Serial.println(tmp);
    }


    // char out_msg[4];
    // while(1){ 
    //     xQueueReceive(msg_out_q, out_msg, portMAX_DELAY);
    //     // Serial.println(out_msg);
    // }
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

void replay(void * pvParameters){

    const TickType_t xFrequency = 1000/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // gets autoupdated by xTaskDelayUntil

    

    while(1){   
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
      if(start_replay)
      {
        
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
    DMA::DMABuffer = (uint32_t*) malloc(BUFFER_SIZE*sizeof(uint32_t));
    DMA::DMAHalfBuffPtr = &DMA::DMABuffer[HALF_BUFFER_SIZE];
    DMA::DMACurrBuffPtr = DMA::DMABuffer;
    DMA::NextDMABuffer = (uint32_t*) malloc(HALF_BUFFER_SIZE*sizeof(uint32_t)); 
    DMA::DMATMP = DMA::DMAHalfBuffPtr;
    // for (uint8_t i = 0; i < BUFFER_SIZE; i++){
    //     DMA::DMABuffer[i] = 0;
    // }

    // uint16_t phaseAcc = 0;
    // uint16_t increment = 1310;
    // for (uint8_t i = 0; i < BUFFER_SIZE; i++){
    //     DMA::DMABuffer[i] = 0;//phaseAcc >> 4;
    //     phaseAcc += increment;
    // }
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


    HAL_Init();
    HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );

    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_DAC1_Init();
    MX_TIM2_Init();


    TaskHandle_t sampleGeneratorHandle = NULL;
    xTaskCreate( DMA::sampleGeneratorTask, "sampleGenerator", 256, NULL, 9, &DMA::xDMAHandle );

    HAL_TIM_Base_Start( &htim2 );


    HAL_DAC_Start_DMA( &hdac1, DAC1_CHANNEL_1, DMA::DMABuffer, BUFFER_SIZE, DAC_ALIGN_12B_R );

    // set up DMA callbacks

    // my setups
    // interrupt service routines 
    // TIM_TypeDef * Instance = TIM1;
    // HardwareTimer *sampleTimer = new HardwareTimer(Instance);

    // interrupts
    // sampleTimer->setOverflow(22000, HERTZ_FORMAT);
    // sampleTimer->attachInterrupt(sampleISR);

    // threads

    TaskHandle_t displayUpdateHandle = NULL;
    xTaskCreate( displayUpdateTask, "displayUpdate", 256, NULL, 1, &displayUpdateHandle );
    
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate( scanKeysTask, "scanKeys", 64, NULL, 2, &scanKeysHandle );

    TaskHandle_t msgOutHandle = NULL;
    xTaskCreate( msgOutTask, "msgOut", 32, NULL, 2, &msgOutHandle );

    TaskHandle_t serialDecoderHandle = NULL;
    xTaskCreate( serialDecoderTask, "serialDecoder", 64, NULL, 3, &serialDecoderHandle );

    TaskHandle_t replayHandle = NULL;
    xTaskCreate( replay, "replay", 256, NULL, 3, &replayHandle );

    // // declare mutexes and other structures
    key_array_mutex = xSemaphoreCreateMutex(); // could use xSemaphoreCreateMutexStatic for it to be faster
    msg_out_q = xQueueCreate( 8, 4 );
    display_q = xQueueCreate( 10, 4 );
    // // start interrupts and scheduler
    // // sampleTimer->resume();
    HAL_DMA_RegisterCallback( &hdma_dac_ch1, HAL_DMA_XFER_CPLT_CB_ID, &DMA::DMA_Buffer_End_Callback);
    HAL_DMA_RegisterCallback( &hdma_dac_ch1, HAL_DMA_XFER_HALFCPLT_CB_ID, &DMA::DMA_Buffer_Half_Callback);
    hdma_dac_ch1.XferCpltCallback     = DMA::DMA_Buffer_End_Callback;
    hdma_dac_ch1.XferHalfCpltCallback = DMA::DMA_Buffer_Half_Callback;

    // done to make semaphores work

    Serial.println("Setup complete");
    vTaskStartScheduler();
    Serial.println("ERROR: Insufficient RAM");
        //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

}

void loop() {
    // put your main code here, to run repeatedly:

    //Update display
    while(1){
        //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        //delayMicroseconds(1000000);
    }
}


void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */
  /* USER CODE END DMA1_Channel3_IRQn 0 */
//   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  HAL_DMA_IRQHandler(&hdma_dac_ch1);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    myError_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    myError_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    myError_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    myError_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    myError_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 181;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    myError_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    myError_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    myError_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void myError_Handler(void)
{
  /* USER CODE BEGIN myError_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  Serial.println("HError");
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END myError_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
