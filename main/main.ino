
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include <STM32FreeRTOS.h>
#include "src/main.h"
//#include <stm32l4xx_hal.h>
#include "pins.h"


#define SOUND false
#define DMA_REQUEST_DAC1_CH1 6U
#define BUFFER_SIZE 400
#define HALF_BUFFER_SIZE BUFFER_SIZE/2 
#define SHIFTED_HALF_BUFFER_SIZE HALF_BUFFER_SIZE<<24
#define NSAM 256
#define LOAD(ptr) __atomic_load_n(ptr, __ATOMIC_RELAXED)
#define STORE(ptr, val) __atomic_store_n(ptr, val, __ATOMIC_RELAXED)


DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;


#define RECORDING_BUFFER_MAX_SIZE 1000




/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);

void DMA1_Channel3_IRQHandler(void);
void DMA_Buffer_End_ISR( DMA_HandleTypeDef* hdma );
uint8_t decodeKnobChange(const uint8_t& state, const uint8_t& past_change);
class Buffer;
class ReplayBuffer;

//Function to set outputs via matrix
void setOutMuxBit(const uint8_t bitIdx, const uint8_t value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}
//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);
namespace Sound{
    const uint32_t STEP_SIZES [] = {/*C*/ 51076057, /*C#*/ 54113197, /*D */ 57330935, 
                                   /*D#*/ 60740010, /*E */ 64351798, /*F */ 68178356,
                                   /*F#*/ 72232452, /*G */ 76527617, /*G#*/ 81078186, 
                                   /*A */ 85899346, /*A#*/ 91007187, /*B */ 96418756};
    const char NOTE_NAMES[13][3] = { "C ", "C#", "D ", "D#", "E ", "F ", "F#", "G ", "G#", "A ", "A#", "B ", "  "};
    const char INT_TO_HEX[] = "0123456789ABCDEF";

    uint32_t SINE_WAVE_12_BIT_256[NSAM] = {2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398, 2447, 2496, 2545, 2594, 2642, 2690, 2737, 2784, 2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143, 3185, 3226, 3267, 3307, 3346, 3385, 3423, 3459, 3495, 3530, 3565, 3598, 3630, 3662, 3692, 3722, 3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919, 3939, 3958, 3975, 3992, 4007, 4021, 4034, 4045, 4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094, 4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065, 4056, 4045, 4034, 4021, 4007, 3992, 3975, 3958, 3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777, 3750, 3722, 3692, 3662, 3630, 3598, 3565, 3530, 3495, 3459, 3423, 3385, 3346, 3307, 3267, 3226, 3185, 3143, 3100, 3057, 3013, 2968, 2923, 2877, 2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496, 2447, 2398, 2348, 2298, 2248, 2198, 2148, 2098, 2048, 1997, 1947, 1897, 1847, 1797, 1747, 1697, 1648, 1599, 1550, 1501, 1453, 1405, 1358, 1311, 1264, 1218, 1172, 1127, 1082, 1038, 995, 952, 910, 869, 828, 788, 749, 710, 672, 636, 600, 565, 530, 497, 465, 433, 403, 373, 345, 318, 291, 266, 242, 219, 197, 176, 156, 137, 120, 103, 88, 74, 61, 50, 39, 30, 22, 15, 10, 6, 2, 1, 0, 1, 2, 6, 10, 15, 22, 30, 39, 50, 61, 74, 88, 103, 120, 137, 156, 176, 197, 219, 242, 266, 291, 318, 345, 373, 403, 433, 465, 497, 530, 565, 600, 636, 672, 710, 749, 788, 828, 869, 910, 952, 995, 1038, 1082, 1127, 1172, 1218, 1264, 1311, 1358, 1405, 1453, 1501, 1550, 1599, 1648, 1697, 1747, 1797, 1847, 1897, 1947, 1997};
    uint32_t* SQUARE_WAVE_12_BIT_256;
    uint32_t* TRIANGLE_WAVE_12_BIT_256;
    const uint32_t SAMPLE_INCREMENTS_256[] = {/*C */ 50844943, /*C#*/ 53868341, /*D */ 57071519, /*D#*/ 60465168,
                                            /*E */ 64060614, /*F */ 67869857, /*F#*/ 71905608, /*G */ 76181338,
                                            /*G#*/ 80711316, /*A */ 85510661, /*A#*/ 90595390, /*B */ 95982472,};
}

namespace Mutex{
    SemaphoreHandle_t decoder_key_array_mutex;
}

class Buffer{

  private:
    uint32_t data[RECORDING_BUFFER_MAX_SIZE];
    uint32_t saved_data[RECORDING_BUFFER_MAX_SIZE];
    uint16_t saved_data_size;
    uint16_t current_pointer;
    uint8_t empty; // flag for is buffer empty
    uint8_t saved_empty; // flag for is the permanent buffer empty
    uint8_t full; // flag for is buffer full

  public:
    Buffer(): saved_data_size(0), current_pointer(0), empty(true), saved_empty(true), full(false) {}

  ///push data in. If successfully stored then return true otherwise if full then dont store and return false.
    bool push(uint32_t value){
        if(current_pointer  < RECORDING_BUFFER_MAX_SIZE -1 ) { 
            if (!empty)
                current_pointer++;
            empty = false;
            data[current_pointer] = value;
            return true;
        }
        else{
            full = true;
            return false;
        }
    };


  bool issavedEmpty(){ return saved_empty; }

  uint32_t sizeOfSaved(){ return saved_data_size; }

  uint32_t readSavedBuffer(const uint32_t& i){ return saved_data[i]; }

  void save(){
    saved_data_size = current_pointer + 1;
    saved_empty = false;
    memcpy ( &saved_data, &data, saved_data_size * 4 );
  }

  void clearBuffer(){
    empty = true;
    full = false;
    current_pointer = 0;
  }
};

class ReplayBuffer : public Buffer
{
  public:
    ReplayBuffer(): Buffer(){}

    ///input a 12 bit key presses and input time in millisecond. Time can be maximum of 20 bit number. If time is greater than 20bit then the last 12 msb gets cut off. If save not successful then return false otherwise true.
    bool saveKeyPress(const uint32_t& keys , const uint32_t& time ){
      uint32_t value = (time << 12) + (keys & 0xFFF);
      return push(value);
    }

    ///return a tuple type. First tuple is saved key press (12bit) and 2nd is time(20bit) in milliseconds.
    void readSavedKeypressAndTime(const uint32_t& i, uint32_t& val, uint32_t& time){
      uint32_t value = readSavedBuffer(i);        
      val = value & 0b111111111111; 
      time = (value>>12);
    }

};

namespace KeyVars{
    volatile uint32_t pressed_keys = 0; // value with array of binary flags representing each key being pressed with 0 or 1
    volatile uint8_t  key_array[12] = {0};
    volatile uint8_t  volume_knob_position = 0; // position of each knob
    volatile uint8_t  decoder_key_array[12] = {0};
    QueueHandle_t     message_out_queue;

    volatile uint8_t mute = false;
    volatile uint8_t  current_wave_idx = 0;
}

namespace Recording{
    ReplayBuffer record_buffer;
    volatile uint8_t record_play = false;
    volatile uint8_t is_recording = false;
    volatile uint8_t override_key_add = false;
    volatile uint16_t override_keys = 0;
}

namespace TaskHandle{
    inline TaskHandle_t replayHandle = NULL;
}

namespace DMA {
    uint32_t* DMABuffer;                 // start of buffer for DMA pointer
    uint32_t* DMAHalfBuffPtr;            // pointer to halfway through DMA buffer
    uint32_t* DMAModifiableBuffer;       // array for data being operated on which will then placed in DMABuffer
    volatile uint32_t* DMACurrBuffPtr;   // pointer to the current half of the buffer we are allowed to modify
    volatile uint32_t* DMALastBuffPtr;   // pointer to last half of buffer we modified (used to check if we are in sync)
    TaskHandle_t xDMATaskHandle = NULL;  // pointer to sampleGeneratorTask as seen by freeRTOS

    void sampleGeneratorTask( void* pvParameters ){
        uint32_t key_ptrs[12] = {0};
        uint32_t local_pressed_keys = 0;

        uint32_t decoder_key_ptrs[12] = {0};
        uint8_t local_decoder_key_array[12] = {0};

        uint8_t n_keys;

        uint32_t phase_accs[12] = {0};
        uint32_t decoder_phase_accs[12] = {0};

        uint32_t* sound_waves_ptrs[4] = {NULL, Sound::SQUARE_WAVE_12_BIT_256, Sound::TRIANGLE_WAVE_12_BIT_256, Sound::SINE_WAVE_12_BIT_256};
        uint32_t* current_sound_ptr = NULL;

        while(1){
            n_keys = 0;
            local_pressed_keys = LOAD( &KeyVars::pressed_keys );

            if(xSemaphoreTake( Mutex::decoder_key_array_mutex, 2/portTICK_PERIOD_MS)){
                memcpy( &local_decoder_key_array, (uint8_t*) &KeyVars::decoder_key_array, 12*sizeof(uint8_t) );
                xSemaphoreGive( Mutex::decoder_key_array_mutex );
            }
            else{
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            }

            if(LOAD(&KeyVars::mute))
                local_pressed_keys = local_pressed_keys & ~0xFFF;
            
            // zero out the array
            memset( (void*) DMA::DMAModifiableBuffer, 0, sizeof(uint32_t)*HALF_BUFFER_SIZE );
            // select current wave
            current_sound_ptr = sound_waves_ptrs[ LOAD( &KeyVars::current_wave_idx ) ];
            if ( current_sound_ptr ){ // if !=NULL : sound comes from generator buffer
                for (uint8_t i = 0; i < 12; i++){
                    if ( (local_pressed_keys >> i) & 1 ){ // if key i is pressed
                        uint32_t increment = Sound::SAMPLE_INCREMENTS_256[i]; // load the pointer increment
                        n_keys++; // icrease key count
                        for (uint32_t j = 0; j < HALF_BUFFER_SIZE; j++){ // for each element of the buffer that we are filling
                            key_ptrs[i] += increment; // increment the pointer (it automatically circles around to the start by overflowing)
                            DMA::DMAModifiableBuffer[j] += (current_sound_ptr[key_ptrs[i] >> 24]); // read the sound element
                        }
                    }

                    if (local_decoder_key_array[i]){
                        Serial.println((int8_t) local_decoder_key_array[i] - 5);
                        uint32_t increment; // get phase_acc increment depending on octave
                        if (local_decoder_key_array[i] >= 5)
                            increment = Sound::SAMPLE_INCREMENTS_256[i] << ( local_decoder_key_array[i] - 5);
                        else
                            increment = Sound::SAMPLE_INCREMENTS_256[i] >> ( 5 - local_decoder_key_array[i]);

                        n_keys++;
                        for (uint32_t j = 0; j < HALF_BUFFER_SIZE; j++){
                            decoder_key_ptrs[i] += increment;
                            DMA::DMAModifiableBuffer[j] += (current_sound_ptr[decoder_key_ptrs[i] >> 24]);
                        }
                    }
                }
            }
            else { // sawtooth
                for (uint8_t i = 0; i < 12; i++){ // for each key
                    if ( (local_pressed_keys >> i) & 1 ){ // if key i is pressed
                        uint32_t increment = Sound::STEP_SIZES[i]; // get the phase_acc incrementer for key i
                        n_keys++; // increase the count of keys being played
                        for (uint32_t j = 0; j < HALF_BUFFER_SIZE; j++){ // for each element in the half buffer
                            phase_accs[i] += increment; // increment phase_acc of that key 
                            DMA::DMAModifiableBuffer[j] += phase_accs[i] >> 20; // place phase_acc in the buffer to be read
                        }
                    }

                    if (local_decoder_key_array[i]){ // if key i is being played by the serial decoder
                        uint32_t increment;  // get phase_acc increment depending on octave
                        if (local_decoder_key_array[i] >= 5)
                            increment = Sound::STEP_SIZES[i] << ( local_decoder_key_array[i] - 5);
                        else
                            increment = Sound::STEP_SIZES[i] >> ( 5 - local_decoder_key_array[i]);
                        n_keys++; // increase count of keys being played
                        for (uint32_t j = 0; j < HALF_BUFFER_SIZE; j++){ // foe each element in the half buffer we are filling
                            decoder_phase_accs[i] += increment; // update phase_acc for the decoder
                            DMA::DMAModifiableBuffer[j] += decoder_phase_accs[i] >> 20; // palce phase_acc in the buffer 
                        }
                    }
                }
            }
            // adjust for volume
            if (n_keys){
                uint32_t vol_change = (12 - ( LOAD(&KeyVars::volume_knob_position)>>1));
                for (uint32_t j = 0; j < HALF_BUFFER_SIZE; j++)
                    DMA::DMAModifiableBuffer[j] = (DMA::DMAModifiableBuffer[j] >> vol_change)/n_keys;
            }

            // wait for next loop to start
            xTaskNotifyWait(pdFALSE, ULONG_MAX, NULL, portMAX_DELAY);

            memcpy( (void*) LOAD( &DMA::DMACurrBuffPtr ), (void*) DMA::DMAModifiableBuffer, HALF_BUFFER_SIZE*sizeof(uint32_t) );

            if (DMA::DMALastBuffPtr == LOAD(&DMA::DMACurrBuffPtr) ){
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            }
            DMA::DMALastBuffPtr = DMA::DMACurrBuffPtr;

        }
    }

    void DMA_Buffer_End_Callback( DMA_HandleTypeDef* hdma ){
        STORE( &DMA::DMACurrBuffPtr, (uint32_t*) DMA::DMAHalfBuffPtr);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint32_t ulStatusRegister;
        xTaskNotifyFromISR( DMA::xDMATaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
    void DMA_Buffer_Half_Callback( DMA_HandleTypeDef* hdma ){
        STORE( &DMA::DMACurrBuffPtr, (uint32_t*) DMA::DMABuffer);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint32_t ulStatusRegister;
        xTaskNotifyFromISR( DMA::xDMATaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}



namespace Utils{
    // reads all bits in a certain row of the pin matrix
    inline uint8_t readCols(){
        uint8_t out = (digitalRead(C3_PIN) << 3) + (digitalRead(C2_PIN) << 2) + (digitalRead(C1_PIN) << 1) + (digitalRead(C0_PIN));
        return out; }

    // Selects which row to read from
    inline void setRow(const uint8_t& row){
        digitalWrite(REN_PIN, 0);
        static const int pins [] = {RA0_PIN, RA1_PIN, RA2_PIN};
        for (uint8_t i = 0; i < 3; i++)
            digitalWrite( pins[i], (row >> i) & 1 );
        digitalWrite(REN_PIN, 1); 
    }

    inline uint32_t readKeys(){
        uint32_t reading = 0;
        setRow(6); delayMicroseconds(3);
        reading = readCols();
        setRow(5); delayMicroseconds(3);
        reading = (reading << 4) + readCols();
        setRow(4); delayMicroseconds(3);
        reading = (reading << 4) + readCols();
        setRow(3); delayMicroseconds(3);
        reading = (reading << 4) + readCols();
        setRow(2); delayMicroseconds(3);
        reading = (reading << 4) + readCols();
        setRow(1); delayMicroseconds(3);
        reading = (reading << 4) + readCols();
        setRow(0); delayMicroseconds(3);
        reading = (reading << 4) + readCols();
        return ~reading;
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

}


// scan keys being pressed
void scanKeysTask(void * pvParameters){
    uint32_t local_pressed_keys = 0, last_pressed_keys = 0;
    uint16_t first_12_keys = 0, last_12_keys = 0;
    uint8_t local_knob_positions[4] = {0,0,0,0}, last_knob_states[4] = {0,0,0,0}, knob_states[4] = {0,0,0,0}, knob_changes[4] = {0,0,0,0};
    uint8_t local_wave_knob_position=0, last_wave_knob_state=0, knob_wave_state=0, knob_wave_change=0;

    uint8_t local_mute = false;
    uint8_t is_playing_recording = false;
    uint8_t set_up = true;
    uint8_t record_last_keypress = false;
    uint32_t last_time_changed = 0;
    uint8_t local_volume_knob_position = 0, last_volume_knob_state = 0, volume_knob_state = 0, volume_knob_change = 0;

    // setup interval timer
    const TickType_t xFrequency    = 20/portTICK_PERIOD_MS;
    TickType_t       xLastWakeTime = xTaskGetTickCount(); // gets autoupdated by xTaskDelayUntil
    uint8_t          pressed_key_index_last = -1;


    while(1){   vTaskDelayUntil( &xLastWakeTime, xFrequency );
        // reading = onehot-encoded value with active keys = 1  ---------------
        local_pressed_keys = Utils::readKeys();

        if(LOAD(&Recording::record_play)) // if we are playing back
            local_pressed_keys = local_pressed_keys | LOAD( &Recording::override_keys ); // add the recorded keys to the played ones

        STORE( &KeyVars::pressed_keys, local_pressed_keys );

        // update message-------------------------------------------------------------
        uint32_t pressed_differences = local_pressed_keys ^ last_pressed_keys; // find differences 
        for (uint8_t i = 0; i < 12; i++){
            if ( (pressed_differences >> i) & 0b1 ){ // means there have been changes to this key
                // create message and update increments------------------------
                char tmp_message[4] = "xxx";
                if ( (local_pressed_keys >> i) & 0b1 ) // means the transition was from unplayed to played
                    tmp_message[0] = 'P';
                else 
                    tmp_message[0] = 'R';
                tmp_message[1] = '4';
                tmp_message[2] = Sound::INT_TO_HEX[i];
                xQueueSend( KeyVars::message_out_queue, tmp_message, portMAX_DELAY );
            }
        }

        //mute button on knob0 press
        uint8_t knob0_press = (local_pressed_keys >> 24) & 0b1;
        uint8_t prev_knob0_press = (last_pressed_keys >> 24) & 0b1;
        if(prev_knob0_press==1 && knob0_press==0)
            STORE( &KeyVars::mute, !LOAD( &KeyVars::mute) );

        // if knob 3 pressed, switch between recording and not
        uint8_t knob3_press = (local_pressed_keys >> 21) & 0b1;
        uint8_t prev_knob3_press = (last_pressed_keys >> 21) & 0b1;
        if(prev_knob3_press==1 && knob3_press==0){
            uint8_t local_is_recording = !LOAD( &Recording::is_recording );
            STORE( &Recording::is_recording, local_is_recording ); // toggle between recording or not recording

            if ( local_is_recording ){ // occurs when first entering recording phase
                Recording::record_buffer.clearBuffer(); // clear the recording buffer
                last_time_changed = millis(); // initialize when the last time there was a change in the keys was
            }
            else { // stopping recording
                uint32_t _time = millis();
                Recording::record_buffer.saveKeyPress( local_pressed_keys & 0xFFF , _time - last_time_changed );  // save the last keypress
                Recording::record_buffer.save(); // move data from temporary buffer to permanent buffer
                xTaskNotifyGive(TaskHandle::replayHandle); // play back the recording
            }
        }

        // record key pressed  --------------------------------------------------------------------------------------
        if ( LOAD(&Recording::is_recording) && (pressed_differences & 0xFFF)){ // if recording and there is a keychange
            uint32_t _time = millis(); // get current time
            if( !Recording::record_buffer.saveKeyPress( last_pressed_keys & 0xFFF , _time - last_time_changed )  ) // save the keypress
                STORE(&Recording::is_recording, false); // if the buffer is full, stop recording
            last_time_changed = _time;
        }


        // playback -------------------------------------------------------------------------------------------------
        uint8_t knob2_press = (local_pressed_keys >> 20) & 0b1;
        uint8_t prev_knob2_press = (last_pressed_keys >> 20) & 0b1;
        if(prev_knob2_press==1 && knob2_press==0 && !LOAD(&Recording::record_play) && !Recording::record_buffer.issavedEmpty() ) // if knob is pressed, and we are not playing, and the recording buffer is not empty, start the replay
            xTaskNotifyGive(TaskHandle::replayHandle);


        // extract knob readings -----------------------------------------------------------------------------------
            // decode knob volume knob
            volume_knob_state = (local_pressed_keys >> 18) & 0b11;
            volume_knob_change = Utils::decodeKnobChange( (last_volume_knob_state << 2) | volume_knob_state, volume_knob_change );
            local_volume_knob_position += volume_knob_change;
            last_volume_knob_state = volume_knob_state;
            if (local_volume_knob_position > 140)         local_volume_knob_position = 0;
            else if (local_volume_knob_position > 24)     local_volume_knob_position = 24;
            // Store it in global variables
            STORE(&KeyVars::volume_knob_position, local_volume_knob_position);
        
        uint8_t i = 2;
            // decode knob value
            knob_states[i] = (local_pressed_keys >> (12 + i*2)) & 0b11;
            knob_changes[i] = Utils::decodeKnobChange( (last_knob_states[i] << 2) | knob_states[i], knob_changes[i] );
            local_knob_positions[i] += knob_changes[i] * 32;
            last_knob_states[i] = knob_states[i];
            STORE( (uint8_t*) &KeyVars::current_wave_idx, local_knob_positions[i] >> 6 );


        last_pressed_keys = local_pressed_keys;
    }
}

void displayUpdateTask(void * pvParameters){
    uint32_t local_pressed_keys = 0;
    const char* const sound_wave_names[] = { "Sawtooth", "Square", "Triangle", "Sine" };
    // setup interval timer
    const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
    uint8_t local_mute = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // gets autoupdated by xTaskDelayUntil
    int note_pos = 35;
    
    while(1){   vTaskDelayUntil( &xLastWakeTime, xFrequency );

        
        // operate on display
        local_mute = __atomic_load_n( &KeyVars::mute, __ATOMIC_RELAXED);
        local_pressed_keys = LOAD( &KeyVars::pressed_keys );
        u8g2.clearBuffer();         // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

        u8g2.drawStr(2, 10, "Vol: ");
        u8g2.setCursor(27, 10);
        u8g2.print(LOAD( &KeyVars::volume_knob_position ) );
        

        // print note being played on keyboard
        u8g2.drawStr(2, 20, "Note: ");
        if ( local_pressed_keys & 0xFFF ) {
            note_pos = 35;
            for (uint8_t i = 0; i < 12; i++) {
                if ( (local_pressed_keys >> i) & 0b1 ){
                    u8g2.drawStr(note_pos, 20, Sound::NOTE_NAMES[ i ] );
                    note_pos += 15;
                }
            }
        } 

        if(!local_mute) 
            u8g2.drawStr(75, 10, "Unmuted");
        else
            u8g2.drawStr(75, 10, "Muted");

        if(LOAD(&Recording::is_recording))
            u8g2.drawFilledEllipse(105,25,3,3,U8G2_DRAW_ALL);

        if(LOAD(&Recording::record_play))
            u8g2.drawTriangle(115,20,115,30,120,25);
        
        // print waveform being played
        u8g2.drawStr(2, 30, "Wave: ");
        u8g2.setCursor(40, 30);
        u8g2.print( sound_wave_names[ LOAD(&KeyVars::current_wave_idx) ]);


        u8g2.sendBuffer();          // transfer internal memory to the display
    }
}

void msgOutTask(void * pvParameters){

    char out_msg[4];
    while(1){ 
        xQueueReceive( KeyVars::message_out_queue, out_msg, portMAX_DELAY);
        Serial.println(out_msg);
    }
}


void serialDecoderTask(void * pvParameters){
    uint64_t local_decoder_keys = 0;
    uint8_t local_decoder_key_array[12] = {0}; // has 12 values, one per key. each value is the octave being played
    // setup interval timer
    const TickType_t xFrequency = 10/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount(); // gets autoupdated by xTaskDelayUntil
    while(1){   vTaskDelayUntil( &xLastWakeTime, xFrequency );
        char in_msg[] = "xxx";
        while(Serial.available() > 0){
            for (uint8_t it = 0 ; it < 3; it++)
                in_msg[it] = Serial.read();

            if ( Serial.read() != '\n' )
                continue;
            
            if (in_msg[0] == 'P'){
                uint8_t note = strtol( &in_msg[2], NULL, 16); // extract note
                int8_t octave = ((uint8_t) in_msg[1]) - 47; // extract octave + 1
                local_decoder_key_array[ note ] = octave;
            }
            else if (in_msg[0] == 'R'){
                uint8_t note = strtol( &in_msg[2], NULL, 16); // extract note
                local_decoder_key_array[ note ] = 0; // octave = 0 means no sound
            }
        }
        // copy key_array for the decoder
        xSemaphoreTake( Mutex::decoder_key_array_mutex, portMAX_DELAY ) ;
        memcpy( (void*) KeyVars::decoder_key_array, local_decoder_key_array, 12*sizeof(uint8_t) );
        xSemaphoreGive( Mutex::decoder_key_array_mutex );
    }
}

void replayTask(void * pvParameters){
    uint32_t keys, time;
    while(1){
        uint8_t start = ulTaskNotifyTake( pdTRUE, (TickType_t) portMAX_DELAY );
        if(start > 0)
        {
            STORE(&Recording::record_play, true);

            for(int i = 0 ; i < Recording::record_buffer.sizeOfSaved() ; i++)
            {         
                Recording::record_buffer.readSavedKeypressAndTime(i, keys, time);
                STORE( &Recording::override_keys, keys);
                vTaskDelay(time / portTICK_PERIOD_MS);
            }
            STORE(&Recording::record_play, false);
        }
    }
    
}



void setup() {
    // Sound ---------------------------------------------------------------
    // generate square wave
    Sound::SQUARE_WAVE_12_BIT_256 = (uint32_t*) malloc( NSAM * sizeof(uint32_t));
    memset( Sound::SQUARE_WAVE_12_BIT_256, 0, NSAM/2 * sizeof(uint32_t));
    for (uint32_t i = NSAM/2; i < NSAM; i++){
        Sound::SQUARE_WAVE_12_BIT_256[i] = 4095;}
    
    // generate triangle wave
    Sound::TRIANGLE_WAVE_12_BIT_256 = (uint32_t*) malloc( NSAM * sizeof(uint32_t));
    for (uint32_t i = 0; i < NSAM/2; i++){
        Sound::TRIANGLE_WAVE_12_BIT_256[i] = i*4095>>7;}
    for (uint32_t i = 0; i < NSAM/2; i++){
        Sound::TRIANGLE_WAVE_12_BIT_256[i+128] = (128-i)*4095>>7;}
    // preliminary --------------------------------------------------------
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
    Serial.println( Sound::TRIANGLE_WAVE_12_BIT_256[128]);


    // DMA -----------------------------------------------------------------
    DMA::DMABuffer = (uint32_t*) malloc(BUFFER_SIZE*sizeof(uint32_t));
    DMA::DMAHalfBuffPtr = &DMA::DMABuffer[HALF_BUFFER_SIZE];
    DMA::DMACurrBuffPtr = DMA::DMABuffer;
    DMA::DMAModifiableBuffer = (uint32_t*) malloc(HALF_BUFFER_SIZE*sizeof(uint32_t)); 
    DMA::DMALastBuffPtr = NULL;
    memset(DMA::DMABuffer, 2048, BUFFER_SIZE*sizeof(uint32_t));
    KeyVars::current_wave_idx = 0;

    HAL_Init();
    HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );

    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_DAC1_Init();
    MX_TIM2_Init();

    TaskHandle_t sampleGeneratorHandle = NULL;
    xTaskCreate( DMA::sampleGeneratorTask, "sampleGenerator", 256, NULL, 9, &DMA::xDMATaskHandle );

    HAL_TIM_Base_Start( &htim2 );
    HAL_DAC_Start_DMA( &hdac1, DAC1_CHANNEL_1, DMA::DMABuffer, BUFFER_SIZE, DAC_ALIGN_12B_R );

    
    // threads ---------------------------------------------------------------

    TaskHandle_t displayUpdateHandle = NULL;
    xTaskCreate( displayUpdateTask, "displayUpdate", 256, NULL, 1, &displayUpdateHandle );
    
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate( scanKeysTask, "scanKeys", 64, NULL, 2, &scanKeysHandle );

    TaskHandle_t msgOutHandle = NULL;
    xTaskCreate( msgOutTask, "msgOut", 32, NULL, 2, &msgOutHandle );

    TaskHandle_t serialDecoderHandle = NULL;
    xTaskCreate( serialDecoderTask, "serialDecoder", 64, NULL, 4, &serialDecoderHandle );

    

    xTaskCreate( replayTask, "record", 32, NULL, 2, &TaskHandle::replayHandle );



    // declare mutexes and other structures
    Mutex::decoder_key_array_mutex = xSemaphoreCreateMutex();
    KeyVars::message_out_queue = xQueueCreate( 12, 4 );

    // Register DMA Callbacks
    HAL_DMA_RegisterCallback( &hdma_dac_ch1, HAL_DMA_XFER_CPLT_CB_ID, &DMA::DMA_Buffer_End_Callback);
    HAL_DMA_RegisterCallback( &hdma_dac_ch1, HAL_DMA_XFER_HALFCPLT_CB_ID, &DMA::DMA_Buffer_Half_Callback);
    hdma_dac_ch1.XferCpltCallback     = DMA::DMA_Buffer_End_Callback;
    hdma_dac_ch1.XferHalfCpltCallback = DMA::DMA_Buffer_Half_Callback;

    Serial.println("Setup complete");
    vTaskStartScheduler();
    Serial.println("ERROR: Insufficient RAM");

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

//   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  HAL_DMA_IRQHandler(&hdma_dac_ch1);


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



  DAC_ChannelConfTypeDef sConfig = {0};


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


}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{



  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};


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
