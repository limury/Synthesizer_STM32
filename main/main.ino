
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include <STM32FreeRTOS.h>
#include "src/main.h"
//#include <stm32l4xx_hal.h>
#include "pins.h"

#define LOAD(ptr) __atomic_load_n(ptr, __ATOMIC_RELAXED)
#define STORE(ptr, val) __atomic_store_n(ptr, val, __ATOMIC_RELAXED)

#define SOUND false
#define DMA_REQUEST_DAC1_CH1 6U
#define BUFFER_SIZE 400
#define HALF_BUFFER_SIZE BUFFER_SIZE/2 
#define SHIFTED_HALF_BUFFER_SIZE HALF_BUFFER_SIZE<<24
#define NSAM 256

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


namespace KeyVars{
    volatile uint32_t pressed_keys = 0; // value with array of binary flags representing each key being pressed with 0 or 1
    volatile uint8_t  key_array[12] = {0};
    volatile uint8_t  volume_knob_position = 0; // position of each knob
    volatile uint8_t  decoder_key_array[12] = {0};
    QueueHandle_t     message_out_queue;
    volatile uint8_t  current_wave_idx = 0;
}
namespace Mutex{
    SemaphoreHandle_t key_array_mutex;
    SemaphoreHandle_t decoder_key_array_mutex;
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
            xSemaphoreTake( Mutex::decoder_key_array_mutex, portMAX_DELAY);
            memcpy( &local_decoder_key_array, (uint8_t*) &KeyVars::decoder_key_array, 12*sizeof(uint8_t) );
            xSemaphoreGive( Mutex::decoder_key_array_mutex );
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
                // Serial.println("Error: Out of sync DMA");
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            }
            DMA::DMALastBuffPtr = DMA::DMACurrBuffPtr;

        }
    }

    void DMA_Buffer_End_Callback( DMA_HandleTypeDef* hdma ){
        __atomic_store_n( &DMA::DMACurrBuffPtr, (uint32_t*) DMA::DMAHalfBuffPtr, __ATOMIC_RELAXED );

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint32_t ulStatusRegister;
        xTaskNotifyFromISR( DMA::xDMATaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

    void DMA_Buffer_Half_Callback( DMA_HandleTypeDef* hdma ){
        __atomic_store_n( &DMA::DMACurrBuffPtr, (uint32_t*) DMA::DMABuffer, __ATOMIC_RELAXED );

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint32_t ulStatusRegister;
        xTaskNotifyFromISR( DMA::xDMATaskHandle, 0, eNoAction, &xHigherPriorityTaskWoken );
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}


namespace Recording{
    class Recorder{
      public:
        Recorder(){

        }
    };
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
        digitalWrite(REN_PIN, 1); }

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
    uint8_t local_knob_positions[4] = {0,0,0,0}, last_knob_states[4] = {0,0,0,0}, knob_states[4] = {0,0,0,0}, knob_changes[4] = {0,0,0,0};
    uint8_t local_volume_knob_position = 0, last_volume_knob_state = 0, volume_knob_state = 0, volume_knob_change = 0;
    

    // setup interval timer
    const TickType_t xFrequency    = 20/portTICK_PERIOD_MS;
    TickType_t       xLastWakeTime = xTaskGetTickCount(); // gets autoupdated by xTaskDelayUntil
    uint8_t          pressed_key_index_last = -1;
    while(1){   vTaskDelayUntil( &xLastWakeTime, xFrequency );
        // reading = onehot-encoded value with active keys = 1  ---------------
        local_pressed_keys = Utils::readKeys();
        STORE( &KeyVars::pressed_keys, local_pressed_keys );

        // set pointer increment size and update message-------------------------------------------------------------
        uint32_t pressed_differences = local_pressed_keys ^ last_pressed_keys; // find differences 
        for (uint8_t i = 0; i < 12; i++){
            if ( (pressed_differences >> i) & 0b1 ){ // means there have been changes to this key
                // create message and update increments------------------------
                char tmp_message[4] = "xxx";
                if ( (local_pressed_keys >> i) & 0b1 ){ // means the transition was from unplayed to played
                    tmp_message[0] = 'P';
                }
                else {
                    tmp_message[0] = 'R';
                }
                tmp_message[1] = '4';
                tmp_message[2] = Sound::INT_TO_HEX[i];
                xQueueSend( KeyVars::message_out_queue, tmp_message, portMAX_DELAY );
            }
        }

        // extract knob readings -----------------------------------------------------------------------------------
            // decode knob volume knob
            volume_knob_state = (local_pressed_keys >> 18) & 0b11;
            volume_knob_change = Utils::decodeKnobChange( (last_volume_knob_state << 2) | volume_knob_state, volume_knob_change );
            local_volume_knob_position += volume_knob_change;
            last_volume_knob_state = volume_knob_state;
            if (local_volume_knob_position > 140)         local_volume_knob_position = 0;
            else if (local_volume_knob_position > 24)     local_volume_knob_position = 24;
            // Store it in global variables
            __atomic_store_n( &KeyVars::volume_knob_position, local_volume_knob_position, __ATOMIC_RELAXED);
        
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
    TickType_t xLastWakeTime = xTaskGetTickCount(); // gets autoupdated by xTaskDelayUntil
    while(1){   vTaskDelayUntil( &xLastWakeTime, xFrequency );
        local_pressed_keys = LOAD( &KeyVars::pressed_keys );
        // operate on display
        u8g2.clearBuffer();         // clear the internal memory
        u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

        // print volume
        u8g2.drawStr(2, 10, "Volume: ");
        u8g2.setCursor(60, 10);
        u8g2.print(LOAD( &KeyVars::volume_knob_position ) );

        // print lowest note being played on keyboard
        u8g2.drawStr(2, 20, "Note: ");
        if ( local_pressed_keys & 0xFFF )
            for (uint8_t i = 0; i < 12; i++){
                if ( (local_pressed_keys >> i) & 0b1 ){
                    u8g2.drawStr(60, 20, Sound::NOTE_NAMES[ i ] );
                    break;
                }
            }
        
        // print waveform being played
        u8g2.drawStr(2, 30, "Wave: ");
        u8g2.setCursor(60, 30);
        u8g2.print( sound_wave_names[ LOAD(&KeyVars::current_wave_idx) ]);

        u8g2.sendBuffer();          

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
    xTaskCreate( serialDecoderTask, "serialDecoder", 64, NULL, 3, &serialDecoderHandle );

    // declare mutexes and other structures
    Mutex::key_array_mutex = xSemaphoreCreateMutex();
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
