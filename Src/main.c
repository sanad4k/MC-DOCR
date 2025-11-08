#include "main.h"

#define sample_times 12.00

// Semaphore for the main loop
volatile bool Sign = false;

// Ping pong buffers for the adc 
static float adc_Current_data_A[(int)sample_times];
static float adc_Current_data_B[(int)sample_times];
static float adc_Voltage_data_A[(int)sample_times];
static float adc_Voltage_data_B[(int)sample_times];

// Buffer selection
volatile uint8_t active_buffer = 0;

// To dynamically set the time period for the phase locked loop
volatile uint32_t g_current_period = 20000; // The 1MHz ticks for one cycle

// Hardware Handles
TIM_HandleTypeDef adc_trigger;
ADC_HandleTypeDef adc_handle;
TIM_HandleTypeDef zero_handle;

// Set based on the direction
volatile bool toTrip = false;

int main (void){
    // Initialize HAL
    HAL_Init();
    SystemClock_Config();
    // Variables for persistant metrics
    static double progress = 0;
    static constTable ktable[7];
    static double ptable[760];

    // Setup the constant table
    TableSetup(ktable);

    // The sin and cos lookup tables built on bootup
    static float cos_table[(int)sample_times];
    static float sin_table[(int)sample_times];

    // Built the sin and cos tables based on the sample times
    setupTrig(cos_table, sin_table);

    // The Relay object
    relayType curRelay = { // hardcode for now
        .current_pickup = 1.5,
        .time_delay = 24000.0,
        .type = CO2,
        .direction_angle = M_PI/3.00,
    };

    double pickup_squared = pow(curRelay.current_pickup, 2);

    // Populate the progress lookup table
    buildProgress(ptable, ktable, &curRelay);

    // Initialize the peripherals : the ADC, the RELAY, the PLL, and the trigger TIMER
    adc_init();
    relay_init();
    pll_init();
    timer_init();
    indicator_init();
    // start all the interrupts and timers
    HAL_TIM_IC_Start_IT(&zero_handle, TIM_CHANNEL_4);
    HAL_TIM_Base_Start(&adc_trigger);
    HAL_ADC_Start_IT(&adc_handle);

    // Turn on the indicator
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    while(1){
        // Is the semaphore set
        if(Sign == true){

            uint8_t buffer_to_process;

            // Disable the ADC interrupt to prevent 'active_buffer'
            // and 'Sign' from being changed while we read them.
            HAL_NVIC_DisableIRQ(ADC_IRQn);

            // Latch the buffer that is ready for processing
            buffer_to_process = active_buffer; 
            // Clear the flag *inside* the critical section
            Sign = false; 

            // Re-enable the interrupt
            HAL_NVIC_EnableIRQ(ADC_IRQn);
            complexNum current_filt;
            complexNum voltage_filt;

            if(buffer_to_process == 0){
                current_filt = getFiltered(adc_Current_data_A, cos_table, sin_table);
                voltage_filt = getFiltered(adc_Voltage_data_A, cos_table, sin_table);
            }

            else{
                voltage_filt = getFiltered(adc_Voltage_data_B, cos_table, sin_table);
                current_filt = getFiltered(adc_Current_data_B, cos_table, sin_table);
            }

            // Get the power for the directional over current relay
            double C_setting = cos(curRelay.direction_angle);
            double S_setting = sin(curRelay.direction_angle);

            double P_meas = (voltage_filt.real* current_filt.real) + (voltage_filt.img * current_filt.img);
            double Q_meas = (voltage_filt.real* current_filt.img) - (voltage_filt.img * current_filt.real);

            // The directional score that determines if forward or backward
            double directional_score = (P_meas * C_setting) + (Q_meas * S_setting);

            toTrip = directional_score > 0;

            double fund_sqcurrent = getRMSquared(current_filt);

            if(fund_sqcurrent > pickup_squared){
                double psm = sqrt(fund_sqcurrent/pickup_squared);
                double delta_T = (double)(g_current_period / sample_times) / 1000000.0;
                double norm_progress = ptable[PSM_TO_I(psm)];
                progress += norm_progress * delta_T;
                if(progress >= 65535 && toTrip){
                    quickTrip();
                }
            }

            else{
                progress = 0;
            }

        }

    }
    return 0;
}

// Interrupt callback for ADC the interrupt must call this internally i guess
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    // current or voltage current is rank 1 so current first [current = 0]
    static int interrupt_current_count = 0;
    static int interrupt_voltage_count = 0;

    static uint8_t which = 0;
    double value;

    if(!which){
        uint32_t adc_Current_val = HAL_ADC_GetValue(hadc);
        value = map(adc_Current_val, 0, 1023, 0, 3.3);
        if(!active_buffer){
            adc_Current_data_B[interrupt_current_count] = value;
        } else {
            adc_Current_data_A[interrupt_current_count] = value;
        }
        interrupt_current_count++;
        which = 1; // Voltage is next
    } else {
        uint32_t adc_Voltage_val = HAL_ADC_GetValue(hadc);
        value = map(adc_Voltage_val, 0, 1023, 0, 3.3);
        if(!active_buffer){
            adc_Voltage_data_B[interrupt_voltage_count] = value;
        } else {
            adc_Voltage_data_A[interrupt_voltage_count] = value;
        }
        interrupt_voltage_count++;
        which = 0; // Current is next
    }
    // wait for 12 samples
    if(interrupt_current_count== sample_times && interrupt_voltage_count == sample_times) {
        interrupt_current_count = 0;
        interrupt_voltage_count = 0;
        active_buffer = !active_buffer; // <-- SWAP THE ACTIVE BUFFER
        // result is ready signal the main
        Sign = true;
    }
}

// Interrupt call back for the capture timer
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    static uint32_t last_capture=0;
    // Make sure the interrupt came from TIM3
    if (htim->Instance == TIM3) {

        // Read the time that was automatically captured
        uint32_t current_capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

        // (The timer automatically handles the 16-bit rollover)
        uint32_t period = current_capture - last_capture;
        if(period > 10000) {
            // Save the current time for the *next* interrupt
            last_capture = current_capture;
            // store the current period
            g_current_period = period;
            // Calculate the new sample interval
            uint32_t new_sample_period = period / sample_times; 

            // This macro instantly updates TIM2's period (ARR)
            __HAL_TIM_SET_AUTORELOAD(&adc_trigger, new_sample_period);
        }
    }
}

// Sliding DFT filter
complexNum getFiltered(float *adc_t, float *cos_table, float *sin_table){

    complexNum result;
    result.real = 0;
    result.img = 0;

    for(int i = 0; i < sample_times; i++){

        result.real += adc_t[i]*cos_table[i];

        result.img += adc_t[i]*-sin_table[i];
    }
    result.real *= 2.00/sample_times;
    result.img *= 2.00/sample_times;

    return result;

}

// To find the RMS square of the fundamental current
double getRMSquared(complexNum current_fund){
    double real_sq = current_fund.real * current_fund.real;
    double img_sq = current_fund.img * current_fund.img;
    return (real_sq + img_sq)/2.0;
}

// To quickly trip the breaker
void quickTrip(){

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); 

}

// Calculate the expected relay trip time
double getTime(constTable *curTable, relayType *curRelay, double current_PSM){
    double time = 0.0;
    if(current_PSM >= 1.5){
        time = curTable[curRelay->type].constT;
        time += curTable[curRelay->type].constK/pow((current_PSM-curTable[curRelay->type].constC),curTable[curRelay->type].constP);
        time *= (curRelay->time_delay/24000.0);
    }
    else {
        time = curTable[curRelay->type].constR/((current_PSM)-1);
        time *= curRelay->time_delay/24000.0;
    }
    return time;
}

// Build the progress table
void buildProgress(double *progress, constTable *calTable, relayType *calRelay){
    for(int i = 0; i < 760; i++){
        double psm = 1 + i*(1.00/40.00);
        double time_i = getTime(calTable,calRelay,psm);
        if (time_i > 0) {
            progress[i] = 65535.0 / time_i;
        } else {
            progress[i] = 0; // Avoid divide-by-zero
        }
    }
}

// Setup the cos and sine tables on boot
void setupTrig(float *cos_table, float *sin_table){
    for(int i = 0; i < sample_times; i++){
        float angle = (2*M_PI/sample_times)*i;
        cos_table[i] = cos(angle);
        sin_table[i] = sin(angle);
    }
}

// Setup the table of constants
void TableSetup(constTable *mytable){

    mytable[CO2].constT = 111.99;
    mytable[CO2].constK = 735.00;
    mytable[CO2].constC = 0.675;
    mytable[CO2].constP = 1;
    mytable[CO2].constR = 501;

    mytable[CO5].constT = 8196.67;
    mytable[CO5].constK = 13768.94;
    mytable[CO5].constC = 1.130;
    mytable[CO5].constP = 1;
    mytable[CO5].constR = 22705;

    mytable[CO6].constT = 784.52;
    mytable[CO6].constK = 671.01;
    mytable[CO6].constC = 1.190;
    mytable[CO6].constP = 1;
    mytable[CO6].constR = 1475;

    mytable[CO7].constT = 524.84;
    mytable[CO7].constK = 3120.56;
    mytable[CO7].constC = 0.800;
    mytable[CO7].constP = 1;
    mytable[CO7].constR = 2491;

    mytable[CO8].constT = 477.84;
    mytable[CO8].constK = 4122.08;
    mytable[CO8].constC = 1.270;
    mytable[CO8].constP = 1;
    mytable[CO8].constR = 9200;

    mytable[CO9].constT = 310.01;
    mytable[CO9].constK = 2756.06;
    mytable[CO9].constC = 1.350;
    mytable[CO9].constP = 1;
    mytable[CO9].constR = 9342;

    mytable[CO11].constT = 110.00;
    mytable[CO11].constK = 17640.00;
    mytable[CO11].constC = 0.500;
    mytable[CO11].constP = 2;
    mytable[CO11].constR = 8875;

}

// Map function for the ADC
double map(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Intitialize the relay
void relay_init(void){

    // Initialize PA3 as an digital output pin
    GPIO_InitTypeDef GPIO_InitStruct = {
        // Pin 3
        .Pin = GPIO_PIN_3, 
        // Set as analog mode
        .Mode = GPIO_MODE_OUTPUT_PP, 
        // No push pull
        .Pull = GPIO_PULLUP, 
        // LOW doesnt matter anyways
        .Speed = GPIO_SPEED_FAST,
    };

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Initialize the trigger time of the PLL
void timer_init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    // Use Timer 2
    adc_trigger.Instance = TIM2;
    // for 1 Mhz
    adc_trigger.Init.Prescaler = 83; 
    // Count up
    adc_trigger.Init.CounterMode = TIM_COUNTERMODE_UP;
    // Let the period be 0 we will update once zero crosser is ready
    adc_trigger.Init.Period = g_current_period/(int)sample_times; 
    // no div
    adc_trigger.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    adc_trigger.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    // high priority will adjust later
    HAL_TIM_Base_Init(&adc_trigger);

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&adc_trigger, &sMasterConfig);

}

void indicator_init(void){
    // Initialize PA15 as an digital output pin
    GPIO_InitTypeDef GPIO_InitStruct = {
        // Pin 3
        .Pin = GPIO_PIN_15, 
        // Set as analog mode
        .Mode = GPIO_MODE_OUTPUT_PP, 
        // No push pull
        .Pull = GPIO_PULLUP, 
        // LOW doesnt matter anyways
        .Speed = GPIO_SPEED_LOW,
    };

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// setup a phase locked loops to trigger the adc based on incoming exti interrupts
void pll_init(void){
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE(); // already enabled for ADC
    GPIO_InitTypeDef GPIO_InitStruct = {
        // Pin 0
        .Pin = GPIO_PIN_1, 
        // Set as analog mode
        .Mode = GPIO_MODE_AF_PP, 
        // No push pull
        .Pull = GPIO_NOPULL,
        // High freq for the zero cross detector
        .Speed = GPIO_SPEED_FREQ_HIGH,
        // connect to the tim3
        .Alternate = GPIO_AF2_TIM3,
    };

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    zero_handle.Instance = TIM3;
    // 1MHz clock 
    zero_handle.Init.Prescaler = 83;
    // count up
    zero_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    // start count from zero and count till overflow
    zero_handle.Init.Period = 0xFFFF;
    // Also irrelevant
    zero_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    // Irrelevant
    // .RepetitionCounter =  
    zero_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&zero_handle);

    TIM_IC_InitTypeDef sConfigIC = {0};

    // Trigger on the rising edge
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;

    // Connect the pin TI1 to the timer's capture register
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI; 

    // Don't skip any events
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1; 

    // No filter (for a clean ZC signal)
    sConfigIC.ICFilter = 0; 
    HAL_TIM_IC_ConfigChannel(&zero_handle, &sConfigIC, TIM_CHANNEL_4);

    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0); // High priority
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

}

// Setup the adc and return the handle
void adc_init(void){

    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Initialize PA0 as an analog pin
    GPIO_InitTypeDef GPIO_InitStruct = {
        // Pin 0 
        .Pin = GPIO_PIN_0 | GPIO_PIN_1,
        // Set as analog mode
        .Mode = GPIO_MODE_ANALOG, 
        // No push pull
        .Pull = GPIO_NOPULL, 
        // LOW doesnt matter anyways
        .Speed = GPIO_SPEED_FREQ_LOW,
    };

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Initialize the ADC instance
    adc_handle.Instance = ADC1;
    // Peripheral clock scaled down by 2
    adc_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    // 10 Bit like in the paper
    adc_handle.Init.Resolution = ADC_RESOLUTION_10B;
    // Big Endian easier DFT
    adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    // We need to convert multiple channels simultaneously for CT and PT
    adc_handle.Init.ScanConvMode = ENABLE;
    // Interrupt for each conversion first for the current and then for the voltage
    adc_handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    // ADC samples all channels and stops and waits for the external trigger to start the sequence again
    adc_handle.Init.ContinuousConvMode = DISABLE;
    // For single phase we will deal with three phase systems later
    adc_handle.Init.NbrOfConversion = 2;
    // Convert the entire seq in one go
    adc_handle.Init.DiscontinuousConvMode = DISABLE;
    // Externally triggered by an exti interrupt whenver zero crossing occurs 
    adc_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO; 
    // Trigger at timer rising edge,
    adc_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    //Do not Re Arm the ADC by itself and continue, till CPU intervention
    adc_handle.Init.DMAContinuousRequests = DISABLE;

    __HAL_RCC_ADC1_CLK_ENABLE();


    HAL_ADC_Init(&adc_handle);

    // The current channel
    ADC_ChannelConfTypeDef ADC_Channel_Current_InitStruct = {
        // Specify the ADC channel
        .Channel = ADC_CHANNEL_0,
        // The rank of data to sample
        .Rank = 1,
        // Not sure about this one I think its samples every 84 cycles
        // Clock div is 2 so for 16MHz hsi thats like 8Mhz for the adc basically every 10.5 uS
        .SamplingTime = ADC_SAMPLETIME_84CYCLES,
        // Future use set to 0
        .Offset = 0,
    };

    // The voltage channel
    ADC_ChannelConfTypeDef ADC_Channel_Voltage_InitStruct = {
        .Channel = ADC_CHANNEL_1,
        .Rank = 2,
        .SamplingTime = ADC_SAMPLETIME_84CYCLES,
    };

    HAL_ADC_ConfigChannel(&adc_handle, &ADC_Channel_Voltage_InitStruct);

    HAL_ADC_ConfigChannel(&adc_handle, &ADC_Channel_Current_InitStruct);

    __HAL_ADC_ENABLE(&adc_handle);
    __HAL_ADC_ENABLE_IT(&adc_handle, ADC_IT_EOC);
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

}

static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  //if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  //{
  //  Error_Handler();
  //}
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  //if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  //{
  //  Error_Handler();
  //}
}
