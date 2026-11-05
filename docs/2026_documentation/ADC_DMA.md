# ADC Implementation with DMA Documentation

## 1. Introduction

**ADC (Analog-to-Digital Converter)**:

This is a peripheral used to convert an analog signal (voltage) into a digital value represnting its magnitudes so that the microcontroller can process it. It reads voltages within a specified range (between 0 V and the reference voltage 3.3V for us) and converts them into binary values. The ADC's functionality can vary depending on the microcontroller, and it is commonly used to digitize sensor inputs or other analog signals.

For example our 12 bit ADC can have output a range of [0, 4096) in refernce to our refernce voltage (3.3V). We can convert between the ADC output to voltage by multiplying our ADC 12-bit * Vref+/4096. 

However, keep in mind that our ADC input will be a continuous signal while our output is discrete time, this leads to resolution issues. 

**DMA (Direct Memory Access)** :

The DMA controller is a specialized peripheral used for transferring large amounts of data directly between memory and other peripherals (such as ADC, UART, etc.) without burdening the CPU. This way, the CPU doesn't need to read every byte of data and can focus on other tasks while the DMA handles the transfer. This is especially useful when transferring continuous streams of data, such as ADC readings, to memory. There is some initial overhead in setting up the DMA, so it's most beneficial when dealing with large, continuous data transfers.

In essence, DMA helps offload the CPU when dealing with large data transfers, while ADC helps convert analog signals into digital data that the microcontroller can process. 

**ADC + DMA <3** :

In our project, we essentially want to read our accelerometer data (for x, y, z axes) as digital voltage inputs in 3 channels and convert all three in a triple simeoultaneous regular ADC conversion, its timing dictated by a timer. Our DMA is set up in a circular ping-pong buffer so that we are able to do our calculations on half of the completed buffer when the half complete interrupt is raised while DMA completes data transfer on the other half of the buffer. Another interrupt is generated after a full conversion so we can work on that half while DMA overwrites the first half of our buffer. 

In addition, we also want to sample the cooler current and voltage of our battery, but we only have 3 ADC channels. We will use an injected conversion on one ADC to be triggered by a separate timer to sample our voltage and current. Our injected conversion ensures our regular conversion (the axes ones) will not be effected, and this is useful as we do not require a continuous stream of data for slow changing values unlike with our accelerometer. 

---

In this implementation, we will set up ADC1, ADC2, ADC3 to sample our data. ADC1 will be the "master" and ADC2/3 are the "slaves" in a triple regular conversion mode, which basically means when ADC1 converts ADC2/3 do so simeoultaneously. The data is stored in one buffer interleaved using DMA. This process ensures fast, continuous data transfer without requiring CPU intervention. The DMA controller automatically transfers ADC data to the buffers, which are then split into 3 fft buffers for each axis and processed for fast fourier transform. 

## 2. Setup

### 2. 1 Create a new  Project

1. File > New > STM32 Project
2. Commercial Part Number: "NUCLEO -F446RE"
3. Select board
   1. Name your project
   2. Language: C
   3. Binary Type: Executable
   4. Project Type: STM32Cube

### 2.2 IOC Setup

After we create the project, we want to first configure our I/O channels. We select PC1, PC2, PC3 for our x, y, and z axes respectively. In ADC1 we go to ADCs_Common_Settings and select Triple Combined Regular Simeoultaneous + Injected Simeoultaenous mode. For each of the selected pin numbers, we set up its corresponding channel, for PC1 -> IN11, PC2 -> IN12, PC3 -> IN13 press the check mark next to them in each ADC tab. For our injected conversion, we select PA2 for ADC1_IN2 and select IN2 in ADC1 (cooler current). We also select PB1 for ADC2_IN9 and select the IN9 channel in ADC2 (battery voltage). 

Since we want max resolution (12 bits), we need at least 15 clock cycles of the ADC clock (ADCCLK, common to all ADC) as a sampling time. Our APB2 (equivalent to our ADCCLK) is set to 72 Mhz and our prescaler to 4 (so ADCCLK runs at PCLK2/4 = 72 MHz/4 = 18 MHz). 

This gives a minimum sampling time of 15 clock cycles * 1/18MHz = 0.83 us

For DMA, we can use continuous polling mode to request data from ADC to copy them over to memory. Go to the DMA configuration window and add a DMA channel select DMA2 stream 0 and in circular mode (remember our ping pong). After this in the parameters settings, enable the “DMA continuous requests” option to allow the DMA to keep the ADC sampling continuously. 

We are also going to set up timers to trigger ADC triple conversion. Go to timers and select TIM8 with an internal clock source. We will sample at 1 MHz so we do a prescaler of 72-1 and a counter mode of 100-1 (100 ticks) for a 10kHz sampling time. Then under ADC1, select TIM8 as the external trigger source under ADC regular conversion mode. Now we will also set up the timer to trigger the injected conversion, go to TIM5 and type a prescaler of 7200-1 for 10kHz and a counter period of 5000-1 for a 2Hz sampling time for current + voltage. Then under ADC1, select TIM5 as the external trigger source under ADC injected conversion mode.

## 3. Now for the actual code which we all came here for

> Full Implementation in legacy > main > Core > Src > main.c

**3.1 Initialization of ADCs and DMA:**
The three ADCs (`hadc1`, `hadc2`, and `hadc3`) are initialized with DMA enabled. This configuration ensures that each ADC can convert analog inputs to digital values and store the results in a dedicated buffer. The following settings are crucial for each ADC:

* **Resolution:** 12-bit for high accuracy.
* **DMA Requests:** Continuous requests are enabled so that the ADC continuously fills the DMA buffer.
* **External Trigger:** ADC conversions are triggered by a timer (`TIM8` and 'TIM5'), ensuring consistent sampling intervals.
* **Data Alignment:** Data is right-aligned to fit within the 16-bit buffer.

Before we start, here is our ADC handler struct we will be using, refernced as variable 'adc'
```
typedef struct
{
	ADC_HandleTypeDef *hadc1;
	ADC_HandleTypeDef *hadc2;
	ADC_HandleTypeDef *hadc3;
	TIM_HandleTypeDef *htim;
	volatile uint16_t vbat_sample;
	volatile uint16_t current_sample;
} PL_ADC_Handler;
```
**3.2 Buffer Configuration:**
The ADCs store sampled data into the following buffers:

```
// Accelerometer buffer which DMA will write to. Needs to be shared a bit, so declared in main.
volatile uint16_t accelerometer_buffer[ACCELEROMETER_SAMPLE_SIZE_TRIPLE]; //enum is equal to 4096*3, remeber our data is interleaved so [x1,y1,z1,x2,y2....]
```


**3.3 DMA Start:**
DMA transfers are started for each ADC using the following commands:

* `HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);`
* This function starts the ADC (2 and 3)
* `HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length)`
* This function starts ADC1 as a master of the triple ADC multimode with DMA

```
//in ADC.c
    // Start ADCs
    // Starting ADC1 in triple synchronous multimode with DMA takes the values from the other two ADCs 
    HAL_StatusTypeDef adc2_status = HAL_ADC_Start(hadc2);
    HAL_StatusTypeDef adc3_status = HAL_ADC_Start(hadc3);
    HAL_StatusTypeDef adc1_status = HAL_ADCEx_MultiModeStart_DMA(hadc1, (uint32_t *) accelerometer_buffer, ACCELEROMETER_SAMPLE_SIZE_TRIPLE);

    // Start ADC injected conversion sample timer
    HAL_StatusTypeDef tim_status = HAL_TIM_Base_Start_IT(htim);
```

These commands configure the DMA to fill one buffer `ACCELEROMETER_SAMPLE_SIZE_TRIPLE` with interleaved data from the axes. 

**3.4 Half and Full Buffer Completion Handling:**
The DMA controller triggers two key interrupts during its operation:

* **Half Completion Interrupt:** This interrupt is triggered when the first half of the buffer is filled. The respective callback function (`HAL_ADC_ConvHalfCpltCallback`) is called for each ADC.
* **Full Completion Interrupt:** This interrupt is triggered when the entire buffer is filled. The `HAL_ADC_ConvCpltCallback` is called for each ADC. 

```
//in accelerometer.c, called inside of half conversion and full
// in half called with accelerometer_buffer, in full called with &accelerometer_buffer[FFT_SIZE_TRIPLE]
// where FFT_SIZE_TRIPLE is basically half of ACCELEROMETER_SAMPLE_SIZE_TRIPLE
// we have an accelerometer handler struct that contains the 3 buffers we want to use

void PL_Accelerometer_Record(PL_Accelerometer_Handler *accel, volatile uint16_t buffer[FFT_SIZE_TRIPLE])
{
    // Copy the data from the buffer to the FFT buffers
    for (int i = 0; i < FFT_SIZE_TRIPLE; i += 3)
    {
        accel->fft_buffer_x[i / 3] = buffer[i];
        accel->fft_buffer_y[i / 3] = buffer[i + 1];
        accel->fft_buffer_z[i / 3] = buffer[i + 2];
    }
    // Flag that the analysis is ready
    accel->analysis_ready = true;
}
```

In this callback, a flag is set to indicate that half of the buffer is ready for processing. Same idea with the full completion interrupt.

I hope you didn't forget about the injected converesion!
We likewise have an injected conversion interrupt from our timer, and we quickly collect and record our data

```
// in ADC.c, used in callback function void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) in main.c after checking if it is raised by TIM5/ADC1 
void PL_ADC_InjectedConversion(PL_ADC_Handler *adc)
{
    adc->current_sample = (uint16_t) HAL_ADCEx_InjectedGetValue(adc->hadc1, ADC_INJECTED_RANK_1);
    adc->vbat_sample = (uint16_t) HAL_ADCEx_InjectedGetValue(adc->hadc2, ADC_INJECTED_RANK_1);
}
```
We then store those raw ADC values in our ADC struct until we call them for conversion to voltage and amps

**3.4 Half and Full Buffer Completion Handling:**

The functions we call for conversion of injected mode variables

```
// from ADC.c, uses adc struct
float PL_ADC_GetBatteryVoltage(PL_ADC_Handler *adc)
{
    return ADC_RAW_TO_VOLTAGE(adc->vbat_sample) * BATTERY_VOLTAGE_FACTOR;
}

float PL_ADC_GetCoolerCurrent(PL_ADC_Handler *adc)
{
    // This formula is slightly inaccurate for low currents
    return (ADC_RAW_TO_VOLTAGE(adc->current_sample) - COOLER_CURRENT_OFFSET) / COOLER_CURRENT_FACTOR;
}
```

## 4. Useful links
[Getting Started with STM32 - Working with ADC and DMA](https://www.digikey.ca/en/maker/projects/getting-started-with-stm32-working-with-adc-and-dma/f5009db3a3ed4370acaf545a3370c30c)

[STM32 Timer-based ADC sampling at a specific rate, with debug output](https://youtu.be/r9UlXjrAOtc?si=GdtByExQN0a1NJmm)















