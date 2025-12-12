//###########################################################################
//
// FILE:   hotzenblitz_current_sensing_with_pwm_output.c
//
// TITLE:  Motor Current Sensing + 10kHz PWM Signal Output
//
// DESCRIPTION:
//   - Samples 2 current sensors every 1ms using ePWM1 triggered ADC
//   - ePWM1: 1kHz timer for ADC triggering - OUTPUT on GPIO0 (J4 Pin 40)
//   - ePWM2: 10kHz PWM signal with 50% duty cycle - OUTPUT on GPIO2 (J4 Pin 38)
//   - Both signals visible on oscilloscope (2 channels)
//   - F28003x running at 120 MHz
//
//###########################################################################

#include "driverlib.h"
#include "device.h"

//*****************************************************************************
// Pin Configuration - F28003x LaunchPad
//*****************************************************************************
// ADC INPUTS:
// - Sensor 1 → J3 Pin 24 (ADCINA2)
// - Sensor 2 → J3 Pin 26 (ADCINA14)

// PWM OUTPUTS for Oscilloscope:
// - PWM1A (1kHz timer) → GPIO0 (J4 Pin 40) - ADC Timer Signal
// - PWM2A (10kHz, 50%) → GPIO2 (J4 Pin 38) - High Frequency PWM Signal

// SENSOR 1: ADCA Module, SOC0
#define SENSOR1_ADC_MODULE      ADCA_BASE
#define SENSOR1_ADC_CHANNEL     ADC_CH_ADCIN2    // J3 Pin 24 (ADCINA2)
#define SENSOR1_RESULT_BASE     ADCARESULT_BASE
#define SENSOR1_SOC_NUMBER      ADC_SOC_NUMBER0

// SENSOR 2: ADCA Module, SOC1
#define SENSOR2_ADC_MODULE      ADCA_BASE
#define SENSOR2_ADC_CHANNEL     ADC_CH_ADCIN14   // J3 Pin 26 (ADCINA14)
#define SENSOR2_RESULT_BASE     ADCARESULT_BASE
#define SENSOR2_SOC_NUMBER      ADC_SOC_NUMBER1

// PWM OUTPUT Configuration
#define PWM1_OUTPUT_GPIO        0                 // GPIO0 for PWM1A (1kHz timer)
#define PWM1_OUTPUT_PIN_CONFIG  GPIO_0_EPWM1_A   // ePWM1A function
#define PWM2_OUTPUT_GPIO        2                 // GPIO2 for PWM2A (10kHz)
#define PWM2_OUTPUT_PIN_CONFIG  GPIO_2_EPWM2_A   // ePWM2A function

//*****************************************************************************
// Global Variables
//*****************************************************************************
uint16_t sensor1_raw = 0;      // Raw ADC value for sensor 1 (0-4095)
uint16_t sensor2_raw = 0;      // Raw ADC value for sensor 2 (0-4095)

float current_phase_a = 0.0f;  // Calculated current in Amps
float current_phase_b = 0.0f;  // Calculated current in Amps
float total_current = 0.0f;    // Total motor current

uint32_t sample_count = 0;     // Count of samples taken

//*****************************************************************************
// Function Prototypes
//*****************************************************************************
void InitEPWM1(void);          // 1kHz timer for ADC triggering
void InitEPWM2_10kHz(void);    // 10kHz PWM output signal
void InitADC(void);
void InitADCSOC(void);
void ConfigurePWMOutputGPIO(void);
__interrupt void ADCA1_ISR(void);

//*****************************************************************************
// Main Function
//*****************************************************************************
void main(void)
{
    //
    // Step 1: Initialize device clock and peripherals
    //
    Device_init();

    //
    // Step 2: Initialize GPIO
    //
    Device_initGPIO();

    //
    // Step 3: Configure GPIO for PWM output
    //
    ConfigurePWMOutputGPIO();

    //
    // Step 4: Initialize PIE (Peripheral Interrupt Expansion) and clear all flags
    //
    Interrupt_initModule();
    Interrupt_initVectorTable();

    //
    // Step 5: Initialize ADC
    //
    InitADC();

    //
    // Step 6: Initialize ePWM1 (1kHz timer for ADC)
    //
    InitEPWM1();

    //
    // Step 7: Initialize ePWM2 (10kHz PWM output for oscilloscope)
    //
    InitEPWM2_10kHz();

    //
    // Step 8: Configure ADC SOCs (Start of Conversion)
    //
    InitADCSOC();

    //
    // Step 9: Register the interrupt handler
    //
    Interrupt_register(INT_ADCA1, &ADCA1_ISR);

    //
    // Step 10: Enable ADC interrupt in PIE
    //
    Interrupt_enable(INT_ADCA1);

    //
    // Step 11: Enable global interrupts and real-time debug
    //
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    //
    // Step 12: Main loop - system runs here
    //
    while(1)
    {
        // Main loop can do other tasks
        // - ePWM1 (1kHz) triggers ADC every 1ms AND outputs pulse to GPIO0
        // - ePWM2 (10kHz) generates continuous PWM output on GPIO2
        // - Both signals visible on oscilloscope (2 channels)
        
        // Optional: Add your background tasks here
        // Examples:
        // - Communication protocols (CAN, UART)
        // - State machine logic
        // - Display updates
        // - Safety monitoring

        NOP; // No operation - just wait for interrupts
    }
}

//*****************************************************************************
// ConfigurePWMOutputGPIO - Setup GPIO pins for both PWM outputs
//*****************************************************************************
void ConfigurePWMOutputGPIO(void)
{
    //
    // Configure GPIO0 as ePWM1A output (1kHz timer signal for oscilloscope)
    //
    GPIO_setPadConfig(PWM1_OUTPUT_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(PWM1_OUTPUT_PIN_CONFIG);
    
    //
    // Configure GPIO2 as ePWM2A output (10kHz PWM signal)
    //
    GPIO_setPadConfig(PWM2_OUTPUT_GPIO, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(PWM2_OUTPUT_PIN_CONFIG);
}

//*****************************************************************************
// InitEPWM1 - Configure ePWM1 for 1ms period (ADC trigger + GPIO output)
// Outputs a pulse waveform to GPIO0 to visualize timer on oscilloscope
//*****************************************************************************
void InitEPWM1(void)
{
    //
    // Enable ePWM1 clock
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);

    //
    // Disable ePWM1 clock to TBCLK (configure while stopped)
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Set Time Base Period Register for 1kHz (1ms)
    // TBPRD = (SYSCLK / Prescaler / Desired_Frequency) / 2
    // TBPRD = (120,000,000 / 1 / 1000) / 2 = 60,000
    //
    EPWM_setTimeBasePeriod(EPWM1_BASE, 60000);

    //
    // Set Time Base Counter to 0
    //
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0);

    //
    // Set Time Base Clock Prescaler to divide by 1 (full 120 MHz)
    //
    EPWM_setClockPrescaler(EPWM1_BASE,
                          EPWM_CLOCK_DIVIDER_1,    // CLKDIV = /1
                          EPWM_HSCLOCK_DIVIDER_1); // HSPCLKDIV = /1

    //
    // Set Count Mode to Up-Down (symmetrical)
    //
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    //
    // Set Phase to 0
    //
    EPWM_setPhaseShift(EPWM1_BASE, 0);

    //
    // Disable phase loading (free-running)
    //
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);

    //
    // Set Emulation Mode (free run - continue in debug mode)
    //
    EPWM_setEmulationMode(EPWM1_BASE, EPWM_EMULATION_FREE_RUN);

    //
    // Create a narrow pulse at the ADC trigger point
    // This pulse shows when ADC sampling occurs
    // Pulse occurs at TBPRD (counter peak) when ADC is triggered
    //
    EPWM_setCounterCompareValue(EPWM1_BASE, 
                                EPWM_COUNTER_COMPARE_A, 
                                59000);  // Pulse near the peak

    //
    // Set PWM Action Qualifier to create a pulse at trigger point
    // Set output HIGH when counter = CMPA going UP
    // Set output LOW when counter hits PERIOD (at ADC trigger)
    //
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);

    //
    // Configure ADC SOC Trigger
    // Trigger SOCA when counter equals period (at the peak/center)
    // This generates a 1kHz trigger pulse for ADC sampling
    //
    EPWM_setADCTriggerSource(EPWM1_BASE,
                            EPWM_SOC_A,                    // SOCA signal
                            EPWM_SOC_TBCTR_PERIOD);        // Trigger at TBCTR = TBPRD

    //
    // Set ADC SOC trigger event prescaler (trigger every event)
    //
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);

    //
    // Enable ADC SOC A trigger
    //
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    //
    // Enable ePWM1 clock to TBCLK (start the timer)
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

//*****************************************************************************
// InitEPWM2_10kHz - Configure ePWM2 for 10kHz, 50% duty cycle OUTPUT
//*****************************************************************************
void InitEPWM2_10kHz(void)
{
    //
    // Enable ePWM2 clock
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);

    //
    // Stop TBCLK during configuration
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Set Period for 10 kHz in UP-DOWN mode
    // TBPRD = 6000  → 10 kHz
    //
    EPWM_setTimeBasePeriod(EPWM2_BASE, 6000);

    //
    // Reset counter
    //
    EPWM_setTimeBaseCounter(EPWM2_BASE, 0);

    //
    // Clock prescaler = 1 (full 120 MHz)
    //
    EPWM_setClockPrescaler(EPWM2_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // UP-DOWN counting mode
    //
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    //
    // No phase shift
    //
    EPWM_setPhaseShift(EPWM2_BASE, 0);
    EPWM_disablePhaseShiftLoad(EPWM2_BASE);

    //
    // CMPA for 50% duty cycle
    // In up-down mode: Duty = CMPA / TBPRD
    // So CMPA = TBPRD / 2 = 6000/2 = 3000
    //
    EPWM_setCounterCompareValue(EPWM2_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                3000);

    //
    // Action Qualifier (UP-DOWN mode)
    // Typical symmetric PWM:
    // - Clear output (LOW) when counter = CMPA going UP
    // - Set output (HIGH) when counter = CMPA going DOWN
    //
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    //
    // Free-run during debug
    //
    EPWM_setEmulationMode(EPWM2_BASE, EPWM_EMULATION_FREE_RUN);

    //
    // Start TBCLK
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

//*****************************************************************************
// InitADC - Power up ADC and configure resolution and signal mode
//*****************************************************************************
void InitADC(void)
{
    //
    // Initialize ADCA Module
    //
    // Set ADCCLK divider to /4 (SYSCLK = 120MHz, ADCCLK = 30MHz)
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);

    // Set pulse positions to late (recommended for 12-bit)
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);

    // Power up the ADCA
    ADC_enableConverter(ADCA_BASE);

    //
    // Delay for 1ms to allow ADC to power up
    // At 120 MHz, 1ms = 120,000 cycles
    //
    DEVICE_DELAY_US(1000);
}

//*****************************************************************************
// InitADCSOC - Configure ADC Start of Conversion channels
//*****************************************************************************
void InitADCSOC(void)
{
    //
    // Configure SOC0 for Sensor 1 (ADCA)
    //
    ADC_setupSOC(SENSOR1_ADC_MODULE,          // ADCA_BASE
                 SENSOR1_SOC_NUMBER,          // SOC0
                 ADC_TRIGGER_EPWM1_SOCA,      // Triggered by ePWM1 SOCA
                 SENSOR1_ADC_CHANNEL,         // ADCIN2
                 15);                         // Sample window = 15 cycles

    //
    // Configure SOC1 for Sensor 2 (ADCA)
    //
    ADC_setupSOC(SENSOR2_ADC_MODULE,          // ADCA_BASE (same module)
                 SENSOR2_SOC_NUMBER,          // SOC1 (different slot)
                 ADC_TRIGGER_EPWM1_SOCA,      // Triggered by ePWM1 SOCA
                 SENSOR2_ADC_CHANNEL,         // ADCIN14
                 15);                         // Sample window = 15 cycles

    //
    // Configure interrupt to trigger when SOC1 completes
    // This ensures both SOC0 and SOC1 have completed
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, SENSOR2_SOC_NUMBER);

    //
    // Enable continuous mode (interrupt every time SOC completes)
    //
    ADC_enableContinuousMode(ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Enable ADCA interrupt 1
    //
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Clear any pending interrupt flags
    //
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}

//*****************************************************************************
// ADCA1_ISR - ADC Interrupt Service Routine (runs every 1ms)
//*****************************************************************************
__interrupt void ADCA1_ISR(void)
{
    //
    // Step 1: Read ADC results from both sensors
    //
    sensor1_raw = ADC_readResult(SENSOR1_RESULT_BASE, SENSOR1_SOC_NUMBER);
    sensor2_raw = ADC_readResult(SENSOR2_RESULT_BASE, SENSOR2_SOC_NUMBER);

    //
    // Step 2: Convert ADC values to actual current
    //
    // Assumption: Current sensors output 0-3.3V for 0-100A range
    // ADC: 0-4095 represents 0-3.3V
    // Therefore: Current (A) = (ADC_value / 4095) * 100
    //
    // IMPORTANT: Adjust this formula based on YOUR sensor specifications!
    //
    current_phase_a = ((float)sensor1_raw / 4095.0f) * 100.0f;  // 0-100A
    current_phase_b = ((float)sensor2_raw / 4095.0f) * 100.0f;  // 0-100A

    //
    // Step 3: Calculate total motor current
    //
    total_current = current_phase_a + current_phase_b;

    //
    // Step 4: Optional - Process or store data
    //
    sample_count++;  // Increment sample counter

    // Add your custom processing here:
    // - Store in circular buffer for logging
    // - Calculate RMS current
    // - Implement over-current protection
    // - Update control algorithms
    // - Send data over communication bus

    //
    // Example: Simple over-current protection
    //
    #define MAX_CURRENT 150.0f  // Maximum allowed current in Amps

    if(total_current > MAX_CURRENT)
    {
        // SAFETY: Disable motor drive or take protective action
        // GPIO_writePin(MOTOR_ENABLE_PIN, 0);  // Disable motor
        // Set error flag
    }

    //
    // Step 5: Clear ADC interrupt flag
    //
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Step 6: Acknowledge interrupt to PIE
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//*****************************************************************************
// End of File
//*********************################################################################
