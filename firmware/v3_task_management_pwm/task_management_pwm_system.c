//###########################################################################
//
// FILE:   task_management_pwm_system.c
//
// TITLE:  Task Management System with 6-Channel Synchronized PWM Control
//
// DESCRIPTION:
//   - Task management system with IDLE and RUN modes
//   - 6 independent PWM outputs (ePWM1, 2, 6, 7, 4, 5) with synchronized operation
//   - Configurable switching frequency: 50 Hz to 50 kHz (example: 10 kHz)
//   - Configurable duty cycle: 0% to 100% (example: 50%)
//   - All PWM signals share same frequency and duty cycle
//   - Asynchronous task execution in infinite loop
//   - F28003x running at 120 MHz
//
//###########################################################################

#include "driverlib.h"
#include "device.h"

//*****************************************************************************
// System Configuration
//*****************************************************************************
#define SYSCLK_FREQ         120000000UL  // 120 MHz system clock

// PWM Frequency Limits
#define MIN_SWITCHING_FREQ  50           // Minimum 50 Hz
#define MAX_SWITCHING_FREQ  50000        // Maximum 50 kHz
#define DEFAULT_FREQ        10000        // Default 10 kHz

// Duty Cycle Limits
#define MIN_DUTY_CYCLE      0.0f         // 0%
#define MAX_DUTY_CYCLE      100.0f       // 100%
#define INITIAL_DUTY_CYCLE  0.0f         // Start at 0%
//*****************************************************************************
// IDLE → RUN Timing Measurement
//*****************************************************************************
volatile uint32_t idleStartCount = 0;
volatile uint32_t idleEndCount   = 0;
volatile float idleToRunTime_us  = 0.0f;

//*****************************************************************************
// PWM Output Pin Configuration - F28003x LaunchPad
//*****************************************************************************
// PWM1A → GPIO0  (J4 Pin 40)
// PWM2A → GPIO2  (J4 Pin 38)
// PWM6A → GPIO10 (J4 Pin 30)
// PWM7A → GPIO12 (J4 Pin 28)
// PWM4A → GPIO6  (J4 Pin 34)
// PWM5A → GPIO16 (J2 Pin 35)

typedef struct {
    uint32_t base;           // ePWM base address
    uint32_t gpio;           // GPIO pin number
    uint32_t pinConfig;      // GPIO pin config
} PWM_Channel_t;

// Define all 6 PWM channels - Updated configuration
const PWM_Channel_t PWM_Channels[6] = {
    {EPWM1_BASE, 0,  GPIO_0_EPWM1_A},
    {EPWM2_BASE, 2,  GPIO_2_EPWM2_A},
    {EPWM6_BASE, 10, GPIO_10_EPWM6_A},
    {EPWM7_BASE, 12, GPIO_12_EPWM7_A},
    {EPWM4_BASE, 6,  GPIO_6_EPWM4_A},
    {EPWM5_BASE, 16, GPIO_16_EPWM5_A}
};

//*****************************************************************************
// System State Machine
//*****************************************************************************
typedef enum {
    STATE_IDLE = 0,          // Initialization mode - no PWM output
    STATE_RUN = 1            // Running mode - PWM active
} SystemState_t;

//*****************************************************************************
// Global Variables
//*****************************************************************************
volatile SystemState_t systemState = STATE_IDLE;

// PWM Configuration
uint32_t switchingFrequency = DEFAULT_FREQ;    // Hz (10 kHz default)
float dutyCycle = INITIAL_DUTY_CYCLE;          // % (0% initially)
uint16_t pwmPeriod = 0;                        // Calculated TBPRD value
uint16_t pwmCompare = 0;                       // Calculated CMPA value

// Task flags
volatile bool initComplete = false;
volatile bool pwmConfigured = false;

//*****************************************************************************
// Function Prototypes
//*****************************************************************************
void TaskManager_Init(void);
void TaskManager_Run(void);
void System_StateIdle(void);
void System_StateRun(void);

void PWM_ConfigureAllChannels(uint32_t frequency);
void PWM_ConfigureSingleChannel(uint8_t channel);
void PWM_SetDutyCycle(float duty);
void PWM_StartAll(void);
void PWM_StopAll(void);
void PWM_DisableOutputs(void);

void GPIO_ConfigurePWMPins(void);
uint16_t PWM_CalculatePeriod(uint32_t frequency);
uint16_t PWM_CalculateCompare(uint16_t period, float duty);

//*****************************************************************************
// Main Function
//*****************************************************************************
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();
    Device_initGPIO();

    //
    // Initialize task management system
    //
    TaskManager_Init();

    //
    // Task Management System - Asynchronous Infinite Loop
    //
    while(1)
    {
        TaskManager_Run();
    }
}

//*****************************************************************************
// TaskManager_Init - Initialize the task management system
//*****************************************************************************
void TaskManager_Init(void)
{
    systemState = STATE_IDLE;
    initComplete = false;
    pwmConfigured = false;

    GPIO_ConfigurePWMPins();

    // -----------------------------
    // CPU TIMER 0 CONFIGURATION
    // -----------------------------
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);   // No prescaler
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_startTimer(CPUTIMER0_BASE);
}


//*****************************************************************************
// TaskManager_Run - Main task execution loop (asynchronous)
//*****************************************************************************
void TaskManager_Run(void)
{
    switch(systemState)
    {
        case STATE_IDLE:
            System_StateIdle();
            break;

        case STATE_RUN:
            System_StateRun();
            break;

        default:
            // Safety fallback
            systemState = STATE_IDLE;
            break;
    }
}

//*****************************************************************************
// System_StateIdle - IDLE mode operations
// - Perform all initializations
// - Configure PWM (but don't start)
// - NO PWM signals generated
//*****************************************************************************
void System_StateIdle(void)
{
    if(!initComplete)
    {
        // Capture IDLE entry time
        idleStartCount = CPUTimer_getTimerCount(CPUTIMER0_BASE);
        //
        // Step 1: Configure switching frequency (example: 10 kHz)
        //
        switchingFrequency = DEFAULT_FREQ;

        //
        // Step 2: Initialize duty cycle to 0%
        //
        dutyCycle = INITIAL_DUTY_CYCLE;

        //
        // Step 3: Calculate PWM timing parameters
        //
        pwmPeriod = PWM_CalculatePeriod(switchingFrequency);
        pwmCompare = PWM_CalculateCompare(pwmPeriod, dutyCycle);

        //
        // Step 4: Configure all 6 PWM channels
        //
        PWM_ConfigureAllChannels(switchingFrequency);

        //
        // Step 5: Ensure PWM outputs are disabled
        //
        PWM_DisableOutputs();

        //
        // Mark initialization complete
        //
        initComplete = true;
        pwmConfigured = true;

        //
        // Transition to RUN state
        // PWM will be started in RUN mode, not in IDLE
        //
        systemState = STATE_RUN;
    }
}

//*****************************************************************************
// System_StateRun - RUN mode operations
// - PWM signals are active
// - Duty cycle can be changed dynamically
//*****************************************************************************
void System_StateRun(void)
{
    static bool firstEntry = true;

    if(firstEntry)
    {
        // Capture RUN entry time
        idleEndCount = CPUTimer_getTimerCount(CPUTIMER0_BASE);

        // Calculate time difference
        uint32_t cycles = idleStartCount - idleEndCount;
        idleToRunTime_us = cycles / 120.0f;  // 120 MHz → microseconds

        PWM_StartAll();

        dutyCycle = 50.0f;
        PWM_SetDutyCycle(dutyCycle);

        firstEntry = false;
    }

    NOP;
}


//*****************************************************************************
// GPIO_ConfigurePWMPins - Configure all 6 GPIO pins for PWM output
//*****************************************************************************
void GPIO_ConfigurePWMPins(void)
{
    uint8_t i;

    for(i = 0; i < 6; i++)
    {
        GPIO_setPadConfig(PWM_Channels[i].gpio, GPIO_PIN_TYPE_STD);
        GPIO_setPinConfig(PWM_Channels[i].pinConfig);
    }
}

//*****************************************************************************
// PWM_ConfigureAllChannels - Configure all 6 PWM channels
//*****************************************************************************
void PWM_ConfigureAllChannels(uint32_t frequency)
{
    uint8_t i;

    //
    // Enable all ePWM clocks - Updated for ePWM1, 2, 6, 7, 4, 5
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM7);

    //
    // Disable TBCLK during configuration
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Configure each PWM channel identically
    //
    for(i = 0; i < 6; i++)
    {
        PWM_ConfigureSingleChannel(i);
    }

    //
    // Note: TBCLKSYNC will be enabled in PWM_StartAll()
    //
}

//*****************************************************************************
// PWM_ConfigureSingleChannel - Configure individual PWM channel
//*****************************************************************************
void PWM_ConfigureSingleChannel(uint8_t channel)
{
    uint32_t base = PWM_Channels[channel].base;

    //
    // Set Time Base Period (same for all channels)
    //
    EPWM_setTimeBasePeriod(base, pwmPeriod);

    //
    // Reset counter to 0
    //
    EPWM_setTimeBaseCounter(base, 0);

    //
    // Set clock prescaler to 1 (full 120 MHz)
    //
    EPWM_setClockPrescaler(base,
                          EPWM_CLOCK_DIVIDER_1,
                          EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set count mode to UP-DOWN (symmetrical PWM)
    //
    EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP_DOWN);

    //
    // Set phase shift to 0 (all channels synchronized)
    //
    EPWM_setPhaseShift(base, 0);
    EPWM_disablePhaseShiftLoad(base);

    //
    // Set Compare A value (duty cycle)
    //
    EPWM_setCounterCompareValue(base,
                                EPWM_COUNTER_COMPARE_A,
                                pwmCompare);

    //
    // Configure Action Qualifier for PWM output
    // Clear on CMPA going UP, Set on CMPA going DOWN
    //
    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_setActionQualifierAction(base,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    //
    // Set emulation mode (free run during debug)
    //
    EPWM_setEmulationMode(base, EPWM_EMULATION_FREE_RUN);
}

//*****************************************************************************
// PWM_SetDutyCycle - Update duty cycle for all PWM channels
// duty: 0.0 to 100.0 (percentage)
//*****************************************************************************
void PWM_SetDutyCycle(float duty)
{
    uint8_t i;

    //
    // Clamp duty cycle to valid range
    //
    if(duty < MIN_DUTY_CYCLE)
        duty = MIN_DUTY_CYCLE;
    if(duty > MAX_DUTY_CYCLE)
        duty = MAX_DUTY_CYCLE;

    //
    // Update global duty cycle
    //
    dutyCycle = duty;

    //
    // Calculate new compare value
    //
    pwmCompare = PWM_CalculateCompare(pwmPeriod, duty);

    //
    // Update all PWM channels
    //
    for(i = 0; i < 6; i++)
    {
        EPWM_setCounterCompareValue(PWM_Channels[i].base,
                                    EPWM_COUNTER_COMPARE_A,
                                    pwmCompare);
    }
}

//*****************************************************************************
// PWM_StartAll - Start all PWM channels synchronously
//*****************************************************************************
void PWM_StartAll(void)
{
    uint8_t i;

    //
    // Reset all counters to 0 for perfect synchronization
    //
    for(i = 0; i < 6; i++)
    {
        EPWM_setTimeBaseCounter(PWM_Channels[i].base, 0);
    }

    //
    // Enable TBCLK to start all PWMs simultaneously
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

//*****************************************************************************
// PWM_StopAll - Stop all PWM channels
//*****************************************************************************
void PWM_StopAll(void)
{
    //
    // Disable TBCLK to stop all PWM timers
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Disable outputs
    //
    PWM_DisableOutputs();
}

//*****************************************************************************
// PWM_DisableOutputs - Force all PWM outputs LOW
//*****************************************************************************
void PWM_DisableOutputs(void)
{
    uint8_t i;

    for(i = 0; i < 6; i++)
    {
        //
        // Force output LOW using Action Qualifier
        //
        EPWM_setActionQualifierAction(PWM_Channels[i].base,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    }
}

//*****************************************************************************
// PWM_CalculatePeriod - Calculate TBPRD value for given frequency
// frequency: Switching frequency in Hz
// Returns: TBPRD value
//*****************************************************************************
uint16_t PWM_CalculatePeriod(uint32_t frequency)
{
    uint32_t period;

    //
    // Validate frequency range
    //
    if(frequency < MIN_SWITCHING_FREQ)
        frequency = MIN_SWITCHING_FREQ;
    if(frequency > MAX_SWITCHING_FREQ)
        frequency = MAX_SWITCHING_FREQ;

    //
    // Calculate TBPRD for UP-DOWN mode
    // TBPRD = (SYSCLK / (frequency * 2))
    //
    // For 10 kHz: TBPRD = 120,000,000 / (10,000 * 2) = 6000
    //
    period = SYSCLK_FREQ / (frequency * 2);

    return (uint16_t)period;
}

//*****************************************************************************
// PWM_CalculateCompare - Calculate CMPA value for given duty cycle
// period: TBPRD value
// duty: Duty cycle in percentage (0-100)
// Returns: CMPA value
//*****************************************************************************
uint16_t PWM_CalculateCompare(uint16_t period, float duty)
{
    uint32_t compare;

    //
    // Clamp duty cycle
    //
    if(duty < MIN_DUTY_CYCLE)
        duty = MIN_DUTY_CYCLE;
    if(duty > MAX_DUTY_CYCLE)
        duty = MAX_DUTY_CYCLE;

    //
    // Calculate CMPA value
    // CMPA = TBPRD * (duty / 100)
    //
    // For 50% duty with TBPRD=6000: CMPA = 6000 * 0.5 = 3000
    //
    compare = (uint32_t)((float)period * (duty / 100.0f));

    return (uint16_t)compare;
}

//*****************************************************************************
// End of File
//*****************************************************************************
