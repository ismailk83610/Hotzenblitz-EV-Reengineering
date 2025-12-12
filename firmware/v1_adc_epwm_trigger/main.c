//###########################################################################
// FILE:   hotzenblitz_current_sensing.c
// TITLE:  Motor Current Sensing with ePWM Triggered ADC (1ms sampling)
//###########################################################################

#include "driverlib.h"
#include "driverlib/adc.h"
#include "device.h"

//*****************************************************************************
// Sensor Pin / SOC Configuration
//*****************************************************************************
#define SENSOR1_ADC_MODULE      ADCA_BASE
#define SENSOR1_ADC_CHANNEL     ADC_CH_ADCIN2
#define SENSOR1_RESULT_BASE     ADCARESULT_BASE
#define SENSOR1_SOC_NUMBER      ADC_SOC_NUMBER0

#define SENSOR2_ADC_MODULE      ADCA_BASE
#define SENSOR2_ADC_CHANNEL     ADC_CH_ADCIN14
#define SENSOR2_RESULT_BASE     ADCARESULT_BASE
#define SENSOR2_SOC_NUMBER      ADC_SOC_NUMBER1

//*****************************************************************************
// Global Variables
//*****************************************************************************
uint16_t sensor1_raw = 0;
uint16_t sensor2_raw = 0;

float current_phase_a = 0.0f;
float current_phase_b = 0.0f;
float total_current = 0.0f;

uint32_t sample_count = 0;

//*****************************************************************************
// Function Prototypes
//*****************************************************************************
void InitEPWM1(void);
void InitADC(void);
void InitADCSOC(void);
__interrupt void ADCA1_ISR(void);

//*****************************************************************************
// Main Function
//*****************************************************************************
void main(void)
{
    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();
    InitADC();
    InitEPWM1();
    InitADCSOC();
    Interrupt_register(INT_ADCA1, &ADCA1_ISR);
    Interrupt_enable(INT_ADCA1);
    EINT;
    ERTM;

    while(1)
    {
        NOP;
    }
}

//*****************************************************************************
// InitEPWM1 - Configure ePWM1 for triggering ADC
//*****************************************************************************
void InitEPWM1(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    EPWM_setTimeBasePeriod(EPWM1_BASE, 60000);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0);

    EPWM_setClockPrescaler(EPWM1_BASE,
                          EPWM_CLOCK_DIVIDER_1,
                          EPWM_HSCLOCK_DIVIDER_1);

    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_setPhaseShift(EPWM1_BASE, 0);
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);
    EPWM_setEmulationMode(EPWM1_BASE, EPWM_EMULATION_FREE_RUN);

    EPWM_setADCTriggerSource(EPWM1_BASE,
                            EPWM_SOC_A,
                            EPWM_SOC_TBCTR_PERIOD);

    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

//*****************************************************************************
// InitADC - Configure ADC module
//*****************************************************************************
void InitADC(void)
{
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);
}

//*****************************************************************************
// InitADCSOC - Configure Start Of Conversion
//*****************************************************************************
void InitADCSOC(void)
{
    ADC_setupSOC(SENSOR1_ADC_MODULE,
                 SENSOR1_SOC_NUMBER,
                 ADC_TRIGGER_EPWM1_SOCA,
                 SENSOR1_ADC_CHANNEL,
                 15);

    ADC_setupSOC(SENSOR2_ADC_MODULE,
                 SENSOR2_SOC_NUMBER,
                 ADC_TRIGGER_EPWM1_SOCA,
                 SENSOR2_ADC_CHANNEL,
                 15);

    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, SENSOR2_SOC_NUMBER);
    ADC_enableContinuousMode(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}

//*****************************************************************************
// ADCA1_ISR - ADC Interrupt Service Routine
//*****************************************************************************
__interrupt void ADCA1_ISR(void)
{
    sensor1_raw = ADC_readResult(SENSOR1_RESULT_BASE, SENSOR1_SOC_NUMBER);
    sensor2_raw = ADC_readResult(SENSOR2_RESULT_BASE, SENSOR2_SOC_NUMBER);

    current_phase_a = ((float)sensor1_raw / 4095.0f) * 100.0f;
    current_phase_b = ((float)sensor2_raw / 4095.0f) * 100.0f;
    total_current = current_phase_a + current_phase_b;

    sample_count++;

    #define MAX_CURRENT 150.0f
    if(total_current > MAX_CURRENT)
    {
    }

    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//*****************************************************************************
// End of File
//*****************************************************************************
