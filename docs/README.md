# Project Documentation – Hotzenblitz EV Re-Engineering

This folder contains technical reports and measurement documentation
related to the embedded firmware developed for the Hotzenblitz EV
Re-Engineering Project.

---

## 📘 Reports

### Task 1 – ePWM Triggered ADC Sampling (1 ms)
- File: `task1_adc_epwm_report.pdf`
- Description:
  - ePWM1 configured in up-down counter mode
  - ADC triggered every 1 ms using hardware SOC
  - Explanation of why TBPRD = 0.5 ms results in 1 ms period
  - Oscilloscope validation

---

### Task 2 – PWM Signal Generation (10 kHz)
- File: `task2_pwm_output_report.pdf`
- Description:
  - Independent PWM generation using ePWM2
  - 10 kHz frequency, 50% duty cycle
  - Simultaneous ADC sampling and PWM output

---

### Task 2 (i) – Synchronized PWM and ADC
- File: `task2_synced_pwm_report.pdf`
- Description:
  - ePWM1 and ePWM2 synchronized using TBCLKSYNC
  - ADC sampling aligned to PWM timing
  - Multi-channel oscilloscope timing analysis

---

## 📊 Timing Diagrams
The `timing_diagrams/` folder contains oscilloscope screenshots used
to validate ADC sampling and PWM synchronization.


