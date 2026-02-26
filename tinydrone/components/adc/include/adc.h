#ifdef ADC_H_
#define ADC_H_

void adc_init(void); //call only once else mem leak
void battery_check_task(void); //call only once
float get_voltage(void);

#endif