#ifdef ADC_H_
#define ADC_H_

void adc_init(void);
void battery_check_task(void);
float get_voltage(void);

#endif