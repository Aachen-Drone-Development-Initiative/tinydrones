#ifdef ADC_H_
#define ADC_H_

void adc_init(void); //call only (dont know what happens but guard does exist)
void battery_check_task(void); //call as task
float get_voltage(void); //in V

#endif