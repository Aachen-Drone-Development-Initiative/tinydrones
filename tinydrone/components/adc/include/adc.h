#ifndef ADC_H_
#define ADC_H_


/*
 * initializes adc unit & channel for battery voltage
 * should only be called once
 */
void adc_init(void); 

/*
 * Entry point for the battery checking task
 * call something like 
 * `xTaskCreate(battery_check_task, "BATTERY CHECK TASK", 512, 10, &xHandle);`
 *
 * This is only an example task creation call
 */ 
void battery_check_task(void); //call as task

/*
 * returns battery voltage in V
 */
float get_battery_voltage(void); //in V

#endif
