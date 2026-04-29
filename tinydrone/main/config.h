#ifndef CONFIG_H_
#define CONFIG_H_

// KCONFIG notes
//
// generally configs can simply be found as defines in sdkconfig and
// are simply CONFIG_$NAME where the name is the config name in Kconfig
//
// to use simply use the defined name in sdkconfig.h
//
// Kconfig should be implemented on a per component base
// maybe use Kconfig.projbuild instead of Kconfig

// Should you use this file you need to add PRIV_REQUIRES to the CMakeLists.txt
// of the component when it is not already there

// General

// ADC Settings
//#define ADC_CHANNEL ADC1_GPIO35_CHANNEL
//
//#define BATTERY_GUARD CONFIG_BATTERY_GUARD //if battery voltage should be checked
//#define BATTERY_MIN 3.1f //minimum voltage (in V)
//#define CONTROL_GAP 5000 //time between the task checking the battery again (in ms)

#endif
