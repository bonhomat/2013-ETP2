/**************************************************************************//**
 * @file temperature.h
 *
 * @brief temperature functions & defines
 *
 * @author bonhomat@students.zhaw.ch
 *         burrisim@students.zhaw.ch
 * @date   2013-05-18
 * @version 0.1
 *
 * @verbatim
 * temperature used defines, configuration and functions
 * @endverbatim 
 *******************************************************************************
 * @section License
 * <b>OpenSource GPL3</b>
 *******************************************************************************/

 
 
#ifndef __temperature_H
#define __temperature_H


/*******************************************************************************
 * Enums
 *******************************************************************************/

/*******************************************************************************
* @brief Definition of states for temperature sensor mode: active or shutdown
********************************************************************************/
typedef enum 
{
  T_SENS_ACTIVE   = 0,
  T_SENS_SHUTDOWN = 1
 } TempSensorMode_t;
 
 
/**************************************************************************//**
 * @brief Initialize clock system for temperature reading
 * 
 *****************************************************************************/
extern void InitClkTemperature();

/**************************************************************************//**
 * @brief Initialize USART1 for SPI communication 
 *****************************************************************************/
extern void InitUsart1SPI(void);

/**************************************************************************//**
 * @brief setup sensor for air temperature => continuous mode or shutdown
 *****************************************************************************/
extern void InitTSensor(TempSensorMode_t mode);

/**************************************************************************//**
 * @brief get emperature from external sensor
 * @return  temperature result struct
 * @note see datasheet of LM74CIM-3
 *****************************************************************************/
extern TempData_t getTemperature(void);



#endif
