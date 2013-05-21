


#ifndef __ui_H
#define __ui_H


/**************************************************************************//**
 * @brief Enable the Buttons Interrupts
 * 
 *****************************************************************************/
extern void ButtonsEnable(void);


/**************************************************************************//**
 * @brief Disable the Buttons Interrupts
 * 
 *****************************************************************************/
extern void ButtonsDisable(void);


/**************************************************************************//**
 * @brief Initialize input and output ports for Buttons
 * 
 *****************************************************************************/
extern void InitButtons(void);


/**************************************************************************//**
 * @brief Button PB1 Pressed Routine
 * 
 *****************************************************************************/
extern void ButtonPB1pressed(void);




/******************************************************************************
 * @brief Initial settings for all states
 *
 *****************************************************************************/
extern void STATE_INITIALISER(void);


/******************************************************************************
 * @brief UI_Main handels main statemachine changes
 * 
 *****************************************************************************/
extern void UI_Main(void);



/*******************************************************************************
 **************************   INTERRUPT HANDLERS   *****************************
 ******************************************************************************/


/**************************************************************************//**
 * @brief Pushbutton 0 GPIO_EVEN_IRQHandler
 *
 *****************************************************************************/
extern void GPIO_EVEN_IRQHandler(void);

/******************************************************************************
 * @brief Pushbutton 1 GPIO_ODD_IRQHandler
 * 
 *****************************************************************************/
extern void GPIO_ODD_IRQHandler(void);



#endif