/*
 * button.h
 *
 *  Created on: Jul 18, 2016
 *      Author: Eric Middleton
 *
 * @edit: Phillip Jones 05/30/2019 : Removed uneeded helper functions
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include <stdint.h>
#include <inc/tm4c123gh6pm.h>

// initialize the push buttons
void button_init();

// initialize the push buttons interrupts service
void button_init_interrupts();


// Non-blocking call
// Returns highest value button being pressed, 0 if no button pressed
uint8_t button_getButton();


// non-blocking call
// returns all the button data
uint8_t button_getButtons();


#endif /* BUTTON_H_ */
