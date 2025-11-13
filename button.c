#include "button.h"
#include <inc/tm4c123gh6pm.h>
#include <stdbool.h> // why tf does "driverlib/interrupt.h" not compilable
#include "driverlib/interrupt.h"


volatile int button_data;

/**
 * Initialize PORTE and configure bits 0-3 to be used as inputs for the buttons.
 */
void button_init() {
	static uint8_t initialized = 0;

	//Check if already initialized
	if(initialized){
		return;
	}
	
	// Reading: To initialize and configure GPIO PORTE, visit pg. 656 in the 
	// Tiva datasheet.
	
	// Follow steps in 10.3 for initialization and configuration. Some steps 
	// have been outlined below.
	
	// Ignore all other steps in initialization and configuration that are not 
	// listed below. You will learn more about additional steps in a later lab.

	// 1) Turn on PORTE system clock, do not modify other clock enables
	SYSCTL_RCGCGPIO_R |= 0b00010000;

	// 2) Set the buttons as inputs, do not modify other PORTE wires
	GPIO_PORTE_DIR_R &= ~0b00001111;
	
	// 3) Enable digital functionality for button inputs, 
	//    do not modify other PORTE enables
	GPIO_PORTE_DEN_R |= 0b00001111;

	initialized = 1;
}


/**
 * Interrupt handler -- executes when a GPIO PortE hardware event occurs (i.e., for this lab a button is pressed)
 */
void gpioe_handler() {
    // update button_event = 1;
    button_data = ~(GPIO_PORTE_DATA_R & 0x0F);

    // Clear interrupt status register
    GPIO_PORTE_ICR_R |= 0x0F;
}


/**
 * Initialize and configure PORTE interupts
 */
void button_init_interrupts() {
    // In order to configure GPIO ports to detect interrupts, you will need to visit pg. 656 in the Tiva datasheet.
    // Notice that you already followed some steps in 10.3 for initialization and configuration of the GPIO ports in the function button_init().
    // Additional steps for setting up the GPIO port to detect interrupts have been outlined below.
    // TODO: Complete code below

    // 1) Mask the bits for pins 0-3
    GPIO_PORTE_IM_R &= ~0x0F;

    // 2) Set pins 0-3 to use edge sensing
    GPIO_PORTE_IS_R &= ~0x0F;

    // 3) Set pins 0-3 to use both edges. We want to update the LCD
    //    when a button is pressed, and when the button is released.
    GPIO_PORTE_IBE_R |= 0x0F;

    // 4) Clear the interrupts
    GPIO_PORTE_ICR_R |= 0x0F;

    // 5) Unmask the bits for pins 0-3
    GPIO_PORTE_IM_R |= 0x0F;

    // TODO: Complete code below
    // 6) Enable GPIO port E interrupt
    NVIC_EN0_R |= 0x10;

    // Bind the interrupt to the handler.
    IntRegister(INT_GPIOE, gpioe_handler);
}



/**
 * Returns the position of the rightmost button being pushed.
 * @return the position of the rightmost button being pushed. 4 is the rightmost button, 1 is the leftmost button.  0 indicates no button being pressed
 */
uint8_t button_getButton() {
    if (button_data & 8) return 4;
    if (button_data & 4) return 3;
    if (button_data & 2) return 2;
	if (button_data & 1) return 1;

	return 0;
}

uint8_t button_getButtons() {
    return ~(GPIO_PORTE_DATA_R & 0x0F);
}




