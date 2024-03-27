/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 * This code is for UART testing
 */

#include <asf.h>
#include <string.h>
#include "conf_clocks.h"
#include "delay.h"

#define UART_MODULE       SERCOM4
#define UART_SERCOM_MUX   USART_RX_3_TX_2_XCK_3
#define UART_SERCOM_GCLK  SERCOM4_GCLK_ID_CORE
#define BUFFER_SIZE 64

#define UART_TX_PIN PINMUX_PB10D_SERCOM4_PAD2 //SAM W25 TX line
#define UART_RX_PIN PINMUX_PB11D_SERCOM4_PAD3 //SAM W25 RX line

static uint8_t rx_buffer[BUFFER_SIZE];
static uint8_t tx_buffer[BUFFER_SIZE] = "UART!\r\n";

static struct usart_module usart_instance;
static int receive_flag = 0;

// UART read callback function (receive data)
static void usart_read_callback(struct usart_module *const module) {
	usart_read_buffer_job(&usart_instance, rx_buffer, BUFFER_SIZE);
	// Set receive flag while reading new data comes in
	if (port_pin_get_input_level(BUTTON_0_PIN) == !BUTTON_0_ACTIVE) receive_flag = 1;
}

// Configure UART communication
static void configure_uart(void) {
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	// Configure USART with the settings defined above
	config_usart.mux_setting = UART_SERCOM_MUX;
	config_usart.pinmux_pad0 = UART_TX_PIN; // TX pin
	config_usart.pinmux_pad1 = UART_RX_PIN; // RX pin

	// The program will run while(1) empty loop when initialization was unsuccessful
	while (usart_init(&usart_instance, UART_MODULE, &config_usart) != STATUS_OK) {} 
	usart_enable(&usart_instance);

	// Register and enable USART callback
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
	usart_register_callback(&usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	
	// Start reception
	usart_read_buffer_job(&usart_instance, rx_buffer, BUFFER_SIZE);
}


int main (void)
{
	// Initialize system and modules
	system_init();
	delay_init();
	configure_uart();

	usart_write_buffer_job(&usart_instance, tx_buffer, strlen((const char *)tx_buffer));

	while (1) {
		/* Is button pressed? */
		if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
			/* Yes, send UART data packet. */
			receive_flag = 0;
			usart_write_buffer_job(&usart_instance, tx_buffer, strlen((const char *)tx_buffer));
		}
		/*if the data has been received, blink the on board LED*/
		if (receive_flag == 1){
			port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
			delay_ms(200);
			port_pin_set_output_level(LED_0_PIN, !LED_0_ACTIVE);
			receive_flag = 0;
		}
	}
}



/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 * This code is for UART testing
 */

#include <asf.h>
#include <string.h>
#include "conf_clocks.h"
#include "delay.h"

#define UART_MODULE       SERCOM4
#define UART_SERCOM_MUX   USART_RX_3_TX_2_XCK_3
#define UART_SERCOM_GCLK  SERCOM4_GCLK_ID_CORE
#define BUFFER_SIZE 64

static uint8_t rx_buffer[BUFFER_SIZE];
static uint8_t tx_buffer[BUFFER_SIZE] = "UART!\r\n";

static struct usart_module usart_instance;
static int receive_flag = 0;

///////////////////////////
////UART
static void usart_read_callback(struct usart_module *const module) {
	// Reception completed
	usart_read_buffer_job(&usart_instance, rx_buffer, BUFFER_SIZE);
	if (port_pin_get_input_level(BUTTON_0_PIN) == !BUTTON_0_ACTIVE)
	{
	   receive_flag = 1;
	   }
	
}

static void configure_uart(void) {
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);

	config_usart.mux_setting = UART_SERCOM_MUX;
	config_usart.pinmux_pad0 = PINMUX_PB10D_SERCOM4_PAD2;
	config_usart.pinmux_pad1 = PINMUX_PB11D_SERCOM4_PAD3;

	while (usart_init(&usart_instance, UART_MODULE, &config_usart) != STATUS_OK) {}

	usart_enable(&usart_instance);

	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
	usart_register_callback(&usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_read_buffer_job(&usart_instance, rx_buffer, BUFFER_SIZE);
}

void configure_led(void) {
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	pin_conf.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &pin_conf);
}

/////////////////////////////
int main (void)
{
	system_init();
	configure_led();
	delay_init();
	configure_uart();

	usart_write_buffer_job(&usart_instance, tx_buffer, strlen((const char *)tx_buffer));

	while (1) {
		/* Is button pressed? */
		if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
			/* Yes, send UART data packet. */
			usart_write_buffer_job(&usart_instance, tx_buffer, strlen((const char *)tx_buffer));
		}
		/*if the data has been received, blink the on board LED*/
		if (receive_flag == 1){
			port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
			delay_ms(200);
			port_pin_set_output_level(LED_0_PIN, !LED_0_ACTIVE);
			receive_flag = 0;
		}
	}
}
