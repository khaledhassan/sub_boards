#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

//libopencm3 uses OR masks

uint8_t check_header_checksum(void);
uint8_t check_payload_checksum(void);


//STATES
//0 waiting for start byte 1
//1 waiting for start byte 2
//2 receiving destination ID
//3 receiving flags
//4 receiving header checksum
//5 receiving payload length
//6 receiving payload
//7 receiving payload checksum
int rx_state = 0;
uint8_t rx_address = 0;
uint8_t rx_flags = 0;
uint8_t rx_header_checksum = 0;
uint8_t my_address = 69;
uint8_t rx_payload_length = 0;
uint8_t rx_received_payload = 0;
uint8_t rx_payload[20] = {0};
uint8_t rx_payload_checksum = 0;

int main(void)
{
	//starting at the top of STM32F3 disovery LEDs and going clockwise
	//PE9 - red - LD3
	//PE10 - orange - LD5
	//PE11 - green - LD7
	//PE12 - blue - LD9
	//PE13 - red - LD10
	//PE14 - orange - LD8
	//PE15 - green - LD6
	//PE8 - blue - LD4

	//USART2TX = PA2
	//USART2RX = PA3
	//in this example we're using USART2 instead of USART1 because USART1 is taken by the st-link

	//Setup clock
	rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_64MHZ]);  //select 8MHz clock (I think, haven't figured this part out yet)

	//Enable GPIOE Clock
	rcc_periph_clock_enable(RCC_GPIOE); //you have to turn on clock for anything you want to use in STM32FX processors, here we want LEDs on port E

	//Enable GPIOA Clock (for USART)
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);

	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 | GPIO14 | GPIO15);
	//gpio_set(GPIOE, GPIO8 | GPIO10 | GPIO12 | GPIO14);

	nvic_enable_irq(NVIC_USART2_EXTI26_IRQ); //enable USART2 interrupt
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	usart_enable_rx_interrupt(USART2);

	usart_enable(USART2);

    while(1)
    {
    	int i;
    	gpio_toggle(GPIOE, GPIO15); //just to show that it was working
    	for(i = 0; i < 1000000; i++)
    		__asm__("nop");
    }
}

void usart2_exti26_isr(void)
{
	static uint8_t data = 'A';

	// Check if we were called because of RXNE.
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_ISR(USART2) & USART_ISR_RXNE) != 0)) {
		// Indicate that we got data.
		gpio_toggle(GPIOE, GPIO8); //RX led if available

		// Retrieve the data from the peripheral.
		data = usart_recv(USART2);
		if(data == 0xB)
			rx_state = 0;
		switch(rx_state)
		{
		case 0: //start byte 1
			if(data == '^') //start byte 1 received
				rx_state = 1;
		case 1: //start byte 2
			if(data == '^') //start byte 2 received
				rx_state = 2;
		case 2: //destination ID (address)
			if(rx_state != 0xB)
			{
				rx_address = data;
				rx_state = 3;
			}
			break;
		case 3: //flags
			rx_flags = data;
			rx_state = 4;
			break;
		case 4: //header_checksum
			rx_header_checksum = data;
			rx_state = 5;
			if(rx_address == my_address)
			{
				if(check_header_checksum())
					usart_enable_tx_interrupt(USART2);
				else
					rx_state = 0;
			}
			break;
		case 5: //payload length
			rx_payload_length = data;
			rx_received_payload = 0;
			rx_state = 6;
			break;
		case 6: //payload
			rx_payload[rx_received_payload++] = data;
			if(rx_received_payload == rx_payload_length)
				rx_state = 7;
			break;
		case 7: //payload checksum
			rx_payload_checksum = data;
			if(rx_address == my_address)
			{
				if(check_payload_checksum())
					usart_enable_tx_interrupt(USART2);
				else
					rx_state = 0; //kind of redundant
			}
			rx_state = 0;
			break;
		}
	}

	// Check if we were called because of TXE.
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
	    ((USART_ISR(USART2) & USART_ISR_TXE) != 0)) {

		gpio_toggle(GPIOE, GPIO9);
		// Put data into the transmit register.
		usart_send(USART2, data);

		// Disable the TXE interrupt as we don't need it anymore.
		usart_disable_tx_interrupt(USART2);
	}
}

uint8_t check_header_checksum()
{
	return 1; //check to see if header checksum == XOR of address and flags
}

uint8_t check_payload_checksum()
{
	return 1; //check to see if payload checksum == XOR of payload, payload length
}
