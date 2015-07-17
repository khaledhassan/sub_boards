#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "Queue.hpp"

//libopencm3 uses OR masks

//STATES
//0 waiting for start byte 1
//1 waiting for start byte 2
//2 receiving destination ID
//3 receiving flags
//4 receiving header checksum
//5 receiving payload length
//6 receiving payload
//7 receiving payload checksum
typedef enum {
	START_BYTE1,
	START_BYTE2,
	DEST_ID,
	FLAGS,
	HEADER_CHECK,
	PAYLOAD_LEN,
	PAYLOAD,
	PAYLOAD_CHECK
} packet_state_t;

const uint8_t MAX_PAYLOAD_LEN = 20;

typedef struct {
	uint8_t dest_address = 0;
	uint8_t flags = 0;
	uint8_t header_checksum = 0;
	uint8_t payload_length = 0;
	uint8_t payload[MAX_PAYLOAD_LEN] = {0};
	uint8_t payload_checksum = 0;
} packet_t;

packet_state_t rx_state = START_BYTE1;
packet_state_t tx_state = START_BYTE1;

Queue<packet_t, 3> rx_packet_queue;
Queue<packet_t, 3> tx_packet_queue;

const uint8_t MY_ADDRESS = 69;
const uint8_t MASTER_ADDRESS = 42;

bool check_header_checksum(packet_t& pkt)
{
	return (pkt.dest_address ^ pkt.flags) == pkt.header_checksum; //check to see if header checksum == XOR of address and flags
}

bool check_payload_checksum(packet_t& pkt)
{
	uint8_t checksum = pkt.payload_length;
	for (uint8_t i = 0; i != pkt.payload_length; ++i) {
		checksum ^= pkt.payload[i];
	}

	return checksum == pkt.payload_checksum; //check to see if payload checksum == XOR of payload, payload length
}

uint8_t calculate_header_checksum(packet_t& pkt)
{
	return pkt.dest_address ^ pkt.flags;
}

uint8_t calculate_payload_checksum(packet_t& pkt)
{
	uint8_t checksum = pkt.payload_length;
	for (uint8_t i = 0; i != pkt.payload_length; ++i) {
		checksum ^= pkt.payload[i];
	}

	return checksum;
}

void zeroize_packet(packet_t& pkt)
{
	pkt.dest_address = 0;
	pkt.flags = 0;
	pkt.header_checksum = 0;
	pkt.payload_length = 0;
	pkt.payload[MAX_PAYLOAD_LEN] = {0};
	pkt.payload_checksum = 0;
}

static void clock_setup(void)
{
    rcc_clock_setup_in_hsi_out_48mhz();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART1);
}

static void gpio_setup(void)
{
    /* Set RS485 DE/RE to 'output push-pull' and low-level (receiver enabled). */
    gpio_clear(GPIOA, (GPIO4));
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, (GPIO4));

    /* Setup GPIO pins for USART1. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9); //TX
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10); //RX

    /* Setup USART1 TX/RX pins as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO10);

    /* Set HARDKILL (2)/SOFTKILL (3) to input. */
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, (GPIO2 | GPIO3));

	/* Set CTRL_Blue and CTRL_Green to 'output push-pull' for debug
	 * CTRL_Blue will toggle on packet processing, CTRL_Green will toggle on SysTick */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, (GPIO1 | GPIO0));
}

static void usart_setup(void)
{
    /* Setup USART1 parameters. */
    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	usart_enable_rx_interrupt(USART1);

    /* Finally enable the USART. */
    usart_enable(USART1);
}

/* Called when systick fires */
void sys_tick_handler(void)
{
	static uint32_t hardkill_count = 0;
	static uint32_t softkill_count = 0;
	// XXX: increment a counter if switch is low, once count over a threshold, toggle a boolean value. if switch is high, reset count
	gpio_toggle(GPIOA, GPIO1);
}

/* Set up timer to fire freq times per second */
static void systick_setup(int freq)
{
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	/* clear counter so it starts right away */
	STK_CVR = 0;

	systick_set_reload(rcc_ahb_frequency / freq);
	systick_counter_enable();
	systick_interrupt_enable();
}

int main(void)
{
	clock_setup();
	systick_setup(100);
	gpio_setup();
	usart_setup();

    while(1)
    {
		if (!rx_packet_queue.isEmpty()) {
			packet_t pkt = rx_packet_queue.dequeue();
			if (pkt.payload[0] == 'b') {
				uint8_t button_gpio = gpio_get(GPIOA, (GPIO1 | GPIO0));
				// XXX: READ SWITCHES, SEND NEW PACKET
			}
			gpio_toggle(GPIOA, GPIO0); //just to show that it was working -- CTRL_Blue

		}
		//for(int i = 0; i < 1000000; i++)
		//	__asm__("nop");
    }
	return 0;
}

void usart1_isr(void)
{
    static uint8_t rx_pos = 0;
    static uint8_t tx_pos = 0;
    static packet_t rx_pkt;
    static packet_t tx_pkt;

	static uint8_t data = 'A';

	// Check if we were called because of RXNE.
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
		((USART_ISR(USART1) & USART_ISR_RXNE) != 0)) {
		// Retrieve the data from the peripheral.
		data = usart_recv(USART1);
		switch(rx_state)
		{
			case START_BYTE1: //start byte 1
				if (data == '^') //start byte 1 received
				{
					rx_state = START_BYTE2;
					zeroize_packet(rx_pkt);
					rx_pos = 0;
				}
				break;
			case START_BYTE2: //start byte 2
				if (data == '^') //start byte 2 received
					rx_state = DEST_ID;
				else
					rx_state = START_BYTE1; // didn't get two in a row, so start over
				break;
			case DEST_ID: //destination ID (address)
				rx_pkt.dest_address = data;
				rx_state = FLAGS;
				break;
			case FLAGS: //flags
				rx_pkt.flags = data;
				rx_state = HEADER_CHECK;
				break;
			case HEADER_CHECK: //header_checksum
				rx_pkt.header_checksum = data;
				if (check_header_checksum(rx_pkt)) // if good checksum
				{
					rx_state = PAYLOAD_LEN;
				} else {
					rx_state = START_BYTE1;
				}
				break;
			case PAYLOAD_LEN: //payload length
				rx_pkt.payload_length = data;
				if (rx_pkt.payload_length > MAX_PAYLOAD_LEN) {
					rx_state = START_BYTE1; // payload too big!
				} else {
					rx_state = PAYLOAD;
				}
				break;
			case PAYLOAD: //payload
				rx_pkt.payload[rx_pos++] = data;
				if (rx_pos == rx_pkt.payload_length)
					rx_state = PAYLOAD_CHECK;
				break;
			case PAYLOAD_CHECK: //payload checksum
				rx_pkt.payload_checksum = data;
				if (rx_pkt.dest_address == MY_ADDRESS)
				{
					if (check_payload_checksum(rx_pkt))
						rx_packet_queue.enqueue(rx_pkt);
				}
				rx_state = START_BYTE1;
				break;
		}
	}

	// Check if we were called because of TXE.
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_ISR(USART1) & USART_ISR_TXE) != 0)) {

		switch(tx_state) {
			case START_BYTE1: //start byte 1
				if (tx_packet_queue.isEmpty()) {
					usart_disable_tx_interrupt(USART1);
					gpio_clear(GPIOA, (GPIO4)); // set DE/~RE to enable receiver
					break;
				} else {
					tx_pkt = tx_packet_queue.dequeue();
				}
				usart_send(USART1, '^');
				tx_state = START_BYTE2;
				tx_pos = 0;
				break;
			case START_BYTE2: //start byte 2
				usart_send(USART1, '^');
				tx_state = DEST_ID;
				break;
			case DEST_ID: //destination ID (address)
				usart_send(USART1, tx_pkt.dest_address);
				tx_state = FLAGS;
				break;
			case FLAGS: //flags
				usart_send(USART1, tx_pkt.flags);
				tx_state = HEADER_CHECK;
				break;
			case HEADER_CHECK: //header_checksum
				usart_send(USART1, calculate_header_checksum(tx_pkt));
				tx_state = PAYLOAD_LEN;
				break;
			case PAYLOAD_LEN: //payload length
				usart_send(USART1, tx_pkt.payload_length);
				tx_state = PAYLOAD;
				break;
			case PAYLOAD: //payload
				usart_send(USART1, tx_pkt.payload[tx_pos++]);
				if (tx_pos == tx_pkt.payload_length)
					tx_state = PAYLOAD_CHECK;
				break;
			case PAYLOAD_CHECK: //payload checksum
				usart_send(USART1, calculate_payload_checksum(tx_pkt));
				tx_state = START_BYTE1;
				break;
		}
		// Put data into the transmit register.
		usart_send(USART1, data);

		// Disable the TXE interrupt as we don't need it anymore.
		usart_disable_tx_interrupt(USART1);
	}
}
