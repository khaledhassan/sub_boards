
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

static void clock_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USART1);
}

static void gpio_setup(void)
{
    /* Set all actuator pins low */
    gpio_clear(GPIOA, (GPIO4 | GPIO5 | GPIO6 | GPIO7)); // A1, A2, A3, A4
    gpio_clear(GPIOB, (GPIO1 | GPIO2)); // A5, A6

    /* Set actuator pins to 'output push-pull'. */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, (GPIO4 | GPIO5 | GPIO6 | GPIO7)); // A1, A2, A3, A4
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, (GPIO1 | GPIO2)); // A5, A6

    /* Set USART1_TX_EN to 'output push-pull' and low-level (false). */
    gpio_clear(GPIOB, (GPIO12));
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, (GPIO12));

    /* Setup GPIO pins for USART1. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);

    /* Setup USART1 TX/RX pins as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF1, GPIO9);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO10);
}

static void usart_setup(void)
{
	/* Setup USART1 parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_CR2_STOP_1_0BIT);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

uint8_t read(void)
{
    uint8_t b1 = usart_recv_blocking(USART1);
    while (1) {
        uint8_t b2 = usart_recv_blocking(USART1);
        if (b2 == (b1 ^ 0xff)) {
            return b1;
        }
        b1 = b2;
    }
}

void write(uint8_t data)
{
    gpio_set(GPIOB, (GPIO12)); // USART_TX_EN true

    usart_send_blocking(USART1, data);
    usart_send_blocking(USART1, data ^ 0xFF);

    /* Flush USART1 RX buffer (RS485 RX always true, so we'd receive what we send) */
    usart_recv(USART1);
    usart_recv(USART1);
    usart_recv(USART1);

    gpio_clear(GPIOB, (GPIO12)); // USART_TX_EN false
}

void set_valves(uint8_t valves){
    for (uint8_t i = 0; i != 6; ++i) {
        uint8_t valve = valves & (1 << i);
        switch (i) {
            case 0:
                if (valve) {
                    gpio_set(GPIOA, GPIO4);
                } else {
                    gpio_clear(GPIOA, GPIO4);
                }
                break;
            case 1:
                if (valve) {
                    gpio_set(GPIOA, GPIO5);
                } else {
                    gpio_clear(GPIOA, GPIO5);
                }
                break;
            case 2:
                if (valve) {
                    gpio_set(GPIOA, GPIO6);
                } else {
                    gpio_clear(GPIOA, GPIO6);
                }
                break;
            case 3:
                if (valve) {
                    gpio_set(GPIOA, GPIO7);
                } else {
                    gpio_clear(GPIOA, GPIO7);
                }
                break;
            case 4:
                if (valve) {
                    gpio_set(GPIOB, GPIO1);
                } else {
                    gpio_clear(GPIOB, GPIO1);
                }
                break;
            case 5:
                if (valve) {
                    gpio_set(GPIOB, GPIO2);
                } else {
                    gpio_clear(GPIOB, GPIO2);
                }
                break;
            default:
                break;
        }
    }
}

int main(void)
{
    clock_setup();
	gpio_setup();
	usart_setup();

	while (1) {
        uint8_t cmd = read();
        if (cmd == 0x00) { // read switches
            continue;
        } else if ((cmd >= 0x80) && (cmd <= 0xBF)) { // set valves
            set_valves(cmd);
            write(cmd);
        } else if (cmd == 0x40) { // ping
            write(0x40);
        } else { // unknown command...
            continue;
        }
	}

	return 0;
}
