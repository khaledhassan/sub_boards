
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#define PORT_LED GPIOB
#define PORT_KILL GPIOB
#define PIN_LED GPIO3
#define PIN_KILL GPIO2

static void clock_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART2);
}

static void gpio_setup(void)
{
    /* Set kill default false (active-low) */
    gpio_set(PORT_KILL, PIN_KILL);

    /* Set kill/LED pins to 'output push-pull'. */
    gpio_mode_setup(PORT_KILL, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_KILL);
    gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);

    /* Set RS485 DE/RE to 'output push-pull' and low-level (receiver enabled). */
    gpio_clear(GPIOA, (GPIO4));
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, (GPIO4));

    /* Setup GPIO pins for USART2. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2); //TX
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3); //RX

    /* Setup USART2 TX/RX pins as alternate function. */
    gpio_set_af(GPIOA, GPIO_AF1, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO3);
}

static void usart_setup(void)
{
    /* Setup USART2 parameters. */
    usart_set_baudrate(USART2, 115200);
    usart_set_databits(USART2, 8);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_stopbits(USART2, USART_CR2_STOP_1_0BIT);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART2);
}

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();

    while (1) {

    }


    return 0;
}
