#include <stdint.h>
#include <nrf.h>
#include <string.h>
#include <inttypes.h>
#include "printf.h"

// Forward declarations.
static void twi_write(uint8_t dev_addr, volatile uint8_t *tx_data, unsigned int n_tx_data);
static void twi_read(uint8_t dev_addr, volatile uint8_t *rx_buffer, unsigned int n_rx_data);
static void twi_write_read(uint8_t dev_addr, volatile uint8_t *tx_data, unsigned int n_tx_data, volatile uint8_t *rx_buffer, unsigned int n_rx_data);
static void wait_ms(uint16_t ms);
static void configure_timer(void);
static void configure_uart(void);

// Defines.
#define TICKS_PER_MS 63             // rounded (62,5)
#define DEV_ADDR UINT8_C(0x1D)      // device address according to datasheet
#define REG_CHIP_ID UINT8_C(0x0D)   // WHO_AM_I Register Address
#define REG_CONTROL UINT8_C(0x2A)   // Control register (0x2A)
#define MODE_INACTIVE UINT8_C(0x00) // Control mode inactive
#define MODE_ACTIVE UINT8_C(0x01)   // Control mode active
#define REG_CONFIG UINT8_C(0x0E)    // Configuration register (0x0E)
#define RANGE_2G UINT8_C(0x00)      // Set range +/- 2g (0x00)
#define REG_OUT_X_MSB UINT8_C(0x01) // Register address of OUT_X_MSB (0x01)

// Read and write buffers.
volatile static uint8_t write_buffer[2];
volatile static uint8_t read_buffer[6];

int main(void)
{
    // Configure the UART peripheral for printf.
    configure_uart();
    // Configure the TIMER peripheral for the wait_ms() function and an interrupt
    // every 0.5 seconds.
    configure_timer();

    //*************************************************************************
    //* I2C / TWI configuration.
    //*************************************************************************

    // Configure the PSEL.SCL register.
    NRF_TWIM0->PSEL.SCL = (27 << TWIM_PSEL_SCL_PIN_Pos) |
                          (0 << TWIM_PSEL_SCL_PORT_Pos) |
                          (TWIM_PSEL_SCL_CONNECT_Connected << TWIM_PSEL_SCL_CONNECT_Pos);
    // Configure the PSEL.SDA register.
    NRF_TWIM0->PSEL.SDA = (26 << TWIM_PSEL_SDA_PIN_Pos) |
                          (0 << TWIM_PSEL_SDA_PORT_Pos) |
                          (TWIM_PSEL_SDA_CONNECT_Connected << TWIM_PSEL_SDA_CONNECT_Pos);
    // Configure the FREQUENCY register.
    NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K250 << TWIM_FREQUENCY_FREQUENCY_Pos;

    // Configure the GPIO pins used with the TWIM peripheral according to Table 124, page 473.
    NRF_P0->PIN_CNF[27] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                          (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                          (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
                          (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    NRF_P0->PIN_CNF[26] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                          (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
                          (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
                          (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    // Store the address of the control register in our TX buffer.
    write_buffer[0] = REG_CONTROL;
    // Store the value representing the active operation mode in our TX buffer.
    write_buffer[1] = MODE_ACTIVE;
    // Write the buffer contents to the accelerometer.
    twi_write(DEV_ADDR, write_buffer, 2);

    // Store the address of the configuration register in our TX buffer.
    write_buffer[0] = REG_CONFIG;
    // Store the value representing the +/- 2g range in our TX buffer.
    write_buffer[1] = RANGE_2G;
    // Write the buffer contents to the accelerometer.
    twi_write(DEV_ADDR, write_buffer, 2);

    // Run forever.
    while (1)
    {
        // Go into a low power state (i.e., sleep) until an interrupt occurs
        // (WFI = "wait for interrupt" is a processor instruction).
        __WFI();
    }
}

// This is the interrupt service routine (ISR or IRQ) that is executed when a
// timer event occurs and interrupts are enables.
void TIMER1_IRQHandler(void)
{
    // Check if our specific event triggered the interrupt.
    if ((NRF_TIMER1->EVENTS_COMPARE[0] & TIMER_EVENTS_COMPARE_EVENTS_COMPARE_Msk) == TIMER_EVENTS_COMPARE_EVENTS_COMPARE_Generated)
    {
        // Clear interrupt event.
        NRF_TIMER1->EVENTS_COMPARE[0] = TIMER_EVENTS_COMPARE_EVENTS_COMPARE_NotGenerated << TIMER_EVENTS_COMPARE_EVENTS_COMPARE_Pos;

        // Store the address of the acceleration data (starts at register OUT_X_MSB)
        // in our TX buffer.
        write_buffer[0] = REG_OUT_X_MSB;

        // Write to the accelerometer in order to setup the read transfer.
        // Start a read transfer to receive 6 bytes of acceleration data.
        twi_write_read(DEV_ADDR, write_buffer, 1, read_buffer, 6);

        // Combine LSB and MSB sensor values for the X axis.
        uint16_t xval = read_buffer[0] << 4 | read_buffer[1] >> 4;
        int16_t accX = (xval > 2047) ? xval - 4096 : xval;

        // Combine LSB and MSB sensor values for the Y axis.
        uint16_t yval = read_buffer[2] << 4 | read_buffer[3] >> 4;
        int16_t accY = (yval > 2047) ? yval - 4096 : yval;

        // Combine LSB and MSB sensor values for the Z axis.
        uint16_t zval = read_buffer[4] << 4 | read_buffer[5] >> 4;
        int16_t accZ = (zval > 2047) ? zval - 4096 : zval;

        // Print acceleration values.
        printf("accX = %d   accY = %d   accZ = %d\n", accX, accY, accZ);

        // Stores last orientation.
        // 1: X UP, 2: X DOWN, 3: Y UP, 4: Y DOWN, 5: Z UP, 6: Z DOWN
        static uint8_t state;

        // Compute the current orientation.
        if (accX > 490)
        {
            if (state != 1)
            {
                state = 1;
                printf("X axis is UP!\n");
            }
        }
        else if (accX < -490)
        {
            if (state != 2)
            {
                state = 2;
                printf("X axis is DOWN!\n");
            }
        }
        else if (accY > 490)
        {
            if (state != 3)
            {
                state = 3;
                printf("Y axis is UP!\n");
            }
        }
        else if (accY < -490)
        {
            if (state != 4)
            {
                state = 4;
                printf("Y axis is DOWN!\n");
            }
        }
        else if (accZ > 490)
        {
            if (state != 5)
            {
                state = 5;
                printf("Z axis is UP!\n");
            }
        }
        else if (accZ < -490)
        {
            if (state != 6)
            {
                state = 6;
                printf("Z axis is DOWN!\n");
            }
        }

        // Clear to ensure the timer counter is 0.
        NRF_TIMER1->TASKS_CLEAR = TIMER_TASKS_CLEAR_TASKS_CLEAR_Trigger << TIMER_TASKS_CLEAR_TASKS_CLEAR_Pos;
        // Update the target count value to trigger the interrupt again after 0.5 seconds.
        NRF_TIMER1->CC[0] = 31250;
    }
}

// Write n_data bytes of the buffer pointed to by data to the slave device with
// the address dev_addr.
static void twi_write(uint8_t dev_addr, volatile uint8_t *tx_data, unsigned int n_tx_data)
{
    // Enable the TWIM peripheral.
    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;
    // Set the shortcut to stop transmitting after the last byte.
    NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTTX_STOP_Enabled << TWIM_SHORTS_LASTTX_STOP_Pos;
    // Set the device address in the ADDRESS register.
    NRF_TWIM0->ADDRESS = dev_addr;
    // Set the number of bytes we want to transmit.
    NRF_TWIM0->TXD.MAXCNT = n_tx_data;
    // Provide a pointer to a buffer where the transmit data is stored.
    NRF_TWIM0->TXD.PTR = (uint32_t)tx_data;
    // Clear events.
    NRF_TWIM0->EVENTS_STOPPED = TWIM_EVENTS_STOPPED_EVENTS_STOPPED_NotGenerated << TWIM_EVENTS_STOPPED_EVENTS_STOPPED_Pos;
    NRF_TWIM0->EVENTS_ERROR = TWIM_EVENTS_ERROR_EVENTS_ERROR_NotGenerated << TWIM_EVENTS_ERROR_EVENTS_ERROR_Pos;

    // Start transmit task.
    NRF_TWIM0->TASKS_STARTTX = TWIM_TASKS_STARTTX_TASKS_STARTTX_Trigger << TWIM_TASKS_STARTTX_TASKS_STARTTX_Pos;

    // Wait until the TWIM peripheral has stopped (STOP event after the last byte).
    while (NRF_TWIM0->EVENTS_STOPPED != TWIM_EVENTS_STOPPED_EVENTS_STOPPED_Generated);

    // Disable the TWIM peripheral.
    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
}

// Read n_data bytes from the slave device with the address dev_addr and store the
// data at the location pointed to by buffer.
static void twi_read(uint8_t dev_addr, volatile uint8_t *rx_buffer, unsigned int n_rx_data)
{
    // Enable the TWIM peripheral.
    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;
    // Set the shortcut to stop receiving after the last byte.
    NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos;
    // Set the device address in the ADDRESS register.
    NRF_TWIM0->ADDRESS = dev_addr;
    // Set the number of bytes we want to receive.
    NRF_TWIM0->RXD.MAXCNT = n_rx_data;
    // Provide a pointer to a buffer where the received data can be stored.
    NRF_TWIM0->RXD.PTR = (uint32_t)rx_buffer;
    // Clear events.
    NRF_TWIM0->EVENTS_STOPPED = TWIM_EVENTS_STOPPED_EVENTS_STOPPED_NotGenerated << TWIM_EVENTS_STOPPED_EVENTS_STOPPED_Pos;
    NRF_TWIM0->EVENTS_ERROR = TWIM_EVENTS_ERROR_EVENTS_ERROR_NotGenerated << TWIM_EVENTS_ERROR_EVENTS_ERROR_Pos;

    // Start receive task.
    NRF_TWIM0->TASKS_STARTRX = TWIM_TASKS_STARTRX_TASKS_STARTRX_Trigger << TWIM_TASKS_STARTRX_TASKS_STARTRX_Pos;

    // Wait until the TWIM peripheral has stopped (STOP event after the last byte).
    while (NRF_TWIM0->EVENTS_STOPPED != TWIM_EVENTS_STOPPED_EVENTS_STOPPED_Generated);

    // Disable the TWIM peripheral.
    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
}

// Write n_data bytes of the buffer pointed to by data to the slave device with
// the address dev_addr without sending the stop condition.
// Then read n_data bytes from the slave device with the address dev_addr and store the
// data at the location pointed to by buffer.
static void twi_write_read(uint8_t dev_addr, volatile uint8_t *tx_data, unsigned int n_tx_data, volatile uint8_t *rx_buffer, unsigned int n_rx_data)
{
    // Enable the TWIM peripheral.
    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;
    // Set the shortcut to start receiving after the last byte is transmitted
    // and to stop receiving after the last byte.
    NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTTX_STARTRX_Enabled << TWIM_SHORTS_LASTTX_STARTRX_Pos |
                        TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos;
    // Set the device address in the ADDRESS register.
    NRF_TWIM0->ADDRESS = dev_addr;
    // Set the number of bytes we want to transmit.
    NRF_TWIM0->TXD.MAXCNT = n_tx_data;
    // Provide a pointer to a buffer where the transmit data is stored.
    NRF_TWIM0->TXD.PTR = (uint32_t)tx_data;
    // Set the number of bytes we want to receive.
    NRF_TWIM0->RXD.MAXCNT = n_rx_data;
    // Provide a pointer to a buffer where the received data can be stored.
    NRF_TWIM0->RXD.PTR = (uint32_t)rx_buffer;
    // Clear events.
    NRF_TWIM0->EVENTS_STOPPED = TWIM_EVENTS_STOPPED_EVENTS_STOPPED_NotGenerated << TWIM_EVENTS_STOPPED_EVENTS_STOPPED_Pos;
    NRF_TWIM0->EVENTS_ERROR = TWIM_EVENTS_ERROR_EVENTS_ERROR_NotGenerated << TWIM_EVENTS_ERROR_EVENTS_ERROR_Pos;

    // Start transmit task.
    NRF_TWIM0->TASKS_STARTTX = TWIM_TASKS_STARTTX_TASKS_STARTTX_Trigger << TWIM_TASKS_STARTTX_TASKS_STARTTX_Pos;

    // Wait until the TWIM peripheral has stopped (STOP event after the last byte).
    while (NRF_TWIM0->EVENTS_STOPPED != TWIM_EVENTS_STOPPED_EVENTS_STOPPED_Generated);

    // Disable the TWIM peripheral.
    NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
}

//*************************************************************************
//* UART configuration.
//* 	- TX GPIO pin connection: P0.06
//* 	- baudrate: 115200 Baud (bit/s)
//* 	- hardware flow control: disabled
//* 	- stop bit(s): 1
//* 	- with no parity
//*************************************************************************
static void configure_uart(void)
{
    NRF_UART0->PSEL.TXD = (6 << UART_PSEL_TXD_PIN_Pos) |
                          (0 << UART_PSEL_TXD_PORT_Pos) |
                          (UART_PSEL_TXD_CONNECT_Connected << UART_PSEL_TXD_CONNECT_Pos);
    NRF_UART0->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud115200 << UART_BAUDRATE_BAUDRATE_Pos;
    NRF_UART0->CONFIG = (UART_CONFIG_HWFC_Disabled << UART_CONFIG_HWFC_Pos) |
                        (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos) |
                        (UART_CONFIG_STOP_One << UART_CONFIG_STOP_Pos);

    // Configure the TX GPIO pin according to Table 132, page 503.
    NRF_P0->PIN_CNF[6] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                         (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                         (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                         (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                         (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    // Set the output value of the GPIO pin (UART idle state).
    NRF_P0->OUTSET = GPIO_OUTSET_PIN6_Set << GPIO_OUTSET_PIN6_Pos;
}

//*************************************************************************
//* Timer0 configuration:
//* 	- frequency: 62.500 Hz
//* 	- bit width: 16 bit = 65.536
//
//* Timer1 configuration:
//* 	- frequency: 62.500 Hz
//* 	- bit width: 16 bit = 65.536
//*     - interrupts are enabled and trigger every 0.5 seconds
//*************************************************************************
static void configure_timer(void)
{
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER0->PRESCALER = 8 << TIMER_PRESCALER_PRESCALER_Pos; // f_TIMER = 62.5 kHz

    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER1->PRESCALER = 8 << TIMER_PRESCALER_PRESCALER_Pos; // f_TIMER = 62.5 kHz

    // Enable interrupts for timer events.
    NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos;
    // Enable interrupts for the timer peripheral in the interrupt controller.
    NVIC_EnableIRQ(TIMER1_IRQn);

    // Clear to ensure the timer counter is 0.
    NRF_TIMER1->TASKS_CLEAR = TIMER_TASKS_CLEAR_TASKS_CLEAR_Trigger << TIMER_TASKS_CLEAR_TASKS_CLEAR_Pos;

    // Set target count value (when to trigger the interrupt).
    NRF_TIMER1->CC[0] = 31250; // 500ms

    // Start timer.
    NRF_TIMER1->TASKS_START = TIMER_TASKS_START_TASKS_START_Trigger << TIMER_TASKS_START_TASKS_START_Pos;
}

// Delays the execution between 1 and 999 ms.
static void wait_ms(uint16_t ms)
{
    if (ms > 1000)
        return;

    // Clear and start timer.
    NRF_TIMER0->TASKS_CLEAR = TIMER_TASKS_CLEAR_TASKS_CLEAR_Trigger;
    NRF_TIMER0->TASKS_START = TIMER_TASKS_START_TASKS_START_Trigger;

    // Set target counter value.
    uint16_t counter = 0;
    uint16_t target_count = TICKS_PER_MS * ms;

    // Check timer counter and wait until target_count value ticks have passed.
    while (counter < target_count)
    {
        NRF_TIMER0->TASKS_CAPTURE[0] = TIMER_TASKS_CAPTURE_TASKS_CAPTURE_Trigger;
        counter = (uint16_t)NRF_TIMER0->CC[0];
    }

    // Stop timer.
    NRF_TIMER0->TASKS_STOP = TIMER_TASKS_STOP_TASKS_STOP_Trigger;
}
