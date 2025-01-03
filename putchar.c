#include <stdint.h>
#include <nrf.h>

// This function is called internally by the printf function for each character.
// Here we can redirect the printf output stream to whereever we want.
void _putchar(char character)
{
    // Enable the UART peripheral.
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos;
    // Write the character into the TXD register.
    NRF_UART0->TXD = character;
    // Start the UART transfer (i.e., trigger the UART start task).
    NRF_UART0->TASKS_STARTTX = UART_TASKS_STARTTX_TASKS_STARTTX_Trigger << UART_TASKS_STARTTX_TASKS_STARTTX_Pos;

    // Wait until the end of the transmission by checking the TXRDY event.
    while (NRF_UART0->EVENTS_TXDRDY == UART_EVENTS_TXDRDY_EVENTS_TXDRDY_NotGenerated);

    // Reset the event.
    NRF_UART0->EVENTS_TXDRDY = UART_EVENTS_TXDRDY_EVENTS_TXDRDY_NotGenerated << UART_EVENTS_TXDRDY_EVENTS_TXDRDY_Pos;
    // Stop the transmission by triggering the stop task.
    NRF_UART0->TASKS_STOPTX = UART_TASKS_STOPTX_TASKS_STOPTX_Trigger << UART_TASKS_STOPTX_TASKS_STOPTX_Pos;
    // Disable the UART peripheral.
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos;
}