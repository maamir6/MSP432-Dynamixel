/*
 * Duplex.h
 *
 *  Created on: Nov 28, 2018
 *      Author: Mohammed
 */

#ifndef LIBRARIES_DUPLEX_H_
#define LIBRARIES_DUPLEX_H_

#include <stdint.h>

#include "driverlib.h"
#include "msp.h"

#define DXL_DIR_PIN_RX 0
#define DXL_DIR_PIN_TX 1

#define RX_DUPLEX_BUFFER_SIZE 512

const eUSCI_UART_Config uartConfig =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
    3, // BRDIV = 78
    4, // UCxBRF = 2
    2, // UCxBRS = 0
    EUSCI_A_UART_NO_PARITY, // No Parity
    EUSCI_A_UART_LSB_FIRST, // MSB First
    EUSCI_A_UART_ONE_STOP_BIT, // One stop bit
    EUSCI_A_UART_MODE, // UART mode
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // Oversampling
}; // set for dynamixel default of 57600


#ifdef __cplusplus

class Duplex
{
public:
    Duplex();
    virtual ~Duplex();

    void begin(void);
    uint16_t available(void);
    uint8_t read(void);
    void write(uint8_t data);
    void flush(void);



private:

    uint16_t* pRxDuplexBufferIndex;
    uint16_t* pRxDuplexBufferUnprocessed;
    uint8_t* pRxDuplexBuffer;

};

#endif /*end __cplusplus */

void pinMode(uint8_t dir_pin, uint8_t mode);
void digitalWrite(uint8_t dir_pin, uint8_t direction);
void EUSCIA1_IRQHandler(void);


#endif /* LIBRARIES_DUPLEX_H_ */
