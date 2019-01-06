/*
 * Duplex.cpp
 *
 *  Created on: Nov 28, 2018
 *      Author: Mohammed
 */

#include "Duplex.h"

uint8_t EUSCIA1_rxBuffer[RX_DUPLEX_BUFFER_SIZE];
uint16_t EUSCIA1_rxBufferIndex = 0;
uint16_t EUSCIA1_rxBufferUnprocessed = 0;

Duplex::Duplex()
{
    // TODO Auto-generated constructor stub

}

Duplex::~Duplex()
{
    // TODO Auto-generated destructor stub
}

void Duplex::begin(void)
{

    pRxDuplexBuffer = EUSCIA1_rxBuffer;
    pRxDuplexBufferIndex = &EUSCIA1_rxBufferIndex;
    pRxDuplexBufferUnprocessed = &EUSCIA1_rxBufferUnprocessed;

    Interrupt_disableMaster();

    // Start clock in main function

    /* For now, set both Tx and Rx to UART. @mohammed change Tx to input with PullUp for Rx */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN2 | GPIO_PIN3,
                                                   GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN5);

    UART_initModule(EUSCI_A1_BASE, &uartConfig);

    UART_enableModule(EUSCI_A1_BASE);

    UART_clearInterruptFlag(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT );

    UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    Interrupt_registerInterrupt(INT_EUSCIA1, EUSCIA1_IRQHandler);
    Interrupt_enableInterrupt(INT_EUSCIA1);
    Interrupt_enableMaster();

}

uint16_t Duplex::available(void)
{
    return *pRxDuplexBufferUnprocessed;
}

uint8_t Duplex::read(void)
{
    return pRxDuplexBuffer[*pRxDuplexBufferIndex - (*pRxDuplexBufferUnprocessed)--];
}

void Duplex::write(uint8_t data) {

    UART_clearInterruptFlag(EUSCI_A1_BASE, EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT);
    UART_transmitData(EUSCI_A1_BASE, data);

    while(!(UCA1IFG & EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT)){
        __no_operation();
    }  // ensure byte is sent
}

void Duplex::flush(void)
{
    /* Wait for dynamixel to send stuff  */
    int jj = 5000;
    while (jj > 0) { jj--;/* wait for UART interrupt handler to send shit */ }
}


void pinMode(uint8_t dir_pin, uint8_t mode) {
    // Do nothing for the one time this fucking thing is called
    __no_operation();
}

void digitalWrite(uint8_t dir_pin, uint8_t direction) {

    if (direction == DXL_DIR_PIN_TX) {
        // @working GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5);

        //GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN2);
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN3,
                                                       GPIO_PRIMARY_MODULE_FUNCTION);
        Interrupt_disableInterrupt(INT_EUSCIA1);



    } else if (direction == DXL_DIR_PIN_RX) {

        // @working GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5);

        EUSCIA1_rxBufferIndex = 0;

        GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN2,
                                                       GPIO_PRIMARY_MODULE_FUNCTION);
        UART_receiveData(EUSCI_A1_BASE);
        Interrupt_enableInterrupt(INT_EUSCIA1);

    }
}

void EUSCIA1_IRQHandler(void) {

 int status = UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

     UART_clearInterruptFlag(EUSCI_A1_BASE, status);

     if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG) {

         EUSCIA1_rxBuffer[EUSCIA1_rxBufferIndex++] = UART_receiveData(EUSCI_A1_BASE);
         EUSCIA1_rxBufferUnprocessed++;

     }
}


