/* Heat control daughter board for Raspberry Pi 

T0 on ADC2
T1 on ADC1
T2 on ADC0

Read request = receipt of key
Reply message = key, T0, T1, T2

*/

#define CPU ATmega168
#define TERMINAL_BAUD_RATE 38400

#include <inttypes.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
/* #include <util/delay_basic.h>*/
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

// message key
uint8_t key0 = 0xAA;
uint8_t key1 = 0x55;

/* ADC Routines */

unsigned int adc_data;

// ADC interrupt service routine
//  **cvavr** interrupt [ADC_INT] void adc_isr(void)
ISR(ADC_vect)
{
// Read the AD conversion result
adc_data=ADCW;
};

// Read the AD conversion result
// with noise canceling

#define ADC_VREF_TYPE 0xC0  // 2.56V reference

uint16_t read_adc(uint8_t adc_input)
{
  ADCSRB = 0x00;

  ADMUX=adc_input | (ADC_VREF_TYPE & 0xff);
  ADCSRA = (ADCSRA & 0xF8) | 0x06;  // Set prescaler to 64
  /* Enable ADC; Start Coversion; Enable Interrupt  */
  ADCSRA |= ((1<<ADEN) | (1<<ADIE));
  // Delay needed for the stabilization of the ADC input voltage
  _delay_us( 10);
  
  set_sleep_mode(SLEEP_MODE_ADC);
  sleep_enable();
  // Enter Sleep mode which triggers ADC
  // Wakes up on interrupt
  sleep_cpu();
  sleep_disable();
  return adc_data;
}

//  UART I/O

/* UART Control Register B Bits */

#define  RXCIE  7
#define  TXCIE  6
#define  UDRIE  5
#define  RXEN   4
#define  TXEN   3
#define  CHR9   2
#define  RXB8   1
#define  TXB8   0

/* UART Status Register A Bits */

#define RXC   7
#define TXC   6
#define UDRE  5
#define FE    4
#define OVR   3
#define UPE   2
#define U2X   1
#define MPCM  0

/* USART Control Register C Bit */

#define UMSEL1 7
#define UMSEL0 6
#define UPM1   5
#define UPM0   4
#define USBS   3
#define UCSZ1  2
#define UCSZ0  1
#define UCPOL  0

/* UART Buffer Defines */

#define UART_RX_BUFFER_SIZE  128  // Must be 2^n
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK)
#error RX buffer size is not a power of 2
#endif

#define UART_TX_BUFFER_SIZE  128  // Must be 2^n
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK)
#error TX buffer size is not a power of 2
#endif

/* Static Variables  for USART0 = terminal I/O */

static uint8_t UART_RxBuf0[UART_RX_BUFFER_SIZE];
static volatile uint8_t  UART_RxHead0;
static volatile uint8_t  UART_RxTail0;

static uint8_t UART_TxBuf0[UART_TX_BUFFER_SIZE];
static volatile uint8_t  UART_TxHead0;
static volatile uint8_t  UART_TxTail0;


/* Initialize UART0 */


void InitUART0(void){
  uint8_t x;
  #undef BAUD
  #define BAUD TERMINAL_BAUD_RATE
  #include <util/setbaud.h>
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;
  #if USE_2X
  UCSR0A |= (1 << U2X);
  #else
  UCSR0A &= ~(1 << U2X);
  #endif
  UCSR0C = 0x06;  //Async, no parity, 1 stop bit, 8 data bits
  UCSR0B = ( (1<<RXCIE) | (1<<RXEN) | (1<<TXEN) );
  x = 0;   // Flush the receive buffer
  UART_RxTail0 = x;
  UART_RxHead0 = x;
  UART_TxTail0 = x;
  UART_TxHead0 = x;
}

  /* UART0 Interrupt Handlers */

ISR(USART_RX_vect) {
  uint8_t data;
  uint8_t tmphead;
  data = UDR0;   //Read the received data
  // Calculate buffer index
  tmphead = (UART_RxHead0 + 1) & UART_RX_BUFFER_MASK;
  UART_RxHead0 = tmphead;  // Save the new index
  if (tmphead == UART_RxTail0) {
    /* Error! Receive buffer overflow */
  }
  UART_RxBuf0[tmphead] = data;  //Store the data in the buffer
}
ISR(USART_UDRE_vect){
  uint8_t tmptail;
  if (UART_TxHead0 != UART_TxTail0) {  // Check if all data is transmitted
    tmptail = (UART_TxTail0 +1) & UART_TX_BUFFER_MASK;
    UART_TxTail0 = tmptail;
    UDR0 = UART_TxBuf0[tmptail];  // Start sending next byte
  }
  else {
    UCSR0B &= ~(1<<UDRIE);  // Clear UDRE interrupt
  }
}

/* Read USART0 function */

uint8_t ReceiveByte0(void)
// uint8_t ReceiveByte0()
  {
  uint8_t tmptail;
  while (UART_RxHead0 == UART_RxTail0);  // Wait for incoming data
  tmptail = (UART_RxTail0 + 1) & UART_RX_BUFFER_MASK;
  UART_RxTail0 = tmptail;
  return UART_RxBuf0[tmptail];
}

/* Write USART0 function */

int16_t TransmitByte0(uint8_t data) 
// void TransmitByte0(uint8_t data)
  {
  uint8_t tmphead;
  tmphead = (UART_TxHead0 +1) & UART_TX_BUFFER_MASK;
  while (tmphead == UART_TxTail0);  // Wait for space in buffer
  UART_TxBuf0[tmphead] = data;
  UART_TxHead0 = tmphead;
  UCSR0B |= (1<<UDRIE);  // Enable UDRE interrupt
  return 0;
}

/* Data available USART0 function */

uint8_t DataInReceiveBuffer0(void) {
  return (UART_RxHead0 != UART_RxTail0);
}


uint8_t fetchByte(void){  

  while (!DataInReceiveBuffer0());
  return ReceiveByte0();
}


void waitForMeasurementRequest(void) {

  uint8_t c0 = 0;
  uint8_t c1 = 0;
  do {
    c0 = c1;
    c1 = fetchByte();
  } while ((c0 != key0) || (c1 != key1));
}

void sendReadings(void){

  uint16_t reading[3];
  uint8_t i;
  union byte_int{
    uint16_t i;
    uint8_t b[2];
  };
  union byte_int C;

  for (i = 0; i < 3; i++){
    reading[i] = read_adc(i);
  }
  TransmitByte0(key0);
  TransmitByte0(key1);
  for (i = 2; i >= 0; i--){
    C.i = reading[i];
    TransmitByte0(C.b[0]);
    TransmitByte0(C.b[1]);
  }
}



int main(void){

  /* Initialize System */


  // Disable the watchdog timer

  MCUSR = 0;
  wdt_reset();
  //wdt_disable();
  wdt_enable(WDTO_500MS);
  sei();
 
  InitUART0();

  sei();


  /*  Loop Forever */

  for (;;) {

    waitForMeasurementRequest();

    sendReadings();

  }
}
