#define F_CPU 16000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h> // PSTR

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define  INPUT2(port,pin)   DDR ## port &= ~_BV(pin) 
#define OUTPUT2(port,pin)   DDR ## port |=  _BV(pin) 
#define  CLEAR2(port,pin)  PORT ## port &= ~_BV(pin) 
#define    SET2(port,pin)  PORT ## port |=  _BV(pin) 
#define   READ2(port,pin) ((PIN ## port & _BV(pin))?1:0)

#define  INPUT(x)  INPUT2(x) 
#define OUTPUT(x) OUTPUT2(x)
#define  CLEAR(x)  CLEAR2(x)
#define    SET(x)    SET2(x)
#define   READ(x)   READ2(x)
#define  WRITE(x,b) ((b)?(SET2(x)):(CLEAR2(x)))

#define SK6812_DATA_PIN        B,0
#define SHIFT_595_DATA_PIN     B,1
#define SHIFT_595_CLOCK_PIN    B,2
#define SHIFT_595_LATCH_PIN    B,3

// IN12b: 0 1 2 3 4 5 6 7 8 9 .
// IN15a: μ n % П k M m + - P nc
uint16_t nixie_pins[] = {(1<<8), (1<<11), (1<<9), (1<<3), (1<<4), (1<<5), (1<<0), (1<<7), (1<<2), (1<<6), (1<<10)};

void push_nixie_symbol(uint8_t i) {
    uint16_t data = nixie_pins[i];
    for (int8_t j=15; j>=0; j--) {
        CLEAR(SHIFT_595_CLOCK_PIN);
        _delay_us(10);
        if ((data>>j)&1) {
            SET(SHIFT_595_DATA_PIN);
        } else {
            CLEAR(SHIFT_595_DATA_PIN);
        }
        _delay_us(10);
        SET(SHIFT_595_CLOCK_PIN);
        _delay_us(10);
    }
}

void clock_nixie_latch() {
    SET(SHIFT_595_LATCH_PIN);
    _delay_us(10);
    CLEAR(SHIFT_595_LATCH_PIN);
    _delay_us(10);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void adc_init() {
    ADMUX = (1<<REFS0); // AREF = AVcc
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // ADC Enable and prescaler of 128
}

uint16_t adc_read(uint8_t ch) {
    ch &= 7;                     // prevent ch being >7
    ADMUX = (ADMUX & 0xF8) | ch; // clear 3 lower bits before ORing
    ADCSRA |= (1<<ADSC);         // start single convertion
    while (ADCSRA & (1<<ADSC));  // wait for the conversion to complete
    return ADC;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void uart_write(char x) {
    while ((UCSR0A & (1<<UDRE0))==0); // wait for empty receive buffer
    UDR0 = x; // send
}

uint8_t uart_char_is_waiting() { // returns 1 if a character is waiting, 0 otherwise
    return (UCSR0A & (1<<RXC0));
}

char uart_read() {
    while (!uart_char_is_waiting());
    char x = UDR0;
    return x;
}

int uart_putchar(char c, FILE *stream __attribute__((unused))) {
    uart_write(c);
    return 0;
}

int uart_getchar(FILE *stream __attribute__((unused))) {
    return uart_read();
}

void uart_init() {
    UBRR0H = 0;        // For divisors see table 19-12 in the atmega328p datasheet.
    UBRR0L = 16;       // U2X0, 16 -> 115.2k baud @ 16MHz. 
    UCSR0A = 1<<U2X0;  // U2X0, 207 -> 9600 baud @ 16Mhz.
    UCSR0B = 1<<TXEN0; // Enable  the transmitter. Reciever is disabled.
    UCSR0C = (1<<UDORD0) | (1<<UCPHA0);
    fdevopen(&uart_putchar, &uart_getchar);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  main loop  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define ASM_STRIP_PIN2(port,pin)  "I" (_SFR_IO_ADDR(PORT ## port)), "I" (pin)
#define ASM_STRIP_PIN(x) ASM_STRIP_PIN2(x)  

/** led_strip_write sends a series of colors to the LED strip, updating the LEDs.
 The colors parameter should point to an array of rgb_color structs that hold the colors to send.
 The count parameter is the number of colors to send.
 This function takes about 1.1 ms to update 30 LEDs.
 Interrupts must be disabled during that time, so any interrupt-based library
 can be negatively affected by this function.
 Timing details at 20 MHz (the numbers slightly different at 16 MHz and 8MHz):
  0 pulse  = 400 ns
  1 pulse  = 850 ns
  "period" = 1300 ns
 */
void __attribute__((noinline)) led_strip_write(uint8_t *colors, uint16_t count) {
    cli();   // Disable interrupts temporarily because we don't want our pulse timing to be messed up.
    while (count--) {
        // Send a color to the LED strip.
        // The assembly below also increments the 'colors' pointer,
        // it will be pointing to the next color at the end of this loop.
        asm volatile(
                "ld __tmp_reg__, %a0+\n"
                "rcall led_strip_send_byte%=\n"  // Send green component.
                "ld __tmp_reg__, %a0+\n"
                "rcall led_strip_send_byte%=\n"  // Send red component.
                "ld __tmp_reg__, %a0+\n"
                "rcall led_strip_send_byte%=\n"  // Send blue component.
                "rjmp led_strip_asm_end%=\n"     // Jump past the assembly subroutines.

                // led_strip_send_byte subroutine:  Sends a byte to the LED strip.
                "led_strip_send_byte%=:\n"
                "rcall led_strip_send_bit%=\n"  // Send most-significant bit (bit 7).
                "rcall led_strip_send_bit%=\n"
                "rcall led_strip_send_bit%=\n"
                "rcall led_strip_send_bit%=\n"
                "rcall led_strip_send_bit%=\n"
                "rcall led_strip_send_bit%=\n"
                "rcall led_strip_send_bit%=\n"
                "rcall led_strip_send_bit%=\n"  // Send least-significant bit (bit 0).
                "ret\n"

                // led_strip_send_bit subroutine:  Sends single bit to the LED strip by driving the data line
                // high for some time.  The amount of time the line is high depends on whether the bit is 0 or 1,
                // but this function always takes the same time (2 us).
                "led_strip_send_bit%=:\n"
                "sbi %2, %3\n"                           // Drive the line high.
                "rol __tmp_reg__\n"                      // Rotate left through carry.
                "nop\n" "nop\n"
                "brcs .+2\n" "cbi %2, %3\n"              // If the bit to send is 0, drive the line low now.
                "nop\n" "nop\n" "nop\n" "nop\n" "nop\n"
                "brcc .+2\n" "cbi %2, %3\n"              // If the bit to send is 1, drive the line low now.
                "ret\n"
                "led_strip_asm_end%=: "
                : "=b" (colors)
                : "0" (colors),         // %a0 points to the next color to display
                ASM_STRIP_PIN(SK6812_DATA_PIN) // %2 is the port register (e.g. PORTB), %3 is the pin number (0-8)
                       );
    }
    sei();          // Re-enable interrupts now that we are done.
    _delay_us(80);  // Send the reset signal.
}


#define LED_COUNT 12
uint8_t red[]   = {0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0};
uint8_t green[] = {16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0,16,0,0};
uint8_t gray[]  = {16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16};
uint8_t blue[] = {0,0,128,0,0,128,0,0,128,0,0,128,0,0,128,0,0,128,0,0,128,0,0,128,128,0,0,128,128,0,0,128,128,0,0,128,128,0,0,128};

uint8_t colors[]  = {128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128};

int main(void) {
    OUTPUT(SHIFT_595_DATA_PIN);
    OUTPUT(SHIFT_595_CLOCK_PIN);
    OUTPUT(SHIFT_595_LATCH_PIN);
    OUTPUT(SK6812_DATA_PIN);

    CLEAR(SHIFT_595_DATA_PIN);
    CLEAR(SHIFT_595_CLOCK_PIN);
    CLEAR(SHIFT_595_LATCH_PIN);
    CLEAR(SK6812_DATA_PIN);

    adc_init();
    uart_init();
    FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
    stdin = stdout = &uart_stream;

    while(1) {
#if 0
        for (uint8_t i=0; i<11; i++) {
            push_nixie_symbol(i);
            for (uint8_t j=0; j<LED_COUNT; j++) {
                if (j%4==0) {
                    colors[((i+j)*3  )%(LED_COUNT*3)] = red[0];
                    colors[((i+j)*3+1)%(LED_COUNT*3)] = red[1];
                    colors[((i+j)*3+2)%(LED_COUNT*3)] = red[2];
                } else if (j%4==1) {
                    colors[((i+j)*3  )%(LED_COUNT*3)] = green[0];
                    colors[((i+j)*3+1)%(LED_COUNT*3)] = green[1];
                    colors[((i+j)*3+2)%(LED_COUNT*3)] = green[2];
                } else if (j%4==2) {
                    colors[((i+j)*3  )%(LED_COUNT*3)] = blue[0];
                    colors[((i+j)*3+1)%(LED_COUNT*3)] = blue[1];
                    colors[((i+j)*3+2)%(LED_COUNT*3)] = blue[2];
                } else {
                    colors[((i+j)*3  )%(LED_COUNT*3)] = gray[0];
                    colors[((i+j)*3+1)%(LED_COUNT*3)] = gray[1];
                    colors[((i+j)*3+2)%(LED_COUNT*3)] = gray[2];
                }
            }
                led_strip_write(colors, LED_COUNT);
            /*
            if (i%3==0) {
                led_strip_write(green, LED_COUNT);
            } else if (i%3==1) {
                led_strip_write(red, LED_COUNT);
            } else {
                led_strip_write(gray, LED_COUNT);
            }
            */
            clock_nixie_latch();
            _delay_ms(1000);
        }
        continue;
#else
        uint16_t v0 = adc_read(2);
        uint16_t v1 = adc_read(1);
        uint16_t v2 = adc_read(0);
        int8_t t0 = v0<341 ? -1 : (v0>682 ? 1 : 0);
        int8_t t1 = v1<341 ? -1 : (v1>682 ? 1 : 0);
        int8_t t2 = v2<341 ? -1 : (v2>682 ? 1 : 0);
        int8_t value1 = t0+t1*3+t2*9;

        uint16_t v3 = adc_read(5);
        uint16_t v4 = adc_read(4);
        uint16_t v5 = adc_read(3);
        int8_t t3 = v3<341 ? -1 : (v3>682 ? 1 : 0);
        int8_t t4 = v4<341 ? -1 : (v4>682 ? 1 : 0);
        int8_t t5 = v5<341 ? -1 : (v5>682 ? 1 : 0);
        int8_t value2 = t3+t4*3+t5*9;

        uint8_t ns0 = abs(value1)%10;
        uint8_t ns1 = abs(value1)/10;
        if (!ns1) ns1 = 10;
        uint8_t ns2 = value1>0?7:(value1<0?8:10);

        uint8_t ns3 = abs(value2)%10;
        uint8_t ns4 = abs(value2)/10;
        if (!ns4) ns4 = 10;
        uint8_t ns5 = 9;//value2>0?7:(value2<0?8:10);

        uint8_t *c1 = value1>0 ? green : (value1<0 ? red : gray);
        uint8_t *c2 = value2>0 ? green : (value2<0 ? red : gray);

        for (int8_t i=0; i<LED_COUNT*3; i++) {
            colors[i] = (i<LED_COUNT*3/2 ? c1[i] : c2[i]);
        }

        led_strip_write(colors, LED_COUNT);

        push_nixie_symbol(ns3);
        push_nixie_symbol(ns4);
        push_nixie_symbol(ns5);
        push_nixie_symbol(ns0);
        push_nixie_symbol(ns1);
        push_nixie_symbol(ns2);
        clock_nixie_latch();

        fprintf_P(&uart_stream, PSTR("%d,%d,%d,%d, %d %d %d\r\n"), adc_read(0), adc_read(1), adc_read(2), value1, ns2, ns1, ns0);
        _delay_ms(100);
#endif
    }

    return 0;
}

