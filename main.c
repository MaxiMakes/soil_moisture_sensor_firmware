//#define F_CPU 32768L
/*  
    you might need to download the support pack for avr-gcc from http://packs.download.atmel.com/
    add include/avr/iotn402.h to /usr/avr/include/avr
    add gcc/dev/attiny402/device-specs/specs-attiny402 to /usr/lib/gcc/avr/<VERSION>/device-specs/
    add gcc/dev/attiny402/avrxmega3/short-calls/ to /usr/lib/avr/lib/  
*/  
//avrdude -c jtag2updi -p t402 -P /dev/ttyUSB0 -Uflash:w:soil-moisture-code.hex
#define USART_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)



#include <avr/io.h> 
#include <util/delay.h>

uint16_t a_val = 1;

void init(void){
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCULP32K_gc;
    CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_64X_gc;
    
    
    PORTA.DIR &= ~PIN2_bm;  //DI: input
    PORTA.DIR |= PIN6_bm;   //SENSOR-IN: output
    PORTA.OUT |= PIN6_bm; 
    PORTA.DIR &= ~PIN7_bm;  //SENSOR-OUT: input
    //Disable digital input buffer on PA7
    PORTA.PIN7CTRL |= 0x4;
    //Disable digital input buffer on PA3
    //PORTA.PIN3CTRL |= 0x4; 
    //Set USART to PA1 and PA2
    PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc;
    //Set Baudrate 
    //USART0.BAUD = 64;
    USART0.BAUD = (uint16_t)USART_BAUD_RATE(9600);
    //Set USART mode to IRCOM
    USART0.CTRLC |= USART_CMODE_IRCOM_gc;

    PORTA.DIR |= PIN1_bm;   //DO: output
    //Set TCB0 WO0 to PA6(SENSOR-IN) to PWM
    PORTMUX.CTRLD &= ~0b00000001; 


    //TODO enable USART0.CTRLB MPCM
    //TODO Set read IR-address from userfuses for MPCM
    //Turn on IRCOM Event
    USART0.EVCTRL |= USART_IREI_bm;
}

uint16_t readSensor(){
    //TODO set fast clockspeed
    //TODO Turn PWM on and wait a bit
    VREF.CTRLA  = VREF_ADC0REFSEL_4V34_gc;
    ADC0.CTRLB  = ADC_SAMPNUM_ACC8_gc;
    ADC0.CTRLC  &= ~0x3 << 4;//set internal reference
    ADC0.CTRLC  |= ADC_PRESC_DIV4_gc;
    ADC0.MUXPOS = ADC_MUXPOS_AIN7_gc;     //set AIN7 (PA7) as ADC0 input
    ADC0.CTRLA  |= ADC_ENABLE_bm;
    ADC0.COMMAND |= ADC_STCONV_bm;
    while(ADC0.COMMAND & 0x1);
    //TODO Turn PWM off
    //TODO set low clockspeed
    return ADC0.RES;
}

//TODO adapt to clock change
void generatePWM(uint8_t on){
    //CPU_CCP = CCP_IOREG_gc;
    //CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCULP32K_gc;
    //CPU_CCP = CCP_IOREG_gc;
    //CLKCTRL.MCLKCTRLB &= ~0x1;
    //CPU_CCP = CCP_IOREG_gc;
    //CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_10X_gc | 0x1;
    if(on){
        TCB0.CCMPL = 1;
        TCB0.CCMPH = 1;
        TCB0.CTRLB |= TCB_CCMPEN_bm;
        TCB0.CTRLB |= TCB_CNTMODE_PWM8_gc;
        //TCB0.CTRLA |= 0b00000011; //ENABLE + CLKDIV2
        TCB0.CTRLA |= 0b00000001; //ENABLE
    }
    else{
        TCB0.CTRLA &= ~0x1;
    }
    
}

void delay_us(uint16_t us){
    for(uint32_t i = 0; i < us; ++i)
        _delay_us(1);
}
/*TODO
char rx_data(){
    //TODO disable tx
    //TODO enable USART0.CTRLB SFDEN (start of frame detection)
    //TODO enable USART0.CTRLB MPCM
    while (!(USART1.STATUS & USART_RXCIF_bm))asm("nop;");
    return USART1.RXDATAL;
}
*/

void tx_data(){
    //TODO disable rx
    USART0.BAUD = (uint16_t)USART_BAUD_RATE(200);//50000;
    //USART0.CTRLC |= USART_CMODE_IRCOM_gc;
    PORTA.DIR |= PIN1_bm;   //DO: output
    USART0.CTRLB |= USART_TXEN_bm; // enable tx
    //USART0.CTRLB |= USART_ODME_bm; // enable open drain mode
    while(!(USART0.STATUS & USART_DREIF_bm))asm("nop;");
    USART0.TXDATAL = 0b10101010;
    //USART0.TXDATAH = 0b1;
    //USART0.TXDATAH = 0b10101010;
    //_delay_us(300);
    while(!(USART0.STATUS & USART_TXCIF_bm))asm("nop;");
    USART0.STATUS |= USART_TXCIF_bm;
}

int main(void){
    init();
    generatePWM(1);
    PORTA.DIR |= PIN3_bm;
    uint16_t baudrate =  USART0.BAUD;

    while(1){
        //if(baudrate >= 65535) baudrate = 64;
        //for(int i=0; i < 255; ++i) 
        generatePWM(0);
        tx_data();
        //baudrate += 1;
        //USART0.BAUD = baudrate;
        //_delay_ms(100);
        //generatePWM(1);
        /*
        a_val = readSensor();
        PORTA.OUTSET = PIN3_bm;
        delay_us(a_val);
        PORTA.OUTCLR = PIN3_bm;
        delay_us(a_val);
        */
        //generatePWM(0);
    }
    return 0;
}