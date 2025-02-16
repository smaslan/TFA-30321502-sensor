#ifndef MAIN_H
#define MAIN_H

// general macros
#define sbi(port,pin) {(port)|=(1<<pin);}
#define cbi(port,pin) {(port)&=~(1<<pin);}
#define min(a,b) ((a<b)?(a):(b))
#define max(a,b) ((a>b)?(a):(b))
#define bcopy(dreg,dpin,srreg,srpin) if(bit_is_set((srreg),(srpin))){sbi(dreg,dpin);}else{cbi(dreg,dpin);}
#define bcopy_v(dreg,dpin,srreg,srpin) if((srreg)&(1<<srpin)){sbi(dreg,dpin);}else{cbi(dreg,dpin);}
#define low(x) ((x) & 0xFFu)
#define high(x) (((x)>>8) & 0xFFu)

// I2C
#define I2C_port PORTC
#define I2C_ddr DDRC
#define I2C_pin PINC
#define I2C_SDA PC4
#define I2C_SCL PC5

// radio TX data
#define DATA_port PORTC
#define DATA_ddr DDRC
#define DATA PC3

// power enable (active low)
#define PWR_port PORTB
#define PWR_ddr DDRB
#define PWR PB1

// power monitor ADC
#define VMON_ADC_MUX 0x07 /* VMON adc channel mux input */

// status LED
#define LEDAR_port PORTD
#define LEDAR_ddr DDRD
#define LEDAR PD0
#define LEDAG_port PORTC
#define LEDAG_ddr DDRC
#define LEDAG PC1

// sync button
#define SYNC_port PORTB
#define SYNC_ddr DDRB
#define SYNC_pin PINB
#define SYNC PB2
#define SYNC_PCIE PCIE0
#define SYNC_PCMSK PCMSK0 
#define SYNC_ISR PCINT0_vect

// DIP: channel
#define CH_port PORTD
#define CH_ddr DDRD
#define CH_pin PIND
#define CH0 PD1
#define CH1 PD2
#define CH_msk 0x03

// DIP: address id
#define A_port PORTD
#define A_ddr DDRD
#define A_pin PIND
#define A0 PD3
#define A1 PD4
#define A2 PD5
#define A3 PD6
#define A_msk 0x0F

// DIP: speed
#define S0_port PORTD
#define S0_ddr DDRD
#define S0_pin PIND
#define S0 PD7
#define S1_port PORTB
#define S1_ddr DDRB
#define S1_pin PINB
#define S1 PB0


// ADC averaging
#define ADC_AVG 16 /* 10bit*64 to scale to full 16bit */


#define I2C_FREQ 100000 /* I2C bus clock rate [Hz] */

#define VMON_MIN 2.4 /* minimum battery voltage [V] */

#define TICK_REFCLK 32768.0 /* reference clock rate [Hz] */
#define TICK_PER 5.0 /* mean wakeup tick rate [s] */
#define TICK_PER_RAND 0.25 /* wakeup tick rate randomization [s] */





#endif
