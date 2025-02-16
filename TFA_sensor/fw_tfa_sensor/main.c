/*
 * TFA Dostmann 30.3215.02 compatible sensor transmitter.
 * This code uses TH06 ambient sensor to periodically read temperature and RH
 * and sends packet compatible with TFA Dostmann 30.3215.02 sensors via any 433.92MHz radio module.
 *
 * Sensor channel code 1 to 4 is defined by DIP switches.
 * Sensor identifier is set either manually by DIPs in range 1 to 15 or random after each reset as for original sensor when DIPs set to 0.
 * Reporting period can be set to 1, 2, 5 or 10 minutes by another DIPs. Actual period is randomized +/-5%.
 *
 * Designed for ATmega48PB, which is about minimum to fit.
 * Wakeup timer uses 32768Hz crystal on Timer 2.
 * Main clock is internal 1MHz RC clock (default fuses).
 * It is designed for a 3V lithium battery Renata 2450n with ~500mAh capacity.
 * It should last for about a year with 1 minute reporting period (consumes ~1uAh per transmission).
 * Idle power is few uA. Actual lifetime depends on transmitter power.
 *
 * More details on HW on project GitHub url:
 * https://github.com/smaslan/TFA-30321502-sensor
 *
 * (c) 2025, Stanislav Maslan, s.maslan@seznam.cz
 * V1.2, 2025-02-15
 *
 * The code and all its part are distributed under MIT license
 * https://opensource.org/licenses/MIT.
 */ 

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sfr_defs.h>
#include <util/twi.h>
#include <util/atomic.h>
#include <util/delay_basic.h>
#include <util/delay.h>
#include <stdlib.h>

#include "main.h"

// --- jump to address ---
#define boot_start(boot_addr) {goto *(const void*)boot_addr;}



//----------------------------------------------------------------------------------
// analog ADC stuff
//----------------------------------------------------------------------------------

// ADC data and stuff
typedef struct{
	volatile uint16_t a_vmon;
	volatile uint8_t rand;
}TADC;
TADC adc_data;

#define ADC_VREF 1.1f
#define ADC_VMON_DIV (10000.0/(10000.0 + 24000.0))
#define ADC_GAIN (ADC_VREF/(ADC_AVG*1024.0f))
#define ADC_VMON_CAL (ADC_GAIN/ADC_VMON_DIV)

// ADC conversion done ISR
ISR(ADC_vect)
{
	// average buffer
	static uint8_t avg_cnt = ADC_AVG;

	// get new value
	uint16_t adc = ADC;
		
	static uint16_t vmon = 0;
	vmon += adc;
	avg_cnt--;
	if(!avg_cnt)
	{
		avg_cnt = ADC_AVG;
		adc_data.a_vmon = vmon; vmon = 0;

		// generate random-like number
		uint8_t rng = (adc&0xFF)^(adc>>8);
		rng = (rng>>4)^(rng&0x0F);
		rng = (rng>>2)^(rng&0x03);
		rng = (rng>>1)^(rng&0x01);
		adc_data.rand = (adc_data.rand<<1) | rng;
	}
	
	// start next conversion
	sbi(ADCSRA,ADIF);
	sbi(ADCSRA,ADSC);
}

// get VMON voltage [V]
float adc_get_vmon()
{
	uint16_t a_vmon = 0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		a_vmon = adc_data.a_vmon;
	}
	return(ADC_VMON_CAL*a_vmon);
}


// init ADC
void adc_init()
{
	// initial ADC setup (AVcc ref)
	ADMUX = (3<<REFS0)|(0<<ADLAR)|(VMON_ADC_MUX<<MUX0);
	ADCSRA = (1<<ADEN)|(0<<ADATE)|(1<<ADIE)|(2<<ADPS0);
	ADCSRB = (0<<ADTS0);
	
	// start ADC conversion loop
	sbi(ADCSRA,ADSC);
	
	// initial measured value
	adc_data.a_vmon = 0;
}

// idle ADC before sleep
void adc_idle()
{
	ADCSRA = 0;
}


//----------------------------------------------------------------------------------
// DIP switches stuff
//----------------------------------------------------------------------------------
// init DIP switches
void dip_init()
{
	// channel
	CH_ddr &= ~(CH_msk<<CH0);
	CH_port |= (CH_msk<<CH0);

	// address id
	A_ddr &= ~(A_msk<<A0);
	A_port |= (A_port<<A0);

	// speed code
	cbi(S0_ddr, S0);
	sbi(S0_port, S0);
	cbi(S1_ddr, S1);
	sbi(S1_port, S1);
}

// set DIP to power down mode
void dip_idle()
{
	CH_port &= ~(CH_msk<<CH0);
	A_port &= ~(A_msk<<A0);
	cbi(S0_port, S0);
	cbi(S1_port, S1);
}

// get address id
uint8_t dip_get_addr()
{
	return(15 - ((A_pin >> A0) & A_msk));
}

// get channel id
uint8_t dip_get_chn()
{
	return(3 - ((CH_pin >> CH0) & CH_msk));
}

// get speed code
uint8_t dip_get_speed()
{
	uint8_t speed = 0;
	bcopy(speed, 0, S0_pin, S0);
	bcopy(speed, 1, S1_pin, S1);
	return(3 - speed);
}





//----------------------------------------------------------------------------------
// I2C STUFF
//----------------------------------------------------------------------------------

#define i2c_start() TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTA)
#define i2c_stop() TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO)
#define i2c_data_out(data)	TWDR = (data); TWCR = (1<<TWINT) | (1<<TWEN)
#define i2c_read(ack) TWCR = (1<<TWINT) | (1<<TWEN) | ((ack)<<TWEA)

// set timeout value [ms]
volatile uint16_t i2c_timeout;
void i2c_timeout_set(uint16_t timeout)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		i2c_timeout = timeout;
	}
}

// timeout done?
uint8_t i2c_timeout_done()
{
	uint16_t timeout;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		timeout = i2c_timeout;
	}
	return(timeout == 0);
}

// timeout timer ISR
ISR(TIMER0_COMPA_vect, ISR_NOBLOCK)
{
	uint16_t time = i2c_timeout;
	if(time)
		time--;
	i2c_timeout = time;
}

// initialize I2C
void i2c_init()
{
	// I2C as master
	TWCR = (1<<TWEN)|(1<<TWEA);
	TWBR = (uint8_t)((F_CPU - 16.0*I2C_FREQ)/(2.0*I2C_FREQ));
	TWSR = (0<<TWPS0);

	// timeout timer
	i2c_timeout = 0;
	TCCR0A = (2<<WGM00);
	TCCR0B = (0<<WGM02)|(2<<CS00);
	OCR0A = (uint8_t)((1.0e-3*F_CPU/8) - 1);
	TIMSK0 = (1<<OCIE0A);
	
	// enable pull-ups
	sbi(I2C_port,I2C_SDA);
	sbi(I2C_port,I2C_SCL);
}

// idle I2C stuff
void i2c_idle()
{
	// stop I2C
	TWCR = 0;
	
	// stop timeout timer
	TCCR0B = (0<<WGM02)|(0<<CS00);
	
	// idle lines
	cbi(I2C_port,I2C_SDA);
	cbi(I2C_port,I2C_SCL);
}



// wait for I2C state change with timeout (ISR must be enabled!)
uint8_t i2c_wait(uint16_t timeout_ms)
{
	if(timeout_ms)
	{
		// timeout mode
		i2c_timeout_set(timeout_ms);
		while(!bit_is_set(TWCR,TWINT) && !i2c_timeout_done());
		return(i2c_timeout_done());
	}
	else
		while(!bit_is_set(TWCR,TWINT)); // no timeout mode

	return(0);
}

// I2C write and read back routine with timeouts (ISR must be enabled)
// reads to the same data register, if no read needed, set rd_count to zero
uint8_t i2c_write_read_data(uint8_t address,uint8_t *data,uint8_t wr_count,uint8_t rd_count,uint16_t timeout_ms)
{
	// start
	i2c_start();
	if(i2c_wait(timeout_ms))
	{
		// timeout
		i2c_stop();
		return(1);
	}
	if((TW_STATUS) != TW_START)
	{
		i2c_stop();
		return(1);
	}

	// send address + write
	i2c_data_out(address<<1);
	if(i2c_wait(timeout_ms))
	{
		// timeout
		i2c_stop();
		return(2);
	}
	if((TW_STATUS) != TW_MT_SLA_ACK)
	{
		i2c_stop();
		return(2);
	}

	// transmit data bytes
	for(uint8_t k = 0;k < wr_count;k++)
	{
		// send byte
		i2c_data_out(data[k]);
		if(i2c_wait(timeout_ms))
		{
			// timeout
			i2c_stop();
			return(3);
		}
		if((TW_STATUS) != TW_MT_DATA_ACK)
		{
			i2c_stop();
			return(3);
		}
	}

	// optional read data
	if(rd_count)
	{
		// restart
		i2c_start();
		if(i2c_wait(timeout_ms))
		{
			// timeout
			i2c_stop();
			return(4);
		}
		if((TW_STATUS) != TW_REP_START)
		{
			i2c_stop();
			return(4);
		}

		// send address + read
		i2c_data_out((address<<1) | 0x01);
		if(i2c_wait(timeout_ms))
		{
			// timeout
			i2c_stop();
			return(5);
		}
		if((TW_STATUS) != TW_MR_SLA_ACK)
		{
			i2c_stop();
			return(5);
		}

		// read data bytes
		for(uint8_t k = 0;k < rd_count;k++)
		{
			// read byte
			i2c_read(k < rd_count-1);
			if(i2c_wait(timeout_ms))
			{
				// timeout
				i2c_stop();
				return(6);
			}
			if((TW_STATUS) != TW_MR_DATA_ACK && (TW_STATUS) != TW_MR_DATA_NACK)
			{
				i2c_stop();
				return(6);
			}
			data[k] = TWDR;
		}

	}

	// stop
	i2c_stop();

	
	return(0);
}

// timeout in case I2C fails [ms]
#define I2C_TIMEOUT 100
#define TH06_I2C_ADDRESS 0x40u
#define TH06_CMD_TEMP_READ_HOLD 0xE3u
#define TH06_CMD_RH_READ_HOLD 0xE5u
#define TH06_CMD_TEMP_READ_LAST 0xE0u

// random number from TH06 sensor
uint8_t th06_rng;

// get TH06 relative humidity (call before temperature read - it measures both)
uint8_t TH06_get_rh(uint8_t *rh)
{
	// default return
	*rh = 0.0;

	// read temp blocking
	uint8_t data[2] = {TH06_CMD_RH_READ_HOLD,0};
	uint8_t err = i2c_write_read_data(TH06_I2C_ADDRESS, data, 1, 2, I2C_TIMEOUT);
	if(err)
		return(err);
	
	// convert to float
	uint16_t read = data[1] + (data[0]<<8);
	*rh = (uint8_t)(125.0*read/65536.0 - 6.0);

	// update random generator
	th06_rng = data[0] ^ data[1];
	
	return(err);
}

// get TH06 temperature (last measured by previous RH measurement)
uint8_t TH06_get_temp(int16_t *temp)
{
	// default return
	*temp = 0;

	// read temp blocking
	uint8_t data[2] = {TH06_CMD_TEMP_READ_LAST,0};
	uint8_t err = i2c_write_read_data(TH06_I2C_ADDRESS, data, 1, 2, I2C_TIMEOUT);
	if(err)
	return(err);
	
	// convert to float
	uint16_t read = data[1] + (data[0]<<8);
	*temp = (int16_t)((175.72*read/65536.0 - 46.85)*10.0);

	// update random generator
	th06_rng = data[0] ^ data[1];
	
	return(err);
}





//----------------------------------------------------------------------------------
// Transmitter stuff
//----------------------------------------------------------------------------------

typedef struct{
	uint8_t data[5];
	uint8_t bit_pos;
	uint8_t state;
	uint8_t pulse;
	uint8_t packet;
}TTX;
TTX tx;

#define TX_STATE_IDLE 0
#define TX_STATE_START 1
#define TX_STATE_DATA 2
#define TX_STATE_STOP 3
#define TX_STATE_FINISHED 4

#define TX_BIT_W_MS 0.5e-3
#define TX_BIT_W ((uint16_t)(TX_BIT_W_MS*F_CPU))
#define TX_BIT_START_W ((uint16_t)((9.0e-3 - TX_BIT_W_MS)*F_CPU))
#define TX_BIT_STOP_W ((uint16_t)((1.5e-3 - TX_BIT_W_MS)*F_CPU))
#define TX_BIT_HIGH_W ((uint16_t)((4.2e-3 - TX_BIT_W_MS)*F_CPU))
#define TX_BIT_LOW_W ((uint16_t)((2.3e-3 - TX_BIT_W_MS)*F_CPU))

// bit timer
ISR(TIMER1_COMPA_vect)
{
	// send last iteration pulse out
	if(tx.pulse)
		sbi(DATA_port, DATA)
	else
		cbi(DATA_port, DATA)		

	if(tx.pulse)
	{
		// start pulse
		tx.pulse = 0;
		OCR1A = TX_BIT_W;
	}
	else
	{
		// pulse ended

		// initiate new pulse?
		if(tx.state != TX_STATE_IDLE)
		{
			tx.pulse = 1;
		}

		
		if(tx.state == TX_STATE_START)
		{
			// start bit
			OCR1A = TX_BIT_START_W;
			tx.state = TX_STATE_DATA;
		}
		else if(tx.state == TX_STATE_DATA)
		{
			// data bits
			static const uint8_t mask[] PROGMEM = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};
			uint8_t bit_pos = tx.bit_pos - 1;
			uint8_t data = tx.data[bit_pos >> 3] & pgm_read_byte(&mask[bit_pos & 0x07]);
			if(data)
				OCR1A = TX_BIT_HIGH_W;
			else
				OCR1A = TX_BIT_LOW_W;
			
			if(!bit_pos)
			{				
				// last bit out
				bit_pos = 36;
				tx.packet--;				
				tx.state = TX_STATE_STOP;
			}
			tx.bit_pos = bit_pos;
			
		}
		else if(tx.state == TX_STATE_STOP)
		{
			// stop bit
			OCR1A = TX_BIT_STOP_W;
			if(tx.packet)
			{
				// start next packet repetition
				tx.state = TX_STATE_START;
			}
			else
			{
				// all packets out - finish after stop pulse
				tx.state = TX_STATE_IDLE;
			}
		}
		else
		{
			// all finished
			tx.state = TX_STATE_FINISHED;
			
			// stop bit timer
			TCCR1B = (1<<WGM12)|(0<<CS10);
			TIMSK1 = 0;
		}
	}	
}

// init transmitter
void tx_init()
{
	// init TX pin
	sbi(DATA_ddr, DATA);
	cbi(DATA_port, DATA);
	tx.state = TX_STATE_IDLE;
}

// set transmitter to idle state
void tx_idle()
{
	// data port to idle
	cbi(DATA_port, DATA);

	// stop bit timer
	TCCR1B = (1<<WGM12)|(0<<CS10);
	TIMSK1 = 0;
}

// start data packet transmission
void tx_start()
{
	// start bit timer
	OCR1A = 65535;
	TCNT1 = 0;
	TCCR1A = (0<<WGM10);
	TCCR1B = (1<<WGM12)|(1<<CS10);
	TIFR1 = 0xFF;
	TIMSK1 = (1<<OCIE1A);
		
	// init transmitter state machine
	tx.state = TX_STATE_START;
	tx.bit_pos = 36;
	tx.packet = 7;
	tx.pulse = 1;
	
	// start transmission
	TCCR1C = (1<<FOC1A);
}

// check if transmission done
uint8_t tx_is_done()
{
	return(tx.state == TX_STATE_FINISHED);
}


//----------------------------------------------------------------------------------
// system timer
//----------------------------------------------------------------------------------

// update timer period
volatile uint16_t tick_periods;
volatile uint16_t tick_div;
void timer_set(uint16_t period)
{
	// timer tick rate [s]
	const float tick = 1024.0/TICK_REFCLK;
	// timer period randomization const
	const uint8_t tick_rand = (uint8_t)(2.0*TICK_PER_RAND/tick);
	// timer period const
	const uint8_t tick_int = (uint8_t)((TICK_PER - TICK_PER_RAND)/tick);
	// set new rate	
	OCR2A = tick_int + (rand() % tick_rand);

	// timer periods multiplier
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		tick_periods = max((uint8_t)((float)period/TICK_PER),1);	
		tick_div = tick_periods;
	}
}

// init system timer
volatile uint8_t tick_flag;
void timer_init()
{
	// default period
	timer_set(5);

	// init with external crystal
	TCCR2A = (2<<WGM20);
	TCCR2B = (0<<WGM22) | (7<<CS20);
	ASSR = (0<<EXCLK) | (1<<AS2);
	TIMSK2 = 1<<OCIE2A;
	tick_flag = 0;
}

// wakeup timer
ISR(TIMER2_COMPA_vect)
{
	tick_div--;
	if(!tick_div)
	{
		tick_div = tick_periods;
		tick_flag = 1;
	}
}


//----------------------------------------------------------------------------------
// Other
//----------------------------------------------------------------------------------
// power off
void pwr_idle()
{
	sbi(PWR_ddr, PWR);
	sbi(PWR_port, PWR);
}

// power on
void pwr_enable()
{
	sbi(PWR_ddr, PWR);
	cbi(PWR_port, PWR);
}

// set LED state
#define LED_OFF 0
#define LED_GREEN 1
#define LED_RED 2
void led_state(uint8_t state)
{
	sbi(LEDAR_ddr, LEDAR);
	sbi(LEDAG_ddr, LEDAG);
	if(state == LED_GREEN)
	{
		sbi(LEDAG_port, LEDAG);
		cbi(LEDAR_port, LEDAR);	
	}
	else if(state == LED_RED)
	{
		sbi(LEDAR_port, LEDAR);
		cbi(LEDAG_port, LEDAG);
	}
	else
	{
		cbi(LEDAR_port, LEDAR);
		cbi(LEDAG_port, LEDAG);
	}
}

// init SYNC button
volatile uint8_t sync_flag;
void sync_init()
{
	// sync input with pullup
	cbi(SYNC_ddr, SYNC);
	sbi(SYNC_port, SYNC);
	
	// enable ISR
	SYNC_PCMSK = (1<<SYNC);
	PCICR = (1<<SYNC_PCIE);

	// no sync request yet
	sync_flag = 0;
}

// SYNC button ISR
ISR(SYNC_ISR)
{
	if(!bit_is_set(SYNC_pin, SYNC))
		sync_flag = 1;
}




//----------------------------------------------------------------------------------
// MAIN
//----------------------------------------------------------------------------------
int main(void)
{
	// disable WDT if enabled (sometimes it fucks up ...)
	uint8_t rst_flag = MCUSR;
	MCUSR = 0;
	wdt_disable();

	// disable all ports
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;
	
	if(bit_is_set(rst_flag,BORF))
	{
		// brown-out reset
		_delay_ms(200);
	}

	PRR = (1<<PRUSART0) | (1<<PRSPI);
		
	// init ADC
	adc_init();
	adc_idle();

	// init DIP switches
	dip_init();
	dip_idle();

	// init transmitter
	tx_init();
	tx_idle();

	// init I2C bus
	i2c_init();
	i2c_idle();

	// idle power stage
	pwr_idle();

	// init SYNC button
	sync_init();

	// init periodic wakeup timer
	timer_init();

	// state LED
	led_state(LED_OFF);

	// default sensor ID
	uint8_t addr_rand = 0;

	// initial channel id unknown
	uint8_t chn_id_last = 255;
	
	// enable ISR
	sei();
			
	
	while(1)
	{		
		// init stuff
		pwr_enable();
		i2c_init();
		adc_init();
		tx_init();

		// wait for power up
		_delay_ms(100);
						
		// read temperature and RH from TH06
		int16_t temp = 0;
		uint8_t rh = 0;
		TH06_get_rh(&rh);
		TH06_get_temp(&temp);
							
		// get system voltage (should be updated at this point)
		float vmon = adc_get_vmon();
		uint8_t low_batt = vmon < VMON_MIN;

		// idle ADC, init DIPs
		adc_idle();
		dip_init();

		// get sensor channel
		uint8_t chn_id = dip_get_chn();
		if(chn_id != chn_id_last)
		{
			// randomizing sensor id whenever channel changed (and after reset)
			
			// regenerate seed?			
			uint8_t seed = TCNT2 ^ adc_data.rand ^ th06_rng;
			if(chn_id_last == 255)
				srand(seed);
						
			// generate new sensor id			
			addr_rand = (seed&0x0F) ^ (seed>>4);
		}
		chn_id_last = chn_id;
		
		// select sensor id (fixed or random)
		uint8_t addr = dip_get_addr();
		if(!addr)
			addr = addr_rand;

		// update auto report period
		uint8_t speed_code = dip_get_speed();
		static const uint16_t speed_list[] PROGMEM = {60, 120, 300, 600};
		uint16_t speed = pgm_read_word(&speed_list[speed_code]);
		timer_set(speed);

		// make data packet
		tx.data[4] = 0x09;
		tx.data[3] = 0x00 | addr;
		tx.data[2] = (low_batt<<7) | (sync_flag<<6) | (chn_id<<4) | ((temp >> 8) & 0x0F);
		tx.data[1] = temp & 0xFF;
		tx.data[0] = rh;

		dip_idle();
			
		// start transmission
		tx_start();

		// blink state led based on battery state
		led_state((low_batt)?LED_RED:LED_GREEN);
		_delay_ms(10);
		led_state(LED_OFF);

		// wait till transmission is finished		
		while(!tx_is_done())
			_delay_ms(1);		

		// idle transmitter
		tx_idle();

		// idle I2C bus
		i2c_idle();

		// idle power stage
		pwr_idle();

		// clear auto report tick flag
		tick_flag = 0;
		sync_flag = 0;

		do{
			// go to sleep
			cli();
			MCUCR = (1<<BODSE)|(1<<BODS);
			MCUCR = (1<<BODS);
			set_sleep_mode(SLEEP_MODE_PWR_SAVE);
			sleep_enable();
			sleep_bod_disable();
			sei();
			sleep_cpu();
			sleep_disable();
			
			// wait till SYNC or auto report tick (ignoring wakeup timer ticks until report period reached)
		}while(!sync_flag && !tick_flag);		
		
	}

}

