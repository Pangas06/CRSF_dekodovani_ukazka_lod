/*
Tento program slouí pro dekódování pøijatıch dat pøes UART v protokolu CRSF, a následné ovládání rybáøské zakrmovací lodì.
*/

#define F_CPU 20000000UL			// definice pro knihovnu delay.h
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>   //printf
#include <avr/pgmspace.h>

//-------CRSF--------//
#define BAUDRATE 420000UL			// Rychlost komunikace

// Vıpoèet hodnoty BAUDVAL potøednou pro inicializaci UARTu
#if BAUDRATE < F_CPU/16
#if (((10*4*F_CPU)/BAUDRATE) % 10) > 4
#define BAUDVAL (((4*F_CPU)/BAUDRATE)+1)
#else
#define BAUDVAL ((4*F_CPU)/BAUDRATE)
#endif
#else
#warning "Baudrate is out of range ! (max baudrate without CLK2X is F_CPU/16 (1.25Mb/s if F_CPU=20MHz)"
#define BAUDVAL 0
#endif

#define CRSF_VELIKOST_PAYLOADU 24	// Maximální velikost payloadu v CRSF
#define CRSF_HEADER 0xC8			// Hlavièka CRSF
#define NUM_CHANNELS 16				// Poèet kanálù
#define CRSF_ZACATEK_KANALU 3		// První kanál zaèíná na 3. bajtu payloadu



//-------PWM--------//
#define SERVO_0_DEGREE 3749			// PWM hodnota pro 0° (napø. 1 ms pulz)//3750
#define SERVO_90_DEGREE 1800		// PWM hodnota pro 90° (napø. 2 ms pulz)//4500

#define MIN_PWM 3749				// Motor se netoèí
#define MAX_PWM 7499				// Maximální rychlost motoru dopøedu
#define MAX_REVERSE_PWM 1500		// Maximální rychlost motoru dozadu
#define RAMP_STEP 10				// Maximální zmìna PWM hodnoty za jedno volání



//-------AD pøevodník--------//
#define MAX_NAPETI 4096				// Baterie 100 %
#define MIN_NAPETI 3170				// Baterie 0 %


//-------LED pásek----------//
#define NUM_LEDS 20
#define L_PATTERN 0b01110000		// Patern pro zasílání 0 pøes SPI pro LED pásek
#define H_PATTERN 0b01111110		// Patern pro zasílání 1 pøes SPI pro LED pásek



volatile uint8_t crsf_buffer[CRSF_VELIKOST_PAYLOADU];
volatile uint8_t crsf_index = 0;
volatile uint8_t crsf_new_data = 0;

volatile uint16_t napeti = 0;
volatile uint8_t procento_napeti = 0;

uint8_t colors[NUM_LEDS * 3];

volatile uint16_t milis_cnt=0;

//-------Deklarace funkcí---------//

void clock_20MHz(void);

void usart_init(void);
int usart_putchar(char var, FILE *stream);
static FILE usart_stream = FDEV_SETUP_STREAM(usart_putchar, NULL, _FDEV_SETUP_WRITE);

void send_channel_data(void);
uint16_t hodnota_kanalu(uint8_t *data, uint8_t channel);
uint16_t * dekodovani(void);

uint16_t milis(void);

void pwm_init(void);
void servo_set_position(uint8_t position);

void motory_dopredu(uint16_t rychlost, uint16_t zatoceni);
void motory_dozadu(uint16_t rychlost, uint16_t zatoceni);

void ad_prevodnik(void);
void test(uint8_t* data, uint16_t delka);
void sparovani(void);
void LED_pasek(void);
void inicializace(void);


int main(void) {
	clock_20MHz();
	pwm_init();
	usart_init();
	inicializace();
	
	stdout = &usart_stream;  // Pøesmìrování printf na UART
	
	sei();							// povolení pøerušení

	static uint8_t stav = 0;
	
	static uint16_t time = 0;
	static uint16_t time_motory = 0;
	static uint16_t time_sparovani = 0;
	
	time = milis();
	time_motory = milis();
	time_sparovani = milis();
	
	static uint8_t led_on = 0;				// 0 = LED vypnuté, 1 = LED zapnuté
	static uint16_t time_osvetleni = 0;     // Èas aktivace osvìtlení
	
	sparovani();
	
	while (1) {
		milis();
		
		uint16_t *channels = dekodovani();	// Tetno ukazatel ukazuje na pole dekódovanıch kanálù
		
		uint16_t channel_0 = channels[0];		// Zatáèení pravı joystick min 174, stred 968, max 1808
		uint16_t channel_1 = channels[1];		// Rychlost pravı joystick min 174, stred 968, max 1797
		uint16_t channel_2 = channels[2];		// Rychlost levı  joystick min 174, stred 968, max 1797
		uint16_t channel_3 = channels[3];		// Zatáèení levı  joystick min 174, stred 968, max 1773
		uint16_t channel_4 = channels[4];		// SA 191 vypnuto, 1792 zapnuto
		uint16_t channel_5 = channels[5];		// SB 191 zapnuto, 1792 vypnuto
		uint16_t channel_6 = channels[6];		// SC 191 zapnuto, 1792 vypnuto
		uint16_t channel_7 = channels[7];		// SD 1792 vypnuto, 191 zapnuto
		uint16_t channel_8 = channels[8];		// SE 1792 stisk,   191 zapnuto
		uint16_t channel_9 = channels[9];		// SF 1792 stisk,   191 vypnuto
		
		//printf_P(PSTR("Kanál %u: %x\r\n"), 0, channel_0);		//odesílání hodnot pøes UART pro vıpis v terminálovém emulátoru (PuTTY), pro kontrolu
		
		if (milis()-time_motory > 500 && channel_5 == 191)
		{
			motory_dopredu(channel_1,channel_3);			// Ovládání motorù pøi smìru vpøed
			time_motory=milis();
		}
		
		if (milis()-time_motory > 500 && channel_5 == 1792)
		{
			motory_dozadu(channel_1,channel_3);					// Ovládání motorù pøi smìru vpøed
			time_motory=milis();
		}
		
		if (crsf_new_data) {									// Kdy byla pøijatá nová data
			crsf_new_data = 0;
			
			// Vypuštìní návnady
			if (channel_9 == 1792 && stav == 0) {
				// Uloíme èas a nastavíme stav
				time_osvetleni = milis();
				stav = 1;
				
				if (!led_on) {									// Rozsvítíme svìtla jednorázovì, pokud ještì nejsou zapnuté, signalizace vypuštìní návnady
					PORTD_OUTSET = PIN0_bm;						// Zapnutí zadních svìtel
					for (int i = 0; i < NUM_LEDS * 3; i++) {	//Zápis hodnot pro rozsvícení LED pásku bílou barvou
						colors[i] = 0xFF;
					}
					test(colors, sizeof(colors));				// Odeslání pole colors do LED pásku
					led_on = 1;
				}
			}
			
			// Ovládání osvìtlení, pokud není právì vypouštìna návnada
			if (stav == 0) {
				if (channel_4 == 1792 && channel_7 == 191) {	// Zapnutá svìtla
					PORTD_OUTSET = PIN0_bm;						// Zapnutí zadních svìtel
					for (int i = 0; i < NUM_LEDS * 3; i++) {	// Zapnutí LED pásku - bílá barva
						colors[i] = 0xFF;
					}
					test(colors, sizeof(colors));				// Odeslání pole colors do LED pásku
				}
				if (channel_4 == 191 && channel_7 == 191) {		// Vypnutá svìtla
					// Vypnutí LED pro další osvìtlení
					PORTD_OUTCLR = PIN0_bm;						// Vypnutí zadních svìtel
					for (int i = 0; i < NUM_LEDS * 3; i++) {	// Zápis hodnot pro zhasnutí LED pásku
						colors[i] = 0x00;
					}
					test(colors, sizeof(colors));				// Odeslání pole colors do LED pásku
				}
			}
			
			ad_prevodnik();										// Kontrola stavu nabití baterie
		}														// konec zpracování novıch paketù

		// Kontinuální aktualizace pøi vypuštìní návnady
		if (stav == 1) {
			servo_set_position(1);								// Vypuštìní návnady
			
			if (milis() - time_osvetleni >= 50000) {			// Pokud uplynul poadovanı èas deaktivujeme reim
				stav = 0;
			}
			} else {											// stav == 0 -> právì se nevypouští návnada
			servo_set_position(0);								// Servo do vıchozí polohy
			
			if (led_on) {										// Pokud byly LED aktivovány pøi vypouštìní návnady, jednorázovì je vypneme
				PORTD_OUTCLR = PIN0_bm;
				for (int i = 0; i < NUM_LEDS * 3; i++) {
					colors[i] = 0x00;
				}
				test(colors, sizeof(colors));
				led_on = 0;
			}
		}
		
		if (milis()-time_sparovani>50000)			// Kontrola spárování
		{
			time_sparovani = milis();
			sparovani();
		}
	}
}

// Rutina pøerušení pro pøíjem dat z UARTu
ISR(USART0_RXC_vect) {
	uint8_t data = USART0.RXDATAL;					// Pøeètení pøijatého bajtu z UARTu, doèasné uloení do promìnné data

	if (crsf_index == 0 && data != CRSF_HEADER) {	// Pokud adresovı bajt není roven C8 -> chybnı paket, vrácení
		return;
	}

	crsf_buffer[crsf_index++] = data;				// Uloení dat do bufferu
	
	if (crsf_index > CRSF_VELIKOST_PAYLOADU) {		// Reset buffer, kdy je uloeno a moc dat
		crsf_index = 0;
		return;
	}
	
	if (crsf_index == CRSF_VELIKOST_PAYLOADU) {		// Kompletní paket
		crsf_index = 0;								// Pøipravení pro další paket
		crsf_new_data = 1;
	}
}

// Inicializace UART
void usart_init(void) {
	PORTA.DIRCLR = PIN1_bm;  // Rx
	PORTA.OUTSET = PIN0_bm;	 // Nastavení Rx do logické 1 pro pøíjem dat
	PORTA.DIRSET = PIN0_bm;  // Tx
	
	USART0.BAUD = BAUDVAL;	 // Nastavení rychlosti komunikace
	USART0.CTRLB |= USART_RXEN_bm;
	USART0.CTRLA |= USART_RXCIE_bm;  // Povolit pøerušení od Rx
	USART0.CTRLB |= USART_TXEN_bm;
}

// Pøepnutí mikroèipu na frekvenci 20 MHz
void clock_20MHz(void) {
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;
	CCP = CCP_IOREG_gc;
	CLKCTRL.OSCHFCTRLA = CLKCTRL_FRQSEL_20M_gc;
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB = 0;
}

// Rutina pøerušní pro millis()
ISR(TCB0_INT_vect){
	TCB0.INTFLAGS = TCB_CAPT_bm;
	milis_cnt++;
}

// funkce pro mìøení èasu
uint16_t milis(void){
	uint16_t tmp;
	TCB0.INTCTRL &=~ TCB_CAPT_bm;	//vynulovat capt bit = vypnout pøerušení od TCB
	tmp = milis_cnt;
	TCB0.INTCTRL = TCB_CAPT_bm;		//zapnout zpìt pøerušení od TCB
	return tmp;
}

// Funkce pro odesílání dat pøes UART do
int usart_putchar(char var, FILE *stream) {
	while (!(USART0.STATUS & USART_DREIF_bm)) {}
	USART0.TXDATAL = var;
	return 0;
}

//Inicializace
void inicializace(void){
	// Nastavení pro funkci milis()
	TCB0.CTRLA = TCB_CLKSEL_DIV1_gc | TCB_ENABLE_bm;
	TCB0.CTRLB = TCB_CNTMODE_INT_gc;
	TCB0_INTCTRL = TCB_CAPT_bm;
	TCB0_CCMP = 999;
	
	// Nastavení SPI pro ovládání LED pásku
	PORTA.DIRSET = PIN4_bm;
	SPI0.CTRLB = SPI_SSD_bm | SPI_MODE_3_gc | SPI_BUFEN_bm;
	SPI0.CTRLA = SPI_MASTER_bm | SPI_PRESC_DIV4_gc |SPI_CLK2X_bm| SPI_ENABLE_bm;
	
	// Nastavení AD pøevodníku se vzorkovací frekvencí 10 ms, Pin PA1, referenèní napìtí 4,096 V
	PORTD.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;		//vypnutí vstupního bufferu
	VREF.ADC0REF = VREF_REFSEL_4V096_gc;
	ADC0.CTRLC = ADC_PRESC_DIV20_gc;
	ADC0.SAMPCTRL = 10;
	ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
	ADC0.CTRLA = ADC_ENABLE_bm;
	
	
	// Nastavení vıstupních pinù
	PORTD_DIRSET = PIN7_bm;			// 0 red	- vıstraná
	PORTD_DIRSET = PIN6_bm;			// 1 green, > 25 %
	PORTD_DIRSET = PIN5_bm;			// 2 green, > 25 %
	PORTD_DIRSET = PIN4_bm;			// 3 green, > 50 %
	PORTD_DIRSET = PIN3_bm;			// 4 green, > 75 %
	PORTD_DIRSET = PIN2_bm;			// Spárovací LED
	PORTD_DIRSET = PIN1_bm;			// Vstup AD pøevodník
	PORTD_DIRSET = PIN1_bm;			// Zadní svìtla
	
	PORTD_OUTCLR = PIN0_bm;			// vypnutí zadních svìtel
}

// Inicializace PWM
void pwm_init(void) {
	TCA0.SINGLE.PER = 49999; // Nastavení periody PWM na 20 ms (50 Hz)

	TCA0.SINGLE.CMP0 = 3749; // Start na 0° (šíøka pulzu = 1 ms)
	TCA0.SINGLE.CMP1 = 3749; // Start na 0° (šíøka pulzu = 1 ms)
	TCA0.SINGLE.CMP2 = 3749; // Start na 0° (šíøka pulzu = 1 ms)
	
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_CMP2EN_bm |// Povolení vıstupu Compare Channel 0 (PA0)
	TCA_SINGLE_WGMODE_SINGLESLOPE_gc; // Reim: Single-slope PWM
	
	TCA0.SINGLE.CTRLA = (1 << TCA_SINGLE_ENABLE_bp) | (TCA_SINGLE_CLKSEL_DIV8_gc);			//Nastavení pøeddìlièky a povolení timeru
	
	PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTC_gc;		//Pøesmìrování PWM na port C

	PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm;		// Nastavení pinu PC0, PC1, PC2 jako vıstupního pro PWM
}

// Funkce pro ovládání motorù pøi chodu vpøed
void motory_dopredu(uint16_t rychlost, uint16_t zatoceni) {
	// MIN_PWM = 1.5ms			(neutrální – motory stojí)
	// MAX_PWM = 2.0ms			(maximální vpøed)
	// MAX_REVERSE_PWM = 1.0ms  (maximální zpìtnı chod)
	static uint16_t last_zatoceni = 970;
	static uint16_t last_rychlost = 174;
	// Pøedchozí hodnoty PWM pro vyhlazení zmìn
	static uint16_t last_left_pwm = MIN_PWM;
	static uint16_t last_right_pwm = MIN_PWM;
	
	uint16_t base_pwm;
	uint16_t rozdil;
	uint16_t procenta;
	uint16_t left_pwm, right_pwm;
	
	// Pokud je rychlost pod minimem, nastavíme neutrální hodnoty (motory nestojí)
	if (rychlost < 174) {
		left_pwm = MIN_PWM;
		right_pwm = MIN_PWM;
		} else {
		// Vıpoèet základní PWM hodnoty pro vpøed
		base_pwm = MIN_PWM + ((rychlost - 174) / 2);
		if (base_pwm > MAX_PWM)
		base_pwm = MAX_PWM;
		if (base_pwm < MIN_PWM)
		base_pwm = last_rychlost;
		
		rozdil = base_pwm - MIN_PWM;
		
		// Zatáèení doleva – pokud je zatoceni < 470
		// pøi maximálním zatáèení (zatoceni = 200) bude levı motor v plném reverse
		if (zatoceni < 470) {
			procenta = (470 - zatoceni) * 100UL / (470 - 200);				// procentuální zmìna od neutrálu
			left_pwm = MIN_PWM - (procenta * (base_pwm - MIN_PWM)) / 100;
			right_pwm = base_pwm;
		}
		// Mírné zatáèení doleva (470 < zatoceni < 968)
		else if (zatoceni < 968) {
			procenta = (968 - zatoceni) * 100UL / (968 - 470);
			uint16_t zp = (procenta * rozdil) / 100;
			left_pwm = base_pwm - zp;										// levı motor zpomaluje vpøed
			if (left_pwm < MIN_PWM)
			left_pwm = MIN_PWM;
			right_pwm = base_pwm;
		}
		// Neutrální oblast: 968 < zatoceni < 973
		else if (zatoceni <= 973) {
			left_pwm = base_pwm;
			right_pwm = base_pwm;
		}
		// Mírné zatáèení doprava (973 < zatoceni < 1400)
		else if (zatoceni <= 1400) {
			procenta = (zatoceni - 973) * 100UL / (1400 - 973);
			uint16_t zp = (procenta * rozdil) / 100;
			left_pwm = base_pwm;
			right_pwm = base_pwm - zp;  // pravı motor deceleruje vpøed
			if (right_pwm < MIN_PWM)
			right_pwm = MIN_PWM;
		}
		// Zatáèení doprava – pokud je zatoceni > 1400
		// pøi maximálním zatáèení (zatoceni = 1700) bude pravı motor v plném zpìtném chodu
		else {
			procenta = (zatoceni - 1400) * 100UL / (1700 - 1400); // 0 a 100
			right_pwm = MIN_PWM - (procenta * (base_pwm - MIN_PWM)) / 100;
			left_pwm = base_pwm;
		}
	}
	
	// Zajištìní zmìny rychlosti motorù pouze o malou zmìnu, kvùli plynulému ovládání motorù
	if (left_pwm > last_left_pwm) {
		if (left_pwm - last_left_pwm > RAMP_STEP)
		left_pwm = last_left_pwm + RAMP_STEP;
		} else if (left_pwm < last_left_pwm) {
		if (last_left_pwm - left_pwm > RAMP_STEP)
		left_pwm = last_left_pwm - RAMP_STEP;
	}
	
	if (right_pwm > last_right_pwm) {
		if (right_pwm - last_right_pwm > RAMP_STEP)
		right_pwm = last_right_pwm + RAMP_STEP;
		} else if (right_pwm < last_right_pwm) {
		if (last_right_pwm - right_pwm > RAMP_STEP)
		right_pwm = last_right_pwm - RAMP_STEP;
	}
	
	// Nastavení PWM vıstupù pro motory
	TCA0.SINGLE.CMP1 = left_pwm;
	TCA0.SINGLE.CMP2 = right_pwm;
	
	// Uloení aktuálních hodnot pro další volání
	last_zatoceni = zatoceni;
	last_rychlost = base_pwm;
	last_left_pwm = left_pwm;
	last_right_pwm = right_pwm;
}

// Funkce pro ovládání motorù pøi zpìtném chodu
void motory_dozadu(uint16_t rychlost, uint16_t zatoceni) {
	static uint16_t last_zatoceni = 970;
	static uint16_t last_rychlost = 174;
	static uint16_t last_left_pwm = MIN_PWM;
	static uint16_t last_right_pwm = MIN_PWM;
	
	uint16_t base_pwm;
	uint16_t procenta;
	uint16_t zpomaleni;
	uint16_t left_pwm, right_pwm;
	// Rozdíl mezi neutrálem a maximální reversí
	uint16_t rozdil = MIN_PWM - MAX_REVERSE_PWM;
	
	// Pøi nízké rychlosti nastavíme neutrální hodnotu – motory stojí
	if (rychlost < 174) {
		left_pwm = MIN_PWM;
		right_pwm = MIN_PWM;
		} else {
		// Vıpoèet základní PWM hodnoty pro zpìtnı chod:
		// Pøi rostoucí hodnotì "rychlost" se base_pwm sniuje od neutrálu smìrem k MAX_REVERSE_PWM.
		base_pwm = MIN_PWM - ((rychlost - 174) / 2);
		if (base_pwm < MAX_REVERSE_PWM)
		base_pwm = MAX_REVERSE_PWM;
		if (base_pwm > MIN_PWM)
		base_pwm = last_rychlost; // Tato vìtev by nemìla nastat
		
		// Diferenciální øízení podle zatáèení
		if (zatoceni < 470) {
			// Maximální levé zatáèení:
			procenta = (470 - zatoceni) * 100UL / (470 - 200);
			zpomaleni = (procenta * rozdil) / 100;
			left_pwm = base_pwm - zpomaleni;   // ještì více reverse
			right_pwm = base_pwm + zpomaleni;    // ménì reverse
			if (right_pwm > MIN_PWM) right_pwm = MIN_PWM;
		}
		else if (zatoceni < 968) {
			// Mírné zatáèení doleva:
			procenta = (968 - zatoceni) * 100UL / (968 - 470);
			uint16_t diff = MIN_PWM - base_pwm;
			zpomaleni = (procenta * diff) / 100;
			left_pwm = base_pwm - zpomaleni;
			right_pwm = base_pwm + zpomaleni;
			if (right_pwm > MIN_PWM) right_pwm = MIN_PWM;
		}
		else if (zatoceni <= 973) {
			// Neutrální oblast – ádná diferenciace
			left_pwm = base_pwm;
			right_pwm = base_pwm;
		}
		else if (zatoceni <= 1400) {
			//Mírné zatáèení doprava:
			procenta = (zatoceni - 973) * 100UL / (1400 - 973);
			uint16_t diff = MIN_PWM - base_pwm;
			zpomaleni = (procenta * diff) / 100;
			left_pwm = base_pwm + zpomaleni;		// Levı motor se "zmíròuje" smìrem k neutrálu.
			right_pwm = base_pwm - zpomaleni;		// Pro zatáèení doprava v reimu reverse: pravı motor jde více zpìt
			if (left_pwm > MIN_PWM) left_pwm = MIN_PWM;
		}
		else {
			//Maximální pravé zatáèení:
			procenta = (zatoceni - 1400) * 100UL / (1700 - 1400);
			zpomaleni = (procenta * rozdil) / 100;
			left_pwm = base_pwm + zpomaleni;	 // levı motor se "zmíròuje"
			right_pwm = base_pwm - zpomaleni;    // pravı motor jde ještì více reverse
			if (left_pwm > MIN_PWM) left_pwm = MIN_PWM;
		}
	}
	
	// Zajištìní zmìny rychlosti motorù pouze o malou zmìnu, kvùli plynulému ovládání motorù
	if (left_pwm > last_left_pwm) {
		if (left_pwm - last_left_pwm > RAMP_STEP)
		left_pwm = last_left_pwm + RAMP_STEP;
		} else if (left_pwm < last_left_pwm) {
		if (last_left_pwm - left_pwm > RAMP_STEP)
		left_pwm = last_left_pwm - RAMP_STEP;
	}
	
	if (right_pwm > last_right_pwm) {
		if (right_pwm - last_right_pwm > RAMP_STEP)
		right_pwm = last_right_pwm + RAMP_STEP;
		} else if (right_pwm < last_right_pwm) {
		if (last_right_pwm - right_pwm > RAMP_STEP)
		right_pwm = last_right_pwm - RAMP_STEP;
	}
	
	// Nastavení PWM vıstupù
	TCA0.SINGLE.CMP1 = left_pwm;
	TCA0.SINGLE.CMP2 = right_pwm;
	
	// Uloení aktuálních hodnot pro další volání
	last_zatoceni = zatoceni;
	last_rychlost = base_pwm;
	last_left_pwm = left_pwm;
	last_right_pwm = right_pwm;
}

// Funkce pro ovládání otáèení servomotorù
void servo_set_position(uint8_t position) {
	if (position == 0) {
		TCA0.SINGLE.CMP0 = SERVO_0_DEGREE;		// Neutrální pozice
		} else if (position == 1) {
		TCA0.SINGLE.CMP0 = SERVO_90_DEGREE;		// Vypuštìní návnady
	}
}

// Funkce pro indikaci stavu nabití baterie zobrazovaného na bargrafu
void ad_prevodnik(void)
{
	ADC0.COMMAND = ADC_STCONV_bm;				// Zahájení mìøení
	while(ADC0.COMMAND & ADC_STCONV_bm){}		// Èekáme na zmìnu spconv
	napeti = ADC0.RES;							// Uloení zmìøené hodnoty
	if(napeti>MIN_NAPETI){
		procento_napeti = (((uint32_t)napeti-MIN_NAPETI) * 100) / (MAX_NAPETI-MIN_NAPETI);	// Pøepoèet na procenta
		}else{
		procento_napeti = 0;
	}
	
	if (procento_napeti>25)			// Pokud jsou procenta vìtší ne 25 -> vypni vıstranou LED, rozsvi LED 1 a 2
	{
		PORTD_OUTCLR = PIN7_bm;
		PORTD_OUTSET = PIN6_bm;
		PORTD_OUTSET = PIN5_bm;
		if (procento_napeti>50)		// Pokud jsou procenta vìtší ne 50 -> rozsvi LED 3
		{
			PORTD_OUTSET = PIN4_bm;
			if (procento_napeti>75)	// Pokud jsou procenta vìtší ne 75 -> rozsvi LED 4
			{
				PORTD_OUTSET = PIN3_bm;
				}else{					// Pokud jsou procenta menší ne 75 -> zhasni LED 4
				PORTD_OUTCLR = PIN3_bm;
			}
			}else{						// Pokud jsou procenta menší ne 50 -> zhasni LED 3
			PORTD_OUTCLR = PIN4_bm;
		}
		
		}else{							// Pokud jsou procenta menší ne 25 -> zhasni všechny LED, rozsvi vıstranou LED
		PORTD_OUTSET = PIN7_bm;
		
		PORTD_OUTCLR = PIN6_bm;
		PORTD_OUTCLR = PIN5_bm;
		PORTD_OUTCLR = PIN4_bm;
		PORTD_OUTCLR = PIN3_bm;
	}
	
}

// Funkce pro získání 11bitové hodnoty kanálu
uint16_t hodnota_kanalu(uint8_t *data, uint8_t channel) {

	uint16_t bitOffset = channel * 11;		// Kolik je pøed kanálem bitù pøedchozích kanálùs
	uint8_t byteOffset = bitOffset / 8;		// Urèení bajtového offsetu
	uint8_t bitInByte = bitOffset % 8;		// Kolik bitù je posunuto v rámci daného bajtu
	uint32_t raw = data[byteOffset]			// Naètení 3 bajtù, protoe 11 bitù mùe bıt rozmístìno maximálnì pøes 3 bajty
	| (data[byteOffset + 1] << 8)
	| (data[byteOffset + 2] << 16);
	
	return (raw >> bitInByte) & 0x07FF;		// Posuneme hodnotu doprava a maskujeme nejniších 11 bitù
}

// Funkce pro dekodování kanálù
uint16_t * dekodovani(void) {
	static uint16_t channels[NUM_CHANNELS];
	uint8_t *data = &crsf_buffer[CRSF_ZACATEK_KANALU];	// Ukazatel na data kanálù v crsf_buffer, zaèíná od definovaného offsetu

	for (uint8_t i = 0; i < NUM_CHANNELS; i++) {		// Dekódujeme kanály 0 a 15 (co odpovídá kanálùm 1 a 16)
		channels[i] = hodnota_kanalu(data, i);
	}

	return channels;
}

// Funkce pro odesílání dat pro LED pásek
void test(uint8_t* data, uint16_t length){
	uint8_t mask;
	cli();			 // Vypnutí pøerušení
	while(length){   // Pro všechny bajty z vstupného rozsahu
		length--;
		mask=0b10000000; // Pro všechny bity v bajtu
		while(mask){
			while (!(SPI0.INTFLAGS & SPI_DREIF_bm)){}	// Èekání na prázdnı SPI buffer
			if(mask & data[length]){					// Poslání pulzu s s koresponující délkou pro "L" nebo "H"
				SPI0.DATA = H_PATTERN;
				}else{
				SPI0.DATA = L_PATTERN;
			}
			mask = mask >> 1;							// Bitovı posun, abychom mohli maskovat další bit
		}
	}
	sei();			// Zapnutí pøerušení
}

// Funkce pro ovìøení spárování zaøízení
void sparovani(void){
	uint8_t pocet_sparovani = 0;
	static uint16_t time_led = 0;
	static uint16_t time_ledpasek = 0;
	time_ledpasek = milis();
	time_led = milis();
	while (1)
	{
		uint8_t data = USART0.RXDATAL;					// Naètení pøijatého bajtu
		// Kontrola hlavièkového bajtu, pokud se nerovná -> spuštìní animace LED a blikání LED indikující párování
		if (data != CRSF_HEADER) {
			
			if (milis()-time_ledpasek>500)
			{
				LED_pasek();
				time_ledpasek = milis();
				if (milis()-time_led>3000)
				{
					PORTD.OUTTGL=PIN2_bm;
					time_led = milis();
					pocet_sparovani=0;
				}
			}
			
			}else{
			pocet_sparovani+=1;
			if (pocet_sparovani>50)						// Ovìøení spárování
			{
				PORTD_OUTSET = PIN2_bm;					// Zapnutí LED bargrafu pro signalizaci spárování
				break;									// Zaøízení je spárováno
			}
			
		}
	}
}

// Funkce pro animace na LED pásku
void LED_pasek(void)
{
	static int pos = 0;         // pozice hlavní LED
	static int direction = 1;   // smìr: 1 = doprava, -1 = doleva
	static uint8_t time_pasek = 0;
	time_pasek = milis();
	// vypnutí všech LED
	for (int i = 0; i < NUM_LEDS * 3; i++) {
		colors[i] = 0x00;
	}
	
	//urèení barev LED
	colors[pos * 3 + 0] = 0x00;  // B
	colors[pos * 3 + 1] = 0xFF;  // R
	colors[pos * 3 + 2] = 0x00;  // G
	
	if (pos > 0) {
		colors[(pos - 1) * 3 + 0] = 0x00;
		colors[(pos - 1) * 3 + 1] = 0x40;
		colors[(pos - 1) * 3 + 2] = 0x00;
	}
	if (pos < NUM_LEDS - 1) {
		colors[(pos + 1) * 3 + 0] = 0x00;
		colors[(pos + 1) * 3 + 1] = 0x40;
		colors[(pos + 1) * 3 + 2] = 0x00;
	}
	
	test(colors, sizeof(colors));			// Odeslání pole, která LED a jakou barvou má svítit
	
	_delay_ms(40);
	
	pos += direction;						// Zmìna pozice animace
	
	if (pos >= NUM_LEDS - 1 || pos <= 0) {	// Pokud dojedeme nakonec LED pásku, zmìníme smìr chodu animace
		direction = -direction;
	}
}

