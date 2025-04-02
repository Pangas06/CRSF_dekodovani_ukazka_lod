/*
Tento program slou�� pro dek�dov�n� p�ijat�ch dat p�es UART v protokolu CRSF, a n�sledn� ovl�d�n� ryb��sk� zakrmovac� lod�.
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

// V�po�et hodnoty BAUDVAL pot�ednou pro inicializaci UARTu
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

#define CRSF_VELIKOST_PAYLOADU 24	// Maxim�ln� velikost payloadu v CRSF
#define CRSF_HEADER 0xC8			// Hlavi�ka CRSF
#define NUM_CHANNELS 16				// Po�et kan�l�
#define CRSF_ZACATEK_KANALU 3		// Prvn� kan�l za��n� na 3. bajtu payloadu



//-------PWM--------//
#define SERVO_0_DEGREE 3749			// PWM hodnota pro 0� (nap�. 1 ms pulz)//3750
#define SERVO_90_DEGREE 1800		// PWM hodnota pro 90� (nap�. 2 ms pulz)//4500

#define MIN_PWM 3749				// Motor se neto��
#define MAX_PWM 7499				// Maxim�ln� rychlost motoru dop�edu
#define MAX_REVERSE_PWM 1500		// Maxim�ln� rychlost motoru dozadu
#define RAMP_STEP 10				// Maxim�ln� zm�na PWM hodnoty za jedno vol�n�



//-------AD p�evodn�k--------//
#define MAX_NAPETI 4096				// Baterie 100 %
#define MIN_NAPETI 3170				// Baterie 0 %


//-------LED p�sek----------//
#define NUM_LEDS 20
#define L_PATTERN 0b01110000		// Patern pro zas�l�n� 0 p�es SPI pro LED p�sek
#define H_PATTERN 0b01111110		// Patern pro zas�l�n� 1 p�es SPI pro LED p�sek



volatile uint8_t crsf_buffer[CRSF_VELIKOST_PAYLOADU];
volatile uint8_t crsf_index = 0;
volatile uint8_t crsf_new_data = 0;

volatile uint16_t napeti = 0;
volatile uint8_t procento_napeti = 0;

uint8_t colors[NUM_LEDS * 3];

volatile uint16_t milis_cnt=0;

//-------Deklarace funkc�---------//

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
	
	stdout = &usart_stream;  // P�esm�rov�n� printf na UART
	
	sei();							// povolen� p�eru�en�

	static uint8_t stav = 0;
	
	static uint16_t time = 0;
	static uint16_t time_motory = 0;
	static uint16_t time_sparovani = 0;
	
	time = milis();
	time_motory = milis();
	time_sparovani = milis();
	
	static uint8_t led_on = 0;				// 0 = LED vypnut�, 1 = LED zapnut�
	static uint16_t time_osvetleni = 0;     // �as aktivace osv�tlen�
	
	sparovani();
	
	while (1) {
		milis();
		
		uint16_t *channels = dekodovani();	// Tetno ukazatel ukazuje na pole dek�dovan�ch kan�l�
		
		uint16_t channel_0 = channels[0];		// Zat��en� prav� joystick min 174, stred 968, max 1808
		uint16_t channel_1 = channels[1];		// Rychlost prav� joystick min 174, stred 968, max 1797
		uint16_t channel_2 = channels[2];		// Rychlost lev�  joystick min 174, stred 968, max 1797
		uint16_t channel_3 = channels[3];		// Zat��en� lev�  joystick min 174, stred 968, max 1773
		uint16_t channel_4 = channels[4];		// SA 191 vypnuto, 1792 zapnuto
		uint16_t channel_5 = channels[5];		// SB 191 zapnuto, 1792 vypnuto
		uint16_t channel_6 = channels[6];		// SC 191 zapnuto, 1792 vypnuto
		uint16_t channel_7 = channels[7];		// SD 1792 vypnuto, 191 zapnuto
		uint16_t channel_8 = channels[8];		// SE 1792 stisk,   191 zapnuto
		uint16_t channel_9 = channels[9];		// SF 1792 stisk,   191 vypnuto
		
		//printf_P(PSTR("Kan�l %u: %x\r\n"), 0, channel_0);		//odes�l�n� hodnot p�es UART pro v�pis v termin�lov�m emul�toru (PuTTY), pro kontrolu
		
		if (milis()-time_motory > 500 && channel_5 == 191)
		{
			motory_dopredu(channel_1,channel_3);			// Ovl�d�n� motor� p�i sm�ru vp�ed
			time_motory=milis();
		}
		
		if (milis()-time_motory > 500 && channel_5 == 1792)
		{
			motory_dozadu(channel_1,channel_3);					// Ovl�d�n� motor� p�i sm�ru vp�ed
			time_motory=milis();
		}
		
		if (crsf_new_data) {									// Kdy� byla p�ijat� nov� data
			crsf_new_data = 0;
			
			// Vypu�t�n� n�vnady
			if (channel_9 == 1792 && stav == 0) {
				// Ulo��me �as a nastav�me stav
				time_osvetleni = milis();
				stav = 1;
				
				if (!led_on) {									// Rozsv�t�me sv�tla jednor�zov�, pokud je�t� nejsou zapnut�, signalizace vypu�t�n� n�vnady
					PORTD_OUTSET = PIN0_bm;						// Zapnut� zadn�ch sv�tel
					for (int i = 0; i < NUM_LEDS * 3; i++) {	//Z�pis hodnot pro rozsv�cen� LED p�sku b�lou barvou
						colors[i] = 0xFF;
					}
					test(colors, sizeof(colors));				// Odesl�n� pole colors do LED p�sku
					led_on = 1;
				}
			}
			
			// Ovl�d�n� osv�tlen�, pokud nen� pr�v� vypou�t�na n�vnada
			if (stav == 0) {
				if (channel_4 == 1792 && channel_7 == 191) {	// Zapnut� sv�tla
					PORTD_OUTSET = PIN0_bm;						// Zapnut� zadn�ch sv�tel
					for (int i = 0; i < NUM_LEDS * 3; i++) {	// Zapnut� LED p�sku - b�l� barva
						colors[i] = 0xFF;
					}
					test(colors, sizeof(colors));				// Odesl�n� pole colors do LED p�sku
				}
				if (channel_4 == 191 && channel_7 == 191) {		// Vypnut� sv�tla
					// Vypnut� LED pro dal�� osv�tlen�
					PORTD_OUTCLR = PIN0_bm;						// Vypnut� zadn�ch sv�tel
					for (int i = 0; i < NUM_LEDS * 3; i++) {	// Z�pis hodnot pro zhasnut� LED p�sku
						colors[i] = 0x00;
					}
					test(colors, sizeof(colors));				// Odesl�n� pole colors do LED p�sku
				}
			}
			
			ad_prevodnik();										// Kontrola stavu nabit� baterie
		}														// konec zpracov�n� nov�ch paket�

		// Kontinu�ln� aktualizace p�i vypu�t�n� n�vnady
		if (stav == 1) {
			servo_set_position(1);								// Vypu�t�n� n�vnady
			
			if (milis() - time_osvetleni >= 50000) {			// Pokud uplynul po�adovan� �as deaktivujeme re�im
				stav = 0;
			}
			} else {											// stav == 0 -> pr�v� se nevypou�t� n�vnada
			servo_set_position(0);								// Servo do v�choz� polohy
			
			if (led_on) {										// Pokud byly LED aktivov�ny p�i vypou�t�n� n�vnady, jednor�zov� je vypneme
				PORTD_OUTCLR = PIN0_bm;
				for (int i = 0; i < NUM_LEDS * 3; i++) {
					colors[i] = 0x00;
				}
				test(colors, sizeof(colors));
				led_on = 0;
			}
		}
		
		if (milis()-time_sparovani>50000)			// Kontrola sp�rov�n�
		{
			time_sparovani = milis();
			sparovani();
		}
	}
}

// Rutina p�eru�en� pro p��jem dat z UARTu
ISR(USART0_RXC_vect) {
	uint8_t data = USART0.RXDATAL;					// P�e�ten� p�ijat�ho bajtu z UARTu, do�asn� ulo�en� do prom�nn� data

	if (crsf_index == 0 && data != CRSF_HEADER) {	// Pokud adresov� bajt nen� roven C8 -> chybn� paket, vr�cen�
		return;
	}

	crsf_buffer[crsf_index++] = data;				// Ulo�en� dat do bufferu
	
	if (crsf_index > CRSF_VELIKOST_PAYLOADU) {		// Reset buffer, kdy� je ulo�eno a� moc dat
		crsf_index = 0;
		return;
	}
	
	if (crsf_index == CRSF_VELIKOST_PAYLOADU) {		// Kompletn� paket
		crsf_index = 0;								// P�ipraven� pro dal�� paket
		crsf_new_data = 1;
	}
}

// Inicializace UART
void usart_init(void) {
	PORTA.DIRCLR = PIN1_bm;  // Rx
	PORTA.OUTSET = PIN0_bm;	 // Nastaven� Rx do logick� 1 pro p��jem dat
	PORTA.DIRSET = PIN0_bm;  // Tx
	
	USART0.BAUD = BAUDVAL;	 // Nastaven� rychlosti komunikace
	USART0.CTRLB |= USART_RXEN_bm;
	USART0.CTRLA |= USART_RXCIE_bm;  // Povolit p�eru�en� od Rx
	USART0.CTRLB |= USART_TXEN_bm;
}

// P�epnut� mikro�ipu na frekvenci 20 MHz
void clock_20MHz(void) {
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;
	CCP = CCP_IOREG_gc;
	CLKCTRL.OSCHFCTRLA = CLKCTRL_FRQSEL_20M_gc;
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB = 0;
}

// Rutina p�eru�n� pro millis()
ISR(TCB0_INT_vect){
	TCB0.INTFLAGS = TCB_CAPT_bm;
	milis_cnt++;
}

// funkce pro m��en� �asu
uint16_t milis(void){
	uint16_t tmp;
	TCB0.INTCTRL &=~ TCB_CAPT_bm;	//vynulovat capt bit = vypnout p�eru�en� od TCB
	tmp = milis_cnt;
	TCB0.INTCTRL = TCB_CAPT_bm;		//zapnout zp�t p�eru�en� od TCB
	return tmp;
}

// Funkce pro odes�l�n� dat p�es UART do
int usart_putchar(char var, FILE *stream) {
	while (!(USART0.STATUS & USART_DREIF_bm)) {}
	USART0.TXDATAL = var;
	return 0;
}

//Inicializace
void inicializace(void){
	// Nastaven� pro funkci milis()
	TCB0.CTRLA = TCB_CLKSEL_DIV1_gc | TCB_ENABLE_bm;
	TCB0.CTRLB = TCB_CNTMODE_INT_gc;
	TCB0_INTCTRL = TCB_CAPT_bm;
	TCB0_CCMP = 999;
	
	// Nastaven� SPI pro ovl�d�n� LED p�sku
	PORTA.DIRSET = PIN4_bm;
	SPI0.CTRLB = SPI_SSD_bm | SPI_MODE_3_gc | SPI_BUFEN_bm;
	SPI0.CTRLA = SPI_MASTER_bm | SPI_PRESC_DIV4_gc |SPI_CLK2X_bm| SPI_ENABLE_bm;
	
	// Nastaven� AD p�evodn�ku se vzorkovac� frekvenc� 10 ms, Pin PA1, referen�n� nap�t� 4,096 V
	PORTD.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;		//vypnut� vstupn�ho bufferu
	VREF.ADC0REF = VREF_REFSEL_4V096_gc;
	ADC0.CTRLC = ADC_PRESC_DIV20_gc;
	ADC0.SAMPCTRL = 10;
	ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc;
	ADC0.CTRLA = ADC_ENABLE_bm;
	
	
	// Nastaven� v�stupn�ch pin�
	PORTD_DIRSET = PIN7_bm;			// 0 red	- v�stra�n�
	PORTD_DIRSET = PIN6_bm;			// 1 green, > 25 %
	PORTD_DIRSET = PIN5_bm;			// 2 green, > 25 %
	PORTD_DIRSET = PIN4_bm;			// 3 green, > 50 %
	PORTD_DIRSET = PIN3_bm;			// 4 green, > 75 %
	PORTD_DIRSET = PIN2_bm;			// Sp�rovac� LED
	PORTD_DIRSET = PIN1_bm;			// Vstup AD p�evodn�k
	PORTD_DIRSET = PIN1_bm;			// Zadn� sv�tla
	
	PORTD_OUTCLR = PIN0_bm;			// vypnut� zadn�ch sv�tel
}

// Inicializace PWM
void pwm_init(void) {
	TCA0.SINGLE.PER = 49999; // Nastaven� periody PWM na 20 ms (50 Hz)

	TCA0.SINGLE.CMP0 = 3749; // Start na 0� (���ka pulzu = 1 ms)
	TCA0.SINGLE.CMP1 = 3749; // Start na 0� (���ka pulzu = 1 ms)
	TCA0.SINGLE.CMP2 = 3749; // Start na 0� (���ka pulzu = 1 ms)
	
	TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_CMP2EN_bm |// Povolen� v�stupu Compare Channel 0 (PA0)
	TCA_SINGLE_WGMODE_SINGLESLOPE_gc; // Re�im: Single-slope PWM
	
	TCA0.SINGLE.CTRLA = (1 << TCA_SINGLE_ENABLE_bp) | (TCA_SINGLE_CLKSEL_DIV8_gc);			//Nastaven� p�edd�li�ky a povolen� timeru
	
	PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTC_gc;		//P�esm�rov�n� PWM na port C

	PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm;		// Nastaven� pinu PC0, PC1, PC2 jako v�stupn�ho pro PWM
}

// Funkce pro ovl�d�n� motor� p�i chodu vp�ed
void motory_dopredu(uint16_t rychlost, uint16_t zatoceni) {
	// MIN_PWM = 1.5ms			(neutr�ln� � motory stoj�)
	// MAX_PWM = 2.0ms			(maxim�ln� vp�ed)
	// MAX_REVERSE_PWM = 1.0ms  (maxim�ln� zp�tn� chod)
	static uint16_t last_zatoceni = 970;
	static uint16_t last_rychlost = 174;
	// P�edchoz� hodnoty PWM pro vyhlazen� zm�n
	static uint16_t last_left_pwm = MIN_PWM;
	static uint16_t last_right_pwm = MIN_PWM;
	
	uint16_t base_pwm;
	uint16_t rozdil;
	uint16_t procenta;
	uint16_t left_pwm, right_pwm;
	
	// Pokud je rychlost pod minimem, nastav�me neutr�ln� hodnoty (motory nestoj�)
	if (rychlost < 174) {
		left_pwm = MIN_PWM;
		right_pwm = MIN_PWM;
		} else {
		// V�po�et z�kladn� PWM hodnoty pro vp�ed
		base_pwm = MIN_PWM + ((rychlost - 174) / 2);
		if (base_pwm > MAX_PWM)
		base_pwm = MAX_PWM;
		if (base_pwm < MIN_PWM)
		base_pwm = last_rychlost;
		
		rozdil = base_pwm - MIN_PWM;
		
		// Zat��en� doleva � pokud je zatoceni < 470
		// p�i maxim�ln�m zat��en� (zatoceni = 200) bude lev� motor v pln�m reverse
		if (zatoceni < 470) {
			procenta = (470 - zatoceni) * 100UL / (470 - 200);				// procentu�ln� zm�na od neutr�lu
			left_pwm = MIN_PWM - (procenta * (base_pwm - MIN_PWM)) / 100;
			right_pwm = base_pwm;
		}
		// M�rn� zat��en� doleva (470 < zatoceni < 968)
		else if (zatoceni < 968) {
			procenta = (968 - zatoceni) * 100UL / (968 - 470);
			uint16_t zp = (procenta * rozdil) / 100;
			left_pwm = base_pwm - zp;										// lev� motor zpomaluje vp�ed
			if (left_pwm < MIN_PWM)
			left_pwm = MIN_PWM;
			right_pwm = base_pwm;
		}
		// Neutr�ln� oblast: 968 < zatoceni < 973
		else if (zatoceni <= 973) {
			left_pwm = base_pwm;
			right_pwm = base_pwm;
		}
		// M�rn� zat��en� doprava (973 < zatoceni < 1400)
		else if (zatoceni <= 1400) {
			procenta = (zatoceni - 973) * 100UL / (1400 - 973);
			uint16_t zp = (procenta * rozdil) / 100;
			left_pwm = base_pwm;
			right_pwm = base_pwm - zp;  // prav� motor deceleruje vp�ed
			if (right_pwm < MIN_PWM)
			right_pwm = MIN_PWM;
		}
		// Zat��en� doprava � pokud je zatoceni > 1400
		// p�i maxim�ln�m zat��en� (zatoceni = 1700) bude prav� motor v pln�m zp�tn�m chodu
		else {
			procenta = (zatoceni - 1400) * 100UL / (1700 - 1400); // 0 a� 100
			right_pwm = MIN_PWM - (procenta * (base_pwm - MIN_PWM)) / 100;
			left_pwm = base_pwm;
		}
	}
	
	// Zaji�t�n� zm�ny rychlosti motor� pouze o malou zm�nu, kv�li plynul�mu ovl�d�n� motor�
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
	
	// Nastaven� PWM v�stup� pro motory
	TCA0.SINGLE.CMP1 = left_pwm;
	TCA0.SINGLE.CMP2 = right_pwm;
	
	// Ulo�en� aktu�ln�ch hodnot pro dal�� vol�n�
	last_zatoceni = zatoceni;
	last_rychlost = base_pwm;
	last_left_pwm = left_pwm;
	last_right_pwm = right_pwm;
}

// Funkce pro ovl�d�n� motor� p�i zp�tn�m chodu
void motory_dozadu(uint16_t rychlost, uint16_t zatoceni) {
	static uint16_t last_zatoceni = 970;
	static uint16_t last_rychlost = 174;
	static uint16_t last_left_pwm = MIN_PWM;
	static uint16_t last_right_pwm = MIN_PWM;
	
	uint16_t base_pwm;
	uint16_t procenta;
	uint16_t zpomaleni;
	uint16_t left_pwm, right_pwm;
	// Rozd�l mezi neutr�lem a maxim�ln� revers�
	uint16_t rozdil = MIN_PWM - MAX_REVERSE_PWM;
	
	// P�i n�zk� rychlosti nastav�me neutr�ln� hodnotu � motory stoj�
	if (rychlost < 174) {
		left_pwm = MIN_PWM;
		right_pwm = MIN_PWM;
		} else {
		// V�po�et z�kladn� PWM hodnoty pro zp�tn� chod:
		// P�i rostouc� hodnot� "rychlost" se base_pwm sni�uje od neutr�lu sm�rem k MAX_REVERSE_PWM.
		base_pwm = MIN_PWM - ((rychlost - 174) / 2);
		if (base_pwm < MAX_REVERSE_PWM)
		base_pwm = MAX_REVERSE_PWM;
		if (base_pwm > MIN_PWM)
		base_pwm = last_rychlost; // Tato v�tev by nem�la nastat
		
		// Diferenci�ln� ��zen� podle zat��en�
		if (zatoceni < 470) {
			// Maxim�ln� lev� zat��en�:
			procenta = (470 - zatoceni) * 100UL / (470 - 200);
			zpomaleni = (procenta * rozdil) / 100;
			left_pwm = base_pwm - zpomaleni;   // je�t� v�ce reverse
			right_pwm = base_pwm + zpomaleni;    // m�n� reverse
			if (right_pwm > MIN_PWM) right_pwm = MIN_PWM;
		}
		else if (zatoceni < 968) {
			// M�rn� zat��en� doleva:
			procenta = (968 - zatoceni) * 100UL / (968 - 470);
			uint16_t diff = MIN_PWM - base_pwm;
			zpomaleni = (procenta * diff) / 100;
			left_pwm = base_pwm - zpomaleni;
			right_pwm = base_pwm + zpomaleni;
			if (right_pwm > MIN_PWM) right_pwm = MIN_PWM;
		}
		else if (zatoceni <= 973) {
			// Neutr�ln� oblast � ��dn� diferenciace
			left_pwm = base_pwm;
			right_pwm = base_pwm;
		}
		else if (zatoceni <= 1400) {
			//M�rn� zat��en� doprava:
			procenta = (zatoceni - 973) * 100UL / (1400 - 973);
			uint16_t diff = MIN_PWM - base_pwm;
			zpomaleni = (procenta * diff) / 100;
			left_pwm = base_pwm + zpomaleni;		// Lev� motor se "zm�r�uje" sm�rem k neutr�lu.
			right_pwm = base_pwm - zpomaleni;		// Pro zat��en� doprava v re�imu reverse: prav� motor jde v�ce zp�t
			if (left_pwm > MIN_PWM) left_pwm = MIN_PWM;
		}
		else {
			//Maxim�ln� prav� zat��en�:
			procenta = (zatoceni - 1400) * 100UL / (1700 - 1400);
			zpomaleni = (procenta * rozdil) / 100;
			left_pwm = base_pwm + zpomaleni;	 // lev� motor se "zm�r�uje"
			right_pwm = base_pwm - zpomaleni;    // prav� motor jde je�t� v�ce reverse
			if (left_pwm > MIN_PWM) left_pwm = MIN_PWM;
		}
	}
	
	// Zaji�t�n� zm�ny rychlosti motor� pouze o malou zm�nu, kv�li plynul�mu ovl�d�n� motor�
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
	
	// Nastaven� PWM v�stup�
	TCA0.SINGLE.CMP1 = left_pwm;
	TCA0.SINGLE.CMP2 = right_pwm;
	
	// Ulo�en� aktu�ln�ch hodnot pro dal�� vol�n�
	last_zatoceni = zatoceni;
	last_rychlost = base_pwm;
	last_left_pwm = left_pwm;
	last_right_pwm = right_pwm;
}

// Funkce pro ovl�d�n� ot��en� servomotor�
void servo_set_position(uint8_t position) {
	if (position == 0) {
		TCA0.SINGLE.CMP0 = SERVO_0_DEGREE;		// Neutr�ln� pozice
		} else if (position == 1) {
		TCA0.SINGLE.CMP0 = SERVO_90_DEGREE;		// Vypu�t�n� n�vnady
	}
}

// Funkce pro indikaci stavu nabit� baterie zobrazovan�ho na bargrafu
void ad_prevodnik(void)
{
	ADC0.COMMAND = ADC_STCONV_bm;				// Zah�jen� m��en�
	while(ADC0.COMMAND & ADC_STCONV_bm){}		// �ek�me na zm�nu spconv
	napeti = ADC0.RES;							// Ulo�en� zm��en� hodnoty
	if(napeti>MIN_NAPETI){
		procento_napeti = (((uint32_t)napeti-MIN_NAPETI) * 100) / (MAX_NAPETI-MIN_NAPETI);	// P�epo�et na procenta
		}else{
		procento_napeti = 0;
	}
	
	if (procento_napeti>25)			// Pokud jsou procenta v�t�� ne� 25 -> vypni v�stra�nou LED, rozsvi� LED 1 a 2
	{
		PORTD_OUTCLR = PIN7_bm;
		PORTD_OUTSET = PIN6_bm;
		PORTD_OUTSET = PIN5_bm;
		if (procento_napeti>50)		// Pokud jsou procenta v�t�� ne� 50 -> rozsvi� LED 3
		{
			PORTD_OUTSET = PIN4_bm;
			if (procento_napeti>75)	// Pokud jsou procenta v�t�� ne� 75 -> rozsvi� LED 4
			{
				PORTD_OUTSET = PIN3_bm;
				}else{					// Pokud jsou procenta men�� ne� 75 -> zhasni LED 4
				PORTD_OUTCLR = PIN3_bm;
			}
			}else{						// Pokud jsou procenta men�� ne� 50 -> zhasni LED 3
			PORTD_OUTCLR = PIN4_bm;
		}
		
		}else{							// Pokud jsou procenta men�� ne� 25 -> zhasni v�echny LED, rozsvi� v�stra�nou LED
		PORTD_OUTSET = PIN7_bm;
		
		PORTD_OUTCLR = PIN6_bm;
		PORTD_OUTCLR = PIN5_bm;
		PORTD_OUTCLR = PIN4_bm;
		PORTD_OUTCLR = PIN3_bm;
	}
	
}

// Funkce pro z�sk�n� 11bitov� hodnoty kan�lu
uint16_t hodnota_kanalu(uint8_t *data, uint8_t channel) {

	uint16_t bitOffset = channel * 11;		// Kolik je p�ed kan�lem bit� p�edchoz�ch kan�l�s
	uint8_t byteOffset = bitOffset / 8;		// Ur�en� bajtov�ho offsetu
	uint8_t bitInByte = bitOffset % 8;		// Kolik bit� je posunuto v r�mci dan�ho bajtu
	uint32_t raw = data[byteOffset]			// Na�ten� 3 bajt�, proto�e 11 bit� m��e b�t rozm�st�no maxim�ln� p�es 3 bajty
	| (data[byteOffset + 1] << 8)
	| (data[byteOffset + 2] << 16);
	
	return (raw >> bitInByte) & 0x07FF;		// Posuneme hodnotu doprava a maskujeme nejni���ch 11 bit�
}

// Funkce pro dekodov�n� kan�l�
uint16_t * dekodovani(void) {
	static uint16_t channels[NUM_CHANNELS];
	uint8_t *data = &crsf_buffer[CRSF_ZACATEK_KANALU];	// Ukazatel na data kan�l� v crsf_buffer, za��n� od definovan�ho offsetu

	for (uint8_t i = 0; i < NUM_CHANNELS; i++) {		// Dek�dujeme kan�ly 0 a� 15 (co� odpov�d� kan�l�m 1 a� 16)
		channels[i] = hodnota_kanalu(data, i);
	}

	return channels;
}

// Funkce pro odes�l�n� dat pro LED p�sek
void test(uint8_t* data, uint16_t length){
	uint8_t mask;
	cli();			 // Vypnut� p�eru�en�
	while(length){   // Pro v�echny bajty z vstupn�ho rozsahu
		length--;
		mask=0b10000000; // Pro v�echny bity v bajtu
		while(mask){
			while (!(SPI0.INTFLAGS & SPI_DREIF_bm)){}	// �ek�n� na pr�zdn� SPI buffer
			if(mask & data[length]){					// Posl�n� pulzu s s koresponuj�c� d�lkou pro "L" nebo "H"
				SPI0.DATA = H_PATTERN;
				}else{
				SPI0.DATA = L_PATTERN;
			}
			mask = mask >> 1;							// Bitov� posun, abychom mohli maskovat dal�� bit
		}
	}
	sei();			// Zapnut� p�eru�en�
}

// Funkce pro ov��en� sp�rov�n� za��zen�
void sparovani(void){
	uint8_t pocet_sparovani = 0;
	static uint16_t time_led = 0;
	static uint16_t time_ledpasek = 0;
	time_ledpasek = milis();
	time_led = milis();
	while (1)
	{
		uint8_t data = USART0.RXDATAL;					// Na�ten� p�ijat�ho bajtu
		// Kontrola hlavi�kov�ho bajtu, pokud se nerovn� -> spu�t�n� animace LED a blik�n� LED indikuj�c� p�rov�n�
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
			if (pocet_sparovani>50)						// Ov��en� sp�rov�n�
			{
				PORTD_OUTSET = PIN2_bm;					// Zapnut� LED bargrafu pro signalizaci sp�rov�n�
				break;									// Za��zen� je sp�rov�no
			}
			
		}
	}
}

// Funkce pro animace na LED p�sku
void LED_pasek(void)
{
	static int pos = 0;         // pozice hlavn� LED
	static int direction = 1;   // sm�r: 1 = doprava, -1 = doleva
	static uint8_t time_pasek = 0;
	time_pasek = milis();
	// vypnut� v�ech LED
	for (int i = 0; i < NUM_LEDS * 3; i++) {
		colors[i] = 0x00;
	}
	
	//ur�en� barev LED
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
	
	test(colors, sizeof(colors));			// Odesl�n� pole, kter� LED a jakou barvou m� sv�tit
	
	_delay_ms(40);
	
	pos += direction;						// Zm�na pozice animace
	
	if (pos >= NUM_LEDS - 1 || pos <= 0) {	// Pokud dojedeme nakonec LED p�sku, zm�n�me sm�r chodu animace
		direction = -direction;
	}
}

