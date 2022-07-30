#define F_CPU 6000000L
#include <avr/io.h>					// obsługa portów
#include <stdlib.h> 				// abs()
#include <util/delay.h> 		// _delay_ms()
#include <avr/interrupt.h> 	//dołączenie biblioteki z przerwaniami
#include <math.h>   				// pow(a,x)
#include <avr/eeprom.h>

#include "hd44780.c"

#include "dds_library.c"

 // te zmienne muszą być tutaj bo inaczej nie działa
int frq_digits[7];
char str[12];

// te zmienne musza byc globalne,
int dds_st 				    = 1; // 1 czyli przeliczyć on-load
int vfo_id 					= 0;
unsigned long frq[]		= {3700000, 3500000}; // vfo a/b    <----------------------------------------------------------------<<< USTAWIĆ
unsigned long frq_if		= 0;	// częstotliwość pośrednia  <----------------------------------------------------------------<<< USTAWIĆ
int step					= 25; // step startowy 	<----------------------------------------------------------------<<< USTAWIĆ


#define BTN_VFO_A_B		!(PIND & 0b10000000) // PD7
#define BTN_STEP		!(PINB & 0b00000001) // PB0


// ZMIANA WSTĘG
#define BTN_USB_LSB		!(PIND & 0b01000000) // PD6
#define CHANGE_USB 		PORTD ^= 0b00001000
#define IS_USB			(PIND & 0b00001000)

#define CHANGE_LSB 		PORTD ^= 0b00010000
#define IS_LSB			(PIND & 0b00010000)


// Przedwzmacniacz
#define BTN_RF_AMP		!(PIND & 0b00100000) // PD5
#define CHANGE_RF_AMP 	PORTD ^= 0b00000001	 // PD0
#define IS_RF_AMP 	    (PIND & 0b00000001) // PD0
  




////////////////////////////////
//   WYSYŁANIE DANYCH DO DDS  //
////////////////////////////////
void sendDataToDDS(unsigned long delta_reg){
	// PORTB.5 DATA 
	// PORTB.4 CLK 
	// PORTB.3 FQ-UP

	#define DDS_DATA_0 	  	PORTB &= 0b011111 // PB5
	#define DDS_DATA_1 	  	PORTB |= 0b100000

	#define DDS_CLK_ON 	  	PORTB |= 0b010000 // PB4
	#define DDS_CLK_OFF 	PORTB &= 0b101111

	#define DDS_FQ_UD_ON 	PORTB |= 0b001000 // PB3
	#define DDS_FQ_UD_OFF 	PORTB &= 0b110111


	long bit_id 			= 0;  // kolejny bit z z 40-sto bitowego słowa wgrywany do DDS

	bit_id = 0;
	while(bit_id < 40){
		if( bit_id < 32 )
		{
			if(  (delta_reg & 0b1) ) DDS_DATA_1;
			if( !(delta_reg & 0b1) ) DDS_DATA_0;
			delta_reg >>= 1;
		}
		else
		{
			DDS_DATA_0;
		}
		DDS_CLK_ON;
		DDS_CLK_OFF;
		bit_id++;
	}
	DDS_FQ_UD_ON;
	DDS_FQ_UD_OFF;
}// end send_data()







////////////////////////////////
//      OBLICZANIE DELTY      //
////////////////////////////////
unsigned long countDelta(unsigned long frq){

	int  dig_val;						   // tutaj wpadają kolejne cyfry z częstotliwości
	int  dig_id				    = 8;       // numer cyfry z częstotliwości
	int  dds_clk 				= 75;	   // częstotliwość generatora kwarcowego w MHz <----------------------------------------------------------------<<< USTAWIĆ
	unsigned long delta_mod 	= pow(2,32) / (0.005 + dds_clk) * 10; // delta dla 10MHz
	unsigned long delta 		= 0;       // liczba wprowadzana do dds

	dig_id 						= 7;       // idziemy od jedynek herców w strone megaherców // sprawdzić czemu wyżej jest 8 ??????????

	while(dig_id >= 0){
		dig_val = frq % 10;
		frq /= 10;
		frq_digits[ dig_id ] = dig_val;

		dig_id--;
	}

	delta 		= 0; // zerujemy po poprzednich obliczeniach
	dig_id 		= 0; // idziemy od dziesiątek megaherców (index 0) w stronę herców

	while(dig_id < 8){
		delta += delta_mod * frq_digits[ dig_id ];
		delta_mod /= 10;
		dig_id++;
	}
	return delta;
}






////////////////////////////////
//   WYSYŁANIE FRQ DO LCD     //
////////////////////////////////
void LCD_WriteFrq(unsigned long frq){
	int a = 9; //int a = 7;
	int dig_val;						// tutaj wpadają kolejne cyfry z częstotliwości

	while(a >= 0){
		if(frq > 0){
			dig_val = frq % 10 + 48;
			str[a] = dig_val;

			if(a == 3){a--; str[a] = 46;}// to są kropki
			if(a == 7){a--; str[a] = 46;}// to są kropki
		}
		else
		{
			if(a > 4){
				str[a] = 48;
				if(a == 7){a--; str[a] = 46;}// to są kropki
				if(a == 3){a--; str[a] = 46;}// to są kropki
			}else{
				str[a] = 32;
			}

		}
		frq /=10;
		a--;
	}

	LCD_WriteText(str);
}






void operateDisplay(void){

	LCD_Pos(11,0);
	switch (step){
		case 1: 		LCD_WriteText("  1Hz");break;
		case 10: 		LCD_WriteText(" 10Hz");break;
		case 25: 		LCD_WriteText(" 25Hz");break;
		case 50: 		LCD_WriteText(" 50Hz");break;
		case 100: 	LCD_WriteText("100Hz");break;
		case 500: 	LCD_WriteText("500Hz");break;
		case 1000: 	LCD_WriteText(" 1kHz");break;
		case 10000:	LCD_WriteText("10kHz");break;
		default: 	LCD_WriteText("  def");
	}

	// częstotliwosci
	LCD_Pos(0,0); LCD_WriteFrq( frq[0] );
	LCD_Pos(0,1); LCD_WriteFrq( frq[1] );

	//vfo a/b
	LCD_Pos(10,0); LCD_WriteText(" ");
	LCD_Pos(10,1); LCD_WriteText(" ");

	LCD_Pos(10,vfo_id); LCD_WriteText("<");


	if(IS_USB) { LCD_Pos(13,1); LCD_WriteText("USB"); }
	if(IS_LSB) { LCD_Pos(13,1); LCD_WriteText("LSB"); }

	if(IS_RF_AMP)  { LCD_Pos(11,1); LCD_WriteText("+"); }
	if(!IS_RF_AMP) { LCD_Pos(11,1); LCD_WriteText(" "); }
}





void DDS_setOutputFrq(unsigned long frq[], unsigned long frq_if, int vfo_id)
{
	unsigned long frq_dds;

	if(frq_if > 0){
		frq_dds = frq_if + frq[vfo_id];
		//frq_dds =  frq_if - frq[vfo_id];
	}else{
		frq_dds = frq[vfo_id];
	}
	sendDataToDDS( countDelta( frq_dds ) );// ustawiamy Fout
}






/////////////////////////////////////
//  OBSŁUGA PRZERWANIA - ENKODER  ///
/////////////////////////////////////
SIGNAL(SIG_INTERRUPT0) //początek funkcji obsługi przerwania
{
	//przestrajanie
	dds_st = 1; // zmieniono częstotliwość
	frq[vfo_id] = frq[vfo_id]/step*step;
	if(  (PIND & 0b00000010) ) frq[vfo_id] += step;
	if( !(PIND & 0b00000010) ) frq[vfo_id] -= step;

	// ograniczenie krańców pasma
	if(frq[vfo_id] < 30000) 		frq[vfo_id] = 30000;
	if(frq[vfo_id] > 30000000) 	frq[vfo_id] = 30000000;
}






/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
int main(void)
{
	unsigned int dly_1			= 0;
	unsigned int lock_1 		= 0;
	unsigned int lock_2	  = 0;

		// tryby					// stany
		// 1=out/ 0=in

	DDRB  = 0b000000; 	PORTB = 0b111111;		_delay_ms(200); // przeładowanie portów zeby działał dds
	DDRB  = 0b111000; 	PORTB = 0b000001;		_delay_ms(200); // sterowanie DDS-em  i dwa guziki
																					// PB0  STEP

	DDRC  = 0b111111; 	PORTC = 0b000000;		// LCD

	DDRD  = 0b10011001;	PORTD = 0b11100110;	   // PD1, PD2 Impulsator


	// PD1, PD2 Impulsator
	
    // PD7  VFO A/B


	LCD_Initalize();
	LCD_Clear();
	LCD_WriteText("80m QRP TRX"); 
	
	LCD_Pos(0,1);
	LCD_WriteText("AD9850 & NE612 "); //  <----------------------------------------------------------------<<< USTAWIĆ

	_delay_ms(3000);
	LCD_Clear();


	// włączamy dolną wstęgę
	CHANGE_LSB;




	// przerwania dla impulsatora
	// impulsator podpięty do PD2 i PD1;
	MCUCR |= 0b00000011; // w resejstrze MCUCR bity ICS01, ICS00 na H (Zbocze narastające na INT0 generuje przerwanie)
	GICR  |= 0b01000000; // włączamy INT0
	sei(); //globalne włączenie przerwań








	/* Początek nieskończonej pętli */
	while(1)
	{
		//////////////////////////////////////////
		//     jeśli zmieniono częstotliwość    //
		//////////////////////////////////////////
		if(dds_st != 0){

			DDS_setOutputFrq(frq, frq_if, vfo_id);
			operateDisplay();

			dds_st = 0; //zablokuj liczenie do czasu kolejnej zmiany częstotliwości
		}

		/////////////////////////
		//       VFO A/B       //
		/////////////////////////
		else if( BTN_VFO_A_B )
		{
			if(lock_1 == 0)
			{
				if(lock_2 == 10000)
				{
					vfo_id = abs(vfo_id - 1);

					operateDisplay();

					lock_1 = 1;
					dds_st = 1;
				}
				lock_2++;
			}
		}

		/////////////////////////
		//       USB/LSB       //
		/////////////////////////
		else if( BTN_USB_LSB )
		{
			if(lock_1 == 0)
			{
				if(lock_2 == 10000)
				{
					CHANGE_USB;
					CHANGE_LSB;
					operateDisplay();
				}
				lock_2++;
			}
		}

		/////////////////////////
		//       RF-AMP        //
		/////////////////////////
		else if( BTN_RF_AMP )
		{
			if(lock_1 == 0)
			{
				if(lock_2 == 10000)
				{
					CHANGE_RF_AMP;
					operateDisplay();
				}
				lock_2++;
			}
		}


		///////////////////////
    //       STEP        //
    ///////////////////////
		else if( BTN_STEP )
		{
			if(lock_1 == 0)
			{
				if(lock_2 == 10000)
				{
					lock_1 = 1;

					switch (step) {
						case 1: 		step = 10;		break;
					 	case 10: 		step = 25;		break;
					 	case 25: 		step = 50;		break;
						case 50:		step = 1000;	break;
						case 1000:	step = 10000;	break;
						case 10000:	step = 1;	    break;

					  default:		step = 25;
				 	}
				 	operateDisplay();
				}
				lock_2++;
			}
		}
		////////////////////////////////
    //  PO ZWOLNIENIU PRZYCISKÓW  //
    ////////////////////////////////
		else
		{
			dly_1		= 0;
			lock_1	= 0;
			lock_2	= 0;
		}
	}// end main loop

}// end main()
