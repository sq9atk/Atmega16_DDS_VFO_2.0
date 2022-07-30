
void PO_configureIO(void){
    // DDRx  : 0 = IN, 1 = OUT
    // PORTx : 0 = Lo, 1 = Hi

    DDRB  = 0b000000;   PORTB = 0b111111;	_delay_ms(200); // przeładowanie portów zeby działał dds
    DDRB  = 0b111000;   PORTB = 0b000001;   _delay_ms(200); // sterowanie DDS-em  i dwa guziki
                                                                                    // PB0  STEP
    DDRC  = 0b111111;   PORTC = 0b000000;   // LCD

    DDRD  = 0b10011001; PORTD = 0b11100110; // PD1, PD2 Impulsator
}

void PO_enableInterrupts()
{   
    // przerwania dla impulsatora
	// impulsator podpięty do PD2 i PD1;
	MCUCR |= 0b00000011; // w resejstrze MCUCR bity ICS01, ICS00 na H (Zbocze narastające na INT0 generuje przerwanie)
	GICR  |= 0b01000000; // włączamy INT0
	sei(); //globalne włączenie przerwań	
}
	
void PO_helloMessage(void)
{	
	LCD_Clear();
	LCD_WriteText("SQ9ATK");
    _delay_ms(1000);
    LCD_Clear();
    
}

void PO_WriteFrq(unsigned long frq)
{
	int 	a = 9; 
	int 	dig_val;						// tutaj wpadają kolejne cyfry z częstotliwości
	char 	str[] = "00.000.000";

	while(a >= 0){
		if(frq > 0){
			
			dig_val = frq % 10 + 48;
			str[a] = dig_val;

			if (a == 3) {a--; str[a] = 46;} // to są kropki
			if (a == 7) {a--; str[a] = 46;} // to są kropki
			
		}else{
			
			if(a > 4){
				str[a] = 48;
				if (a == 7) {a--; str[a] = 46;} // to są kropki
				if (a == 3) {a--; str[a] = 46;} // to są kropki
			}else{
				str[a] = 32;
			}
		}
		frq /= 10;
		a--;
	}

	LCD_WriteText(str);
}

void PO_operateDisplay(void) 
{
	LCD_Pos(0,0); PO_WriteFrq( frq[0] );
	LCD_Pos(0,1); PO_WriteFrq( frq[1] );
	
	LCD_Pos(10,vfo_id); LCD_WriteText("<");
}




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

    int  frq_digits[7];
	int  dig_val;						   // tutaj wpadają kolejne cyfry z częstotliwości
	int  dig_id				    = 7;       // numer cyfry z częstotliwości
	int  dds_clk 				= 125;	   // częstotliwość generatora kwarcowego w MHz <----------------------------------------------------------------<<< USTAWIĆ
	unsigned long delta_mod 	= pow(2,32) / dds_clk * 10; // delta dla 10MHz
	unsigned long delta 		= 0;       // liczba wprowadzana do dds


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


void PO_operateDDS(void) 
{
	sendDataToDDS( countDelta( frq[vfo_id] ));// ustawiamy Fout );
}


