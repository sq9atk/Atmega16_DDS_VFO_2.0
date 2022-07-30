
#define F_CPU 6000000L
#include <avr/io.h>                    // obsługa portów
#include <stdlib.h>                 // abs()
#include <util/delay.h>         // _delay_ms()
#include <avr/interrupt.h>     //dołączenie biblioteki z przerwaniami

#include "hd44780.c"

int32_t frq[]         = {3722000, 7150000};
int32_t bfo[]         = {8999780, 8997000};
int32_t if_shift[]    = {0, 0};

int16_t dial_div      = 0;

int8_t dds_st         = 0;
int8_t vfo_id         = 0;
int8_t bfo_id         = 1;
int8_t screen_id      = 0;

int8_t on_air         = 0;
int8_t on_air_lock    = 0;




char step_1[]         = "  1Hz";
char step_2[]         = "  5Hz";
char step_3[]         = " 10Hz";
char step_4[]         = " 25Hz";
char step_5[]         = " 50Hz";
char step_6[]         = "100Hz";
char step_7[]         = " 1kHz";
char step_8[]         = "10kHz";
char *step_names[8]   = {step_1, step_2, step_3, step_4, step_5, step_6, step_7, step_8};

int16_t steps[]       = {1,5,10,25,50,100,1000,10000};
int16_t step_id       = 4; 

int8_t  fast_lock_1   = 0;
int16_t fast_lock_2   = 0;
int8_t  fast_step_id  = 7;
int8_t  step_tmp      = -1;

#define BTN_A	!(PIND & 0b00001000) // PB3
#define BTN_B   !(PIND & 0b00010000) // PB4
#define BTN_C   !(PIND & 0b00100000) // PB5
#define BTN_D   !(PIND & 0b01000000) // PB6
#define BTN_E   !(PIND & 0b10000000) // PD7

#define ON_AIR	on_air == 1
#define IS_USB  bfo_id == 0
#define IS_LSB  bfo_id == 1

#define ROTATE_LEFT		!(PIND & 0b00000010)
#define ROTATE_RIGHT    (PIND & 0b00000010)




////////////////////////////////
//   WYSYŁANIE DANYCH DO DDS  //
////////////////////////////////
void sendDataToDDS(int32_t delta_reg, int8_t dds_id){
    // PORTB.5 DATA 
    // PORTB.4 CLK 
    // PORTB.3 FQ-UP VFO
    // PORTB.2 FQ-UP BFO

    #define DDS_DATA_0      PORTB &= 0b011111 // PB5
    #define DDS_DATA_1      PORTB |= 0b100000

    #define DDS_CLK_ON      PORTB |= 0b010000 // PB4
    #define DDS_CLK_OFF     PORTB &= 0b101111

    #define DDS_FQ_UD_ON    PORTB |= 0b001000 // PB3
    #define DDS_FQ_UD_OFF   PORTB &= 0b110111

    #define BFO_FQ_UD_ON    PORTB |= 0b000100 // PB2
    #define BFO_FQ_UD_OFF   PORTB &= 0b111011
    
    int8_t bit_id = 0;  // kolejny bit z z 40-sto bitowego słowa wgrywany do DDS

    bit_id = 0;
    while(bit_id < 40){
        if (bit_id < 32) { 
            if(  (delta_reg & 0b1) ) DDS_DATA_1;
            if( !(delta_reg & 0b1) ) DDS_DATA_0;
            delta_reg >>= 1;
        } else {
            DDS_DATA_0;
        }
        DDS_CLK_ON;
        DDS_CLK_OFF;
        bit_id++;
    }
    
    if(dds_id == 0) { DDS_FQ_UD_ON; DDS_FQ_UD_OFF; }
    if(dds_id == 1) { BFO_FQ_UD_ON; BFO_FQ_UD_OFF; }
}// end send_data()


////////////////////////////////
//      OBLICZANIE DELTY      //
////////////////////////////////
int32_t delta(int32_t frq){
    
    int8_t  position      = 7; // numer cyfry z częstotliwości
    int8_t  digit         = 0; // tutaj wpadają kolejne cyfry z częstotliwości
    int8_t  frq_digits[]  = {0,0,0,0,0,0,0,0};
    int8_t  dds_clk       = 125; // frq xGEN
    
    int32_t delta_mod     = pow(2,32) / dds_clk * 10; // delta dla 10MHz
    int32_t delta         = 0; // liczba wprowadzana do dds

    while (position >= 0) {
        digit = frq % 10;
        frq /= 10;
        frq_digits[ position ] = digit;

        position--;
    }

    delta = 0;    
    // zerujemy po poprzednich obliczeniach
    position = 0; // idziemy od dziesiątek megaherców (index 0) w stronę herców
    while (position < 8) {
        delta += delta_mod * frq_digits[ position ];
        delta_mod /= 10;
        position++;
    }
    
    return delta;
}


void LCD_WriteFrq(int32_t frq){
    
    int negative = 0;    
    if (frq<0) {
        negative = 1;
        frq = abs(frq);
    }
    
    char string[11] = {' '};

    int i;
    for(i=9; i>=0; i--, frq /= 10){        
        if(i%4 == 2) {
            string[i] = '.'; 
            i--;
        }
        string[i] = "0123456789"[frq%10];
    }    
    
    // usuwamy zera wiodące i dodajemy minus jesli wyświetlamy liczbę ujemną
    for(i=0; i<=4; ++i){
        if(string[i] != '0' && string[i] != '.') break;
        string[i] = ' ';
    }
    
    
    for(i=0; i<=10; ++i){
        if(string[i+1] != ' ') {
            if(negative ==1){
                string[i] = '-';
                break;
            }
        }
    }
    
    LCD_WriteText(string);
}


void operateDisplay(void){     
    if (screen_id == 0) {
        
        // frq
        LCD_Pos(0,0); LCD_WriteFrq(frq[0]);
        LCD_Pos(0,1); LCD_WriteFrq(frq[1]);


        //vfo a/b
        LCD_Pos(10, abs(vfo_id-1)); LCD_WriteText(" ");
        LCD_Pos(10, vfo_id);        LCD_WriteText("<");
        
        if (step_tmp >=  0) { LCD_Pos(12,1); LCD_WriteText("F"); }
        if (step_tmp == -1) { LCD_Pos(12,1); LCD_WriteText(" "); }
        
        if (bfo_id == 0) { LCD_Pos(12,0); LCD_WriteText(" LSB"); }
        if (bfo_id == 1) { LCD_Pos(12,0); LCD_WriteText(" USB"); }
        
        
        if (on_air == 0) { LCD_Pos(14,1); LCD_WriteText("Rx"); }
        if (on_air == 1) { LCD_Pos(14,1); LCD_WriteText("Tx"); }
    }
    
    if (screen_id == 1) {
        // step
        LCD_Pos(0,0); LCD_WriteText("VFO-step");
        LCD_Pos(0,1); LCD_WriteText( step_names[step_id] ); 
    }
    
    if (screen_id == 2) {
        // bfo
        LCD_Pos(0,0); LCD_WriteText("BFO-tune");
        LCD_Pos(0,1); LCD_WriteFrq( bfo[bfo_id] );
        
        if (bfo_id == 0) { LCD_Pos(13,0); LCD_WriteText("LSB"); }
        if (bfo_id == 1) { LCD_Pos(13,0); LCD_WriteText("USB"); }
    }
    
    if (screen_id == 3) {
        // bfo
        LCD_Pos(0,0); LCD_WriteText("IF-shift");
        LCD_Pos(0,1); LCD_WriteFrq( if_shift[bfo_id] );
        
        if (bfo_id == 0) { LCD_Pos(13,0); LCD_WriteText("LSB"); }
        if (bfo_id == 1) { LCD_Pos(13,0); LCD_WriteText("USB"); }
    }
}

void DDS_setOutputFrq(int32_t frq)
{
    sendDataToDDS( delta(frq), 0);// ustawiamy VFO Fout (uwzględnic p.cz.)
}

void BFO_setOutputFrq(int32_t frq)
{
    sendDataToDDS( delta(frq), 1);// ustawiamy BFO Fout
}


/////////////////////////////////////
//  OBSŁUGA PRZERWANIA - ENKODER  ///
/////////////////////////////////////
ISR(INT0_vect) //początek funkcji obsługi przerwania
{
    if (screen_id == 0) {// na tym widoku przestrajamy aktualnie VFO
        dds_st      = 1; 
        frq[vfo_id] = frq[vfo_id] / steps[step_id] * steps[step_id]; // wyrównanie kroku
        
        if (ROTATE_RIGHT)   frq[vfo_id] += steps[step_id];
        if (ROTATE_LEFT)    frq[vfo_id] -= steps[step_id];

        if(frq[vfo_id] < 100000)     frq[vfo_id] = 100000;
        if(frq[vfo_id] > 30000000)   frq[vfo_id] = 30000000;
    }
    
    if (screen_id == 1) { // zmiana kroku gałką
        dial_div++;
        if (dial_div == 10) {
            dial_div = 0;
            if ( ROTATE_RIGHT)    {
                step_id++;
                if(step_id > 7) step_id = 0;
            }
            if (ROTATE_LEFT)    {
                step_id--;
                if(step_id < 0) step_id = 7;
            }
            operateDisplay();            
        }
    }
    
    if (screen_id == 2) { // przestrajanie BFO
        dds_st = 1; 
        if (ROTATE_RIGHT)  bfo[bfo_id] += 5;
        if (ROTATE_LEFT)   bfo[bfo_id] -= 5;
    }
    
    if (screen_id == 3) { // ustawienie IF-SHIFT
        dds_st = 1; 
        if (ROTATE_RIGHT)   if_shift[bfo_id] += 5;
        if (ROTATE_LEFT)    if_shift[bfo_id] -= 5;
        
        if(if_shift[bfo_id] > 5000) if_shift[bfo_id] = 5000;
        if(if_shift[bfo_id] < -5000) if_shift[bfo_id] = -5000;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
int main(void)
{
    int8_t  lock_1         = 0;
    int16_t lock_2         = 0;
    //int32_t btn_counter    = 0;


    // DDRx = 1 OUT        PORTx = 1 Hi
    // DDRx = 0 IN        PORTx = 0 Low

    DDRB = 0b111111;     PORTB = 0b000000;   // DDSy
    DDRC = 0b111111;     PORTC = 0b000000;   // LCD
    DDRD = 0b00000000;   PORTD = 0b11111111; // PD1, PD2 Impulsator
                                             // PD5  Screen
                                             // PD6  STEP
                                             // PD7  VFO A/B

    LCD_Initalize();
    LCD_Clear();
    
    LCD_WriteText("DDS by SQ9ATK"); _delay_ms(500);
    LCD_Clear();
    LCD_WriteText("v2.0"); _delay_ms(500);
    LCD_Clear();
    

    // przerwania dla impulsatora
    // impulsator podpięty do PD2 i PD1;
    MCUCR |= 0b00000011; // w resejstrze MCUCR bity ICS01, ICS00 na H (Zbocze narastające na INT0 generuje przerwanie)
    GICR  |= 0b01000000; // włączamy INT0
    sei(); //globalne włączenie przerwań
    
    // puszczamy dwa razy bo cos nie bangla przy starcie procka
    DDS_setOutputFrq( bfo[bfo_id] - frq[vfo_id] + if_shift[bfo_id] ); 
    DDS_setOutputFrq( bfo[bfo_id] - frq[vfo_id] + if_shift[bfo_id] ); 
    
    BFO_setOutputFrq( bfo[bfo_id] + if_shift[bfo_id]); 
    BFO_setOutputFrq( bfo[bfo_id] + if_shift[bfo_id]);
    
    operateDisplay();

    /* Początek nieskończonej pętli */
    while (1) {
        
        if (dds_st == 1) {
            if (ON_AIR) {
                DDS_setOutputFrq( bfo[bfo_id] - frq[vfo_id] );
                BFO_setOutputFrq( bfo[bfo_id] );    
            } else {
                DDS_setOutputFrq( bfo[bfo_id] - frq[vfo_id] + if_shift[bfo_id] );
                BFO_setOutputFrq( bfo[bfo_id] + if_shift[bfo_id] );
            }
            operateDisplay();
            dds_st = 0; 
        } 
                
        if (BTN_A)  { // PD3 RX / TX sensor
            if(on_air_lock == 0){
                on_air = 1;
                on_air_lock = 1;
                operateDisplay();
                dds_st = 1;
            }
        } else {
            if(on_air_lock == 1){
                on_air = 0;
                on_air_lock = 0;
                operateDisplay();
                dds_st = 1;
            }
        }
        
        
        
        
          
        if (BTN_B) { // PD4 FAST STEP
            if (fast_lock_1 == 0) {
                fast_lock_2++;
                if (fast_lock_2 == 10000) {
                    
                    LCD_Clear();
                    
                    fast_lock_1 = 1;
                    if(screen_id == 0){
                        if (step_tmp == -1) {
                            step_tmp = step_id;
                            step_id = fast_step_id;
                        } else {
                            step_id = step_tmp;
                            step_tmp = -1;
                        }
                        LCD_Clear();
                        operateDisplay();
                    }
                    
                }
            }
        }else{
            fast_lock_1 = 0;
            fast_lock_2 = 0;
        }
        
        
        
        
        
        
         
        if (BTN_C) { // PD5 MENU
            if (lock_1 == 0) {
                if (lock_2 == 10000) {
                    lock_1 = 1;
                    screen_id++;
                    if(screen_id > 3) screen_id = 0;
                    LCD_Clear();
                    operateDisplay();
                }
                lock_2++;
            }
        }
        
        else if (BTN_D) { // PD6    
            if (screen_id == 0) { // VFO A/B
                if (lock_1 == 0) {
                    if (lock_2 == 10000) {
                        lock_1 = 1;
                        vfo_id = abs(vfo_id-1);
                        operateDisplay();
                        dds_st = 1;
                    }
                    lock_2++;
                }    
            }
        } 
        
        else if (BTN_E)  { // PD7
            if (screen_id == 0) { // USB/LSB
                if (lock_1 == 0) {
                    if (lock_2 == 10000) {
                        lock_1 = 1;
                        bfo_id = abs(bfo_id-1);
                        operateDisplay();
                        dds_st = 1;
                    }
                    lock_2++;
                }    
            }
            if (screen_id == 1) { 
                if (lock_1 == 0) {
                    if (lock_2 == 10000) {
                        lock_1 = 1;
                        // nic tu nie ma
                    }
                    lock_2++;
                }    
            }
            if (screen_id == 2) { // BFO Tune zmiana wstęg
                if (lock_1 == 0) {
                    if (lock_2 == 10000) {
                        bfo_id = abs(bfo_id-1);
                        operateDisplay();
                        lock_1 = 1;
                        dds_st = 1;
                    }
                    lock_2++;
                }    
            }
            if (screen_id ==3) { // IF-SHIFT zmiana wstęg
                if (lock_1 == 0) {
                    if (lock_2 == 10000) {
                        bfo_id = abs(bfo_id-1);
                        operateDisplay();
                        lock_1 = 1;
                        dds_st = 1;
                    }
                    lock_2++;
                }    
            } 
        }  
        
        else {
            lock_1    = 0;
            lock_2    = 0;
        }
    }// end main loop

}// end main()
