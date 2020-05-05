#include <p18f4431.h>
#include <timers.h>
#include <delays.h>
#include <usart.h>


//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
                                      //CONFIGURATION BIT SETTINGS

#pragma config OSC = HS       //Oscillator set to high speed
#pragma config WDTEN = OFF    // Wathdog timer disable
#pragma config LVP = OFF      // LVP disable
#pragma config SSPMX = RD1


//--------------------------------------------------------------------------------------------------------------------
									  //Definitions


#define CLOCK {PORTDbits.RD6 = 0 ;\ PORTDbits.RD6 = 0 ;\ PORTDbits.RD6 = 1 ;\ PORTDbits.RD6 = 1 ;\ PORTDbits.RD6 = 1 ;}
#define CLOCK_HIGH {PORTDbits.RD6 = 1 ;}	
#define CLOCK_LOW {PORTDbits.RD6 = 0 ;}	
#define IN  PORTDbits.RD7 

/


//--------------------------------------------------------------------------------------------------------------------
									//	GLOBAL VARIABLES

char index = 0; 
unsigned char cnt ;
char *ptr_int_speed_cnt;
unsigned int int_speed_cnt = 0 ;
int *byte0, *byte1;
unsigned int var1 = 0;
unsigned int var2 = 0;

struct DATA 
{
	unsigned D0:1 ;
	unsigned D1:1 ;
	unsigned D2:1 ;
	unsigned D3:1 ;
	unsigned D4:1 ;
	unsigned D5:1 ;
	unsigned D6:1 ;
	unsigned D7:1 ;
} DATA_BYTE_0_bits, DATA_BYTE_1_bits; 

//---------------------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------------------
									//	Function Prototypes

void main(void) ;
void init(void);
void rs232_init(void) ;
void setint(void) ;
void delay(void);
void rs232(char);
void bin_io(void);
void rx_handler (void); //Declare the ISR function


//--------------------------------------------------------------------------------------------------------------------
									// Main Program

void main()
{
  
	//PORTB = 0 ; 
	//LATB = 0 ;
	//TRISBbits.TRISB1 = 0 ; 			//Clock Select, set RB1 as output
	
	
	TRISDbits.TRISD6 = 0 ; 			//TEST PIN


	TRISDbits.TRISD7 = 1 ; 			//set RD2 as input

  TRISCbits.TRISC7 = 1; // RX pin set as input
  TRISCbits.TRISC6 = 0; // TX pin set as output
	
  OpenUSART (USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE &
  USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, 64);

  RCONbits.IPEN = 1;    /* Enable interrupt priority */
  IPR1bits.RCIP = 1;    /* Make receive interrupt high priority */
  INTCONbits.GIEH = 1;  /* Enable all high priority interrupts */

   	CLOCK_HIGH
			
	//setting unused bits of 1st byte to zero
//	DATA_BYTE_1_bits.D5 = 0 ;		//Position Bit14
//	DATA_BYTE_1_bits.D6 = 0 ;		//Position Bit15
//	DATA_BYTE_1_bits.D7 = 0 ;		//Position Bit16
 
	while(1)
	{
		


	}

}







//----------------------------------------------------------------------------
//high priority interrupt vector
  #pragma code rx_interrupt = 0x8
  void rx_int (void)
      {
         _asm goto rx_handler _endasm
      }


//----------------------------------------------------------------------------



//----------------------------------------------------------------------------
// High priority interrupt routine

#pragma code
#pragma interrupt rx_handler
void rx_handler (void)

 {
  unsigned char c;
  c = getcUSART(); //get a single character off the USART line
  while(BusyUSART());

{

			//useful 1st byte
				CLOCK
				DATA_BYTE_0_bits.D7 = IN ;		//Position Bit0
				CLOCK
				DATA_BYTE_0_bits.D6 = IN ;		//Position Bit1
				CLOCK
				DATA_BYTE_0_bits.D5 = IN ;		//Position Bit2
				CLOCK
				DATA_BYTE_0_bits.D4 = IN ;		//Position Bit3
				CLOCK
				DATA_BYTE_0_bits.D3 = IN ;		//Position Bit4
				CLOCK
				DATA_BYTE_0_bits.D2 = IN ;		//Position Bit5
				CLOCK
				DATA_BYTE_0_bits.D1 = IN ;		//Position Bit6	
				CLOCK
				DATA_BYTE_0_bits.D0 = IN ;		//Position Bit7

				//useful 2nd byte
				CLOCK
				DATA_BYTE_1_bits.D7 = IN ;		//Position Bit8
				CLOCK
				DATA_BYTE_1_bits.D6 = IN ;		//Position Bit9
				CLOCK
				DATA_BYTE_1_bits.D5 = IN ;		//Position Bit10
				CLOCK
				DATA_BYTE_1_bits.D4 = IN ;		//Position Bit11
				CLOCK
				DATA_BYTE_1_bits.D3 = IN ;		//Position Bit12
			    CLOCK
				DATA_BYTE_1_bits.D2 = IN ;		//Position Bit10
				CLOCK
				DATA_BYTE_1_bits.D1 = IN ;		//Position Bit11
				CLOCK
				DATA_BYTE_1_bits.D0 = IN ;		//Position Bit12
			

		
				
				byte0 = (int*) &DATA_BYTE_0_bits ;
				byte1 = (int*) &DATA_BYTE_1_bits ;	
		        //var1 <<=8;	 
                //var1 |= *byte0;
                 
			
                //var2 <<=8;	 
               //var2 |= *byte1;
			
	   
        TXREG = *byte0;
		while(TXSTAbits.TRMT == 0 );
		
		TXREG = *byte1& 0xF8;
	    while(TXSTAbits.TRMT == 0 );

        //PORTDbits.RD3 = !PORTDbits.RD3 ;
				
		 PIR1bits.RCIF = 0; //reset the ISR flag.	
		}	
       

}  
	
 
Encoder code.txt
Displaying Encoder code.txt.
