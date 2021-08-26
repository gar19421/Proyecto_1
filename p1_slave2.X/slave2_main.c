/*
 * Archivo:   Lab04_main_master.c
 * Dispositivo: PIC16F887
 * Autor: Brandon Garrido 
 * 
 * Programa: Comunicación esclavo - maestro SPI / ADC
 * 
 * Creado: Agosto 02, 2021
 * Última modificación: Agosto 03, 2021
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>
#include "ADC.h"
#include "I2C.h"


// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT//Oscillator Selection bits(INTOSCIO 
                              //oscillator: I/O function on RA6/OSC2/CLKOUT pin, 
                              //I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT disabled and 
                          //can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR  
                                //pin function is digital input, MCLR internally 
                                //tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code 
                                //protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code 
                                //protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit 
                                //Internal/External Switchover mode is disabled
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit 
                                //(Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF         //Low Voltage Programming Enable bit(RB3/PGM pin 
                                //has PGM function, low voltage programming 
                                //enabled)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)


#define _XTAL_FREQ 8000000
#define CANAL0 0

//------------------------------VARIABLES---------------------------------------
uint8_t valor_ADC = 0;
uint8_t var;

unsigned char Check;
unsigned char uniT=0,decT=0,uniHR=0,decHR=0;
unsigned char  T_byte1, T_byte2,RH_byte1, RH_byte2;
unsigned Sum;




//-----------------------------PROTOTIPOS---------------------------------------
void setup (void);
void StartSignal();
void CheckResponse();
char ReadData();

//---------------------------INTERRUPCION--------------------------------------
void __interrupt()isr(void){
    /*     
    if (ADIF == 1){                    //INTERRUPCION DEL ADC
         
        valor_ADC = ADC_READ();
        
        PIR1bits.ADIF = 0;              //LIMPIAMOS LA BANDERA DEL ADC
    }*/
    
       if(PIR1bits.SSPIF == 1){ 

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            var = SSPBUF;                 // LEEMOS EL VALOR DEL BUFFER Y AGREGAMOS A UNA VARIABLE
            SSPCONbits.SSPOV = 0;       // LIMPIAMOS LA BANDERA DE OVERFLOW
            SSPCONbits.WCOL = 0;        // LIMPIAMOS EL BIT DE COLISION
            SSPCONbits.CKP = 1;         // HABILITAMOS SCL
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            //__delay_us(7);
            var = SSPBUF;                 // LEEMOS EL VALOR DEL BUFFER Y AGREGAMOS A UNA VARIABLE
            //__delay_us(2);
            PIR1bits.SSPIF = 0;         // LIMPIAMOS BANDERA DE INTERUPCION RECEPCION/TRANSMISION SSP
            SSPCONbits.CKP = 1;         // HABILITA LOS PULSOS DEL RELOJ SCL
            while(!SSPSTATbits.BF);     // HASTA QUE LA RECEPCION SE REALICE
           // PORTD = SSPBUF;             // Guardar en el PORTD el valor del buffer de recepción
            __delay_us(250);
            
        }else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            var = SSPBUF;
            BF = 0;
            SSPBUF = T_byte1;
            SSPCONbits.CKP = 1;
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
       
        PIR1bits.SSPIF = 0;    
    }
    
                        
}

void main (void){
    setup(); 
    //ADCON0bits.GO = 1;
    while(1){
       /* PORTB = valor_ADC;
        ADC_CHANNELS(CANAL0,valor_ADC,&valor_ADC);  //POT0
        __delay_ms(10);*/
        
        StartSignal();
        CheckResponse();
        if(Check == 1)
        {
          RH_byte1 = ReadData();
          RH_byte2 = ReadData();
          T_byte1 = ReadData();
          T_byte2 = ReadData();
          Sum = ReadData();

          if(Sum == ((RH_byte1+RH_byte2+T_byte1+T_byte2) & 0XFF))
          {
           decHR=RH_byte1/10;
           uniHR=RH_byte1%10;
           decT=T_byte1/10;
           uniT=T_byte1%10;

           decHR=decHR+0x30;
           uniHR=uniHR+0x30;
           decT=decT+0x30;
           uniT=uniT+0x30;

          }
        }else{
            T_byte1 = 0;
        }
        
        __delay_ms(1000);
    }
}

//---------------------------CONFIGURACION--------------------------------------
void setup(void){
    //CONFIGURACION DE PUERTOS
    ANSEL = 0B0000000;          //RA0 ANALOGICO
    ANSELH = 0X00;              //PINES COMO DIGITALES
    
    TRISA = 0B0000000;          //RA0 INPUT
    TRISD = 0X00;               //PORTD COMO OUTPUT
    TRISE = 0X00;               //PORTE COMO OUTPUT
    TRISB = 0X00;         //PORTB COMO OUTPUT
    
    PORTA = 0X00;                //LIMPIAMOS EL PUERTOA
    PORTB = 0X00;                //LIMPIAMOS EL PUERTOB
    PORTC = 0X00;                //LIMPIAMOS EL PUERTOC
    PORTD = 0X00;                //LIMPIAMOS EL PUERTOD
    PORTE = 0X00;                //LIMPIAMOS EL PUERTOE
    
    //CONFIGURACION DEL OSCIALDOR
    OSCCONbits.IRCF2 = 1;        //OSCILADOR  DE 8 MHZ
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1;         //RELOJ INTERNO 
    
   // ADC_INIT(CANAL0);           //INICIO DEL ADC
    INTCONbits.GIE = 1;         //HABILITAMOS LAS INTERRUPCIONES GLOBALES
    INTCONbits.PEIE = 1;        //HABILITAMOS LAS INTERRUPCIONES PERIFERICAS
    I2C_Slave_Init(0x80);       //DIRECCION DEL I2C ESCLAVO
}



//lectura sensor de temperatura


void StartSignal()
{
  TRISAbits.TRISA0 = 0;   //Configura RA5 como salida
  PORTAbits.RA0 = 0;    //envia un 0 hacia el sensor.
  __delay_ms(18);
  PORTAbits.RA0 = 1;    //Envia un 1 hacia el sensor.
  __delay_us(30);
  TRISAbits.TRISA0 = 1;    //Configura RA5 como entrada.
}

//////////////////////////////
void CheckResponse()
{
  Check = 0;
  __delay_us(40);
  if(PORTAbits.RA0 == 0)
  {
    __delay_us(80);
    if (PORTAbits.RA0 == 1)
    Check = 1;
    __delay_us(40);
  }
}

//////////////////////////////
char ReadData()
{
  
  //TRISAbits.TRISA0 = 1;   //Configura RA5 como salida
  //PORTAbits.RA0 = 0;    //envia un 0 hacia el sensor.
  char i, j;
  for(j = 0; j < 8; j++)
  {
    while(!PORTAbits.RA0 );       //Espera hasta que la entrada DataDHT sea un alto.
    __delay_us(30);
    if(PORTAbits.RA0  == 0)
     i&= ~(1<<(7 - j));    //Clear bit (7-b)  i=i &  ~(1<<(7 - j)) ===> i=i and NOT(1<<(7 - j))
    else
    {
      i|= (1 << (7 - j));  //Set bit (7-b)  i= i | (1 << (7 - j))===> i= i not (1 << (7 - j))
      while(PORTAbits.RA0 );      //Espera hasta que la entrada DataDHT sea un bajo.
    }
  }
 return i;
}