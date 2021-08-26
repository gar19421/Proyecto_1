/*
 * Archivo:   lab04_main_master.c
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
#include "I2C.h"
#include "LCD.h"


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
#define ENTER 13
#define PUNTO 46
#define v 118

//------------------------------VARIABLES---------------------------------------
uint16_t valor_ADC = 0;
uint8_t POS1;
uint8_t POS2;
uint8_t POS3;
uint8_t alarma=0;
uint16_t TEMP=0;
uint16_t T_byte1;



//-----------------------------PROTOTIPOS---------------------------------------
void setup (void);
void VAL (uint16_t var);
void moverVentilador();
void alarmaBuzzer();
//---------------------------INTERRUPCION--------------------------------------
void __interrupt()isr(void){
    di();                   //PUSH
    
    ei();                           //POP
}

void main (void){
    setup(); 
    Lcd_Init();                     //INICIALIZAMOS LA LCD
    Lcd_Clear();  //Limpiar LCD
    Lcd_Set_Cursor(1,1); //cursor fila uno primera posicion 
    Lcd_Write_String(" S1:   S2:   S3:");
    while(1){
        
        I2C_Master_Start();         //INICIALIZAMOS LA COMUNICACION
        I2C_Master_Write(0x71);     //ESCRIBIMOS A LA DIRECCION PARA LEER ESCLAVO1
        valor_ADC = I2C_Master_Read(0); //AGREGAMOS EL VALOR AL PORTD
        I2C_Master_Stop();          //DETENEMOS LA COMUNICACION
        __delay_ms(200);
        
        
        I2C_Master_Start();         //INICIALIZAMOS LA COMUNICACION
        I2C_Master_Write(0x81);     //ESCRIBIMOS A LA DIRECCION PARA LEER ESCLAVO2
        T_byte1 = I2C_Master_Read(0); //AGREGAMOS EL VALOR AL CONTADOR
        I2C_Master_Stop();          //DETENEMOS LA COMUNICACION
        __delay_ms(300);
        
        moverVentilador();
        
        I2C_Master_Start();         //INICIALIZAMOS LA COMUNICACION
        I2C_Master_Write(0x71);     //ESCRIBIMOS A LA DIRECCION PARA LEER ESCLAVO2
        alarma = I2C_Master_Read(0); //AGREGAMOS EL VALOR AL CONTADOR
        I2C_Master_Stop();          //DETENEMOS LA COMUNICACION
        __delay_ms(300);
        
        if(alarma==0xFF){
            alarmaBuzzer();
        }
               
        valor_ADC= valor_ADC*1.961; //MAPEO PARA 5.00V
        VAL(valor_ADC);             //EXTRAER LOS VALORES DEL ADC
        Lcd_Set_Cursor(2,1);        //COLOCAMOS VALORES DEL ADC EN LA LCD
        Lcd_Write_Char(POS1);
        Lcd_Write_Char(PUNTO);
        Lcd_Write_Char(POS2);
        Lcd_Write_Char(POS3);
        Lcd_Write_String("v ");
        
        VAL(T_byte1);             //EXTRAER LOS VALORES CONTADOR
        Lcd_Write_Char(POS1);
        Lcd_Write_Char(POS2);
        Lcd_Write_Char(POS3);
        Lcd_Write_String("\337C  ");
        
        VAL(TEMP);                  //EXTRAER LOS VALORES DE TEMP
        Lcd_Write_Char(83);
        Lcd_Write_Char(79);
        Lcd_Write_Char(83);
        Lcd_Write_String("");
    }
}
//---------------------------CONFIGURACION--------------------------------------
void setup(void){
    //CONFIGURACION DE PUERTOS
    ANSEL = 0B0000000;          //PINES COMO DIGITALES
    ANSELH = 0X00;              //PINES COMO DIGITALES
    
    TRISA = 0B0000000;          //PORTA COMO OUTPUT
    TRISD = 0X00;               //PORTD COMO OUTPUT
    TRISE = 0X00;               //PORTE COMO OUTPUT
    TRISB = 0X00;                //PORTB COMO OUTPUT
    
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
    INTCONbits.GIE = 0;         //DESHABILITAMOS LAS INTERRUPCIONES GLOBALES
    INTCONbits.PEIE = 0;        //DESHABILITAMOS LAS INTERRUPCIONES PERIFERICAS
    I2C_Master_Init(100000);        // INICIALIZAR MASTER A FRECUENCIA DE 100kHz
}
void VAL(uint16_t variable){        // Función para obtener valor decimal
    uint16_t valor;
    valor = variable;                  
    POS1 = (valor/100) ;                // VALOR DE CENTENAS
    valor = (valor - (POS1*100));
    POS2 = (valor/10);              // VALOR DE DECENAS
    valor = (valor - (POS2*10));
    POS3 = (valor);                // UNIDADES
    
    POS1 = POS1 + 48;          // PASARLO A VALORES ASCCII
    POS2 = POS2 + 48;
    POS3 = POS3 + 48;
    
}

void moverVentilador(){
    if(T_byte1>0x19){ // si la temperatura es mayor a 25
        PORTAbits.RA0 = 1;
    }else{
         PORTAbits.RA0 = 0;
    }
}

void alarmaBuzzer(){
    
    
    PORTAbits.RA1 = 1;
    __delay_ms(500);
    PORTAbits.RA1 = 0;

        
    
}