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
#include "USART.h"

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

uint8_t POS1_LDR;
uint8_t POS2_LDR;
uint8_t POS3_LDR;

uint8_t POS1_TMP;
uint8_t POS2_TMP;
uint8_t POS3_TMP;

uint8_t POS1_BZ;
uint8_t POS2_BZ;
uint8_t POS3_BZ;

uint8_t alarma=0;
uint16_t TEMP=0;
uint16_t T_byte1;

uint8_t guia = 0x00;
uint8_t RC_temp;

uint8_t contador;
uint8_t contador_unidad = 0;
uint8_t contador_decena = 0;
uint8_t contador_centena = 0;

uint8_t cont=0;
uint8_t contador;
uint8_t val_USART;
uint8_t bandera_buzzer;


uint8_t u_flag = 1;
uint8_t d_flag = 0;
uint8_t c_flag = 0;
uint8_t unidad = 0;
uint8_t decena = 0;
uint8_t valor_Servo = 0;

//-----------------------------PROTOTIPOS---------------------------------------
void setup (void);
void VAL (uint16_t var);
void moverVentilador();
void alarmaBuzzer();
//---------------------------INTERRUPCION--------------------------------------
void __interrupt()isr(void){

    if (INTCONbits.T0IF){           // INTERRUPCION TMR0
        cont++;
        INTCONbits.T0IF = 0;        // Limpiar la bandera de interrupción TMR0
    }
    
    if(PIR1bits.RCIF == 1){ //Empieza a recibir datos del USART
        //0x0A para el salto de linea \n
        if (RCREG ==  0x0D){
            //PORTD = contador;
            PORTB = 2;
        }
        
        if (RCREG !=  0x0D){
        RC_temp = RCREG;
        
            switch(RC_temp){
                case 48:
                    val_USART = 0;
                    break;
                case 49:
                    val_USART = 1;
                    break;
                case 50:
                    val_USART = 2;
                    break;
                case 51:
                    val_USART = 3;
                    break;
                case 52:
                    val_USART = 4;
                    break;
                case 53:
                    val_USART = 5;
                    break;
                case 54:
                    val_USART = 6;
                    break;
                case 55:
                    val_USART = 7;
                    break;
                case 56:
                    val_USART = 8;
                    break;       
                case 57:
                    val_USART = 9;
                    break;
            }  
            
            if (u_flag == 1){
                contador = val_USART;
                unidad = val_USART;
                u_flag = 0;
                d_flag = 1;
            }
            else if (d_flag == 1){
                contador = (unidad*10)+val_USART;
                decena = val_USART;
                d_flag = 0;
                c_flag = 1;
            }
            else if (c_flag == 1){
                contador = (unidad*100)+(decena*10)+val_USART;
                d_flag = 0;
                c_flag = 1;
            } 
        }
    }
    
    if (TXIF == 1){
        //enviar datos
        if(guia==0x00){
            TXREG = POS1_LDR;
            guia = 0x01;
        }else if(guia==0x01){ 
            TXREG = PUNTO;
            guia =0x02;
        } else if(guia==0x02){
            TXREG = POS2_LDR;
            guia = 0x03;
        } else if(guia==0x03){
            TXREG = POS3_LDR;
            guia = 0x04;
        }else if(guia==0x04){
            TXREG = 0x0D;
            guia = 0x05;
        }else if(guia==0x05){
            TXREG = POS1_TMP;
            guia = 0x06;
        } else if(guia==0x06){
            TXREG = POS2_TMP;
            guia = 0x07;
        } else if(guia==0x07){
            TXREG = POS3_TMP;
            guia = 0x08;
        }else if(guia==0x08){
            TXREG = 0x0D;
            guia = 0x09;
        }else if(guia==0x09){
            TXREG = bandera_buzzer+48;
            guia = 0x0A;
        }else if(guia==0x0A){
            TXREG = 0x0D;
            guia = 0x00;
        }
        
        TXIF = 0; //Se limpia la bandera
    } 
    
     
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
        __delay_ms(310);
        
        
        I2C_Master_Start();         //INICIALIZAMOS LA COMUNICACION
        I2C_Master_Write(0x81);     //ESCRIBIMOS A LA DIRECCION PARA LEER ESCLAVO2
        T_byte1 = I2C_Master_Read(0); //AGREGAMOS EL VALOR AL CONTADOR
        I2C_Master_Stop();          //DETENEMOS LA COMUNICACION
        __delay_ms(310);
        
        moverVentilador();
        
        I2C_Master_Start();         //INICIALIZAMOS LA COMUNICACION
        I2C_Master_Write(0x71);     //ESCRIBIMOS A LA DIRECCION PARA LEER ESCLAVO2
        alarma = I2C_Master_Read(0); //AGREGAMOS EL VALOR AL CONTADOR
        I2C_Master_Stop();          //DETENEMOS LA COMUNICACION
        __delay_ms(310);
        
        if(alarma==0xFF){
            alarmaBuzzer();
            bandera_buzzer = 1;
        }else{
            bandera_buzzer = 0;
            POS1_BZ = 45;
            POS2_BZ = 45;
            POS3_BZ = 45;
        }
        
        valor_ADC= valor_ADC*1.961; //MAPEO PARA 5.00V
                    
        
        VAL(valor_ADC);             //EXTRAER LOS VALORES DEL ADC
        POS1_LDR = POS1;
        POS2_LDR = POS2;
        POS3_LDR = POS3;
        
        // para mover el servo
        if(valor_ADC>0x190){
            valor_Servo = 0x00;
            CCPR2L = (valor_Servo>>1) + 128;//Swift y ajuste de señal
            CCP2CONbits.DC2B1 = 0;
            CCP2CONbits.DC2B0 = 0;
        }else if(valor_ADC<=0x190){
            valor_Servo = 0xFF;
            CCPR2L = (valor_Servo>>1) + 128;//Swift y ajuste de señal
            CCP2CONbits.DC2B1 = 0;
            CCP2CONbits.DC2B0 = 0;
        }
        
        
        Lcd_Set_Cursor(2,1);        //COLOCAMOS VALORES DEL ADC EN LA LCD
        Lcd_Write_Char(POS1_LDR);
        Lcd_Write_Char(PUNTO);
        Lcd_Write_Char(POS2_LDR);
        Lcd_Write_Char(POS3_LDR);
        Lcd_Write_String("v ");
                    
        VAL(T_byte1);             //EXTRAER LOS VALORES TEMPERATURA
        POS1_TMP = POS1;
        POS2_TMP = POS2;
        POS3_TMP = POS3;
        Lcd_Write_Char(POS1_TMP);
        Lcd_Write_Char(POS2_TMP);
        Lcd_Write_Char(POS3_TMP);
        Lcd_Write_String("\337C  ");
        
        VAL(TEMP);                  //EXTRAER LOS VALORES DE TEMP
        Lcd_Write_Char(POS1_BZ);
        Lcd_Write_Char(POS2_BZ);
        Lcd_Write_Char(POS3_BZ);
        Lcd_Write_String("");
        
                
        if(cont > 15){ //Se reinicia el contador después de 45ms y se enciende
         cont = 0; //el enable para enviar datos via USART
         TXIE = 1; 
        }
        
        
        if (PORTBbits.RB1 == 1){
            val_USART = 0;
            contador = 0;
            PORTB = 0;
            u_flag = 1;
            d_flag = 0;
            c_flag = 0;
        }
        
    }
}
//---------------------------CONFIGURACION--------------------------------------
void setup(void){
    
    //CONFIGURACION DEL OSCIALDOR
    OSCCONbits.IRCF2 = 1;        //OSCILADOR  DE 8 MHZ
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1;         //RELOJ INTERNO 
    
    //CONFIGURACION DE PUERTOS
    ANSEL = 0B0000000;          //PINES COMO DIGITALES
    ANSELH = 0X00;              //PINES COMO DIGITALES
    
    TRISA = 0B0000000;          //PORTA COMO OUTPUT
    TRISD = 0X00;               //PORTD COMO OUTPUT
    TRISE = 0X00;               //PORTE COMO OUTPUT
    TRISB = 0X00;                //PORTB COMO OUTPUT
    TRISCbits.TRISC1= 0X00;
    
    PORTA = 0X00;                //LIMPIAMOS EL PUERTOA
    PORTB = 0X00;                //LIMPIAMOS EL PUERTOB 
    PORTD = 0X00;                //LIMPIAMOS EL PUERTOD
    PORTC = 0X00;                //LIMPIAMOS EL PUERTOC
    PORTD = 0X00;                //LIMPIAMOS EL PUERTOD
    PORTE = 0X00;                //LIMPIAMOS EL PUERTOE
    
    
    INTCONbits.GIE = 1;         //HABILITAMOS LAS INTERRUPCIONES GLOBALES
    INTCONbits.PEIE = 1;        //HABILITAMOS LAS INTERRUPCIONES PERIFERICAS
    
    INTCONbits.T0IE = 1;           
    INTCONbits.T0IF = 0;
   
    
    //Configuración de TX y RX
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    
    BAUDCTLbits.BRG16 = 1;
    
    SPBRG = 207;
    SPBRGH = 0;
    
    RCSTAbits.SPEN = 1;//Configuración del USART y Baud Rate
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;
    
    TXSTAbits.TXEN = 1; 
    
     //Configuración TMR0
    OPTION_REGbits.T0CS = 0;        // TMR0 Clock source
    OPTION_REGbits.PSA = 0;         // Prescaler a tmr0
    OPTION_REGbits.PS = 0b111;        // prescaler 1:256
    TMR0 = 10;
    
    
    //Configurar PWM
    PR2 = 250; //Valor inicial de PR2
    CCP1CONbits.P1M = 0; //PWM bits de salida
    CCP1CONbits.CCP1M = 0b00001100; //Se habilita PWM   
    CCP2CONbits.CCP2M = 0b00001100;   
    
    CCPR1L = 0x0F; 
    CCPR2L = 0x0F;
    CCP1CONbits.DC1B = 0; //Bits menos significativos del Duty Cycle
    CCP2CONbits.DC2B1 = 0;
    CCP2CONbits.DC2B0 = 0;
    
    PIR1bits.TMR2IF = 0; //Se limpia la bandera
    T2CONbits.T2CKPS1 = 1; //Prescaler de 16
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.TMR2ON = 1; //Se enciende el TMR2
    
    while (!PIR1bits.TMR2IF); //Se espera una interrupción
    PIR1bits.TMR2IF = 0;
    
    
    
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
    POS1_BZ = 83;
    POS2_BZ = 79;
    POS3_BZ = 83;
        
    
}