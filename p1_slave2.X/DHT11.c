/*
 Codigo para obtener datos del dht11
 */


// send start signal to the sensor
void Start_Signal(void) {
  DHT11_PIN_DIR = 0;     // configure DHT11_PIN as output
  DHT11_PIN = 0;         // clear DHT11_PIN output (logic 0)
 
  __delay_ms(25);        // wait 25 ms
  DHT11_PIN = 1;         // set DHT11_PIN output (logic 1)
 
  __delay_us(30);        // wait 30 us
  DHT11_PIN_DIR = 1;     // configure DHT11_PIN as input
}

void DHT11_check_response()
{
   while(input_state(DHT11_PIN)){};
   while(!input_state(DHT11_PIN)){};
   while(input_state(DHT11_PIN)){};  
}

int DHT11_read_byte()
{
   int i,data = 0;  
   for(i=0;i<8;i++){        
      while(!input_state(DHT11_PIN)){
      }
      delay_us(30);         
      if(input_state(DHT11_PIN)){  
         data = ((data<<1) | 1); 
      }
      else{
         data = (data<<1);  
      }
      while(input_state(DHT11_PIN)){
      }
   }
   return data;
}

int DHT11_read_data(float *hum, float *temp)
{
   char buf[8];
   int result = 0;
   DHT11_start();
   DHT11_response();
   bits[0] = DHT11_read_byte();   //Humedad entero
   bits[1] = DHT11_read_byte();   //Humedad decimal
   bits[2] = DHT11_read_byte();   //Temp entero
   bits[3] = DHT11_read_byte();   //Temp decimal
   bits[4] = DHT11_read_byte();   //Paridad
   result = bits[0] + bits[1] + bits[2] + bits[3];
   if (result == bits[4]){
      sprintf(buf, "%2d.%1d",bits[0],bits[1]);
      *hum = atof(buf);
      sprintf(buf, "%2d.%1d",bits[2],bits[3]);
      *temp = atof(buf);  
      return 1;
   }
   else{
      return 0;
   }
}
