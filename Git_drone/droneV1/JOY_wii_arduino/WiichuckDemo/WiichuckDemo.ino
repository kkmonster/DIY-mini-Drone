/*
 * WiiChuckDemo -- 
 *
 * 2008 Tod E. Kurt, http://thingm.com/
 *
 */

#include <Wire.h>
#include "nunchuck_funcs.h"

int loop_cnt=0;
int raw_accx,raw_accy ;
int accx,accy,joyx,joyy,zbut,cbut;
int buf_accx=0,buf_accy=0,buf_joyx=0,buf_joyy=0,buf_zbut=0,buf_cbut=0;
int cen_accx=0,cen_accy=0 ;
int enable = 0;
int offset_yaw = 0;
int return_yaw = 1;
void setup()
{
    Serial.begin(19200);
    nunchuck_setpowerpins();
    nunchuck_init(); // send the initilization handshake
}

void loop()
{
    if( loop_cnt > 20 ) { // every 20 msecs get new data
        loop_cnt = 0;
        
        
        /*  Read data from wii  */
        nunchuck_get_data();
        accx  = nunchuck_accelx(); // ranges from approx 70 - 182
        accy  = nunchuck_accely(); // ranges from approx 65 - 173
        joyx = nunchuck_joyx() - 136;
        joyy = nunchuck_joyy() - 126;
        zbut = nunchuck_zbutton();
        cbut = nunchuck_cbutton(); 
         
        /*  Compute data  */ 
        
        if ( zbut && !buf_zbut ){
          enable = 1 - enable ;
          cen_accx = accx ;
          cen_accy = accy ;
          offset_yaw = 0  ;
        }
        if ( cbut && !buf_cbut ){
          offset_yaw = buf_joyx ;
          return_yaw = 0 ;
        }
        
        accx -= cen_accx ;
        accy -= cen_accy ;
        joyx += offset_yaw ;
        buf_joyx = joyx ;
        
        if (((offset_yaw-8) < joyx) && (joyx > (offset_yaw+8))) return_yaw = 1;
        
        if (!return_yaw) joyx = offset_yaw ;
        
        if (joyx> 120)joyx= 120 ;
        if (joyx<-120)joyx=-120 ;
       // enable = 0xabcd ;
        
        
        Serial.write(0x7E);
        Serial.write(0x7E);
        
        Serial.write((int8_t)enable);
        Serial.write((int8_t)(enable>>8));
        
        Serial.write((int8_t)accx);
        Serial.write((int8_t)(accx>>8));
        
        Serial.write((int8_t)accy);
        Serial.write((int8_t)(accy>>8));
        
        Serial.write((int8_t)joyx);
        Serial.write((int8_t)(joyx>>8));
        
        Serial.write((int8_t)joyy);
        Serial.write((int8_t)(joyy>>8));
        
        Serial.write(0x03);
        Serial.write(0x03);

        
//          //////////////////// FOR DEBUG ///////////////////////////
//          ////////////////////////////////////////////////////////// 
//        /*  Sent data  */    
//        Serial.print("enable: "); Serial.print((int)enable,DEC);
//        Serial.print("\tacc_x: "); Serial.print((int)accx,DEC);
//        Serial.print("\tacc_y: "); Serial.print((int)accy,DEC);
//        Serial.print("\tjoy_x: "); Serial.print((int)joyx,DEC);
//        Serial.print("\tjoy_y: "); Serial.print((int)joyy,DEC);
//        Serial.print("\tzbut: "); Serial.print((int)zbut,DEC);
//        Serial.print("\tcbut: "); Serial.println((int)cbut,DEC);
//         ////////////////////////////////////////////////////////// 
  
        /*  Update data t-1  */
          buf_zbut = zbut  ;
          buf_cbut = cbut  ;

    }
    loop_cnt++;
    delay(1);
}

