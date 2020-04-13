//#ifndef P253_H_ 
//#define P253_H_ 

#ifndef __P253_H__
#define __P253_H__


#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#include "main.h" 	
	
//#define PROGMEM /* empty */
//#define pgm_read_byte(x) (*(x))
//#define pgm_read_word(x) (*(x))
//#define pgm_read_float(x) (*(x))
//#define PSTR(STR) STR
#endif

//#define BNO055_WR_FUNC_PTR int8_t (*bus_write)(uint8_t, uint8_t, uint8_t *, uint8_t)

//#define BNO055_RD_FUNC_PTR int8_t (*bus_read)(uint8_t, uint8_t, uint8_t *, uint8_t)

//#include "main.h" 





// 253***sign_byte_array****8bit_data_array****16bit_data_array***fraction_data_array8bit***fraction_data_array16bit***fraction_index8bit***fraction_index16bit



typedef struct {
    int8_t  total_space;
    int8_t  total_number;
    int8_t  n_of_frac;
	int8_t  n_of_frac16bit;
    int8_t  n_of16bit;
    int8_t  n_of8bit;
    
    int8_t  fraction_byte_array_index;
	int8_t  sign_byte_array_lenght;
	int8_t  fraction_byte_array_lenght;
	int8_t  sign_byte_array_index;
	int8_t  sign_byte_array[ 30];
    int8_t  length_of_fractions;
    int8_t  fraction_activation_for_8bits[ 10];
	
	int8_t  frac_indx_arry[ 30]; 
	int8_t  frac_indx_arry16[ 30]; 
	int8_t  fraction_byte_array_lenght16bit;
	
	
//	BNO055_WR_FUNC_PTR;/**< bus write function pointer */
	//BNO055_RD_FUNC_PTR;/**<bus read function pointer */
  
}p253_t;


uint8_t p253_init3(p253_t *p253, uint8_t n_of8bit ,uint8_t n_of16bit,   uint8_t n_of_f,   uint8_t n_of_f16, uint8_t *frac_indx_arry, uint8_t *frac_indx_arry16);

uint8_t p253_perform_depackage( p253_t p253,  uint8_t *decoded_packet ,float  *data_raw8bit,float *data_raw16bit);
 


int extract_fraction( float data_f ,uint8_t precision);

int8_t p253_make_prepackage_distill_8_16(p253_t *p253,  float *values_8bit ,uint8_t *a ,uint8_t *b, float *values_16bit ,uint8_t *c ,uint16_t *d);



uint8_t p253_init(p253_t *p253, uint8_t n_of8bit ,uint8_t n_of16bit,   uint8_t n_of_f,   uint8_t n_of_f16);



uint8_t p253_make_prepackage_signs_8_16( p253_t *p253,float *values_8bit ,float *values_16bit,uint8_t *return_array );


uint8_t p253_package( p253_t *p253,uint8_t *values_8bit ,uint16_t *values_16bit,uint8_t *values_fraction,uint8_t *values16bit_fraction,uint8_t *sign_array,uint8_t *frac_indexarray,uint8_t *frac_indexarray16,uint8_t *return_array);


int8_t zip2byte_array2( uint8_t *values ,uint8_t number_of_boolean,int8_t *return_array);

void p253_make_prepackage_create_fraction_index_byte8(p253_t *p253, int8_t *values ,int8_t *return_array );

uint8_t p253_make_prepackage_create_fraction_index_byte16(p253_t *p253, int8_t *values ,int8_t *return_array );


  
	/**************************************************************************************************/
	
	
	float add_fraction( uint8_t decimal_part ,uint8_t fraction_in8bit);


uint8_t p253_depackage( p253_t *p253,uint8_t *example_received_package, uint8_t *values_8bit ,uint16_t *values_16bit,uint8_t *values_fraction,uint8_t *values16bit_fraction,uint8_t *sign_array,uint8_t *frac_indexarray,uint8_t *frac_indexarray16);


int8_t p253_make_prepackage_conjugate_8_16(p253_t *p253,uint8_t *decimalpart_8bit , uint16_t *decimalpart_16bit, uint8_t *fractionpart_8bit, uint8_t *fractionpart_16bit,float *a ,float *b);



uint8_t p253_assign_signs_8_16( p253_t *p253,uint8_t *sign_byte_array,float *data_raw8bit,float *data_raw16bit );
	 
	 #endif


/*


uint8_t p253_make_prepackage_signs_8_16( p253_t *p253,float *values_8bit ,float *values_16bit,uint8_t *return_array ){   //  (-251 -1602)  (-2 -160)      //    paket :  -251 -2  -1602 -160
  
uint8_t   signbyte=0;uint8_t  storage=0;uint8_t  index_memory=0;

 uint8_t  n0f8  =p253->n_of8bit ;
 uint8_t n0f16 = p253->n_of16bit;
 uint8_t sign_byte_array_index  =  p253->sign_byte_array_index;

 int16_t  negative_number=-240;

uint8_t sign_byte_array[10]={0};

   
        for(int i=0;i< n0f8   ;i++){  
                                                                                   
          if( (*(values_8bit+i))<0)   {  signbyte=1 ;  
        
            *(values_8bit+i)= *(values_8bit+i)  *-1 ; }
         else{ 
            signbyte=0;
            }
                                //      Serial.print("i :"); Serial.print(i); Serial.print(" , ");      Serial.print("dbg1 :"); Serial.print(*(values_8bit+i)); Serial.print(" , "); Serial.print("sign_byte :"); Serial.print(signbyte);  Serial.print(" , "); Serial.print("sb_index :"); Serial.print(sign_byte_array_index);   Serial.println( );              
           storage=  ( signbyte<<( i-8*sign_byte_array_index) )|storage;   //  0000 0011
                                  //      Serial.print("storage :"); Serial.print(storage);  Serial.println( ); 

            
            if( i== (8*(sign_byte_array_index+1))-1  )   {      
                                                   //  p253. sign_byte_array[ sign_byte_array_index]  = storage  ; 
                      sign_byte_array[sign_byte_array_index]=  storage  ; 
                                                            //  sign_byte_array[0]=     p253.sign_byte_array[0];
                  //    Serial.print("sign_array : ");    Serial.println(   sign_byte_array[0] );        Serial.println( );                                                            
                          storage=0;   sign_byte_array_index++;
                 //   Serial.print("sign_byte_array_index : ");    Serial.println(   sign_byte_array_index );        Serial.println( );  
            }
            else
            {
                     sign_byte_array[sign_byte_array_index] = storage  ; 
                                    
              }
             index_memory=i;  //i =8      00000001     11011100 
        }
       signbyte=0 ;

    
      //  for(int i=0;i<sign_byte_array_index+1;i++)
      //  { *(return_array + i) = sign_byte_array[i];}
   
  
   // Serial.println( ); Serial.print("16bitlik alan icin index memory  :"); Serial.print(index_memory);   Serial.print("storage :"); Serial.print(storage); Serial.println( );
   
           for(int i=0;i< n0f16   ;i++){  
                         
       
           if( (*(values_16bit+i))<0)   {  signbyte=1 ;  
            *(values_16bit+i)= *(values_16bit+i)  *-1 ;           }
            
         else{ 
            signbyte=0;
            }

//1     11011100 
//01     11011100 i=0
//001     11011100   i=1
//1001     11011100   i=2
//11001     11011100   i=3
//111001     11011100   i=4
//0111001     11011100   i=5
//01011 1001     11011100   i=6
//      11  01011 1001     11011100   i=7

 
    //   Serial.print("i :"); Serial.print(i); Serial.print(" , ");      Serial.print("dbg2 :"); Serial.print(*(values_16bit+i)); Serial.print(" , "); Serial.print("sign_byte :"); Serial.print(signbyte);  Serial.print(" , "); Serial.print("sb_index :"); Serial.print(sign_byte_array_index);   Serial.println( );              
  
          //storage=  ( signbyte<<( i-8*sign_byte_array_index) )|storage;   //  0000 0011               
           storage=  ( signbyte<<( i+index_memory-8*sign_byte_array_index+1) )|storage;   //  0000 0011
    //   Serial.print("storage :"); Serial.print(storage); Serial.println(); 

            if( i+index_memory+1== (8*(sign_byte_array_index+1))-1 )   {   //   if( i== (8*(sign_byte_array_index+1))-1  ) 
                sign_byte_array[ sign_byte_array_index]  = storage  ; 
                storage=0;     sign_byte_array_index++;
            }else
            {
                sign_byte_array[ sign_byte_array_index]  = storage  ; 
              }

        }

        for(int i=0;i<sign_byte_array_index+1;i++)
        {
            // Serial.print("sign_byte_array ");   Serial.print(i);Serial.print(": "); Serial.println( sign_byte_array[i] ); 
                *(return_array + i) = sign_byte_array[i];
        }
 

        
}

*/














	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	


	
	
	
	




/*  \Brief: The API is used as I2C bus write
 *  \Return : Status of the I2C write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *    will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
 
 
 
// s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
// {
  // s32 BNO055_iERROR = BNO055_INIT_VALUE;
  // u8 array[I2C_BUFFER_LEN];
  // u8 stringpos = BNO055_INIT_VALUE;

  // array[BNO055_INIT_VALUE] = reg_addr;
  // for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    // array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] =
      // *(reg_data + stringpos);
  // }


  // s8 I2C_routine(void)
// {
  // bno055.bus_write = BNO055_I2C_bus_write;
  // bno055.bus_read = BNO055_I2C_bus_read;
  // bno055.delay_msec = BNO055_delay_msek;
  // bno055.dev_addr = BNO055_I2C_ADDR1;

  // return BNO055_INIT_VALUE;
// }







// s8  hal_i2c_write( u8 dev_addr,u8  reg_addr,u8  *data,   u8  length) {
  

  // u8 buffer[8];
  // u8 i =0;
  
  // buffer[0] =reg_addr;  
  // for( i =0;i<length;i++)  // 0 için buffer_global(1) e data_gl[0]  i atiyor.    // 1 için 
   // {
     // buffer[i+1]=  (*data+i)  ;
   // }
                        
  
  
    // if  (HAL_I2C_Master_Transmit(&hi2c1,dev_addr,buffer,length+1,100)==HAL_OK  ){return 0;}
   // else {return -1;}
// }





