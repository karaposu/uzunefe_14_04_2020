
//#define BNO055_WR_FUNC_PTR int8_t (*bus_write)(uint8_t, uint8_t, uint8_t *, uint8_t)

//#define BNO055_RD_FUNC_PTR int8_t (*bus_read)(uint8_t, uint8_t, uint8_t *, uint8_t)

#include "p253.h" 
#include <math.h>

//extern float  master_debug[30];  



// 253***sign_byte_array****8bit_data_array****16bit_data_array***fraction_data_array8bit***fraction_data_array16bit***fraction_index8bit***fraction_index16bit






uint8_t p253_init3(p253_t *p253, uint8_t n_of8bit ,uint8_t n_of16bit,   uint8_t n_of_f,   uint8_t n_of_f16, uint8_t *frac_indx_arry, uint8_t *frac_indx_arry16){
  // 253***sign_byte_array****8bit_data_array****16bit_data_array***fraction_data_array***fraction_index

   uint8_t a=0;  uint8_t b=0;  uint8_t c=0;  uint8_t d=0;  uint8_t e=0;uint8_t f=0;  uint8_t g=0;
   a= n_of8bit     ;      b= n_of16bit     ;   c= n_of_f    ;     d=   ((b+a-1)/8) +1 ;  e=   ((c-1)/8) +1 ;        f= n_of_f16;        g=   ((f-1)/8) +1 ;    

if(c==0){e=0;}  
if(f==0){g=0;}  

p253->n_of_frac16bit=   f ;
p253->n_of_frac=   c ;
p253->n_of16bit=   b;
p253->n_of8bit=    a;

p253->total_number= a+b ;
p253->sign_byte_array_lenght=d; 
p253->fraction_byte_array_lenght= e;
p253->fraction_byte_array_lenght16bit= g;
p253->total_space=  a+2*b+c+d+e+f+g;
//n_of8bit+n_of16bit*2  + nof_F+  (n_of8bit+n_of16bit)/8+1  +(( n_of_f-1)/8)+1 +n_of_f16+  (( n_of_f16-1)/8)+1

    for(int i=0;i<a;i++){p253->frac_indx_arry[ i]= *(frac_indx_arry +i)  ; 
    }
    for(int i=0;i<b;i++){p253->frac_indx_arry16[ i]= *(frac_indx_arry16+i)  ; 
    }

                                                        
                                                         
}


        uint8_t fraction_index_for_8bit_values[30]; 
        uint8_t fraction_index_for_16bit_values[30]; 
	
         uint8_t fractionpart_8bit[30];   // storage array for 8bit data to be sent over usart ....
         uint8_t decimalpart_8bit[30];   // storage array for 8bit data to be sent over usart ....

         uint8_t fractionpart_16bit[30];   // storage array for 8bit data to be sent over usart ....
         uint16_t decimalpart_16bit[30];   // storage array for 8bit data to be sent over usart ....
         uint8_t sign_array[30]; 

uint8_t p253_perform_depackage( p253_t p253,  uint8_t *decoded_packet ,float  *data_raw8bit,float *data_raw16bit){
       
	  // uint8_t sign_bytes[30]; 
     //   memcpy(&master_debug[0], decoded_packet, 19);   /* Copy first part */
  //  master_debug[]
       //   master_debug[0] =decoded_packet[0] ;   /* Copy first part */
	
	 p253_depackage( &p253, decoded_packet, decimalpart_8bit , decimalpart_16bit, fractionpart_8bit, fractionpart_16bit, sign_array,fraction_index_for_8bit_values,fraction_index_for_16bit_values);
  
	 p253_make_prepackage_conjugate_8_16(&p253  ,decimalpart_8bit , decimalpart_16bit, fractionpart_8bit, fractionpart_16bit,data_raw8bit,data_raw16bit);
   p253_assign_signs_8_16( &p253,sign_array,data_raw8bit,data_raw16bit );
						
}










int extract_fraction( float data_f ,uint8_t precision){  
 

    int decimalPart =(data_f* (int) pow(10,precision)  ) -floor (data_f )*(int) pow(10,precision)    ;
    return  decimalPart;
}

int8_t p253_make_prepackage_distill_8_16(p253_t *p253,  float *values_8bit ,uint8_t *a ,uint8_t *b, float *values_16bit ,uint8_t *c ,uint16_t *d){  


       uint8_t n0f16 = p253->n_of16bit; uint8_t  n0f8  =p253->n_of8bit ;uint8_t  frac_indx_arry[n0f8]  ; uint8_t  frac_indx_arry16[n0f16]  ;
        float temp;  int8_t  sign=1;  int8_t  memory=0;

 for(int i=0;i<n0f8;i++){ frac_indx_arry[i]=     p253->frac_indx_arry[ i];}//Serial.print( "frac_indx_arry_inside_distill :");  Serial.print( "  ");   for(int i=0;i< n0f8 ;i++){Serial.print(  frac_indx_arry[i] );  Serial.print( " ");delay(10);}  ;Serial.println();Serial.println();
 
 for(int i=0;i<n0f16;i++){frac_indx_arry16[i]=     p253->frac_indx_arry16[ i];}//Serial.print( "frac_indx_arry16_inside_distill :");  Serial.print( "  ");   for(int i=0;i< n0f16 ;i++){Serial.print(  frac_indx_arry16[i] );  Serial.print( " ");delay(10);}  ;Serial.println();Serial.println();
      

                                                            
     for(int i=0;i<n0f8;i++){
       sign=1; 
       temp  =    *(values_8bit+i);
  
       if(frac_indx_arry[i]==1){
       *(a + memory)  =    extract_fraction(temp ,2);memory++;
     
       }
        *(b + i)  =        floor (temp) * sign;

    }
    temp=0;memory=0;
            



   
    for(int i=0;i<n0f16;i++){
       sign=1; 
       temp  =    *(values_16bit+i);
    
       if(frac_indx_arry16[i]==1){
        *(c + memory)  =    extract_fraction(temp ,2);memory++;
       }
       *(d + i)  =        floor (temp) * sign;

    }

                                                            
}






uint8_t p253_make_prepackage_signs_8_16( p253_t *p253,float *values_8bit ,float *values_16bit,uint8_t *return_array ){   //  (-251 -1602)  (-2 -160)      //    paket :  -251 -2  -1602 -160
  
       uint8_t   signbyte=0;uint8_t  storage=0;uint8_t  index_memory=0;

        uint8_t  n0f8  =p253->n_of8bit ;uint8_t n0f16 = p253->n_of16bit;uint8_t sign_byte_array_index  =  p253->sign_byte_array_index; uint8_t sign_byte_array[10]={0};

     

        for(int i=0;i< n0f8   ;i++){  
                                                                                   
          if( (*(values_8bit+i))<0)   {  signbyte=1 ;  *(values_8bit+i)= *(values_8bit+i)  *-1 ; }else{  signbyte=0; }
               storage=  ( signbyte<<( i-8*sign_byte_array_index) )|storage;   //  0000 0011
          if( i== (8*(sign_byte_array_index+1))-1  )   {      sign_byte_array[sign_byte_array_index]=  storage  ; 
                 storage=0;   sign_byte_array_index++;} else {sign_byte_array[sign_byte_array_index] = storage  ; }
             index_memory=i;  //i =8      00000001     11011100 
        }
       signbyte=0 ;

         for(int i=0;i< n0f16   ;i++){   if( (*(values_16bit+i))<0)   {  signbyte=1 ;  *(values_16bit+i)= *(values_16bit+i)  *-1 ;  }else{  signbyte=0;}
         storage=  ( signbyte<<( i+index_memory-8*sign_byte_array_index+1) )|storage;   //  0000 0011
         if( i+index_memory+1== (8*(sign_byte_array_index+1))-1 )   {   sign_byte_array[ sign_byte_array_index]  = storage  ; storage=0;     sign_byte_array_index++;}
		 else{sign_byte_array[ sign_byte_array_index]  = storage  ; }

        }

        for(int i=0;i<sign_byte_array_index+1;i++)
        {
          
                *(return_array + i) = sign_byte_array[i];
        }
 

        
}









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














int8_t zip2byte_array2( uint8_t *values ,uint8_t number_of_boolean,int8_t *return_array){

     uint8_t byte_array_index =0;uint8_t   fid=0; uint8_t  storage=0;uint8_t  index_memory=0;
     uint8_t temp_array[30]={0};

     
  for(int i=0;i< number_of_boolean   ;i++){  
                                                            
            fid=*(values+i);
            storage=  ( fid<<( i-8*byte_array_index) )|storage;   //  0000 0
                                                                                 
            if( i== (8*(byte_array_index+1))-1  )   {  
                                                                                                              
                temp_array[byte_array_index]=  storage  ; 
                 storage=0; byte_array_index++;
            }
      else
            {
            temp_array[byte_array_index] = storage  ;
            }
         index_memory=i;
        }
        for(int i=0;i<byte_array_index+1;i++)
        {       *(return_array + i) = temp_array[i];
           }
    
}

  void p253_make_prepackage_create_fraction_index_byte8(p253_t *p253, int8_t *values ,int8_t *return_array ){  
                                                                      
      int8_t return_array_inception[15];uint8_t  zero=0; uint8_t n = p253->n_of8bit;
      if(n != zero){
        zip2byte_array2( values,  n ,return_array_inception); 
        for(int i=0;i<   ((n-1)/8) +1         ;i++){ *(return_array + i) = return_array_inception[i];}
            
                  }
    }

uint8_t p253_make_prepackage_create_fraction_index_byte16(p253_t *p253, int8_t *values ,int8_t *return_array ){  
                                                                      
      uint8_t nof16 = p253->n_of16bit; int8_t return_array_inception[30]; uint8_t  zero=0;   
      if(nof16 != zero){
      zip2byte_array2( values,nof16 ,return_array_inception);  
       for(int i=0;i<   ((nof16-1)/8) +1         ;i++){ *(return_array + i) = return_array_inception[i];}
      }
    }


  
	
/*************************************************************************/
		
		
		
		
		
		float add_fraction( uint8_t decimal_part ,uint8_t fraction_in8bit){  
 
     float  raw_data= fraction_in8bit/100+decimal_part  ;
 
    return raw_data ;
}


uint8_t p253_depackage( p253_t *p253,uint8_t *example_received_package, uint8_t *values_8bit ,uint16_t *values_16bit,uint8_t *values_fraction,uint8_t *values16bit_fraction,uint8_t *sign_array,uint8_t *frac_indexarray,uint8_t *frac_indexarray16){

//example_received_package
//255 1 2 3 40 5 232 3 208 7 184 11 10 50 30 17 4 
// 253***sign_byte_array****8bit_data_array****16bit_data_array***fraction_data_array8bit***fraction_data_array16bit***fraction_index8bit***fraction_index16bit

  uint8_t  n0f8  =p253->n_of8bit ;         uint8_t n0f16 = p253->n_of16bit;                     uint8_t noff = p253->n_of_frac;     uint8_t total = p253->total_space; 
  uint8_t noff16 = p253->n_of_frac16bit;
  uint8_t sbai = p253->sign_byte_array_lenght;  uint8_t fbai = p253->fraction_byte_array_lenght; uint8_t fbai16 = p253->fraction_byte_array_lenght16bit;
 uint8_t   raw_packet_8bit[total] ;


 uint8_t example_package[total];
// memset( example_package, 0, total*sizeof(uint8_t) );
 
 int  j;  for (j = 0; j<total; j++){example_package[j] = 0;}


  for(int i=0;i<total+1;i++)
 {
   example_package[i]  =*(example_received_package + i) ;
 }
//1111 1111 01010 01010   0000 0000 01010 01010 
         for(int i=0;i<sbai ;i++){     *(sign_array+i)=example_package[i]  ;                        }
         for(int i=0;i<n0f8;i++){   *(values_8bit+i)=example_package[i+sbai]     ;             }
         
         for(int i=0  ;i< n0f16;    i++){  
              *(values_16bit+i)  = ( example_package[(i*2+sbai+n0f8)+0]  |  example_package[(i*2+sbai+n0f8)+1]<< 8);
        
          
         }
         for(int i=0  ;i< noff;    i++){ 
      *(values_fraction+i)=    example_package[sbai+n0f8+n0f16*2+i]; }
          for(int i=0  ;i< noff16;    i++){
       *(values16bit_fraction+i)    = example_package[sbai+n0f8+n0f16*2+noff+i]; }

          
         for(int i=0  ;i< fbai;    i++){   *(frac_indexarray+i) = example_package[sbai+n0f8+n0f16*2+noff+noff16+i]; }
          for(int i=0  ;i< fbai16;    i++){   *(frac_indexarray16+i)= example_package[sbai+n0f8+n0f16*2+noff+noff16+fbai+i]; }
         
       

}



int8_t p253_make_prepackage_conjugate_8_16(p253_t *p253,uint8_t *decimalpart_8bit , uint16_t *decimalpart_16bit, uint8_t *fractionpart_8bit, uint8_t *fractionpart_16bit,float *a ,float *b){  


       uint8_t n0f16 = p253->n_of16bit; uint8_t  n0f8  =p253->n_of8bit ;uint8_t  frac_indx_arry[n0f8]  ; uint8_t  frac_indx_arry16[n0f16]  ;
        float temp;  int8_t  sign=1;  int8_t  memory=0;   float temp_f; 

 for(int i=0;i<n0f8;i++){ frac_indx_arry[i]=     p253->frac_indx_arry[ i];}//Serial.print( "frac_indx_arry_inside_distill :");  Serial.print( "  ");   for(int i=0;i< n0f8 ;i++){Serial.print(  frac_indx_arry[i] );  Serial.print( " ");delay(10);}  ;Serial.println();Serial.println();
 
 for(int i=0;i<n0f16;i++){frac_indx_arry16[i]=     p253->frac_indx_arry16[ i];}//Serial.print( "frac_indx_arry16_inside_distill :");  Serial.print( "  ");   for(int i=0;i< n0f16 ;i++){Serial.print(  frac_indx_arry16[i] );  Serial.print( " ");delay(10);}  ;Serial.println();Serial.println();
      
//    frac_act_indx[NOF_8BIT]={1,0,0,0,1};
// 10 50
                                                        
     for(int i=0;i<n0f8;i++){
        
       temp  =    *(decimalpart_8bit+i);

       if(frac_indx_arry[i]==1){
           temp_f  = *(fractionpart_8bit+memory) ;
           
        *(a + i)  =        temp +  temp_f /100 ; memory++;
              
       }
       else{ *(a + i)  =        temp  ; 
        }
       

    }
    temp=0;memory=0;
            
   for(int i=0;i<n0f16;i++){
        
       temp  =    *(decimalpart_16bit+i);

       if(frac_indx_arry16[i]==1){
           temp_f  = *(fractionpart_16bit+memory) ;
           
        *(b + i)  =        temp +  temp_f /100 ; memory++;
              
       }
     else{ *(b + i)  =        temp  ; 
        }
       

    }

                                                         
}



uint8_t p253_assign_signs_8_16( p253_t *p253,uint8_t *sign_byte_array,float *data_raw8bit,float *data_raw16bit ){    
       uint8_t   signbyte=0;uint8_t  storage=0;uint8_t  index_memory=0;  uint8_t  index=0; uint8_t  temp=0;

        uint8_t  n0f8  =p253->n_of8bit ;uint8_t n0f16 = p253->n_of16bit;uint8_t sign_byte_array_index  =  p253->sign_byte_array_index; 

    //   (1111 1111)

        for(int i=0;i< n0f8   ;i++){
            temp=*(sign_byte_array+index)>>i  ; //  Serial.print( "i:"); Serial.print( i);   Serial.print( "  ");      Serial.print( "*(sign_byte_array+index)>>i):");  Serial.println(temp);  Serial.print( "  ");      Serial.print( "*(sign_byte_array+index)>>i):");  Serial.println((temp)& 0x01);
                        if(  ((temp)& 0x01) ==1){
                              
                          *(data_raw8bit+i)= *(data_raw8bit+i)*-1;
                        }
                        if(i==(8*(index+1))-1 ) {  // 
                          index++;i=0;
                        }
                          index_memory=i;

        }


         for(int i=0;i< n0f16   ;i++){
           temp=*(sign_byte_array+index)>>i+  index_memory+1;
                       if(  ((temp)& 0x01) ==1)
                        {
                          *(data_raw16bit+i)= *(data_raw16bit+i)*-1;
                        }
                      if( i+index_memory+1== (8*(index+1))-1 )  {
                          index++;i=0;
                        }  


         }
                     
    
 

        
}
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	


	
	
	
	




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
  // for( i =0;i<length;i++)  // 0 i�in buffer_global(1) e data_gl[0]  i atiyor.    // 1 i�in 
   // {
     // buffer[i+1]=  (*data+i)  ;
   // }
                        
  
  
    // if  (HAL_I2C_Master_Transmit(&hi2c1,dev_addr,buffer,length+1,100)==HAL_OK  ){return 0;}
   // else {return -1;}
// }
