1，THE SEQUENCE FOR START UP CODEC（ CODEC初始化流程参考）：
/*reset codec*/ 
aucodec_i2c_write(0x00,0x1F);
aucodec_i2c_write(0x45,0x00);
/*set ADC/DAC CLK*/
aucodec_i2c_write(0x01,0x30);//set mclkin,clk_adc/clk_dac
aucodec_i2c_write(0x02,0x10);
aucodec_i2c_write(0x02,0x00);//set digmclk,clock doubler
aucodec_i2c_write(0x03,0x10);//ADC FDMODE,ADC_OSR
aucodec_i2c_write(0x16,0x24);//set ADC_SYNC mode
aucodec_i2c_write(0x04,0x10);//DAC_OSR
aucodec_i2c_write(0x05,0x00);//set adc_mclk,dac_mclk
/*set system power up*/
aucodec_i2c_write(0x0B,0x00);//power up stage A/B time
aucodec_i2c_write(0x0C,0x00);//power up stage B/C time
aucodec_i2c_write(0x10,0x1F);//set vmid/ibias 
aucodec_i2c_write(0x11,0x7F);//set vsel 
aucodec_i2c_write(0x00,0x80);//chip powerup.slave mode.master mode(0xC0)
DELAY_MS(50);
aucodec_i2c_write(0x0D,0x01);//power up analog
aucodec_i2c_write(0x01,0x3F);//power up digital
/*set adc*/
aucodec_i2c_write(0x14,0x14);//dmic off,input selection,PGA gain  
/*set dac*/
aucodec_i2c_write(0x12,0x00);
/*enable HP drive */
aucodec_i2c_write(0x13,0x10);
/*set adc/dac data format*/
aucodec_i2c_write(0x09,0x00);//set dac format=24bit i2s
aucodec_i2c_write(0x0A,0x00);//set adc format=24bit i2s
/*set low or normal power mode*/
aucodec_i2c_write(0x0E,0x02);//enable analog pga/adc modulator
aucodec_i2c_write(0x0F,0x44);//normal power mode
/*set adc */
aucodec_i2c_write(0x15,0x40);//set softramp
aucodec_i2c_write(0x1B,0x0A);//set adc hpf
aucodec_i2c_write(0x1C,0x6A);//set adc hpf,ADC_EQ bypass
aucodec_i2c_write(0x17,0xBF);//set adc digtal vol
/*set dac */
aucodec_i2c_write(0x37,0x48);//set dac softramp,disable DAC_EQ
aucodec_i2c_write(0x32,0xBF);
/*set adc/dac data form*/
aucodec_i2c_write(0x44,0x08);//adc to dac disable,ADCDATA=ADC(L)+ADC(R)
/*only set adc alc funtion for amic record*/
aucodec_i2c_write(0x16,0x22);//set adc gain scale up                
aucodec_i2c_write(0x17,0xCF);//set adc alc maxgain                 
aucodec_i2c_write(0x18,0x87);//adc alc enable,alc_winsize   
aucodec_i2c_write(0x19,0xFB);//set alc target level          
aucodec_i2c_write(0x1A,0x03);//set adc_automute noise gate         
aucodec_i2c_write(0x1B,0xEA);//set adc_automute vol
注：/***
当应用中需要得到更低功耗而不是高性能,同时不需要ES8311输出直接驱动耳机情况下，需要将以下几个寄存器设置寄值覆盖
aucodec_i2c_write(0x10,0x0C);//set vmid/ibias 
aucodec_i2c_write(0x11,0x7B);//set vsel 
aucodec_i2c_write(0x13,0x00);//disable HP drive
aucodec_i2c_write(0x0E,0x0A);//LP vrefbuf
aucodec_i2c_write(0x0F,0xFF);//set in LP mode
aucodec_i2c_write(0x1B,0x05);//set adc hpf
aucodec_i2c_write(0x1C,0x45);//set adc hpf,ADC_EQ bypass
***/
2，THE SEQUENCE FOR START UP ADC （仅初始化ADC流程参考）：
/*reset codec*/ 
aucodec_i2c_write(0x00,0x1F);
aucodec_i2c_write(0x45,0x00);
/*set ADC/DAC CLK*/
aucodec_i2c_write(0x01,0x30);//set mclkin,clk_adc/clk_dac
aucodec_i2c_write(0x02,0x10);
aucodec_i2c_write(0x02,0x00);//set digmclk,clock doubler
aucodec_i2c_write(0x03,0x10);//ADC FDMODE,ADC_OSR
aucodec_i2c_write(0x16,0x24);//set ADC_SYNC mode
aucodec_i2c_write(0x04,0x10);//DAC_OSR
aucodec_i2c_write(0x05,0x00);//set adc_mclk,dac_mclk
/*set system power up*/
aucodec_i2c_write(0x0B,0x7B);//power up stage A/B time
aucodec_i2c_write(0x0C,0x8F);//power up stage B/C time
aucodec_i2c_write(0x10,0x03);//set vmid/ibias
aucodec_i2c_write(0x11,0x7B);//set vsel
aucodec_i2c_write(0x00,0x81);//chip powerup,master mode(0xC1)
DELAY_MS(50);
aucodec_i2c_write(0x0D,0x0A);//power up analog
aucodec_i2c_write(0x01,0x3A);//power up digital
/*set adc*/
aucodec_i2c_write(0x14,0x1A);//dmic off,input selection,PGA gain  
/*set adc data format*/
aucodec_i2c_write(0x0A,0x00);//set adc format=24bit i2s
/*set low power mode*/
aucodec_i2c_write(0x0E,0x0A);//enable analog pga/adc modulator
aucodec_i2c_write(0x0F,0xF9);
/*set adc */
aucodec_i2c_write(0x15,0xA0);//set softramp
aucodec_i2c_write(0x1B,0x0A);//set adc hpf
/*adc alc funtion for amic record*/
aucodec_i2c_write(0x16,0x22);//set adc gain scale up                
aucodec_i2c_write(0x17,0xCF);//set adc alc maxgain                 
aucodec_i2c_write(0x18,0x8A);//adc alc enable,alc_winsize   
aucodec_i2c_write(0x19,0xFB);//set alc target level          
aucodec_i2c_write(0x1A,0x03);//set adc_automute noise gate         
aucodec_i2c_write(0x1B,0xEA);//set adc_automute vol    
/*set adc/dac data form*/
aucodec_i2c_write(0x44,0x08);//adc to dac disable,ADCDATA=ADC(L)+ADC(R)
3，THE SEQUENCE FOR START UP DAC （仅初始化DAC流程参考）：
/*reset codec*/ 
aucodec_i2c_write(0x00,0x1F);
aucodec_i2c_write(0x45,0x00);
/*set ADC/DAC CLK*/
aucodec_i2c_write(0x01,0x30);//set mclkin,clk_adc/clk_dac
aucodec_i2c_write(0x02,0x10);
aucodec_i2c_write(0x02,0x00);//set digmclk,clock doubler
aucodec_i2c_write(0x03,0x10);//ADC FDMODE,ADC_OSR
aucodec_i2c_write(0x16,0x24);//set ADC_SYNC mode
aucodec_i2c_write(0x04,0x10);//DAC_OSR
aucodec_i2c_write(0x05,0x00);//set adc_mclk,dac_mclk
/*set system power up*/
aucodec_i2c_write(0x0B,0x00);//power up stage A/B time
aucodec_i2c_write(0x0C,0x00);//power up stage B/C time
aucodec_i2c_write(0x10,0x1F);//set vmid/ibias
aucodec_i2c_write(0x11,0x5F);//set vsel
aucodec_i2c_write(0x00,0x82);//chip powerup,master mode(0xC2)
DELAY_MS(50);
aucodec_i2c_write(0x0D,0x31);//power up analog
aucodec_i2c_write(0x01,0x35);//power up digital
/*set dac*/
aucodec_i2c_write(0x12,0x28);
/*enable HP drive */
aucodec_i2c_write(0x13,0x10);
/*set dac data format*/
aucodec_i2c_write(0x09,0x00);//set dac format=24bit i2s
/*set low power mode*/
aucodec_i2c_write(0x0F,0x7F);
/*set adc */
aucodec_i2c_write(0x15,0x40);//set softramp
aucodec_i2c_write(0x1B,0x05);//set adc hpf
aucodec_i2c_write(0x1C,0x45);//set adc hpf,ADC_EQ bypass
aucodec_i2c_write(0x17,0xBF);//set adc digtal vol
/*set dac */
aucodec_i2c_write(0x37,0x48);//set dac softramp,disable DAC_EQ
aucodec_i2c_write(0x32,0xBF);
/*set adc/dac data form*/
aucodec_i2c_write(0x44,0x08);//adc to dac disable,ADCDATA=ADC(L)+ADC(R) 
4，THE SEQUENCE FOR STANDBY（CODEC休眠流程参考）：
aucodec_i2c_write(0x32,0x00);//set dac digtal vol -96db
aucodec_i2c_write(0x17,0x00);//set adc digtal vol -96db
aucodec_i2c_write(0x0E,0xFF);//power down analog pga/ADC
aucodec_i2c_write(0x12,0x02);//power down DAC
aucodec_i2c_write(0x14,0x00);//set ADC_SYNC mode
aucodec_i2c_write(0x0D,0xFA);//power down analog
aucodec_i2c_write(0x00,0x00);//power down csm
aucodec_i2c_write(0x00,0x1F);//reset digtal 
aucodec_i2c_write(0x01,0x30);//annlog clk off
aucodec_i2c_write(0x01,0x00);//mclk/bclk off
aucodec_i2c_write(0x45,0x01);//bclk/lrck pullup off
aucodec_i2c_write(0x0D,0xFC);//power down DAC_Vref
