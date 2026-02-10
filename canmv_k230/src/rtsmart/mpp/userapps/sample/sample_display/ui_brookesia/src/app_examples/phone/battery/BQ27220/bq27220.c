#include <math.h>
//#include <vector>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <pthread.h>
#include <rtthread.h>
//#include <rtdevice.h>
#include <poll.h>
#include "fpioa/rt_fpioa.h"
#include "bq27220.h"
#include "bq27220_data_memory.h"
//using namespace std;
/* ioctl */
#define	GPIO_DM_OUTPUT           _IOW('G', 0, int)
#define	GPIO_DM_INPUT            _IOW('G', 1, int)
#define	GPIO_DM_INPUT_PULL_UP    _IOW('G', 2, int)
#define	GPIO_DM_INPUT_PULL_DOWN  _IOW('G', 3, int)
#define	GPIO_WRITE_LOW           _IOW('G', 4, int)
#define	GPIO_WRITE_HIGH          _IOW('G', 5, int)

#define	GPIO_PE_RISING           _IOW('G', 7, int)
#define	GPIO_PE_FALLING          _IOW('G', 8, int)
#define	GPIO_PE_BOTH             _IOW('G', 9, int)
#define	GPIO_PE_HIGH             _IOW('G', 10, int)
#define	GPIO_PE_LOW              _IOW('G', 11, int)

#define RT_I2C_DEV_CTRL_10BIT (0x800 + 0x01)
#define RT_I2C_DEV_CTRL_TIMEOUT (0x800 + 0x03)
#define RT_I2C_DEV_CTRL_RW (0x800 + 0x04)
#define RT_I2C_DEV_CTRL_CLK (0x800 + 0x05)

typedef struct {
    uint16_t addr;
    uint16_t flags;
    uint16_t len;
    uint8_t *buf;

} i2c_msg_t;

typedef struct {
    i2c_msg_t *msgs;
    size_t number;
} i2c_priv_data_t;

static int fd_i2c0=-1;
static int bq27220_read_cmd_data(int fd,uint8_t reg_addr,uint8_t *reg_data, int length)
{
    int ret=0;
        i2c_msg_t msgs[2]=
        {
         {
        .addr=0x55, 
        .flags=0, 
        .len=1, 
        .buf=&reg_addr
        },
        {
        .addr=0x55, 
        .flags=1, 
        .len=(uint16_t)length, 
        .buf=reg_data
        }
        };
        i2c_priv_data_t privdata={msgs, 2};
        if (ioctl(fd, RT_I2C_DEV_CTRL_RW, &privdata) != 0){ //读写数据
            ret=-1;
        }

    return ret;
}
static int bq27220_write_cmd_params(int fd, uint8_t*cmd_params,int length)
{
    int ret=0;
    i2c_msg_t msgs[1]=
        {
        {
        .addr=0x55, 
        .flags=0, 
        .len=(uint16_t)length, 
        .buf=cmd_params
        }
        
        };
        i2c_priv_data_t privdata={msgs, 1};
        if (ioctl(fd, RT_I2C_DEV_CTRL_RW, &privdata) != 0){ //读写数据
            ret=-1;
        }
    return ret;
}






void bq27220_init()
{
    fpioa_set_function(32,IIC0_SCL,-1,-1,-1,-1,-1,-1,-1);
    fpioa_set_function(33,IIC0_SDA,-1,-1,-1,-1,-1,-1,-1);    
    fd_i2c0 = open("/dev/i2c0", O_RDWR);
    if (fd_i2c0> 0)
    {
        uint32_t spped=100000;
     if (ioctl(fd_i2c0, RT_I2C_DEV_CTRL_CLK, &spped) != 0){ //设置速率
        printf("set speed %d failed!\n",spped);
    }
    //init_bq27220
    
    
    
    }

}
uint16_t bq27220_getDeviceNumber(void)  // sub-commands
{
    if(fd_i2c0<=0)
     {
        return 0;
     }
    uint8_t cmd_params[3]={0x00,0x01,0x00};
    bq27220_write_cmd_params(fd_i2c0,cmd_params,3);//trigger measure
    usleep(15000);
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x40,bq27220_data,2);
    //printf("bq27220 device_number:%02x %02x\n",bq27220_data[0],bq27220_data[1]);//0x20 0x02 0x00 0x00
    uint16_t val=((bq27220_data[1]<<8)|bq27220_data[0]);
    return val;
}
uint16_t bq27220_getVoltage(void)
{
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x08,bq27220_data,2);
    uint16_t bq27220_voltage=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_voltage():%02x %02x,value:%dmV\n",bq27220_data[0],bq27220_data[1],bq27220_voltage);
    return bq27220_voltage;
}
int16_t bq27220_getCurrent(void)
{
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x0C,bq27220_data,2);
    int16_t bq27220_current=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_current():%02x %02x,value:%dmA\n",bq27220_data[0],bq27220_data[1],bq27220_current);
    return bq27220_current;  
}

 void bq27220_getControlStatus(BQ27220ControlStatus *ctrl_sta)
 {
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x00,bq27220_data,2);
    (*ctrl_sta).full = ((bq27220_data[1]<<8)|bq27220_data[0]);
 }
 void bq27220_getBatteryStatus(BQ27220BatteryStatus *batt_sta)
 {
    
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x0A,bq27220_data,2);
    (*batt_sta).full = ((bq27220_data[1]<<8)|bq27220_data[0]);  
 }
 void bq27220_getOperationStatus(BQ27220OperationStatus *oper_sta)
 {
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x3A,bq27220_data,2);
    (*oper_sta).full = ((bq27220_data[1]<<8)|bq27220_data[0]);
 
 }
 void bq27220_getGaugingStatus(BQ27220GaugingStatus *gauging_sta)
 {
    uint8_t cmd_params[3]={0x00,0x56,0x00};
    bq27220_write_cmd_params(fd_i2c0,cmd_params,3);//trigger measure
    usleep(15000);
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x40,bq27220_data,2);
    //printf("bq27220 gaug status:%02x %02x\n",bq27220_data[0],bq27220_data[1]);//0x20 0x02 0x00 0x00
     (*gauging_sta).full =((bq27220_data[1]<<8)|bq27220_data[0]);
 }
    float bq27220_getTemperature(void)
    {
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x06,bq27220_data,2);
    float bq27220_temperature_k=(float)((bq27220_data[1]<<8)|bq27220_data[0])/10.0;
    float bq27220_temperature_c=bq27220_temperature_k-273.15;
    //printf("bq27220 get_temperature():%02x %02x,value:%.2fK,value:%.2fC\n",bq27220_data[0],bq27220_data[1],bq27220_temperature_k,bq27220_temperature_c); 
    return  bq27220_temperature_c;
    }
    uint16_t bq27220_getFullChargeCapacity(void)
    {
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x12,bq27220_data,2);
    uint16_t full_charge_capacity=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 full charge capacity:%02x %02x,value:%dmAh\n",bq27220_data[0],bq27220_data[1],full_charge_capacity);
    return full_charge_capacity;
    }
    uint16_t bq27220_getDesignCapacity(void)
    {
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x3C,bq27220_data,2);
    uint16_t design_capacity=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 design capacity:%02x %02x,value:%dmAh\n",bq27220_data[0],bq27220_data[1],design_capacity); 
    return design_capacity;
    }
    uint16_t bq27220_getRemainingCapacity(void)
    {
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x10,bq27220_data,2);
    uint16_t bq27220_remaining_capacity=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_remaining_capacity():%02x %02x,value:%dmAh\n",bq27220_data[0],bq27220_data[1],bq27220_remaining_capacity);
    return bq27220_remaining_capacity; 
    }
    uint16_t bq27220_getStateOfCharge(void)
    {
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x2C,bq27220_data,2);
    uint16_t bq27220_soc=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_soc():%02x %02x,value:%d%%\n",bq27220_data[0],bq27220_data[1],bq27220_soc);
    return bq27220_soc;
    }
    uint16_t bq27220_getStateOfHealth(void)
    {
    uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x2E,bq27220_data,2);
    uint16_t bq27220_soh=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_soh():%02x %02x,value:%d%%\n",bq27220_data[0],bq27220_data[1],bq27220_soh);
    return bq27220_soh;
    }
    uint16_t bq27220_getChargeVoltageMax(void)
    {
     uint8_t bq27220_data[2];
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x30,bq27220_data,2);
    uint16_t bq27220_voltage=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_voltage():%02x %02x,value:%dmV\n",bq27220_data[0],bq27220_data[1],bq27220_voltage);
    return bq27220_voltage;
    }
//////////////////////////////////////

   void bq27220_control_access(int access_mode)
   {
      if(access_mode==3) // seal
      {
    uint8_t cmd_params[3]={0x00,0x30,0x00};
    bq27220_write_cmd_params(fd_i2c0,cmd_params,3);
    usleep(15000);  
      }
      else if(access_mode==2)//unseal
      {
       uint8_t cmd_params[3]={0x00,0x14,0x04};
    bq27220_write_cmd_params(fd_i2c0,cmd_params,3);
    usleep(15000);  
     uint8_t cmd_params1[3]={0x00,0x72,0x36};
    bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    usleep(15000);  
      }
       else if(access_mode==1)//full
      {
      uint8_t cmd_params[3]={0x00,0xff,0xff};
    bq27220_write_cmd_params(fd_i2c0,cmd_params,3);
    usleep(15000);  
     uint8_t cmd_params1[3]={0x00,0xff,0xff};
    bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    usleep(15000);
      }   
  }









static uint8_t bq27220_get_checksum(uint8_t* data, uint16_t len) {
    uint8_t ret = 0;
    for(uint16_t i = 0; i < len; i++) {
        ret += data[i];
    }
    return 0xFF - ret;
}

bool bq27220_parameterCheck(uint16_t address, uint32_t value, size_t size, bool update)
{
    if(!(size == 1 || size == 2 || size == 4)) {
        printf("(%d) Parameter size error\n", __LINE__);
        return false;
    }

    bool ret = false;
    uint8_t buffer[6] = {0};
    uint8_t old_data[4] = {0};
    do {
        buffer[0] = address & 0xFF;
        buffer[1] = (address >> 8) & 0xFF;
        for(size_t i = 0; i < size; i++) {
            buffer[1 + size - i] = (value >> (i * 8)) & 0xFF;
        }
        uint8_t cmd_params[7];
        if(update) {
            cmd_params[0]=0x3E;
            for(int i=0;i<6;i++)
            {
            cmd_params[i+1]=buffer[i];
            }   
            if(bq27220_write_cmd_params(fd_i2c0,cmd_params,size+2+1)==-1) {
                printf("(%d) DM write failed\n", __LINE__);
                break;
            }
            // We must wait, otherwise write will fail
            usleep(15000);

            // Calculate the check sum: 0xFF - (sum of address and data) OR 0xFF
            uint8_t checksum = bq27220_get_checksum(buffer, size + 2);
            // Write the check sum to 0x60 and the total length of (address + parameter data + check sum + length) to 0x61
            buffer[0] = checksum;
            // 2 bytes address, `size` bytes data, 1 byte check sum, 1 byte length
            buffer[1] = 2 + size + 1 + 1;
            cmd_params[0]=0x60;
            for(int i=0;i<6;i++)
            {
            cmd_params[i+1]=buffer[i];
            }
              
            if(bq27220_write_cmd_params(fd_i2c0,cmd_params,size+2+1)==-1) {
                printf("(%d) CRC write failed\n", __LINE__);
                break;
            }
            // We must wait, otherwise write will fail
            usleep(10000);
            ret = true;
        } else {
               cmd_params[0]=0x3E;
            for(int i=0;i<6;i++)
            {
            cmd_params[i+1]=buffer[i];
            }   
            if(bq27220_write_cmd_params(fd_i2c0,cmd_params,2+1)==-1) {
                printf("(%d) DM SelectSubclass for read failed\n", __LINE__);
                break;
            }
            // bqstudio uses 15ms wait delay here
            usleep(15000); 
            uint8_t bq27220_data[2];
    	    memset(bq27220_data,0,2);
            if(bq27220_read_cmd_data(fd_i2c0,0x40,old_data,size)==-1) {
                printf("(%d) DM read failed\n", __LINE__);
                break;
            }
            // bqstudio uses burst reads with continue(CommandSelectSubclass without argument) and ~5ms between burst
            usleep(5000);
            if(*(uint32_t*)&(old_data[0]) != *(uint32_t*)&(buffer[2])) {
                printf(
                    "(%d) Data at 0x%04x(%zu): 0x%08x!=0x%08x\n", __LINE__,
                    address,
                    size,
                    *(uint32_t*)&(old_data[0]),
                    *(uint32_t*)&(buffer[2]));
            } else {
                ret = true;
            }
        }
    } while(0);

    return ret;
}

bool bq27220_dateMemoryCheck(const BQ27220DMData *data_memory, bool update)
{
 if(update) {
        uint8_t cmd_params[3]={0x3E,0x90,0x00};
        if(bq27220_write_cmd_params(fd_i2c0,cmd_params,3)==-1)      
        {
            printf("(%d) ENTER_CFG_UPDATE command failed", __LINE__);
            return false;
        }
        usleep(15000);    
    }
 bool result = true;
    while (data_memory->type != BQ27220DMTypeEnd)
    {
        /*if(data_memory->type == BQ27220DMTypeWait) {
            usleep(data_memory->value.u32);
        } else if(data_memory->type == BQ27220DMTypeU8) {
            result &= bq27220_parameterCheck(data_memory->address, data_memory->value.u8, 1, update);
        } else if(data_memory->type == BQ27220DMTypeU16) {
            result &= bq27220_parameterCheck(data_memory->address, data_memory->value.u16, 2, update);
        } else if(data_memory->type == BQ27220DMTypeU32) {
            result &= bq27220_parameterCheck(data_memory->address, data_memory->value.u32, 4, update);
        } else if(data_memory->type == BQ27220DMTypeI8) {
            result &= bq27220_parameterCheck(data_memory->address, data_memory->value.i8, 1, update);
        } else if(data_memory->type == BQ27220DMTypeI16) {
            result &= bq27220_parameterCheck(data_memory->address, data_memory->value.i16, 2, update);
        } else if(data_memory->type == BQ27220DMTypeI32) {
            result &= bq27220_parameterCheck(data_memory->address, data_memory->value.i32, 4, update);
        } else if(data_memory->type == BQ27220DMTypeF32) {
            result &= bq27220_parameterCheck(data_memory->address, data_memory->value.u32, 4, update);
        } else if(data_memory->type == BQ27220DMTypePtr8) {
            result &= bq27220_parameterCheck(data_memory->address, *(uint8_t*)data_memory->value.u32, 1, update);
        } else if(data_memory->type == BQ27220DMTypePtr16) {
            result &= bq27220_parameterCheck(data_memory->address, *(uint16_t*)data_memory->value.u32, 2, update);
        } else if(data_memory->type == BQ27220DMTypePtr32) {
            result &= bq27220_parameterCheck(data_memory->address, *(uint32_t*)data_memory->value.u32, 4, update);
        } else {
            printf("(%d) Invalid DM Type\n", __LINE__);
        }*/
        data_memory++;
    }

 // Finalize configuration update
    if(update && result) {
        uint8_t cmd_params[3]={0x3E,0x91,0x00};
        if(bq27220_write_cmd_params(fd_i2c0,cmd_params,3)==-1)      
        {
            printf("(%d) ENTER_CFG_UPDATE command failed", __LINE__);
            return false;
        }    
       usleep(15000);       
    }
    return result;
}


























