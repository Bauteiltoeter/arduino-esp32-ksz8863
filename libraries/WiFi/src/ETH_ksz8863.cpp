#include "ETH_ksz8863.h"

#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include <Wire.h>
#include <HardwareSerial.h>
#include <mutex>

#define KSZ8863_ADR 0b1011111
//use register / bit definitions
#define KSZ_REG_PORT1_STATUS0 30
#define KSZ_REG_PORT2_STATUS0 46
#define KSZ_REG_PORT3_STATUS_1 63
#define KSZ_REG_RESET 67
#define KSZ_BIT_LINK_GOOD 5
#define KSZ_BIT_SW_RESET 4
#define KSZ_BIT_SPEED 2
#define KSZ_BIT_DUPLEX 1


static std::mutex i2c_mutex;

void ksz8863_gpio_init(int scl, int sda)
{
    Wire.begin(sda, scl, 100000);
}

//I2C Access functions
static bool ksz8863_read_reg(uint8_t reg, uint8_t* data)
{
    std::lock_guard<std::mutex> lock(i2c_mutex);
    Wire.beginTransmission(KSZ8863_ADR);
    int writtenBytes = Wire.write(reg); //register

    Wire.endTransmission(false);

   

    Wire.requestFrom(KSZ8863_ADR,1);
    

    *data  = Wire.read();

    Wire.endTransmission();
    

    return true;
}

static void ksz_write_reg(uint8_t reg, uint8_t value)
{
    std::lock_guard<std::mutex> lock(i2c_mutex);
    Wire.beginTransmission(KSZ8863_ADR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

bool ksz8863_phy_check_link(void)
{
    // We cannot get a link status on Port 3 so we check Port 1 and Port 2 to see if any link is up
    uint8_t port1_status_0;
    bool readOk = ksz8863_read_reg(KSZ_REG_PORT1_STATUS0, &port1_status_0);
    uint8_t port2_status_0;
    readOk = readOk & ksz8863_read_reg(KSZ_REG_PORT2_STATUS0, &port2_status_0);
    

    bool linkOk= false;
    static bool lastLinkOk = false;


    if ( !readOk)
	    return lastLinkOk;

    if(port1_status_0 & (1<<KSZ_BIT_LINK_GOOD))
    {
        //Serial.printf("Port 1 Link good\n");
        linkOk = true;
    }
    else
    {
       ////Serial.printf("Port 1 Link bad\n");
    }

    if(port2_status_0 & (1<<KSZ_BIT_LINK_GOOD))
    {
       // //Serial.printf("Port 2 Link good\n");
        linkOk = true;
    }
    else
    {
       ////Serial.printf("Port 2 Link bad\n");
    }


    if(linkOk)
    {
       // Serial.printf("Ports link ok\n");
    }
    else
    {
        Serial.printf("ports link bad\n");
    }

    lastLinkOk = linkOk;
    return linkOk;
}


void ksz8863_phy_check_init(void)
{
    //Check for AN done and wait.. but there is no AN on the RMII interface
    vTaskDelay(10/portTICK_RATE_MS);
}

eth_speed_mode_t ksz8863_phy_get_speed_mode(void)
{
    uint8_t reg;
    
    ksz8863_read_reg(KSZ_REG_PORT3_STATUS_1, &reg);

    if( reg & (1<<KSZ_BIT_SPEED))
    {
        Serial.printf( "Port 3 speed mode: 100M");
        return ETH_SPEED_MODE_100M;
    }
    else
    {
        Serial.printf( "Port 3 speed mode: 10M");
        return ETH_SPEED_MODE_10M;
    }
}

eth_duplex_mode_t ksz8863_phy_get_duplex_mode(void)
{
    uint8_t reg;
    ksz8863_read_reg(KSZ_REG_PORT3_STATUS_1, &reg);

    if( reg & (1<<KSZ_BIT_DUPLEX))
    {
        Serial.printf( "Port 3 speed mode: Full Duplex");
        return ETH_MODE_FULLDUPLEX;
    }
    else
    {
        Serial.printf( "Port 3 speed mode: Half Duplex");
        return ETH_MODE_HALFDUPLEX;
    }
}

bool ksz_phy_get_partner_pause_enable(void)
{
    return false; //Not applicable according PHY datasheet
}

void ksz_phy_power_enable(bool enable)
{
}

#define IEEE802_3_TX_FC 5
#define IEEE802_3_RX_FC 4
#define IGMP_SNOOP_ENABLE 6

#define FORCE_FLOW_CONTROL 4 //reg 18/34/50

esp_err_t ksz_phy_init(void)
{
    //Serial.printf( "ksz_phy_init");

    //software reset
    //ESP_LOGW("KSZ", "Except next write to fail because of SW reset");
    ksz_write_reg(KSZ_REG_RESET, (1<<KSZ_BIT_SW_RESET));

    vTaskDelay(100/portTICK_RATE_MS);

    //for(int i=0; i < 220; i++)
    //{
    //  uint8_t reg;
    //  ksz8863_read_reg(i, &reg);
    //  Serial.printf("Reg %d: 0x%02x\n", i, reg);
    //}
    

    uint8_t chipId;
    ksz8863_read_reg(0, &chipId);

    if(chipId != 0x88)
    {
        //ESP_LOGE("KSZ", "Invalid chip id: 0x88 != 0x%02x", chipId);
    }
    else
    {
        Serial.printf( "ChipID ok");
    }

    uint8_t reg198;
    ksz8863_read_reg(198, &reg198);

    Serial.printf("Advanced control registers 198: 0x%02X", reg198);
    reg198 |= 0b00110000;
    ksz_write_reg(198,reg198);

    //Register 3 GLOBAL CONTROL 1
    //Working:      0x04
    //Not working:  0x34
    uint8_t reg;
    ksz8863_read_reg(3, &reg);
    reg &= ~ ( ( 1<< IEEE802_3_TX_FC) | (1<<IEEE802_3_RX_FC));
    ksz_write_reg(3,reg);

    //Register 5 GLOBAL CONTROL 3
    //Working: 0x40
    //N.work : 0x00
    ksz8863_read_reg(5, &reg);
    reg |= (1<<IGMP_SNOOP_ENABLE);
    ksz_write_reg(5, reg);

    //Register 34 Port 2 control 2
    //Working:      0x06
    //not working:  0x16
    ksz8863_read_reg(34, &reg);
    reg &=~(1<<FORCE_FLOW_CONTROL);
    ksz_write_reg(34, reg);


    //External clock
    ksz_write_reg(198,0x01);



    //Example enables Autonegotiation here, but KSZ8863 has it already on

    return ESP_OK;
}
