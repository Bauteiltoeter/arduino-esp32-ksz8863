#ifndef ETH_KSZ8863_H__
#define ETH_KSZ8863_H__

#include "esp_eth.h"

void ksz8863_gpio_init(int scl, int sda);
bool ksz8863_phy_check_link(void);
void ksz8863_phy_check_init(void);
eth_speed_mode_t ksz8863_phy_get_speed_mode(void);
eth_duplex_mode_t ksz8863_phy_get_duplex_mode(void);
bool ksz_phy_get_partner_pause_enable(void);
void ksz_phy_power_enable(bool enable);
esp_err_t ksz_phy_init(void);
#endif