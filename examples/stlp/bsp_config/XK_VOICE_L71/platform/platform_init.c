// Copyright (c) 2022 XMOS LIMITED. This Software is subject to the terms of the
// XMOS Public License: Version 1

/* System headers */
#include <platform.h>

/* App headers */
#include "platform_conf.h"
#include "platform/app_pll_ctrl.h"
#include "platform/driver_instances.h"
#include "platform/platform_init.h"
#include "adaptive_rate_adjust.h"
#include "usb_support.h"

#define PRINT_PS_REG(x)  debug_printf(#x ":\t\t0x%08lX\n", getps(x))
#define PRINT_PSWITCH_REG(x)  do { unsigned tmp; read_pswitch_reg(get_local_tile_id(), x, &tmp); debug_printf(#x ":\t\t0x%08lX\n", tmp); } while (0)
#define PRINT_SSWITCH_REG(x)  do { unsigned tmp; read_sswitch_reg(get_local_tile_id(), x, &tmp); debug_printf(#x ":\t\t0x%08lX\n", tmp); } while (0)

void RegDump(void)
{
    PRINT_PS_REG(XS1_PS_VECTOR_BASE);
    PRINT_PS_REG(XS1_PS_XCORE_CTRL0);
    PRINT_PS_REG(XS1_PS_BOOT_CONFIG);
    PRINT_PS_REG(XS1_PS_BOOT_STATUS);
    PRINT_PS_REG(XS1_PS_SECURITY_CONFIG);
    PRINT_PS_REG(XS1_PS_RING_OSC_CTRL);
    PRINT_PS_REG(XS1_PS_RING_OSC_DATA0);
    PRINT_PS_REG(XS1_PS_RING_OSC_DATA1);
    PRINT_PS_REG(XS1_PS_RING_OSC_DATA2);
    PRINT_PS_REG(XS1_PS_DBG_SSR);
    PRINT_PS_REG(XS1_PS_DBG_SPC);
    PRINT_PS_REG(XS1_PS_DBG_SSP);
    PRINT_PS_REG(XS1_PS_DBG_T_NUM);
    PRINT_PS_REG(XS1_PS_DBG_T_REG);
    PRINT_PS_REG(XS1_PS_DBG_TYPE);
    PRINT_PS_REG(XS1_PS_DBG_DATA);
    PRINT_PS_REG(XS1_PS_DBG_RUN_CTRL);
    PRINT_PS_REG(XS1_PS_DBG_SCRATCH_0);
    PRINT_PS_REG(XS1_PS_DBG_SCRATCH_1);
    PRINT_PS_REG(XS1_PS_DBG_SCRATCH_2);
    PRINT_PS_REG(XS1_PS_DBG_SCRATCH_3);
    PRINT_PS_REG(XS1_PS_DBG_SCRATCH_4);
    PRINT_PS_REG(XS1_PS_DBG_SCRATCH_5);
    PRINT_PS_REG(XS1_PS_DBG_SCRATCH_6);
    PRINT_PS_REG(XS1_PS_DBG_SCRATCH_7);
    PRINT_PS_REG(XS1_PS_DBG_IBREAK_ADDR_0);
    PRINT_PS_REG(XS1_PS_DBG_IBREAK_ADDR_1);
    PRINT_PS_REG(XS1_PS_DBG_IBREAK_ADDR_2);
    PRINT_PS_REG(XS1_PS_DBG_IBREAK_ADDR_3);
    PRINT_PS_REG(XS1_PS_DBG_IBREAK_CTRL_0);
    PRINT_PS_REG(XS1_PS_DBG_IBREAK_CTRL_1);
    PRINT_PS_REG(XS1_PS_DBG_IBREAK_CTRL_2);
    PRINT_PS_REG(XS1_PS_DBG_IBREAK_CTRL_3);
    PRINT_PS_REG(XS1_PS_DBG_DWATCH_ADDR1_0);
    PRINT_PS_REG(XS1_PS_DBG_DWATCH_ADDR1_1);
    PRINT_PS_REG(XS1_PS_DBG_DWATCH_ADDR1_2);
    PRINT_PS_REG(XS1_PS_DBG_DWATCH_ADDR1_3);
    PRINT_PS_REG(XS1_PS_DBG_DWATCH_ADDR2_0);
    PRINT_PS_REG(XS1_PS_DBG_DWATCH_ADDR2_1);
    PRINT_PS_REG(XS1_PS_DBG_DWATCH_ADDR2_2);
    PRINT_PS_REG(XS1_PS_DBG_DWATCH_ADDR2_3);
    PRINT_PS_REG(XS1_PS_DBG_DWATCH_CTRL_0);
    PRINT_PS_REG(XS1_PS_DBG_DWATCH_CTRL_1);
    PRINT_PS_REG(XS1_PS_DBG_DWATCH_CTRL_2);
    PRINT_PS_REG(XS1_PS_DBG_DWATCH_CTRL_3);
    PRINT_PS_REG(XS1_PS_DBG_RWATCH_ADDR1_0);
    PRINT_PS_REG(XS1_PS_DBG_RWATCH_ADDR1_1);
    PRINT_PS_REG(XS1_PS_DBG_RWATCH_ADDR1_2);
    PRINT_PS_REG(XS1_PS_DBG_RWATCH_ADDR1_3);
    PRINT_PS_REG(XS1_PS_DBG_RWATCH_ADDR2_0);
    PRINT_PS_REG(XS1_PS_DBG_RWATCH_ADDR2_1);
    PRINT_PS_REG(XS1_PS_DBG_RWATCH_ADDR2_2);
    PRINT_PS_REG(XS1_PS_DBG_RWATCH_ADDR2_3);
    PRINT_PS_REG(XS1_PS_DBG_RWATCH_CTRL_0);
    PRINT_PS_REG(XS1_PS_DBG_RWATCH_CTRL_1);
    PRINT_PS_REG(XS1_PS_DBG_RWATCH_CTRL_2);
    PRINT_PS_REG(XS1_PS_DBG_RWATCH_CTRL_3);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DEVICE_ID0_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DEVICE_ID1_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DEVICE_ID2_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DEVICE_ID3_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DBG_CTRL_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DBG_INT_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_PLL_CLK_DIVIDER_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_SECU_CONFIG_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DBG_SCRATCH_0_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DBG_SCRATCH_1_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DBG_SCRATCH_2_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DBG_SCRATCH_3_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DBG_SCRATCH_4_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DBG_SCRATCH_5_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DBG_SCRATCH_6_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_DBG_SCRATCH_7_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T0_PC_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T1_PC_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T2_PC_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T3_PC_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T4_PC_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T5_PC_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T6_PC_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T7_PC_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T0_SR_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T1_SR_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T2_SR_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T3_SR_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T4_SR_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T5_SR_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T6_SR_NUM);
    PRINT_PSWITCH_REG(XS1_PSWITCH_T7_SR_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_DEVICE_ID0_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_DEVICE_ID1_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_DEVICE_ID2_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_DEVICE_ID3_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_NODE_CONFIG_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_NODE_ID_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_PLL_CTL_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_CLK_DIVIDER_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_REF_CLK_DIVIDER_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_JTAG_DEVICE_ID_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_XCORE0_GLOBAL_DEBUG_CONFIG_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_XCORE1_GLOBAL_DEBUG_CONFIG_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_SS_APP_PLL_FRAC_N_DIVIDER_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_SS_LPDDR_CONTROLLER_CONFIG_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_MIPI_CLK_DIVIDER_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_MIPI_CFG_CLK_DIVIDER_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_GLOBAL_DEBUG_SOURCE_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_SLINK_0_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_SLINK_1_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_SLINK_2_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_SLINK_3_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_SLINK_4_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_SLINK_5_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_SLINK_6_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_SLINK_7_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_SLINK_8_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_PLINK_0_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_PLINK_1_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_PLINK_2_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_PLINK_3_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_PLINK_4_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_PLINK_5_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_PLINK_6_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_PLINK_7_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_XLINK_0_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_XLINK_1_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_XLINK_2_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_XLINK_3_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_XLINK_4_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_XLINK_5_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_XLINK_6_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_XLINK_7_NUM);
    PRINT_SSWITCH_REG(XS1_SSWITCH_XLINK_8_NUM);
}

static void mclk_init(chanend_t other_tile_c)
{
#if !appconfEXTERNAL_MCLK && ON_TILE(1)
    app_pll_init();
#endif
#if appconfUSB_ENABLED && ON_TILE(USB_TILE_NO)
    adaptive_rate_adjust_init(other_tile_c);
#endif
}

static void flash_init(void)
{
#if ON_TILE(FLASH_TILE_NO)
    rtos_qspi_flash_init(
            qspi_flash_ctx,
            FLASH_CLKBLK,
            PORT_SQI_CS,
            PORT_SQI_SCLK,
            PORT_SQI_SIO,

            /** Derive QSPI clock from the 600 MHz xcore clock **/
            qspi_io_source_clock_xcore,

            /** Full speed clock configuration **/
            5, // 600 MHz / (2*5) -> 60 MHz,
            1,
            qspi_io_sample_edge_rising,
            0,

            /** SPI read clock configuration **/
            12, // 600 MHz / (2*12) -> 25 MHz
            0,
            qspi_io_sample_edge_falling,
            0,

            qspi_flash_page_program_1_4_4);
#endif
}

static void gpio_init(void)
{
    static rtos_driver_rpc_t gpio_rpc_config_t0;
    static rtos_driver_rpc_t gpio_rpc_config_t1;
    rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};

#if ON_TILE(0)
    rtos_gpio_init(gpio_ctx_t0);

    rtos_gpio_rpc_host_init(
            gpio_ctx_t0,
            &gpio_rpc_config_t0,
            client_intertile_ctx,
            1);

    rtos_gpio_rpc_client_init(
            gpio_ctx_t1,
            &gpio_rpc_config_t1,
            intertile_ctx);
#endif

#if ON_TILE(1)
    rtos_gpio_init(gpio_ctx_t1);

    rtos_gpio_rpc_client_init(
            gpio_ctx_t0,
            &gpio_rpc_config_t0,
            intertile_ctx);

    rtos_gpio_rpc_host_init(
            gpio_ctx_t1,
            &gpio_rpc_config_t1,
            client_intertile_ctx,
            1);
#endif
}

static void i2c_init(void)
{
    static rtos_driver_rpc_t i2c_rpc_config;

#if appconfI2C_CTRL_ENABLED
#if ON_TILE(I2C_CTRL_TILE_NO)
    rtos_i2c_slave_init(i2c_slave_ctx,
                        (1 << appconfI2C_IO_CORE),
                        PORT_I2C_SCL,
                        PORT_I2C_SDA,
                        appconf_CONTROL_I2C_DEVICE_ADDR);
#endif
#else
#if ON_TILE(I2C_TILE_NO)
    rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};
    rtos_i2c_master_init(
            i2c_master_ctx,
            PORT_I2C_SCL, 0, 0,
            PORT_I2C_SDA, 0, 0,
            0,
            100);

    rtos_i2c_master_rpc_host_init(
            i2c_master_ctx,
            &i2c_rpc_config,
            client_intertile_ctx,
            1);
#else
    rtos_i2c_master_rpc_client_init(
            i2c_master_ctx,
            &i2c_rpc_config,
            intertile_ctx);
#endif
#endif
}

static void spi_init(void)
{
#if appconfSPI_OUTPUT_ENABLED && ON_TILE(SPI_OUTPUT_TILE_NO)
    rtos_spi_slave_init(spi_slave_ctx,
                        (1 << appconfSPI_IO_CORE),
                        SPI_CLKBLK,
                        SPI_MODE_3,
                        PORT_SPI_SCLK,
                        PORT_SPI_MOSI,
                        PORT_SPI_MISO,
                        PORT_SPI_CS);
#endif
}

static void mics_init(void)
{
    static rtos_driver_rpc_t mic_array_rpc_config;
#if ON_TILE(MICARRAY_TILE_NO)
    rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};
    rtos_mic_array_init(
            mic_array_ctx,
            (1 << appconfPDM_MIC_IO_CORE),
            RTOS_MIC_ARRAY_CHANNEL_SAMPLE);
    rtos_mic_array_rpc_host_init(
            mic_array_ctx,
            &mic_array_rpc_config,
            client_intertile_ctx,
            1);
#else
    rtos_mic_array_rpc_client_init(
            mic_array_ctx,
            &mic_array_rpc_config,
            intertile_ctx);
#endif
}

static void i2s_init(void)
{
#if appconfI2S_ENABLED
#if appconfI2S_MODE == appconfI2S_MODE_MASTER
    static rtos_driver_rpc_t i2s_rpc_config;
#endif
#if ON_TILE(I2S_TILE_NO)
#if appconfI2S_MODE == appconfI2S_MODE_MASTER
    rtos_intertile_t *client_intertile_ctx[1] = {intertile_ctx};
    port_t p_i2s_dout[1] = {
            PORT_I2S_DAC_DATA
    };
    port_t p_i2s_din[1] = {
            PORT_I2S_ADC_DATA
    };

    rtos_i2s_master_init(
            i2s_ctx,
            (1 << appconfI2S_IO_CORE),
            p_i2s_dout,
            1,
            p_i2s_din,
            1,
            PORT_I2S_BCLK,
            PORT_I2S_LRCLK,
            PORT_MCLK,
            I2S_CLKBLK);

    rtos_i2s_rpc_host_init(
            i2s_ctx,
            &i2s_rpc_config,
            client_intertile_ctx,
            1);
#elif appconfI2S_MODE == appconfI2S_MODE_SLAVE
    port_t p_i2s_dout[1] = {
            PORT_I2S_ADC_DATA
    };
    port_t p_i2s_din[1] = {
            PORT_I2S_DAC_DATA
    };
    rtos_i2s_slave_init(
            i2s_ctx,
            (1 << appconfI2S_IO_CORE),
            p_i2s_dout,
            1,
            p_i2s_din,
            1,
            PORT_I2S_BCLK,
            PORT_I2S_LRCLK,
            I2S_CLKBLK);
#endif
#else
#if appconfI2S_MODE == appconfI2S_MODE_MASTER
    rtos_i2s_rpc_client_init(
            i2s_ctx,
            &i2s_rpc_config,
            intertile_ctx);
#endif
#endif
#endif
}

static void usb_init(void)
{
#if appconfUSB_ENABLED && ON_TILE(USB_TILE_NO)
    usb_manager_init();
#endif
}

#if (ON_TILE(0))
#define SYNC()    do {chan_out_byte(other_tile_c, 0xA5);} while(0)
#else
#define SYNC()   do {(void) chan_in_byte(other_tile_c);} while(0)
#endif

void platform_init(chanend_t other_tile_c)
{
    debug_printf("rtos_intertile_init (tile %d on core %d)\n", THIS_XCORE_TILE, portGET_CORE_ID());
    rtos_intertile_init(intertile_ctx, other_tile_c);

    SYNC();
    debug_printf("mclk_init (tile %d on core %d)\n", THIS_XCORE_TILE, portGET_CORE_ID());
    mclk_init(other_tile_c);

    SYNC();
    debug_printf("gpio_init (tile %d on core %d)\n", THIS_XCORE_TILE, portGET_CORE_ID());
    gpio_init();

    SYNC();
    debug_printf("flash_init (tile %d on core %d)\n", THIS_XCORE_TILE, portGET_CORE_ID());
    flash_init();

    SYNC();
    debug_printf("i2c_init (tile %d on core %d)\n", THIS_XCORE_TILE, portGET_CORE_ID());
    i2c_init();

    SYNC();
    debug_printf("spi_init (tile %d on core %d)\n", THIS_XCORE_TILE, portGET_CORE_ID());
    spi_init();

    SYNC();
    debug_printf("mics_init (tile %d on core %d)\n", THIS_XCORE_TILE, portGET_CORE_ID());
    mics_init();

    SYNC();
    debug_printf("i2s_init (tile %d on core %d)\n", THIS_XCORE_TILE, portGET_CORE_ID());
    i2s_init();

    SYNC();
    debug_printf("usb_init (tile %d on core %d)\n", THIS_XCORE_TILE, portGET_CORE_ID());
    usb_init();
}
