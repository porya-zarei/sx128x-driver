#include "sx128x_hal.h"
#include <stdint.h> /* C99 types */
#include <stdio.h>  /* printf fprintf */
#include <stdlib.h> /* malloc free */
#include <unistd.h> /* lseek, close */
#include <fcntl.h>  /* open */
#include <string.h> /* memset */

#ifdef __LINUX_INTERFACE__
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#endif

sx128x_hal_status_t sx128x_hal_setup(const void *context)
{
    const char *com_path;
    void **com_target_ptr;
    int *spi_device = NULL;
    int dev;
    int a = 0, b = 0;
    int i;

    /* check input variables */
    CHECK_NULL(com_path);
    CHECK_NULL(com_target_ptr);

    /* allocate memory for the device descriptor */
    spi_device = malloc(sizeof(int));
    if (spi_device == NULL)
    {
        // DEBUG_MSG("ERROR: MALLOC FAIL\n");
        return SX128X_HAL_STATUS_ERROR;
    }

    /* open SPI device */
    dev = open(com_path, O_RDWR);
    if (dev < 0)
    {
        // DEBUG_PRINTF("ERROR: failed to open SPI device %s\n", com_path);
        return SX128X_HAL_STATUS_ERROR;
    }

    /* setting SPI mode to 'mode 0' */
    i = SPI_MODE_0;
    a = ioctl(dev, SPI_IOC_WR_MODE, &i);
    b = ioctl(dev, SPI_IOC_RD_MODE, &i);
    if ((a < 0) || (b < 0))
    {
        // DEBUG_MSG("ERROR: SPI PORT FAIL TO SET IN MODE 0\n");
        close(dev);
        free(spi_device);
        return SX128X_HAL_STATUS_ERROR;
    }

    /* setting SPI max clk (in Hz) */
    i = SPI_SPEED;
    a = ioctl(dev, SPI_IOC_WR_MAX_SPEED_HZ, &i);
    b = ioctl(dev, SPI_IOC_RD_MAX_SPEED_HZ, &i);
    if ((a < 0) || (b < 0))
    {
        // DEBUG_MSG("ERROR: SPI PORT FAIL TO SET MAX SPEED\n");
        close(dev);
        free(spi_device);
        return SX128X_HAL_STATUS_ERROR;
    }

    /* setting SPI to MSB first */
    i = 0;
    a = ioctl(dev, SPI_IOC_WR_LSB_FIRST, &i);
    b = ioctl(dev, SPI_IOC_RD_LSB_FIRST, &i);
    if ((a < 0) || (b < 0))
    {
        // DEBUG_MSG("ERROR: SPI PORT FAIL TO SET MSB FIRST\n");
        close(dev);
        free(spi_device);
        return SX128X_HAL_STATUS_ERROR;
    }

    /* setting SPI to 8 bits per word */
    i = 0;
    a = ioctl(dev, SPI_IOC_WR_BITS_PER_WORD, &i);
    b = ioctl(dev, SPI_IOC_RD_BITS_PER_WORD, &i);
    if ((a < 0) || (b < 0))
    {
        // DEBUG_MSG("ERROR: SPI PORT FAIL TO SET 8 BITS-PER-WORD\n");
        close(dev);
        return SX128X_HAL_STATUS_ERROR;
    }

    *spi_device = dev;
    *com_target_ptr = (void *)spi_device;
    // DEBUG_MSG("Note: SPI port opened and configured ok\n");
    return SX128X_HAL_STATUS_OK;
}

sx128x_hal_status_t sx128x_hal_end()
{
    void *com_target;
    int spi_device;
    int a;

    /* check input variables */
    CHECK_NULL(com_target);

    /* close file & deallocate file descriptor */
    spi_device = *(int *)com_target; /* must check that spi_target is not null beforehand */
    a = close(spi_device);
    free(com_target);

    /* determine return code */
    if (a < 0)
    {
        // DEBUG_MSG("ERROR: SPI PORT FAILED TO CLOSE\n");
        return SX128X_HAL_STATUS_ERROR;
    }
    else
    {
        // DEBUG_MSG("Note: SPI port closed\n");
        return SX128X_HAL_STATUS_OK;
    }
}

sx128x_hal_status_t sx128x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length, const uint8_t *data, const uint16_t data_length)
{
    void *com_target;
    uint8_t spi_mux_target;

    int spi_device;
    struct spi_ioc_transfer k;
    int a;

    /* check input variables */
    CHECK_NULL(com_target);

    spi_device = *(int *)com_target; /* must check that spi_target is not null beforehand */

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long)command;
    k.len = command_length;
    k.speed_hz = SPI_SPEED;
    k.cs_change = 0;
    k.bits_per_word = 8;
    a = ioctl(spi_device, SPI_IOC_MESSAGE(1), &k);

    /* determine return code */
    if (a != (int)k.len)
    {
        // DEBUG_MSG("ERROR: SPI WRITE FAILURE\n");
        return SX128X_HAL_STATUS_ERROR;
    }
    else
    {
        // DEBUG_MSG("Note: SPI write success\n");
        memcpy(data, &a, data_length);
        return SX128X_HAL_STATUS_OK;
    }
}

sx128x_hal_status_t sx128x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length, uint8_t *data, const uint16_t data_length)
{

    void *com_target;
    uint8_t spi_mux_target;

    int spi_device;
    // uint8_t in_buf[ARRAY_SIZE(out_buf)];
    struct spi_ioc_transfer k;
    int a;

    /* check input variables */
    CHECK_NULL(com_target);
    CHECK_NULL(data);

    spi_device = *(int *)com_target; /* must check that com_target is not null beforehand */

    /* I/O transaction */
    memset(&k, 0, sizeof(k)); /* clear k */
    k.tx_buf = (unsigned long)command;
    k.rx_buf = (unsigned long)data;
    k.len = command_length;
    k.cs_change = 0;
    a = ioctl(spi_device, SPI_IOC_MESSAGE(1), &k);

    /* determine return code */
    if (a != (int)k.len)
    {
        // DEBUG_MSG("ERROR: SPI READ FAILURE\n");
        return SX128X_HAL_STATUS_ERROR;
    }
    else
    {
        // DEBUG_MSG("Note: SPI read success\n");
        return SX128X_HAL_STATUS_OK;
    }
}

sx128x_hal_status_t sx128x_hal_reset(const void *context)
{
}

sx128x_hal_status_t sx128x_hal_wakeup(const void *context)
{
}

// typedef enum
// {
//     RADIO_SLEEP,
//     RADIO_AWAKE
// } radio_sleep_mode_t;

// /*
//  * -----------------------------------------------------------------------------
//  * --- PRIVATE VARIABLES -------------------------------------------------------
//  */

// // This variable will hold the current sleep status of the radio
// static radio_sleep_mode_t radio_mode = RADIO_AWAKE;

// /*
//  * -----------------------------------------------------------------------------
//  * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
//  */

// /**
//  * @brief Wait until radio busy pin is reset to 0
//  */
// static void sx128x_hal_wait_on_busy(void);

// /**
//  * @brief Check if device is ready to receive spi transaction.
//  * @remark If the device is in sleep mode, it will awake it and wait until it is ready
//  */
// static void sx128x_hal_check_device_ready(void);
// /*
//  * -----------------------------------------------------------------------------
//  * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
//  */

// sx128x_hal_status_t sx128x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,
//                                      const uint8_t *data, const uint16_t data_length)
// {
//     sx128x_hal_wakeup(context);

//     // Put NSS low to start spi transaction
//     hal_gpio_set_value(RADIO_NSS, 0);
//     for (uint16_t i = 0; i < command_length; i++)
//     {
//         hal_spi_in_out(RADIO_SPI_ID, command[i]);
//     }
//     for (uint16_t i = 0; i < data_length; i++)
//     {
//         hal_spi_in_out(RADIO_SPI_ID, data[i]);
//     }
//     // Put NSS high as the spi transaction is finished
//     hal_gpio_set_value(RADIO_NSS, 1);

//     // 0x84 - SX128X_SET_SLEEP opcode. In sleep mode the radio dio is struck to 1 => do not test it
//     if (command[0] != 0x84)
//     {
//         sx128x_hal_check_device_ready();
//     }
//     else
//     {
//         radio_mode = RADIO_SLEEP;
//     }

//     return SX128X_HAL_STATUS_OK;
// }

// sx128x_hal_status_t sx128x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,
//                                     uint8_t *data, const uint16_t data_length)
// {
//     sx128x_hal_wakeup(context);

//     // Put NSS low to start spi transaction
//     hal_gpio_set_value(RADIO_NSS, 0);
//     for (uint16_t i = 0; i < command_length; i++)
//     {
//         hal_spi_in_out(RADIO_SPI_ID, command[i]);
//     }
//     for (uint16_t i = 0; i < data_length; i++)
//     {
//         data[i] = hal_spi_in_out(RADIO_SPI_ID, 0);
//     }
//     // Put NSS high as the spi transaction is finished
//     hal_gpio_set_value(RADIO_NSS, 1);

//     return SX128X_HAL_STATUS_OK;
// }

// sx128x_hal_status_t sx128x_hal_reset(const void *context)
// {
//     const sx128x_context_t *sx1280_context = context;
//     hal_gpio_set_value(sx1280_context->nrst_gpio, 0);
//     hal_mcu_wait_us(5000);
//     hal_gpio_set_value(sx1280_context->nrst_gpio, 1);
//     hal_mcu_wait_us(5000);
//     radio_mode = RADIO_AWAKE;
//     return SX128X_HAL_STATUS_OK;
// }

// sx128x_hal_status_t sx128x_hal_wakeup(const void *context)
// {
//     sx128x_hal_check_device_ready();
//     return SX128X_HAL_STATUS_OK;
// }

// /*
//  * -----------------------------------------------------------------------------
//  * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
//  */

// static void sx128x_hal_wait_on_busy(void)
// {
//     while (hal_gpio_get_value(RADIO_BUSY_PIN) == 1)
//     {
//     }
// }

// static void sx128x_hal_check_device_ready(void)
// {
//     if (radio_mode != RADIO_SLEEP)
//     {
//         sx128x_hal_wait_on_busy();
//     }
//     else
//     {
//         // Busy is HIGH in sleep mode, wake-up the device
//         hal_gpio_set_value(RADIO_NSS, 0);
//         sx128x_hal_wait_on_busy();
//         hal_gpio_set_value(RADIO_NSS, 1);
//         radio_mode = RADIO_AWAKE;
//     }
// }
