// Copyright 2021-2022 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

/* System headers */
#include <xcore/assert.h>
#include <xs3a_defines.h>

/* App headers */
#include "app_conf.h"
//#include "platform_conf.h"
#include "platform/driver_instances.h"
#include "xcore_device_memory.h"

size_t model_file_init()
{
    return 1;
}

size_t model_data_load(void *dest, const void *src, size_t size)
{
    xassert(IS_SWMEM(src));

    unsigned int offset = (unsigned int)src - XS1_SWMEM_BASE;
    rtos_qspi_flash_read(qspi_flash_ctx, dest, QSPI_FLASH_MODEL_START_ADDRESS + offset, size);

    return size;
}
