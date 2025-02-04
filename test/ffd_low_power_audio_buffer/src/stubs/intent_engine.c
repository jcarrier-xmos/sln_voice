// Copyright (c) 2023 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public License: Version 1

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include "intent_engine.h"

// Defined by test logic.
void verify_intent_engine_sample_push_args(int32_t *buf, size_t frames);

/* Stub for intent_engine_sample_push */
int32_t intent_engine_sample_push(int32_t *buf, size_t frames)
{
    verify_intent_engine_sample_push_args(buf, frames);
    return 0;
}