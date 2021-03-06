//  Based on https://github.com/apache/mynewt-core/blob/master/hw/drivers/lora/src/utilities.c
/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>

/// Return current time in microseconds
uint32_t timer_get_current_time(void)
{
    //  Convert ticks to milliseconds then microseconds
    return xTaskGetTickCount() * portTICK_PERIOD_MS * 1000;
}

/// Return elased time in microseconds
uint32_t timer_get_elapsed_time(uint32_t saved_time)
{
    return timer_get_current_time() - saved_time;
}
