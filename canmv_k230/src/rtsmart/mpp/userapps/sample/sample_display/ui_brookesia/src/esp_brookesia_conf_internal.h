/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

// *INDENT-OFF*
/* Handle special Kconfig options */
#ifndef ESP_BROOKESIA_KCONFIG_IGNORE
//    #include "sdkconfig.h"
    #ifdef CONFIG_ESP_BROOKESIA_CONF_SKIP
        #define ESP_BROOKESIA_CONF_SKIP
    #endif
#endif

#include "core/esp_brookesia_core_type.h"

/* If "esp_brookesia_conf.h" is not skipped, include it */
#ifndef ESP_BROOKESIA_CONF_SKIP
    #ifdef __has_include                                /* If "esp_brookesia_conf.h" is available from here, try to use it later */
        #if __has_include("esp_brookesia_conf.h")
            #ifndef ESP_BROOKESIA_CONF_INCLUDE_SIMPLE
                #define ESP_BROOKESIA_CONF_INCLUDE_SIMPLE
            #endif
        #elif __has_include("../../esp_brookesia_conf.h")
            #ifndef ESP_BROOKESIA_CONF_INCLUDE_OUTSIDE
                #define ESP_BROOKESIA_CONF_INCLUDE_OUTSIDE
            #endif
        #else
            #define ESP_BROOKESIA_CONF_INCLUDE_INSIDE
        #endif
    #endif

    #ifdef ESP_BROOKESIA_CONF_PATH                             /* If there is a path defined for "esp_brookesia_conf.h" use it */
        #define __TO_STR_AUX(x) #x
        #define __TO_STR(x) __TO_STR_AUX(x)
        #include __TO_STR(ESP_BROOKESIA_CONF_PATH)
        #undef __TO_STR_AUX
        #undef __TO_STR
    #elif defined(ESP_BROOKESIA_CONF_INCLUDE_SIMPLE)           /* Or simply include if "esp_brookesia_conf.h" is available */
        #include "esp_brookesia_conf.h"
    #elif defined(ESP_BROOKESIA_CONF_INCLUDE_OUTSIDE)          /* Or include if "../../ESP_Panel_Conf.h" is available */
        #include "../../esp_brookesia_conf.h"
    #elif defined(ESP_BROOKESIA_CONF_INCLUDE_INSIDE)           /* Or include the default configuration */
        #include "../esp_brookesia_conf.h"
    #endif
#else
    #include "esp_brookesia_conf_kconfig.h"
#endif

// Set default values if not defined, only if the file is not skipped or included from the library
#if !defined(ESP_BROOKESIA_CONF_SKIP) && !defined(ESP_BROOKESIA_CONF_INCLUDE_INSIDE)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// Debug /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ESP_BROOKESIA_CHECK_RESULT_ASSERT
    #define ESP_BROOKESIA_CHECK_RESULT_ASSERT  (0)
#endif

#ifndef ESP_BROOKESIA_LOG_STYLE
    #define ESP_BROOKESIA_LOG_STYLE            (ESP_BROOKESIA_LOG_STYLE_STD)
#endif

#ifndef ESP_BROOKESIA_LOG_LEVEL
    #define ESP_BROOKESIA_LOG_LEVEL            (ESP_BROOKESIA_LOG_LEVEL_INFO)
#endif

/* Enable debug logs for modules */
#if ESP_BROOKESIA_LOG_LEVEL == ESP_BROOKESIA_LOG_LEVEL_DEBUG
    /* Core module */
    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_CORE
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_CORE                     (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_CORE_APP
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_CORE_APP                 (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_CORE_HOME
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_CORE_HOME                (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_CORE_MANAGER
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_CORE_MANAGER             (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_CORE_CORE
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_CORE_CORE                (0)
    #endif

    /* Widgets module */
    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_WIDGETS
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_WIDGETS                  (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_WIDGETS_APP_LAUNCHER
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_WIDGETS_APP_LAUNCHER     (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_WIDGETS_RECENTS_SCREEN
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_WIDGETS_RECENTS_SCREEN   (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_WIDGETS_GESTURE
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_WIDGETS_GESTURE          (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_WIDGETS_NAVIGATION
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_WIDGETS_NAVIGATION       (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_WIDGETS_STATUS_BAR
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_WIDGETS_STATUS_BAR       (0)
    #endif

    /* Phone module */
    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_PHONE
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_PHONE                    (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_PHONE_APP
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_PHONE_APP                (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_PHONE_HOME
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_PHONE_HOME               (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_PHONE_MANAGER
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_PHONE_MANAGER            (0)
    #endif

    #ifndef ESP_BROOKESIA_LOG_ENABLE_DEBUG_PHONE_PHONE
        #define ESP_BROOKESIA_LOG_ENABLE_DEBUG_PHONE_PHONE              (0)
    #endif
#endif /* ESP_BROOKESIA_LOG_LEVEL == ESP_BROOKESIA_LOG_LEVEL_DEBUG */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// Memory /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ESP_BROOKESIA_MEMORY_INCLUDE
    #define ESP_BROOKESIA_MEMORY_INCLUDE   <stdlib.h>
#endif

#ifndef ESP_BROOKESIA_MEMORY_MALLOC
    #define ESP_BROOKESIA_MEMORY_MALLOC    malloc
#endif

#ifndef ESP_BROOKESIA_MEMORY_FREE
    #define ESP_BROOKESIA_MEMORY_FREE      free
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// Squareline ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ESP_BROOKESIA_SQUARELINE_USE_INTERNAL_UI_HELPERS
    #define ESP_BROOKESIA_SQUARELINE_USE_INTERNAL_UI_HELPERS   (0)
#endif

#if ESP_BROOKESIA_SQUARELINE_USE_INTERNAL_UI_HELPERS
// Check if multiple Squareline and LVGL versions are defined
#if defined(ESP_BROOKESIA_SQ1_3_4_LV8_2_0) + defined(ESP_BROOKESIA_SQ1_3_4_LV8_3_3) + defined(ESP_BROOKESIA_SQ1_3_4_LV8_3_4) + \
    defined(ESP_BROOKESIA_SQ1_3_4_LV8_3_6) + defined(ESP_BROOKESIA_SQ1_4_0_LV8_3_6) + defined(ESP_BROOKESIA_SQ1_4_0_LV8_3_11) + \
    defined(ESP_BROOKESIA_SQ1_4_1_LV8_3_6) + defined(ESP_BROOKESIA_SQ1_4_1_LV8_3_11) > 1
    #error "Multiple Squareline and LVGL versions are defined"
#endif
#endif

#endif /* !defined(ESP_BROOKESIA_CONF_SKIP) && !defined(ESP_BROOKESIA_CONF_INCLUDE_INSIDE) */
