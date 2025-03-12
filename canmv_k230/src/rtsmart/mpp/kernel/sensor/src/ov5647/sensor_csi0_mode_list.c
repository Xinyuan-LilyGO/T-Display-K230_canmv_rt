// sensor ae info
static k_sensor_ae_info sensor_csi0_ae_info[] = {
    /* 0: 2592x1944@10 */
    {
        .frame_length = 2051,
        .cur_frame_length = 2051,
        .one_line_exp_time = 0.00003250,
        .gain_accuracy = 1024,
        .min_gain = 1.0,
        .max_gain = 8.0,
        .int_time_delay_frame = 2,
        .gain_delay_frame = 2,
        .color_type = SENSOR_COLOR,    //color sensor
        .integration_time_increment = 0.00003250,
        .gain_increment = (1.0f/16.0f),
        .max_long_integraion_line = 2051 - 12,
        .min_long_integraion_line = 2,
        .max_integraion_line = 2051 - 12,
        .min_integraion_line = 2,
        .max_long_integraion_time = 0.00003250 * (2051 - 12),
        .min_long_integraion_time = 0.00003250 * 2,
        .max_integraion_time = 0.00003250 * (2051 - 12),
        .min_integraion_time = 0.00003250 * 2,
        .cur_long_integration_time = 0.0,
        .cur_integration_time = 0.0,
        .cur_long_again = 0.0,
        .cur_long_dgain = 0.0,
        .cur_again = 0.0,
        .cur_dgain = 0.0,
        .a_long_gain = {
            .min = 1.0,
            .max = 8.0,
            .step = (1.0f/16.0f),
        },
        .a_gain = {
            .min = 1.0,
            .max = 8.0,
            .step = (1.0f/16.0f),
        },

        .d_long_gain = {
            .min = 1.0,
            .max = 1.0,
            .step = (1.0f/1024.0f),
        },
        .d_gain = {
            .min = 1.0,
            .max = 1.0,
            .step = (1.0f/1024.0f),
        },
        .cur_fps = 10,
    },
    /* 1: 1080P@30 */
    {
        .frame_length = 1199,
        .cur_frame_length = 1199,
        .one_line_exp_time = 0.000027808,//0.00003025
        .gain_accuracy = 1024,

        .min_gain = 1.0,
        .max_gain = 8.0,//63.9375,

        .int_time_delay_frame = 2,
        .gain_delay_frame = 2,
        .color_type = SENSOR_COLOR,    //color sensor

        .integration_time_increment = 0.000027808,
        .gain_increment = (1.0f/16.0f),

        .max_long_integraion_line = 1199 - 12,
        .min_long_integraion_line = 2,

        .max_integraion_line = 1199 - 12,
        .min_integraion_line = 2,

        .max_long_integraion_time = 0.000027808 * (1199 - 12),
        .min_long_integraion_time = 0.000027808 * 2,

        .max_integraion_time = 0.000027808 * (1199 - 12),
        .min_integraion_time = 0.000027808 * 2,

        .cur_long_integration_time = 0.0,
        .cur_integration_time = 0.0,

        .cur_long_again = 0.0,
        .cur_long_dgain = 0.0,

        .cur_again = 0.0,
        .cur_dgain = 0.0,

        .a_long_gain.min = 1.0,
        .a_long_gain.max = 8.0,
        .a_long_gain.step = (1.0f/16.0f),

        .a_gain.min = 1.0,
        .a_gain.max = 8.0,
        .a_gain.step = (1.0f/16.0f),

        .d_long_gain.max = 1.0,
        .d_long_gain.min = 1.0,
        .d_long_gain.step = (1.0f/1024.0f),

        .d_gain.max = 1.0,
        .d_gain.min = 1.0,
        .d_gain.step = (1.0f/1024.0f),

        .cur_fps = 30,
    },
    /* 2: 1280x960@45 */
    {
        .frame_length = 1639,
        .cur_frame_length = 1639,
        .one_line_exp_time = 0.000020332,
        .gain_accuracy = 1024,

        .min_gain = 1.0,
        .max_gain = 16.0,//63.9375,

        .int_time_delay_frame = 2,
        .gain_delay_frame = 2,
        .color_type = SENSOR_COLOR,    //color sensor

        .integration_time_increment = 0.000020332,
        .gain_increment = (1.0f/16.0f),

        .max_long_integraion_line = 1639 - 12,
        .min_long_integraion_line = 2,

        .max_integraion_line = 1639 - 12,
        .min_integraion_line = 2,

        .max_long_integraion_time = 0.000020332 * (1639 - 12),
        .min_long_integraion_time = 0.000020332 * 2,

        .max_integraion_time = 0.000020332 * (1639 - 12),
        .min_integraion_time = 0.000020332 * 2,

        .cur_long_integration_time = 0.0,
        .cur_integration_time = 0.0,

        .cur_long_again = 0.0,
        .cur_long_dgain = 0.0,

        .cur_again = 0.0,
        .cur_dgain = 0.0,

        .a_long_gain.min = 1.0,
        .a_long_gain.max = 8.0,
        .a_long_gain.step = (1.0f/16.0f),

        .a_gain.min = 1.0,
        .a_gain.max = 16.0,
        .a_gain.step = (1.0f/16.0f),

        .d_long_gain.max = 1.0,
        .d_long_gain.min = 1.0,
        .d_long_gain.step = (1.0f/1024.0f),

        .d_gain.max = 1.0,
        .d_gain.min = 1.0,
        .d_gain.step = (1.0f/1024.0f),

        .cur_fps = 45,
    },
    /* 3: 720P@60 */
    {
        .frame_length = 1701, //30fps, 60fps: 851,
        .cur_frame_length = 1701,
        .one_line_exp_time = 0.000019593,
        .gain_accuracy = 1024,

        .min_gain = 1.0,
        .max_gain = 24.0,//63.9375,

        .int_time_delay_frame = 2,
        .gain_delay_frame = 2,
        .color_type = SENSOR_COLOR,    //color sensor

        .integration_time_increment = 0.000019593,
        .gain_increment = (1.0f/16.0f),

        .max_long_integraion_line = 1701 - 12,
        .min_long_integraion_line = 2,

        .max_integraion_line = 1701 - 12,
        .min_integraion_line = 2,

        .max_long_integraion_time = 0.000019593 * (1701 - 12),
        .min_long_integraion_time = 0.000019593 * 2,

        .max_integraion_time = 0.000019593 * (1701 - 12),
        .min_integraion_time = 0.000019593 * 2,

        .cur_long_integration_time = 0.0,
        .cur_integration_time = 0.0,

        .cur_long_again = 0.0,
        .cur_long_dgain = 0.0,

        .cur_again = 0.0,
        .cur_dgain = 0.0,

        .a_long_gain.min = 1.0,
        .a_long_gain.max = 8.0,
        .a_long_gain.step = (1.0f/16.0f),

        .a_gain.min = 1.0,
        .a_gain.max = 24.0,//63.9375,
        .a_gain.step = (1.0f/16.0f),

        .d_long_gain.max = 1.0,
        .d_long_gain.min = 1.0,
        .d_long_gain.step = (1.0f/1024.0f),

        .d_gain.max = 1.0,
        .d_gain.min = 1.0,
        .d_gain.step = (1.0f/1024.0f),

        .cur_fps = 60,
    },
    /* 4: VGA@90 */
    {
        .frame_length = 1721,//30fps, 90fps:573,
        .cur_frame_length = 1721,
        .one_line_exp_time = 0.000019373,
        .gain_accuracy = 1024,

        .min_gain = 1.0,
        .max_gain = 24.0,//63.9375,

        .int_time_delay_frame = 2,
        .gain_delay_frame = 2,
        .color_type = SENSOR_COLOR,    //color sensor

        .integration_time_increment = 0.000019373,
        .gain_increment = (1.0f/16.0f),

        .max_long_integraion_line = 1721 - 12,
        .min_long_integraion_line = 2,

        .max_integraion_line = 1721 - 12,
        .min_integraion_line = 2,

        .max_long_integraion_time = 0.000019373 * (1721 - 12),
        .min_long_integraion_time = 0.000019373 * 2,

        .max_integraion_time = 0.000019373 * (1721 - 12),
        .min_integraion_time = 0.000019373 * 2,

        .cur_long_integration_time = 0.0,
        .cur_integration_time = 0.0,

        .cur_long_again = 0.0,
        .cur_long_dgain = 0.0,

        .cur_again = 0.0,
        .cur_dgain = 0.0,

        .a_long_gain.min = 1.0,
        .a_long_gain.max = 8.0,
        .a_long_gain.step = (1.0f/16.0f),

        .a_gain.min = 1.0,
        .a_gain.max = 24.0,//63.9375,
        .a_gain.step = (1.0f/16.0f),

        .d_long_gain.max = 1.0,
        .d_long_gain.min = 1.0,
        .d_long_gain.step = (1.0f/1024.0f),

        .d_gain.max = 1.0,
        .d_gain.min = 1.0,
        .d_gain.step = (1.0f/1024.0f),

        .cur_fps = 90,
    }
};

static const k_sensor_mode sensor_csi0_mode_list[] = {
    {
        .index = 0,
        .sensor_type = OV5647_MIPI_CSI0_2592x1944_10FPS_10BIT_LINEAR,
        .size = {
            .bounds_width = 2592,
            .bounds_height = 1944,
            .top = 0,
            .left = 0,
            .width = 2592,
            .height = 1944,
        },
        .fps = 10000,
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .bayer_pattern = BAYER_PAT_GBRG,
        .mipi_info = {
            .csi_id = 2,
            .mipi_lanes = 2,
            .data_type = 0x2B,
        },
#if defined (CONFIG_MPP_SENSOR_OV5647_ON_CSI0_USE_CHIP_CLK)
        .reg_list = ov5647_2592x1944_10bpp,
        .mclk_setting = {
            {
                .mclk_setting_en = K_TRUE,
                .setting.id = CONFIG_MPP_CSI_DEV0_MCLK_NUM,
                .setting.mclk_sel = SENSOR_PLL0_CLK_DIV4,
                .setting.mclk_div = 16,
            },
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi0_ae_info[0],
#else
        .reg_list = ov5647_2592x1944_10bpp,
        .mclk_setting = {{K_FALSE}, {K_FALSE}, {K_FALSE}},
        .sensor_ae_info = &sensor_csi0_ae_info[0],
#endif
    },
    {
        .index = 1,
        .sensor_type = OV5647_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR,
        .size = {
            .bounds_width = 1920,
            .bounds_height = 1080,
            .top = 0,
            .left = 0,
            .width = 1920,
            .height = 1080,
        },
        .fps = 30000,
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .bayer_pattern = BAYER_PAT_GBRG,
        .mipi_info = {
            .csi_id = 2,
            .mipi_lanes = 2,
            .data_type = 0x2B,
        },
#if defined (CONFIG_MPP_SENSOR_OV5647_ON_CSI0_USE_CHIP_CLK)
        .reg_list = ov5647_mipi2lane_1080p_30fps_linear,
        .mclk_setting = {
            {
                .mclk_setting_en = K_TRUE,
                .setting.id = CONFIG_MPP_CSI_DEV0_MCLK_NUM,
                .setting.mclk_sel = SENSOR_PLL0_CLK_DIV4,
                .setting.mclk_div = 16,
            },
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi0_ae_info[1],
#else
        .reg_list = ov5647_mipi2lane_1080p_30fps_linear,
        .mclk_setting = {{K_FALSE}, {K_FALSE}, {K_FALSE}},
        .sensor_ae_info = &sensor_csi0_ae_info[1],
#endif
    },
    {
        .index = 2,
        .sensor_type = OV5647_MIPI_CSI0_1280X960_45FPS_10BIT_LINEAR,
        .size = {
            .bounds_width = 1280,
            .bounds_height = 960,
            .top = 0,
            .left = 0,
            .width = 1280,
            .height = 960,
        },
        .fps = 45000,
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .bayer_pattern = BAYER_PAT_GBRG,
        .mipi_info = {
            .csi_id = 2,
            .mipi_lanes = 2,
            .data_type = 0x2B,
        },
#if defined (CONFIG_MPP_SENSOR_OV5647_ON_CSI0_USE_CHIP_CLK)
        .reg_list = ov5647_1280x960p45_10bpp,
        .mclk_setting = {
            {
                .mclk_setting_en = K_TRUE,
                .setting.id = CONFIG_MPP_CSI_DEV0_MCLK_NUM,
                .setting.mclk_sel = SENSOR_PLL0_CLK_DIV4,
                .setting.mclk_div = 16,
            },
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi0_ae_info[2],
#else
        .reg_list = ov5647_1280x960p45_10bpp,
        .mclk_setting = {{K_FALSE}, {K_FALSE}, {K_FALSE}},
        .sensor_ae_info = &sensor_csi0_ae_info[2],
#endif
    },
    {
        .index = 3,
        .sensor_type = OV5647_MIPI_CSI0_1280X720_60FPS_10BIT_LINEAR,
        .size = {
            .bounds_width = 1280,
            .bounds_height = 720,
            .top = 0,
            .left = 0,
            .width = 1280,
            .height = 720,
        },
        .fps = 60000,
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .bayer_pattern = BAYER_PAT_GBRG,
        .mipi_info = {
            .csi_id = 2,
            .mipi_lanes = 2,
            .data_type = 0x2B,
        },
#if defined (CONFIG_MPP_SENSOR_OV5647_ON_CSI0_USE_CHIP_CLK)
        .reg_list = mode_1280x720_60fps,
        .mclk_setting = {
            {
                .mclk_setting_en = K_TRUE,
                .setting.id = CONFIG_MPP_CSI_DEV0_MCLK_NUM,
                .setting.mclk_sel = SENSOR_PLL0_CLK_DIV4,
                .setting.mclk_div = 16,
            },
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi0_ae_info[3],
#else
        .reg_list = mode_1280x720_60fps,
        .mclk_setting = {{K_FALSE}, {K_FALSE}, {K_FALSE}},
        .sensor_ae_info = &sensor_csi0_ae_info[3],
#endif
    },
    {
        .index = 4,
        .sensor_type = OV5647_MIPI_CSI0_640x480_90FPS_10BIT_LINEAR,
        .size = {
            .bounds_width = 640,
            .bounds_height = 480,
            .top = 0,
            .left = 0,
            .width = 640,
            .height = 480,
        },
        .fps = 90000,
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 10,
        .bayer_pattern = BAYER_PAT_GBRG,
        .mipi_info = {
            .csi_id = 2,
            .mipi_lanes = 2,
            .data_type = 0x2B,
        },
#if defined (CONFIG_MPP_SENSOR_OV5647_ON_CSI0_USE_CHIP_CLK)
        .reg_list = ov5647_640x480_10bpp,
        .mclk_setting = {
            {
                .mclk_setting_en = K_TRUE,
                .setting.id = CONFIG_MPP_CSI_DEV0_MCLK_NUM,
                .setting.mclk_sel = SENSOR_PLL0_CLK_DIV4,
                .setting.mclk_div = 16,
            },
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi0_ae_info[4],
#else
        .reg_list = ov5647_640x480_10bpp,
        .mclk_setting = {{K_FALSE}, {K_FALSE}, {K_FALSE}},
        .sensor_ae_info = &sensor_csi0_ae_info[4],
#endif
    },
};

#if defined (CONFIG_MPP_SENSOR_OV5647_ON_CSI0_USE_CHIP_CLK)
_Static_assert(CONFIG_MPP_CSI_DEV0_MCLK_NUM >= 1 && (CONFIG_MPP_CSI_DEV0_MCLK_NUM <= 3), "Invalid CONFIG_MPP_CSI_DEV0_MCLK_NUM");
#endif
