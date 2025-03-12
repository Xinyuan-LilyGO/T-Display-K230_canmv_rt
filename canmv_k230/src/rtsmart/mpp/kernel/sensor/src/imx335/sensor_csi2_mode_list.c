static k_sensor_ae_info sensor_csi2_ae_info[] = {
    {
        /* 30 fps */
        .frame_length = IMX335_VMAX_LINEAR,
        .cur_frame_length = IMX335_VMAX_LINEAR,
        .one_line_exp_time = 0.000007407,
        .gain_accuracy = 1024,

        .min_gain = 1.0,
        .max_gain = 50.0,

        .int_time_delay_frame = 1,
        .gain_delay_frame = 1,
        .color_type = SENSOR_COLOR,

        .integration_time_increment = 0.000007407,
        .gain_increment = IMX335_AGAIN_STEP,

        .max_long_integraion_line = IMX335_VMAX_LINEAR - 9,
        .min_long_integraion_line = 1,

        .max_integraion_line = IMX335_VMAX_LINEAR - 9,
        .min_integraion_line = 1,

        .max_vs_integraion_line = IMX335_VMAX_LINEAR - 9,
        .min_vs_integraion_line = 1,

        .max_long_integraion_time = 0.000007407 * (IMX335_VMAX_LINEAR - 9),
        .min_long_integraion_time = 0.000007407 * 1,

        .max_integraion_time = 0.000007407 * (IMX335_VMAX_LINEAR - 9),
        .min_integraion_time = 0.000007407 * 1,

        .max_vs_integraion_time = 0.000007407 * (IMX335_VMAX_LINEAR - 9),
        .min_vs_integraion_time = 0.000007407 * 1,

        .cur_long_integration_time = 0.0,
        .cur_integration_time = 0.0,
        .cur_vs_integration_time = 0.0,

        .cur_long_again = 0.0,
        .cur_long_dgain = 0.0,

        .cur_again = 0.0,
        .cur_dgain = 0.0,

        .cur_vs_again = 0.0,
        .cur_vs_dgain = 0.0,

        .a_long_gain.min = 1.0,
        .a_long_gain.max = 100.0,
        .a_long_gain.step = (1.0f / 256.0f),

        .a_gain.min = 1.0,
        .a_gain.max = 100.0,
        .a_gain.step = (1.0f / 256.0f),

        .a_vs_gain.min = 1.0,
        .a_vs_gain.max = 100.0,
        .a_vs_gain.step = (1.0f / 256.0f),

        .d_long_gain.max = 1.0,
        .d_long_gain.min = 1.0,
        .d_long_gain.step = (1.0f / 1024.0f),

        .d_gain.max = 1.0,
        .d_gain.min = 1.0,
        .d_gain.step = (1.0f / 1024.0f),

        .d_vs_gain.max = 1.0,
        .d_vs_gain.min = 1.0,
        .d_vs_gain.step = (1.0f / 1024.0f),

        .cur_fps = 30,
    },
};

static const k_sensor_mode sensor_csi2_mode_list[] = {
    {
        .index = 0,
        .sensor_type = IMX335_MIPI_CSI2_2LANE_1920X1080_30FPS_12BIT_LINEAR,
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
        .bit_width = 12,
        .bayer_pattern = BAYER_PAT_RGGB,
        .mipi_info = {
            .csi_id = 0,
            .mipi_lanes = 2,
            .data_type = 0x2C,
        },
#if defined (CONFIG_MPP_SENSOR_IMX335_ON_CSI2_USE_CHIP_CLK)
        .reg_list = imx335_mipi_2lane_raw12_1920x1080_30fps_mclk_74_25_regs,
        .mclk_setting = {
            {
                .mclk_setting_en = K_TRUE,
                .setting.id = CONFIG_MPP_CSI_DEV2_MCLK_NUM,
                .setting.mclk_sel = SENSOR_PLL1_CLK_DIV4,
                .setting.mclk_div = 8,
            },
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi2_ae_info[0],
#else
        .reg_list = imx335_mipi_2lane_raw12_1920x1080_30fps_mclk_24m_regs,
        .mclk_setting = {
            {K_FALSE},
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi2_ae_info[0],
#endif
    },
    {
        .index = 1,
        .sensor_type = IMX335_MIPI_CSI2_2LANE_2592X1944_30FPS_12BIT_LINEAR,
        .size = {
            .bounds_width = 2592,
            .bounds_height = 1944,
            .top = 0,
            .left = 0,
            .width = 2592,
            .height = 1944,
        },
        .fps = 30000,
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 12,
        .bayer_pattern = BAYER_PAT_RGGB,
        .mipi_info = {
            .csi_id = 0,
            .mipi_lanes = 2,
            .data_type = 0x2C,
        },
#if defined (CONFIG_MPP_SENSOR_IMX335_ON_CSI2_USE_CHIP_CLK)
        .reg_list = imx335_mipi_2lane_raw12_2592x1944_30fps_mclk_74_25_regs,
        .mclk_setting = {
            {
                .mclk_setting_en = K_TRUE,
                .setting.id = CONFIG_MPP_CSI_DEV2_MCLK_NUM,
                .setting.mclk_sel = SENSOR_PLL1_CLK_DIV4,
                .setting.mclk_div = 8,
            },
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi2_ae_info[0],
#else
        .reg_list = imx335_mipi_2lane_raw12_2592x1944_30fps_mclk_24m_regs,
        .mclk_setting = {
            {K_FALSE},
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi2_ae_info[0],
#endif
    },
#if defined (CONFIG_MPP_SENSOR_IMX335_ENABLE_4LANE_CONFIGURE)
    {
        .index = 2,
        .sensor_type = IMX335_MIPI_CSI2_4LANE_2592X1944_30FPS_12BIT_LINEAR,
        .size = {
            .bounds_width = 2592,
            .bounds_height = 1944,
            .top = 0,
            .left = 0,
            .width = 2592,
            .height = 1944,
        },
        .fps = 30000,
        .hdr_mode = SENSOR_MODE_LINEAR,
        .bit_width = 12,
        .bayer_pattern = BAYER_PAT_RGGB,
        .mipi_info = {
            .csi_id = 0,
            .mipi_lanes = 4,
            .data_type = 0x2C,
        },
#if defined (CONFIG_MPP_SENSOR_IMX335_ON_CSI2_USE_CHIP_CLK)
        .reg_list = imx335_mipi_4lane_raw12_2592x1944_30fps_mclk_74_25_regs,
        .mclk_setting = {
            {
                .mclk_setting_en = K_TRUE,
                .setting.id = CONFIG_MPP_CSI_DEV2_MCLK_NUM,
                .setting.mclk_sel = SENSOR_PLL1_CLK_DIV4,
                .setting.mclk_div = 8,
            },
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi2_ae_info[0],
#else
        .reg_list = imx335_mipi_4lane_raw12_2592x1944_30fps_mclk_24m_regs,
        .mclk_setting = {
            {K_FALSE},
            {K_FALSE},
            {K_FALSE},
        },
        .sensor_ae_info = &sensor_csi2_ae_info[0],
#endif
    },
#endif // CONFIG_MPP_SENSOR_IMX335_ENABLE_4LANE_CONFIGURE
};

#if defined (CONFIG_MPP_SENSOR_IMX335_ON_CSI2_USE_CHIP_CLK)
_Static_assert(CONFIG_MPP_CSI_DEV2_MCLK_NUM >= 1 && (CONFIG_MPP_CSI_DEV2_MCLK_NUM <= 3), "Invalid CONFIG_MPP_CSI_DEV2_MCLK_NUM");
#endif
