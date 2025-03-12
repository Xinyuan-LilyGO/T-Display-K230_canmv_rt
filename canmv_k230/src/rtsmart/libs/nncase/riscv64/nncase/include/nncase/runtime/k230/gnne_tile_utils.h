/* Copyright 2019-2021 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include "nncase/runtime/runtime_tensor.h"
#include <nncase/runtime/datatypes.h>

namespace nncase
{
namespace runtime
{
    namespace k230
    {
        struct segment
        {
            int32_t start;
            int32_t end;
            int32_t length;
            padding pad;

            segment()
            {
                start = end = length = 0;
                pad = { 0, 0 };
            }

            segment(int32_t start_, int32_t end_, int32_t length_)
                : start(start_), end(end_), length(length_)
            {
                pad = { 0, 0 };
            }

            segment(int32_t start_, int32_t end_, int32_t length_, padding pad_)
                : start(start_), end(end_), length(length_), pad(pad_) { }

            bool operator==(const segment &other) const
            {
                return start == other.start && end == other.end && length == other.length;
            }

            bool operator!=(const segment &other) const
            {
                return !(*this == other);
            }

            segment operator+(const segment &other) const
            {
                auto min_start = std::min(start, other.start);
                auto max_end = std::max(end, other.end);
                return { min_start, max_end, max_end - min_start };
            }

            segment operator/(const int32_t &scale) const
            {
                return segment { start / scale, end / scale, length / scale, pad };
            }

            segment operator*(const int32_t &scale) const
            {
                return segment { start * scale, end * scale, length * scale, pad };
            }

            size_t hasher() const
            {
                return std::hash<int32_t>()(start) ^ std::hash<int32_t>()(end) ^ std::hash<int32_t>()(length);
            }
        };

        struct tensor4d_segment
        {
            segment dim_0;
            segment dim_1;
            segment dim_2;
            segment dim_3;
            padding p_h;
            padding p_w;

            tensor4d_segment()
            {
                dim_0 = dim_1 = dim_2 = dim_3 = { 0, 0, 0 };
                p_h = p_w = { 0, 0 };
            }

            tensor4d_segment(segment dim_0_, segment dim_1_, segment dim_2_, segment dim_3_)
                : dim_0(dim_0_), dim_1(dim_1_), dim_2(dim_2_), dim_3(dim_3_)
            {
                p_h = p_w = { 0, 0 };
            }

            tensor4d_segment(segment dim_0_, segment dim_1_, segment dim_2_, segment dim_3_, padding p_h_, padding p_w_)
                : dim_0(dim_0_), dim_1(dim_1_), dim_2(dim_2_), dim_3(dim_3_), p_h(p_h_), p_w(p_w_) { }

            int32_t get_shape_size()
            {
                return dim_0.length * dim_1.length * dim_2.length * dim_3.length;
            }

            bool operator==(const tensor4d_segment &other) const
            {
                return dim_0 == other.dim_0
                    && dim_1 == other.dim_1
                    && dim_2 == other.dim_2
                    && dim_3 == other.dim_3;
            }

            tensor4d_segment operator+(const tensor4d_segment &other) const
            {
                return { dim_0 + other.dim_0, dim_1 + other.dim_1, dim_2 + other.dim_2, dim_3 + other.dim_3 };
            }

            size_t hasher() const
            {
                return dim_0.hasher() ^ dim_1.hasher() ^ dim_2.hasher() ^ dim_3.hasher();
            }

            dims_t to_gnne_shape()
            {
                return dims_t { static_cast<unsigned long>(dim_0.length), static_cast<unsigned long>(dim_1.length), static_cast<unsigned long>(dim_2.length), static_cast<unsigned long>(dim_3.length) };
            }
        };

        class segment_hash
        {
        public:
            size_t operator()(const segment &key) const
            {
                return key.hasher();
            }
        };

        class tensor4d_segment_hash
        {
        public:
            size_t operator()(const tensor4d_segment &key) const
            {
                return key.hasher();
            }
        };

        struct ai2d_sram_t
        {
            int32_t sram_len = 256;
            int32_t sram_size = sram_len * sram_len;
        };

        enum class ai2d_format
        {
            YUV420_NV12 = 0,
            YUV420_NV21 = 1,
            YUV420_I420 = 2,
            NCHW_FMT = 3,
            RGB_packed = 4,
            RAW16 = 5,
        };

        enum class ai2d_pad_mode
        {
            constant = 0,
            copy = 1,
            mirror = 2,
        };

        enum class ai2d_data_loc
        {
            glb = 0,
            ddr = 1,
        };

        enum class ai2d_interp_method
        {
            tf_nearest = 0,
            tf_bilinear = 1,
            cv2_nearest = 2,
            cv2_bilinear = 3,
        };

        enum class ai2d_interp_mode
        {
            none = 0,
            align_corner = 1,
            half_pixel = 2,
        };

        union FP32
        {
            unsigned int u;
            float f;
        };

        struct ai2d_shift_param_t
        {
            bool shift_flag = false;
            int32_t shift_val = 0;
        };

        struct ai2d_resize_param_t
        {
            bool resize_flag = false;
            ai2d_interp_method interp_method = ai2d_interp_method::tf_bilinear;
            ai2d_interp_mode interp_mode = ai2d_interp_mode::none;
        };

        struct ai2d_crop_param_t
        {
            bool crop_flag = false;
            int32_t start_x = 0;
            int32_t start_y = 0;
            int32_t width = 0;
            int32_t height = 0;
        };

        struct ai2d_pad_param_t
        {
            bool pad_flag = false;
            paddings_t paddings;
            ai2d_pad_mode pad_mode = ai2d_pad_mode::constant;
            std::vector<int32_t> pad_val; // by channel
        };

        struct ai2d_affine_param_t
        {
            bool affine_flag = false;
            ai2d_interp_method interp_method = ai2d_interp_method::cv2_bilinear;
            uint32_t cord_round = 0;
            uint32_t bound_ind = 0;
            int32_t bound_val = 0;
            uint32_t bound_smooth = 0;
            std::vector<float> M;
        };

        struct ai2d_datatype_t
        {
            ai2d_format src_format;
            ai2d_format dst_format;
            typecode_t src_type;
            typecode_t dst_type;
            ai2d_data_loc src_loc = ai2d_data_loc::ddr;
            ai2d_data_loc dst_loc = ai2d_data_loc::ddr;
        };

        struct ai2d_config
        {
            // 0x00
            uint32_t src_ch0_ptr : 32;
            uint32_t src_ch1_ptr : 32;
            uint32_t src_ch2_ptr : 32;
            uint32_t src_ch3_ptr : 32;
            // 0x10
            uint32_t dst_ch0_ptr : 32;
            uint32_t dst_ch1_ptr : 32;
            uint32_t dst_ch2_ptr : 32;
            uint32_t dst_ch3_ptr : 32;
            // 0x20
            uint32_t src_ch0_width_layout : 16;
            uint32_t src_ch1_width_layout : 16;
            uint32_t src_ch2_width_layout : 16;
            uint32_t src_ch3_width_layout : 16;
            uint32_t dst_ch0_width_layout : 16;
            uint32_t dst_ch1_width_layout : 16;
            uint32_t dst_ch2_width_layout : 16;
            uint32_t dst_ch3_width_layout : 16;
            // 0x30
            uint32_t M0 : 32;
            uint32_t M1 : 32;
            uint32_t M3 : 32;
            uint32_t M4 : 32;
            // 0x40
            uint32_t reserved0 : 32;
            uint32_t reserved1 : 32;
            uint32_t channel : 3;
            uint32_t dst_channel : 3;
            uint32_t cord_round : 2;
            uint32_t interpolation : 2;
            uint32_t pad_mod : 2;
            uint32_t shift : 8;
            uint32_t bound_ind : 4;
            uint32_t src_format : 4;
            uint32_t dst_format : 4;
            uint32_t bound_val : 16;
            uint32_t bound_smooth : 1;
            uint32_t reserved2 : 15;
            // 0x50
            uint32_t yuv2rgb_coef0 : 12;
            uint32_t yuv2rgb_coef1 : 12;
            uint32_t reserved3 : 8;
            uint32_t yuv2rgb_coef2 : 12;
            uint32_t yuv2rgb_coef3 : 12;
            uint32_t reserved4 : 8;
            uint32_t yuv2rgb_coef4 : 12;
            uint32_t yuv2rgb_coef5 : 12;
            uint32_t reserved5 : 8;
            uint32_t yuv2rgb_coef6 : 12;
            uint32_t yuv2rgb_coef7 : 12;
            uint32_t reserved6 : 8;
            // 0x60
            uint32_t yuv2rgb_coef10 : 12;
            uint32_t yuv2rgb_coef11 : 12;
            uint32_t const_pad_ch0 : 8;
            uint32_t const_pad_ch1 : 8;
            uint32_t const_pad_ch2 : 8;
            uint32_t const_pad_ch3 : 8;
            uint32_t src_ind : 1;
            uint32_t dst_ind : 1;
            uint32_t cmd_id : 1;
            uint32_t sign : 1;
            uint32_t reserved7 : 4;
            uint32_t yuv2rgb_coef8 : 12;
            uint32_t yuv2rgb_coef9 : 12;
            uint32_t reserved8 : 8;
            uint32_t reserved9 : 32;
            // 0x70
            uint32_t pad_t : 16;
            uint32_t pad_b : 16;
            uint32_t pad_l : 16;
            uint32_t pad_r : 16;
            uint32_t src_width_shape : 16;
            uint32_t src_height_shape : 16;
            uint32_t dst_width_shape : 16;
            uint32_t dst_height_shape : 14;
            uint32_t intr_mask : 1;
            uint32_t csc_en : 1;
            // 0x80
            uint32_t M2 : 32;
            uint32_t M5 : 32;
            uint32_t src_x : 16;
            uint32_t src_y : 16;
            uint32_t dst_x : 16;
            uint32_t dst_y : 13;
            uint32_t reserved10 : 2;
            uint32_t ai2d_calc_enable : 1;

            ai2d_config()
            {
                src_ch0_ptr = 0;
                src_ch1_ptr = 0;
                src_ch2_ptr = 0;
                src_ch3_ptr = 0;
                dst_ch0_ptr = 0;
                dst_ch1_ptr = 0;
                dst_ch2_ptr = 0;
                dst_ch3_ptr = 0;

                src_ch0_width_layout = 0;
                src_ch1_width_layout = 0;
                src_ch2_width_layout = 0;
                src_ch3_width_layout = 0;
                dst_ch0_width_layout = 0;
                dst_ch1_width_layout = 0;
                dst_ch2_width_layout = 0;
                dst_ch3_width_layout = 0;

                FP32 m;
                m.f = 1024;
                M0 = m.u;
                M1 = 0;
                M3 = 0;
                M4 = m.u;

                reserved0 = 0;
                reserved1 = 0;
                channel = 0;
                dst_channel = 0;
                cord_round = 0;
                interpolation = 0;
                pad_mod = 0;
                shift = 0;
                bound_ind = 0;
                src_format = 0;
                dst_format = 0;
                bound_val = 0;
                bound_smooth = 0;
                reserved2 = 0;

                yuv2rgb_coef0 = 256;
                yuv2rgb_coef1 = 0;
                reserved3 = 0;
                yuv2rgb_coef2 = 292;
                yuv2rgb_coef3 = (1 << 12) - 146;
                reserved4 = 0;
                yuv2rgb_coef4 = 256;
                yuv2rgb_coef5 = (1 << 12) - 101;
                reserved5 = 0;
                yuv2rgb_coef6 = (1 << 12) - 149;
                yuv2rgb_coef7 = 125;
                reserved6 = 0;

                yuv2rgb_coef10 = 0;
                yuv2rgb_coef11 = (1 << 12) - 260;
                const_pad_ch0 = 0;
                const_pad_ch1 = 0;
                const_pad_ch2 = 0;
                const_pad_ch3 = 0;
                src_ind = 0;
                dst_ind = 0;
                cmd_id = 0;
                sign = 0;
                reserved7 = 0;
                yuv2rgb_coef8 = 256;
                yuv2rgb_coef9 = 520;
                reserved8 = 0;
                reserved9 = 0;

                pad_t = 0;
                pad_b = 0;
                pad_l = 0;
                pad_r = 0;
                src_width_shape = 0;
                src_height_shape = 0;
                dst_width_shape = 0;
                dst_height_shape = 0;
                intr_mask = 1;
                csc_en = 0;

                M2 = 0;
                M5 = 0;
                src_x = 0;
                src_y = 0;
                dst_x = 0;
                dst_y = 0;
                reserved10 = 0;
                ai2d_calc_enable = 1;
            }

            uint32_t get_addr_value(uint32_t idx)
            {
                switch (idx)
                {
                case 0:
                {
                    return src_ch0_ptr;
                }
                case 1:
                {
                    return src_ch1_ptr;
                }
                case 2:
                {
                    return src_ch2_ptr;
                }
                case 3:
                {
                    return src_ch3_ptr;
                }
                case 4:
                {
                    return dst_ch0_ptr;
                }
                case 5:
                {
                    return dst_ch1_ptr;
                }
                case 6:
                {
                    return dst_ch2_ptr;
                }
                case 7:
                {
                    return dst_ch3_ptr;
                }
                case 8:
                {
                    return (src_ch1_width_layout << 16) + src_ch0_width_layout;
                }
                case 9:
                {
                    return (src_ch3_width_layout << 16) + src_ch2_width_layout;
                }
                case 10:
                {
                    return (dst_ch1_width_layout << 16) + dst_ch0_width_layout;
                }
                case 11:
                {
                    return (dst_ch3_width_layout << 16) + dst_ch2_width_layout;
                }

                case 12:
                {
                    return M0;
                }
                case 13:
                {
                    return M1;
                }
                case 14:
                {
                    return M3;
                }
                case 15:
                {
                    return M4;
                }
                case 16:
                {
                    return reserved0;
                }
                case 17:
                {
                    return reserved1;
                }
                case 18:
                {
                    return (dst_format << 28)
                        | (src_format << 24)
                        | (bound_ind << 20)
                        | (shift << 12)
                        | (pad_mod << 10)
                        | (interpolation << 8)
                        | (cord_round << 6)
                        | (dst_channel << 3)
                        | (channel);
                }
                case 19:
                {
                    return (reserved2 << 17)
                        | (bound_smooth << 16)
                        | (bound_val);
                }
                case 20:
                {
                    return (reserved3 << 24)
                        | (yuv2rgb_coef1 << 12)
                        | (yuv2rgb_coef0);
                }
                case 21:
                {
                    return (reserved4 << 24)
                        | (yuv2rgb_coef3 << 12)
                        | (yuv2rgb_coef2);
                }
                case 22:
                {
                    return (reserved5 << 24)
                        | (yuv2rgb_coef5 << 12)
                        | (yuv2rgb_coef4);
                }
                case 23:
                {
                    return (reserved6 << 24)
                        | (yuv2rgb_coef7 << 12)
                        | (yuv2rgb_coef6);
                }
                case 24:
                {
                    return (const_pad_ch0 << 24)
                        | (yuv2rgb_coef11 << 12)
                        | (yuv2rgb_coef10);
                }
                case 25:
                {
                    return (reserved7 << 28)
                        | (sign << 27)
                        | (cmd_id << 26)
                        | (dst_ind << 25)
                        | (src_ind << 24)
                        | (const_pad_ch3 << 16)
                        | (const_pad_ch2 << 8)
                        | (const_pad_ch1);
                }
                case 26:
                {
                    return (reserved8 << 24)
                        | (yuv2rgb_coef9 << 12)
                        | (yuv2rgb_coef8);
                }
                case 27:
                {
                    return reserved9;
                }
                case 28:
                {
                    return (pad_b << 16) | pad_t;
                }
                case 29:
                {
                    return (pad_r << 16) | pad_l;
                }
                case 30:
                {
                    return (src_height_shape << 16) | src_width_shape;
                }
                case 31:
                {
                    return (csc_en << 31)
                        | (intr_mask << 30)
                        | (dst_height_shape << 16)
                        | (dst_width_shape);
                }
                case 32:
                {
                    return M2;
                }
                case 33:
                {
                    return M5;
                }
                case 34:
                {
                    return (src_y << 16) | src_x;
                }
                case 35:
                {
                    return (ai2d_calc_enable << 31)
                        | (reserved10 << 29)
                        | (dst_y << 16)
                        | (dst_x);
                }
                default:
                    return 0;
                }
            }

            std::string to_string(uint32_t idx)
            {
                switch (idx)
                {
                case 0:
                {
                    return "src_ch0_ptr: " + std::to_string(src_ch0_ptr);
                }
                case 1:
                {
                    return "src_ch1_ptr: " + std::to_string(src_ch1_ptr);
                }
                case 2:
                {
                    return "src_ch2_ptr: " + std::to_string(src_ch2_ptr);
                }
                case 3:
                {
                    return "src_ch3_ptr: " + std::to_string(src_ch3_ptr);
                }
                case 4:
                {
                    return "dst_ch0_ptr: " + std::to_string(dst_ch0_ptr);
                }
                case 5:
                {
                    return "dst_ch1_ptr: " + std::to_string(dst_ch1_ptr);
                }
                case 6:
                {
                    return "dst_ch2_ptr: " + std::to_string(dst_ch2_ptr);
                }
                case 7:
                {
                    return "dst_ch3_ptr: " + std::to_string(dst_ch3_ptr);
                }
                case 8:
                {
                    return "src_ch1_width_layout: " + std::to_string(src_ch1_width_layout)
                        + "\n"
                        + "src_ch0_width_layout: " + std::to_string(src_ch0_width_layout);
                }
                case 9:
                {
                    return "src_ch3_width_layout: " + std::to_string(src_ch3_width_layout)
                        + "\n"
                        + "src_ch2_width_layout: " + std::to_string(src_ch2_width_layout);
                }
                case 10:
                {
                    return "dst_ch1_width_layout: " + std::to_string(dst_ch1_width_layout)
                        + "\n"
                        + "dst_ch0_width_layout: " + std::to_string(dst_ch0_width_layout);
                }
                case 11:
                {
                    return "dst_ch3_width_layout: " + std::to_string(dst_ch3_width_layout)
                        + "\n"
                        + "dst_ch2_width_layout: " + std::to_string(dst_ch2_width_layout);
                }

                case 12:
                {
                    FP32 m;
                    m.u = M0;
                    return "M0: " + std::to_string(m.f);
                }
                case 13:
                {
                    FP32 m;
                    m.u = M1;
                    return "M1: " + std::to_string(m.f);
                }
                case 14:
                {
                    FP32 m;
                    m.u = M3;
                    return "M3: " + std::to_string(m.f);
                }
                case 15:
                {
                    FP32 m;
                    m.u = M4;
                    return "M4: " + std::to_string(m.f);
                }
                case 16:
                {
                    return "";
                }
                case 17:
                {
                    return "";
                }
                case 18:
                {
                    return "dst_format: " + std::to_string(dst_format) + "\n"
                        + "src_format: " + std::to_string(src_format) + "\n"
                        + "bound_ind: " + std::to_string(bound_ind) + "\n"
                        + "shift: " + std::to_string(shift) + "\n"
                        + "pad_mod: " + std::to_string(pad_mod) + "\n"
                        + "interpolation: " + std::to_string(interpolation) + "\n"
                        + "cord_round: " + std::to_string(cord_round) + "\n"
                        + "dst_channel: " + std::to_string(dst_channel) + "\n"
                        + "channel: " + std::to_string(channel);
                }
                case 19:
                {
                    return "bound_smooth: " + std::to_string(bound_smooth) + "\n"
                        + "bound_val: " + std::to_string(bound_val);
                }
                case 20:
                {
                    return "yuv2rgb_coef1: " + std::to_string(yuv2rgb_coef1) + "\n"
                        + "yuv2rgb_coef0: " + std::to_string(yuv2rgb_coef0);
                }
                case 21:
                {
                    return "yuv2rgb_coef3: " + std::to_string(yuv2rgb_coef3) + "\n"
                        + "yuv2rgb_coef2: " + std::to_string(yuv2rgb_coef2);
                }
                case 22:
                {
                    return "yuv2rgb_coef5: " + std::to_string(yuv2rgb_coef5) + "\n"
                        + "yuv2rgb_coef4: " + std::to_string(yuv2rgb_coef4);
                }
                case 23:
                {
                    return "yuv2rgb_coef7: " + std::to_string(yuv2rgb_coef7) + "\n"
                        + "yuv2rgb_coef6: " + std::to_string(yuv2rgb_coef6);
                }
                case 24:
                {
                    return "const_pad_ch0: " + std::to_string(const_pad_ch0) + "\n"
                        + "yuv2rgb_coef11: " + std::to_string(yuv2rgb_coef11) + "\n"
                        + "yuv2rgb_coef10: " + std::to_string(yuv2rgb_coef10);
                }
                case 25:
                {
                    return "sign: " + std::to_string(sign) + "\n"
                        + "cmd_id: " + std::to_string(cmd_id) + "\n"
                        + "dst_ind: " + std::to_string(dst_ind) + "\n"
                        + "src_ind: " + std::to_string(src_ind) + "\n"
                        + "const_pad_ch3: " + std::to_string(const_pad_ch3) + "\n"
                        + "const_pad_ch2: " + std::to_string(const_pad_ch2) + "\n"
                        + "const_pad_ch1: " + std::to_string(const_pad_ch1);
                }
                case 26:
                {
                    return "yuv2rgb_coef9: " + std::to_string(yuv2rgb_coef9) + "\n"
                        + "yuv2rgb_coef8: " + std::to_string(yuv2rgb_coef8);
                }
                case 27:
                {
                    return "";
                }
                case 28:
                {
                    return "pad_b: " + std::to_string(pad_b) + "\n"
                        + "pad_t: " + std::to_string(pad_t);
                }
                case 29:
                {
                    return "pad_r: " + std::to_string(pad_r) + "\n"
                        + "pad_l: " + std::to_string(pad_l);
                }
                case 30:
                {
                    return "src_height_shape: " + std::to_string(src_height_shape) + "\n"
                        + "src_width_shape: " + std::to_string(src_width_shape);
                }
                case 31:
                {
                    return "csc_en: " + std::to_string(csc_en) + "\n"
                        + "intr_mask: " + std::to_string(intr_mask) + "\n"
                        + "dst_height_shape: " + std::to_string(dst_height_shape) + "\n"
                        + "dst_width_shape: " + std::to_string(dst_width_shape);
                }
                case 32:
                {
                    FP32 m;
                    m.u = M2;
                    return "M2: " + std::to_string(m.f);
                }
                case 33:
                {
                    FP32 m;
                    m.u = M5;
                    return "M5: " + std::to_string(m.f);
                }
                case 34:
                {
                    return "src_y: " + std::to_string(src_y) + "\n"
                        + "src_x: " + std::to_string(src_x);
                }
                case 35:
                {
                    return "ai2d_calc_enable: " + std::to_string(ai2d_calc_enable) + "\n"
                        + "dst_y: " + std::to_string(dst_y) + "\n"
                        + "dst_x: " + std::to_string(dst_x);
                }
                default:
                    return "";
                }
            }

            float u32_to_float(uint32_t u)
            {
                FP32 m;
                m.u = u;
                return m.f;
            }

            float origin_M(uint32_t u)
            {
                return u32_to_float(u) / 1024.f;
            }
        };

        int32_t get_size_from_shape(dims_t shape);
        int32_t get_bytes_from_type(typecode_t type);
        std::vector<segment> get_segment_start_end_length(int32_t start, int32_t chunk_size, int32_t upper_bound);

        class ai2d_utils
        {
        public:
            void update_static_param(ai2d_config &config, dims_t &in_shape, dims_t &out_shape,
                ai2d_datatype_t &ai2d_dtype, ai2d_crop_param_t &crop_param, ai2d_shift_param_t &shift_param, ai2d_pad_param_t &pad_param,
                ai2d_resize_param_t &resize_param, ai2d_affine_param_t &affine_param);
            void update_M_param(ai2d_config &config, tensor4d_segment &ifmap, tensor4d_segment &ofmap, ai2d_datatype_t &ai2d_dtype, ai2d_resize_param_t &resize_param, ai2d_affine_param_t &affine_param);
            void update_dynamic_param(ai2d_config &config, int32_t src_x, int32_t src_y, ai2d_datatype_t &ai2d_dtype, ai2d_pad_param_t &pad_param, tensor4d_segment &ifmap_sram, tensor4d_segment &ofmap_sram, tensor4d_segment &ifmap, tensor4d_segment &ofmap, float offset_M2, float offset_M5, dims_t &input_shape, dims_t &output_shape, bool broadcast_in_channel);
            void resize_sram_search(ai2d_config &config, tensor4d_segment &ofmap, tensor4d_segment &ifmap, std::vector<int32_t> &ret);
            void affine_sram_search(ai2d_config &config, tensor4d_segment &ofmap, tensor4d_segment &ifmap, ai2d_pad_param_t &pad_param, std::vector<int32_t> &ret);
            void inv_M(std::vector<float> &M_ori_scale, std::vector<float> &M_ori_bias, std::vector<float> &M_inv_scale, std::vector<float> &M_inv_bias);
            std::vector<float> M_mul_add(std::vector<float> &M_scale, std::vector<float> &M_bias, std::vector<float> &v_i);
            bool try_allocate_resize_sram(std::vector<float> &M_ori_scale, std::vector<float> &M_ori_bias, int32_t dst_max_h, int32_t dst_max_w, tensor4d_segment &ifmap, tensor4d_segment &ofmap, ai2d_format &src_format);
            bool try_allocate_affine_sram(std::vector<float> &M_ori_scale, std::vector<float> &M_ori_bias, segment &output_h, segment &output_w);
            void update_regs(ai2d_config &config, bool write_all, std::vector<std::vector<uint32_t>> &regs);

        private:
            ai2d_sram_t ai2d_sram;
        };

    }
}
}