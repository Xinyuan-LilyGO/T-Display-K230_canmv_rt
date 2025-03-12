/* Copyright 2020 Canaan Inc.
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
#include "../kernel_utils.h"
#include <cmath>
#include <nncase/runtime/nnil.h>
#include <nncase/runtime/runtime_op_utility.h>
#include <stdexcept>
#include <type_traits>
#include <xtl/xspan.hpp>
#ifdef __riscv
#include "../riscv/k230_kernels.h"
#endif
using namespace knn::runtime::k230;
using namespace nncase::ir::k230;
namespace knn::kernels::k230
{
void gnne_transpose(const bfloat16 *CXX_RESTRICT input, bfloat16 *CXX_RESTRICT output, const runtime_shape_t &in_shape, const mfu_trans_permute &gnne_perm)
{
    auto perm = to_axis(gnne_perm);
    runtime_shape_t out_shape;
    for (size_t i = 0; i < 4; i++)
        out_shape[i] = in_shape[perm[i]];
    runtime_shape_t i, o;
    for (o[3] = 0; o[3] < out_shape[3]; o[3]++)
    {
        i[perm[3]] = o[3];
        for (o[2] = 0; o[2] < out_shape[2]; o[2]++)
        {
            i[perm[2]] = o[2];
            for (o[1] = 0; o[1] < out_shape[1]; o[1]++)
            {
                i[perm[1]] = o[1];
                for (o[0] = 0; o[0] < out_shape[0]; o[0]++)
                {
                    i[perm[0]] = o[0];
                    output[offset(out_shape, o)] = input[offset(in_shape, i)];
                }
            }
        }
    }
}

void gnne_conv2d(const bfloat16 *input, bfloat16 *output, const bfloat16 *weights, const bfloat16 *psum, const bfloat16 *act, const runtime_shape_t &in_shape,
    int32_t groups, int32_t out_channels, int32_t filter_h, int32_t filter_w, int32_t stride_h, int32_t stride_w, int32_t dilation_h, int32_t dilation_w,
    const padding &padding_h, const padding &padding_w, value_range<bfloat16> fused_clamp)
{
    const auto out_h = details::get_windowed_output_size(in_shape[2], filter_h, stride_h, dilation_h, padding_h);
    const auto out_w = details::get_windowed_output_size(in_shape[3], filter_w, stride_w, dilation_w, padding_w);
    const auto g_ic = in_shape[1] / groups;
    const auto g_oc = out_channels / groups;
    for (int32_t batch = 0; batch < in_shape[0]; batch++)
    {
        const bfloat16 *in_batch_p = input + (size_t)batch * in_shape[1] * in_shape[2] * in_shape[3];
        for (int32_t og = 0; og < groups; og++)
        {
            const bfloat16 *in_group_p = in_batch_p + (size_t)og * g_ic * in_shape[2] * in_shape[3];
            const bfloat16 *w_group_p = weights + (size_t)og * g_oc * g_ic * filter_h * filter_w;
            for (int32_t oc = 0; oc < g_oc; oc++)
            {
                const bfloat16 *w_oc_p = w_group_p + (size_t)oc * g_ic * filter_h * filter_w;
                for (int32_t oy = 0; oy < out_h; oy++)
                {
                    for (int32_t ox = 0; ox < out_w; ox++)
                    {
                        const int32_t in_y_origin = (oy * stride_h) - padding_h.before;
                        const int32_t in_x_origin = (ox * stride_w) - padding_w.before;
                        const int32_t filter_y_start = std::max(0, (-in_y_origin + dilation_h - 1) / dilation_h);
                        const int32_t filter_y_end = std::min(filter_h, (in_shape[2] - in_y_origin + dilation_h - 1) / dilation_h);
                        const int32_t filter_x_start = std::max(0, (-in_x_origin + dilation_w - 1) / dilation_w);
                        const int32_t filter_x_end = std::min(filter_w, (in_shape[3] - in_x_origin + dilation_w - 1) / dilation_w);
                        bfloat16 value = (bfloat16)0;
                        for (int32_t ic = 0; ic < g_ic; ic++)
                        {
                            const bfloat16 *in_c_p = in_group_p + (size_t)ic * in_shape[2] * in_shape[3];
                            const bfloat16 *w_ic_p = w_oc_p + (size_t)ic * filter_h * filter_w;
                            for (int32_t ky = filter_y_start; ky < filter_y_end; ky++)
                            {
                                for (int32_t kx = filter_x_start; kx < filter_x_end; kx++)
                                {
                                    const int32_t in_y = in_y_origin + dilation_h * ky;
                                    const int32_t in_x = in_x_origin + dilation_w * kx;
                                    const bfloat16 in_v = in_c_p[in_y * in_shape[3] + in_x];
                                    const bfloat16 w = w_ic_p[ky * filter_w + kx];
                                    value = value + in_v * w;
                                }
                            }
                        }
                        value = details::apply_activation(value, act[oc * 5], act[oc * 5 + 1], act[oc * 5 + 2], act[oc * 5 + 3], act[oc * 5 + 4]);
                        *output++ = psum[og * g_oc + oc] + details::apply_activation(value, fused_clamp);
                    }
                }
            }
        }
    }
}

void gnne_matmul(const bfloat16 *input_a, const bfloat16 *input_b, bfloat16 *output, const bfloat16 *act, int32_t a_rows, int32_t a_cols, int32_t b_cols, const value_range<bfloat16> &fused_activation)
{
    for (int32_t oy = 0; oy < a_rows; oy++)
    {
        for (int32_t ox = 0; ox < b_cols; ox++)
        {
            bfloat16 value = bfloat16(0);

            for (int32_t i = 0; i < a_cols; i++)
            {
                const auto a = input_a[oy * a_cols + i];
                const auto b = input_b[i * b_cols + ox];
                value = value + a * b;
            }

            if (value < act[0])
                value = value * act[1] + act[2];
            else
                value = value * act[3] + act[4];

            output[oy * b_cols + ox] = details::apply_activation(value, fused_activation);
        }
    }
}

template <typename TI, typename TO>
void gnne_load(const TI *input, const TO *output, const runtime_shape_t &in_shape, const int32_t &channel_axis, const bfloat16 *deq_params)
{
    if (std::is_same_v<TI, TO>)
    {
        std::memcpy(output, input, sizeof(TI) * details::compute_size(in_shape));
    }
    else if ((std::is_same_v<TI, float> && std::is_same_v<TO, bfloat16>)
        || (std::is_same_v<TI, bfloat16> && std::is_same_v<TO, float>))
    {
        for (size_t i = 0; i < details::compute_size(in_shape); i++)
            output[i] = static_cast<TO>(input[i]);
    }
    else if ((std::is_same_v<TI, int8_t> || std::is_same_v<TO, uint8_t>)&&(std::is_same_v<TI, bfloat16> || std::is_same_v<TO, float>))
    {
        size_t size_per_channel = 1;
        for (int32_t i = channel_axis + 1; i < 4; i++)
        {
            size_per_channel *= in_shape[i];
        }
        for (size_t i = 0; i < details::compute_size(in_shape); i++)
        {
            auto value = (input[i] - deq_params[(i % size_per_channel) * 2 + 1]) * deq_params[(i % size_per_channel) * 2];
            output[i] = static_cast<TO>(value);
        }
    }
    else
    {
        std::runtime_error("unsupported convert type in gnne_load!");
    }
}

template <typename TI, typename TO>
void gnne_store(const TI *input, const TO *output, const runtime_shape_t &in_shape, const int32_t &channel_axis, const bfloat16 *q_params)
{
    if (std::is_same_v<TI, TO>)
    {
        std::memcpy(output, input, sizeof(TI) * details::compute_size(in_shape));
    }
    else if ((std::is_same_v<TI, float> && std::is_same_v<TO, bfloat16>)
        || (std::is_same_v<TI, bfloat16> && std::is_same_v<TO, float>))
    {
        for (size_t i = 0; i < details::compute_size(in_shape); i++)
            output[i] = static_cast<TO>(input[i]);
    }
    else if ((std::is_same_v<TI, int8_t> || std::is_same_v<TO, uint8_t>)&&std::is_same_v<TI, bfloat16>)
    {
        size_t size_per_channel = 1;
        for (int32_t i = channel_axis + 1; i < 4; i++)
        {
            size_per_channel *= in_shape[i];
        }
        for (size_t i = 0; i < details::compute_size(in_shape); i++)
        {
            auto value = (input[i] - q_params[(i % size_per_channel) * 2 + 1]) / q_params[(i % size_per_channel) * 2];
            output[i] = static_cast<TO>(value);
        }
    }
    else
    {
        std::runtime_error("unsupported convert type in gnne_store!");
    }
}
}
