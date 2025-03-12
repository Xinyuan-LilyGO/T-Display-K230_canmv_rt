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
#include "compiler_defs.h"
#include <string>

BEGIN_NS_NNCASE_RT_K230

enum gnne_reg_id
{
    gnne_reg_gpr0 = 0,
    gnne_reg_gpr1 = 1,
    gnne_reg_host_mem_base_addr0 = 33,
    gnne_reg_host_mem_base_addr1 = 34,
    gnne_reg_host_mem_base_addr2 = 35,
    gnne_reg_host_mem_base_addr3 = 36,
};

typedef struct dequantize_params
{
    int16_t scale;
    int8_t shift;
    uint8_t zero_point;
} dequantize_params_t;

typedef struct quantize_params
{
    bfloat16 scale;
    bfloat16 zero_point;
} quantize_params_t;

enum class mfu_trans_permute
{
    nchw = 0x0, /* 0:重要*/
    ncwh = 0x1, /* 1:重要*/
    nhcw = 0x2, /* 2:重要*/
    nhwc = 0x3, /* 3:重要*/
    nwch = 0x4, /* 4:None*/
    nwhc = 0x5, /* 5:None*/
    cnhw = 0x6, /* 6:None*/
    cnwh = 0x7, /* 7:None*/
    chnw = 0x8, /* 8:None*/
    chwn = 0x9, /* 9:None*/
    cwnh = 0xa, /* 10:None*/
    cwhn = 0xb, /* 11:None*/
    hncw = 0xc, /* 12:None*/
    hnwc = 0xd, /* 13:None*/
    hcnw = 0xe, /* 14:None*/
    hcwn = 0xf, /* 15:None*/
    hwnc = 0x10, /* 16:None*/
    hwcn = 0x11, /* 17:None*/
    wnch = 0x12, /* 18:None*/
    wnhc = 0x13, /* 19:None*/
    wcnh = 0x14, /* 20:None*/
    wchn = 0x15, /* 21:None*/
    whnc = 0x16, /* 22:None*/
    whcn = 0x17 /* 23:None*/
};

enum class mfu_pdp_op
{
    min = 0x1, /* 1:取最小值*/
    max = 0x0, /* 0:取最大值*/
    average = 0x2, /* 2:取平均值*/
    sum = 0x3 /* 3:求和*/
};

enum class mfu_reduce_op
{
    max = 0x0, /* 0:两者取最大值*/
    min = 0x1, /* 1:两者取最小值*/
    add = 0x2, /* 2:两者求和*/
    sub = 0x3, /* 3:左操作数减去右操作数*/
    mul = 0x4 /* 4:两者求其点积*/
};

enum class mfu_reduce_dim
{
    w = 0x0, /* 0:None*/
    hw = 0x1, /* 1:None*/
    chw = 0x2, /* 2: None*/
    nchw = 0x3, /* 3:None*/
};

enum class mfu_crop_resize
{
    biliner = 0x0, /* 0:双线性插值*/
    nearest = 0x1 /* 1:最邻近插值*/
};

enum class mfu_crop_align
{
    none = 0x0, /* 0:tensorflow align_corner=false*/
    corner = 0x1, /* 1:tensorflow align_corner=True*/
    center = 0x2 /* 2:open cv，中心对齐*/
};

enum class mn_round_mode
{
    round = 0,
    floor = 1,
    ceil = 2
};

static std::string to_string(mn_round_mode v)
{
    switch (v)
    {
    case mn_round_mode::round:
        return "round";
    case mn_round_mode::floor:
        return "floor";
    case mn_round_mode::ceil:
        return "ceil";
    default:
        throw std::runtime_error("unimplemented");
    }
}

enum class mn_triangle_mode
{
    sin = 0,
    cos = 1
};

static std::string to_string(mn_triangle_mode v)
{
    switch (v)
    {
    case mn_triangle_mode::sin:
        return "sin";
    case mn_triangle_mode::cos:
        return "cos";
    default:
        throw std::runtime_error("unimplemented");
    }
}

enum class mn_unary_logic_mode
{
    abs = 0,
    sign = 1,
    neg = 2
};

static std::string to_string(mn_unary_logic_mode v)
{
    switch (v)
    {
    case mn_unary_logic_mode::abs:
        return "abs";
    case mn_unary_logic_mode::sign:
        return "sign";
    case mn_unary_logic_mode::neg:
        return "neg";
    default:
        throw std::runtime_error("unimplemented");
    }
}

enum class mn_cmp_mode
{
    min = 0,
    max = 1,
    gt = 2,
    ge = 3,
    lt = 4,
    le = 5,
    eq = 6
};

static std::string to_string(mn_cmp_mode v)
{
    switch (v)
    {
    case mn_cmp_mode::min:
        return "min";
    case mn_cmp_mode::max:
        return "max";
    case mn_cmp_mode::gt:
        return "gt";
    case mn_cmp_mode::ge:
        return "ge";
    case mn_cmp_mode::lt:
        return "lt";
    case mn_cmp_mode::le:
        return "le";
    case mn_cmp_mode::eq:
        return "eq";
    default:
        throw std::runtime_error("unimplemented");
    }
}

enum class mn_sqrt_mode
{
    positive = 0, // 输入符号位直接认为是0，一定是正数
    nan = 1, // 如果是输入符号位位负的，返回NaN
    negative = 2 // 如果输入是正数，正常输出。否则输出负数
};

static std::string to_string(mn_sqrt_mode v)
{
    switch (v)
    {
    case mn_sqrt_mode::positive:
        return "positive";
    case mn_sqrt_mode::nan:
        return "nan";
    case mn_sqrt_mode::negative:
        return "negative";
    default:
        throw std::runtime_error("unimplemented");
    }
}

enum class mn_log_mode
{
    positive = 0, // 输入符号位直接认为是0，一定是正数
    nan = 1, // 如果是输入符号位位负的，返回NaN
    negative = 2 //如果输入是正数，正常输出。否则输出负数
};

static std::string to_string(mn_log_mode v)
{
    switch (v)
    {
    case mn_log_mode::positive:
        return "positive";
    case mn_log_mode::nan:
        return "nan";
    case mn_log_mode::negative:
        return "negative";
    default:
        throw std::runtime_error("unimplemented");
    }
}

enum class mn_binary_logic_islogic
{
    bitwise = 0,
    logic = 1
};

static std::string to_string(mn_binary_logic_islogic v)
{
    switch (v)
    {
    case mn_binary_logic_islogic::bitwise:
        return "bitwise";
    case mn_binary_logic_islogic::logic:
        return "logic";
    default:
        throw std::runtime_error("unimplemented");
    }
}

enum class mn_binary_logic_mode
{
    // add prefix to avoid collision with C++ keywords.
    kand = 0,
    kor = 1,
    knot = 2,
    kxor = 3
};

enum class sparsity
{
    dense = 0, /* 0:直接存储*/
    sparse = 1 /* 1:对0进行压缩编码*/
};

static std::string to_string(mn_binary_logic_mode v, mn_binary_logic_islogic l = mn_binary_logic_islogic::bitwise)
{
    switch (v)
    {
    case mn_binary_logic_mode::kand:
        return l == mn_binary_logic_islogic::bitwise ? "&" : "&&";
    case mn_binary_logic_mode::kor:
        return l == mn_binary_logic_islogic::bitwise ? "|" : "||";
    case mn_binary_logic_mode::knot:
        return l == mn_binary_logic_islogic::bitwise ? "~" : "!";
    case mn_binary_logic_mode::kxor:
        return l == mn_binary_logic_islogic::bitwise ? "^" : "^^";
    default:
        throw std::runtime_error("unimplemented");
    }
}

END_NS_NNCASE_RT_K230
