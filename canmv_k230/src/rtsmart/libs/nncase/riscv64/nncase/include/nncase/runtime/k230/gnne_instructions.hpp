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
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <nncase/codegen/binary_writer.h>
#include <nncase/runtime/bitio.h>
#include <sstream>
#include <vector>

#define GNNE_INST_ASSERT assert

namespace nncase::runtime::k230::isa
{
typedef uint64_t ADDR;
typedef uint16_t REG;
typedef uint32_t BF24;

namespace DATA_STRUCT
{
    /////////////////////////////////
    // auto generated structs

    /**
    STRIDE_GLB
    @brief 在寻址的时候每增加一行需要跳跃的byte数
    - uint64_t stride_glb_h 在寻址的时候每增加一行需要跳跃的byte数
    - uint64_t stride_glb_c 在寻址的时候每增加一列需要跳跃的byte数
    - uint64_t stride_glb_n 在寻址的时候每增加一通道需要跳跃的byte数
**/
    typedef struct stride_glb_t
    {
        uint64_t stride_glb_h : 21; /** 在寻址的时候每增加一行需要跳跃的byte数 **/
        uint64_t stride_glb_c : 21; /** 在寻址的时候每增加一列需要跳跃的byte数 **/
        uint64_t stride_glb_n : 21; /** 在寻址的时候每增加一通道需要跳跃的byte数 **/

        stride_glb_t()
        {
            stride_glb_h = 0;
            stride_glb_c = 0;
            stride_glb_n = 0;
        }

    } STRIDE_GLB;

    /**
    CCRCLR
    @brief 是否需要将某个 CCR 减 1
    - uint64_t ccr_clr 是否需要将某个 CCR 减 1
    - uint64_t ccr 指明使用哪个 CCR
    - uint64_t ccr_acq 是否需要等待某个ccr
**/
    typedef struct ccrclr_t
    {
        uint64_t ccr_clr : 1; /** 是否需要将某个 CCR 减 1 **/
        uint64_t ccr : 6; /** 指明使用哪个 CCR **/
        uint64_t ccr_acq : 1; /** 是否需要等待某个ccr **/

        ccrclr_t()
        {
            ccr_clr = 0;
            ccr = 0;
            ccr_acq = 0;
        }

    } CCRCLR;

    /**
    CCRSET
    @brief 是否需要设置 CCR
    - uint64_t ccr_set 是否需要设置 CCR
    - uint64_t ccr 指明使用哪个 CCR
    - uint64_t ccr_value ccr 置值
**/
    typedef struct ccrset_t
    {
        uint64_t ccr_set : 1; /** 是否需要设置 CCR **/
        uint64_t ccr : 6; /** 指明使用哪个 CCR **/
        uint64_t ccr_value : 4; /** ccr 置值 **/

        ccrset_t()
        {
            ccr_set = 0;
            ccr = 0;
            ccr_value = 0;
        }

    } CCRSET;

    /**
    QARG
    @brief 某个通道的scale数值
    - uint64_t scale_chan 某个通道的scale数值
    - uint64_t bias_chan 某个通道的偏置
    - uint64_t shift_chan 某个通道的shift量
**/
    typedef struct qarg_t
    {
        uint64_t scale_chan : 16; /** 某个通道的scale数值 **/
        uint64_t bias_chan : 8; /** 某个通道的偏置 **/
        uint64_t shift_chan : 8; /** 某个通道的shift量 **/

        qarg_t()
        {
            scale_chan = 0;
            bias_chan = 0;
            shift_chan = 0;
        }

    } QARG;

    /**
    MNCFG
    @brief 如果是1的话，某个输入就取反
    - uint64_t add_sub_0 如果是1的话，某个输入就取反
    - uint64_t add_sub_1 如果是1的话，某个输入就取反
    - uint64_t add_sub_2 如果是1的话，某个输入就取反
    - uint64_t add_sub_3 如果是1的话，某个输入就取反
    - uint64_t mul_neg_0 如果是1输出取反
    - uint64_t mul_neg_1 如果是1输出取反
    - uint64_t mul_neg_2 如果是1输出取反
    - uint64_t mul_neg_3 如果是1输出取反
    - uint64_t div_neg_0 如果是1输出取反
    - uint64_t binary_logic_isLogic 0：按位运算
1：按照WORD进行逻辑运算
    - uint64_t binary_logic_mode 0：And
1：Or
2：NOT
3：XOR
    - uint64_t round_mode 0：round
1：floor
2：ceil
    - uint64_t sqrt_mode 0：输入符号位直接认为是0，一定是正数
1：如果是输入符号位位负的，返回NaN
2：如果输入是正数，正常输出。否者输出负数
    - uint64_t tran_mode 0：sin
1：cos
    - uint64_t log_mod 0：输入符号位直接认为是0，一定是正数
1：如果是输入符号位位负的，返回NaN
2：如果输入是正数，正常输出。否者输出负数
    - uint64_t unary_logic_mode 0：abs
1：sign
2：neg
    - uint64_t cmp_mode_0 0：MIN
1：MAX
2：>
3：>=
4：<
5：<=
6：==
    - uint64_t cmp_mode_1 0：MIN
1：MAX
2：>
3：>=
4：<
5：<=
6：==
**/
    typedef struct mncfg_t
    {
        uint64_t add_sub_0 : 2; /** 如果是1的话，某个输入就取反 **/
        uint64_t add_sub_1 : 2; /** 如果是1的话，某个输入就取反 **/
        uint64_t add_sub_2 : 2; /** 如果是1的话，某个输入就取反 **/
        uint64_t add_sub_3 : 2; /** 如果是1的话，某个输入就取反 **/
        uint64_t mul_neg_0 : 1; /** 如果是1输出取反 **/
        uint64_t mul_neg_1 : 1; /** 如果是1输出取反 **/
        uint64_t mul_neg_2 : 1; /** 如果是1输出取反 **/
        uint64_t mul_neg_3 : 1; /** 如果是1输出取反 **/
        uint64_t div_neg_0 : 1; /** 如果是1输出取反 **/
        uint64_t round_mode : 2; /** 0：round
     1：floor
     2：ceil **/
        uint64_t sqrt_mode : 2; /** 0：输入符号位直接认为是0，一定是正数
1：如果是输入符号位位负的，返回NaN
2：如果输入是正数，正常输出。否者输出负数 **/
        uint64_t tran_mode : 1; /** 0：sin
1：cos **/
        uint64_t log_mode : 2; /** 0：输入符号位直接认为是0，一定是正数
1：如果是输入符号位位负的，返回NaN
2：如果输入是正数，正常输出。否者输出负数 **/
        uint64_t unary_logic_mode : 2; /** 0：abs
1：sign
2：neg **/
        uint64_t cmp_mode_0 : 3; /** 0：MIN
 1：MAX
 2：>
 3：>=
 4：<
 5：<=
 6：== **/
        uint64_t cmp_mode_1 : 3; /** 0：MIN
 1：MAX
 2：>
 3：>=
 4：<
 5：<=
 6：== **/

        mncfg_t()
        {
            add_sub_0 = 0;
            add_sub_1 = 0;
            add_sub_2 = 0;
            add_sub_3 = 0;
            mul_neg_0 = 0;
            mul_neg_1 = 0;
            mul_neg_2 = 0;
            mul_neg_3 = 0;
            div_neg_0 = 0;
            round_mode = 0;
            sqrt_mode = 0;
            tran_mode = 0;
            log_mode = 0;
            unary_logic_mode = 0;
            cmp_mode_0 = 0;
            cmp_mode_1 = 0;
        }

    } MNCFG;

} // namespace DATA_STRUCT

class CCRCLR
{
    typedef DATA_STRUCT::CCRCLR T_CCRCLR;
    T_CCRCLR data;

public:
    CCRCLR(uint64_t value)
    {
        data.ccr_clr = value & 0x1;
        data.ccr = (value >> 1) & 0x3f;
        data.ccr_acq = (value >> 7) & 0x1;
    }

    CCRCLR(uint64_t ccr, bool acq, bool clr = true)
    {
        this->data.ccr = ccr;
        this->data.ccr_acq = acq;
        this->data.ccr_clr = clr;
    }

    uint64_t inline value() const
    {
        uint64_t val = 0;
        val += data.ccr_clr;
        val += (data.ccr) << 1;
        val += (data.ccr_acq) << 7;
        return val;
    }

    operator uint64_t() const { return value(); }

    friend std::ostream &operator<<(std::ostream &os, const CCRCLR &ccrclr)
    {
        os << "ccrclr(ccr: " << ccrclr.data.ccr
           << " , ccr_acq: " << ccrclr.data.ccr_acq
           << " , ccr_clr: " << ccrclr.data.ccr_clr << ")";
        return os;
    }
};

class CCRSET
{
    typedef DATA_STRUCT::CCRSET T_CCRSET;
    T_CCRSET data;

public:
    CCRSET(uint64_t value)
    {
        data.ccr_set = value & 0x1;
        data.ccr = (value >> 1) & 0x3f;
        data.ccr_value = (value >> 7) & 0xf;
    }

    CCRSET(uint16_t ccr, uint16_t value, bool ccr_set = true)
    {
        this->data.ccr_set = ccr_set;
        this->data.ccr = ccr;
        this->data.ccr_value = value;
    }

    uint64_t inline value() const
    {
        uint64_t val = 0;
        val += data.ccr_set;
        val += (data.ccr << 1);
        val += (data.ccr_value) << 7;
        return val;
    }

    operator uint64_t() const { return value(); }

    friend std::ostream &operator<<(std::ostream &os, const CCRSET &ccrset)
    {
        os << "ccrset(ccr: " << ccrset.data.ccr
           << " , ccr_value: " << ccrset.data.ccr_value
           << " , ccr_set: " << ccrset.data.ccr_set << ")";
        return os;
    }
};

template <int SHIFT, int ADDR_WIDTH, int MMU_ITEM_WIDTH>
class GLB_ADDR
{
#pragma pack(1)
    struct T_ADDR
    {
        uint64_t addr : ADDR_WIDTH;
        uint64_t mmu_item : 4;
    };
#pragma pack()
    T_ADDR data;

public:
    inline GLB_ADDR(uint64_t value)
    {
        data.addr = value & ((1 << ADDR_WIDTH) - 1);
        data.mmu_item = (value >> ADDR_WIDTH) & 0xf;
    }

    uint64_t inline value() const
    {
        uint64_t val = 0;
        val += (data.mmu_item << ADDR_WIDTH);
        val += data.addr;
        return val;
    }

    uint64_t inline mmu_item() const
    {
        if (MMU_ITEM_WIDTH)
        {
            return data.mmu_item;
        }
        return 0;
    }

    void inline mmu_item(uint64_t mmu_item)
    {
        if (MMU_ITEM_WIDTH)
        {
            data.mmu_item = mmu_item;
        }
    }

    uint64_t inline addr() const { return data.addr << SHIFT; }

    void inline addr(uint64_t value) { data.addr = value >> SHIFT; }

    operator uint64_t() const { return value(); }

    GLB_ADDR<SHIFT, ADDR_WIDTH, MMU_ITEM_WIDTH> &
    operator+=(const GLB_ADDR<SHIFT, ADDR_WIDTH, MMU_ITEM_WIDTH> &c)
    {
        this->addr(this->addr() + c.addr());
        return *this;
    }
};
template <int SHIFT, int ADDR_WIDTH, int MMU_ITEM_WIDTH>
std::ostream &operator<<(std::ostream &out,
    const GLB_ADDR<SHIFT, ADDR_WIDTH, MMU_ITEM_WIDTH> &c)
{
    if (MMU_ITEM_WIDTH)
        out << "(mmu: " << c.mmu_item() << ", addr: " << c.addr() << ")";
    else
        out << "(addr: " << c.addr() << ")";
    return out;
}

typedef GLB_ADDR<0, 21, 0> ADDR_GLB_8_WITHOUT_BANK;
typedef GLB_ADDR<3, 17, 0> ADDR_GLB_64_WITHOUT_BANK;
typedef GLB_ADDR<0, 21, 4> ADDR_GLB_8_WITH_BANK;
typedef GLB_ADDR<3, 17, 4> ADDR_GLB_64_WITH_BANK;

class UNION_ADDR
{
    union union_data
    {
        ADDR_GLB_8_WITH_BANK glb;
        ADDR ddr;
        uint64_t u64;
        union_data()
            : u64(0) { }
    };

    union union_data data;

public:
    inline UNION_ADDR(uint64_t data) { this->data.u64 = data; }

    bool inline is_ddr() const { return (data.u64 >> 31) & 1; }

    ADDR_GLB_8_WITH_BANK inline glb() const
    {
        GNNE_INST_ASSERT(!is_ddr());
        return data.glb;
    }

    void inline glb(ADDR_GLB_8_WITH_BANK addr) { data.glb = addr; }

    ADDR inline ddr() const
    {
        GNNE_INST_ASSERT(is_ddr());
        return data.ddr & 0x7FFFFFFF;
    }

    void inline ddr(ADDR addr) { data.ddr = 0x80000000 | addr; }

    uint64_t inline value() const { return data.u64; }

    operator uint64_t() const { return value(); }
};

class STRIDE_GLB
{
protected:
    struct stride_glb_t
    {
        uint64_t stride_glb_h : 21;
        uint64_t stride_glb_c : 21;
        uint64_t stride_glb_n : 21;
        stride_glb_t() = default;
    };

    struct stride_glb_t data;

public:
    STRIDE_GLB(uint64_t value = 0)
    {
        data.stride_glb_h = value & 0x1fffff;
        data.stride_glb_c = (value >> 21) & 0x1fffff;
        data.stride_glb_n = (value >> 42) & 0x1fffff;
    }

    void stride_glb_c(uint64_t value) { data.stride_glb_c = value; }

    void stride_glb_n(uint64_t value) { data.stride_glb_n = value; }

    void stride_glb_h(uint64_t value) { data.stride_glb_h = value; }

    uint64_t stride_glb_c() const { return data.stride_glb_c; }

    uint64_t stride_glb_n() const { return data.stride_glb_n; }

    uint64_t stride_glb_h() const { return data.stride_glb_h; }

    uint64_t inline value() const
    {
        uint64_t val = data.stride_glb_n;
        uint64_t tmp = data.stride_glb_c;
        val = val << 42;
        val += (tmp << 21);
        val += data.stride_glb_h;
        return val;
    }

    operator uint64_t() const { return value(); }
};

union stride_union
{
    STRIDE_GLB stride;
    struct layout
    {
        uint64_t w : 16;
        uint64_t h : 16;
        uint64_t c : 16;
        uint64_t n : 16;
    } layout;
    uint64_t value = 0;
    stride_union()
        : value(0) {};
};

inline std::ostream &operator<<(std::ostream &out, const STRIDE_GLB &c)
{
    stride_union tmp;
    tmp.stride = c;
    out << "stride_glb: ( n:" << c.stride_glb_n() << " , c:" << c.stride_glb_c()
        << ", h:" << c.stride_glb_h() << " ) "
        << "layout:("
        << "n: " << tmp.layout.n << ", "
        << "c: " << tmp.layout.c << ", "
        << "h: " << tmp.layout.h << ", "
        << "w: " << tmp.layout.w << ") ";
    return out;
}

inline std::ostream &operator<<(std::ostream &out, const UNION_ADDR &c)
{
    if (c.is_ddr())
    {
        out << "(ddr : " << c.ddr() << ")";
    }
    else
    {
        out << c.glb();
    }
    return out;
}

/////////////////////////////////
// auto generated enums

/**
    MMU.CONF.WIDTH
    @brief
    - 1 : 0x0(0)  1 bank
    - 2 : 0x1(1)  2 banks
    - 4 : 0x2(2)  4 banks
    - 8 : 0x3(3)  8 banks
**/
typedef enum mmu_conf_width_t : uint8_t
{
    MMU_CONF_WIDTH_1 = 0x0, /* 0:1 bank*/
    MMU_CONF_WIDTH_2 = 0x1, /* 1:2 banks*/
    MMU_CONF_WIDTH_4 = 0x2, /* 2:4 banks*/
    MMU_CONF_WIDTH_8 = 0x3 /* 3:8 banks*/
} MMU_CONF_WIDTH;

std::ostream &operator<<(std::ostream &out, const MMU_CONF_WIDTH &c);

/**
    COMPRESSED
    @brief
    - UNCOMPRESSED : 0x0(0)  直接存储
    - COMPRESSED : 0x1(1)  霍夫曼压缩
**/
typedef enum compressed_t : uint8_t
{
    COMPRESSED_UNCOMPRESSED = 0x0, /* 0:直接存储*/
    COMPRESSED_COMPRESSED = 0x1 /* 1:霍夫曼压缩*/
} COMPRESSED;

std::ostream &operator<<(std::ostream &out, const COMPRESSED &c);

/**
    MFU.REDUCE.DIM
    @brief
    - W : 0x0(0)  None
    - HW : 0x1(1)  None
    - CHW : 0x2(2)  None
    - NCHW : 0x3(3)  None
**/
typedef enum mfu_reduce_dim_t : uint8_t
{
    MFU_REDUCE_DIM_W = 0x0, /* 0:None*/
    MFU_REDUCE_DIM_HW = 0x1, /* 1:None*/
    MFU_REDUCE_DIM_CHW = 0x2, /* 2:None*/
    MFU_REDUCE_DIM_NCHW = 0x3 /* 3:None*/
} MFU_REDUCE_DIM;

std::ostream &operator<<(std::ostream &out, const MFU_REDUCE_DIM &c);

typedef enum mfu_mn_op_t : uint8_t
{
    MFU_MN_OP_NULL = 0x0, /* 0:无连接*/
    MFU_MN_OP_VECTOR = 0x1, /* 1:向量*/
    MFU_MN_OP_CONST1 = 0x2, /* 2:常量1*/
    MFU_MN_OP_ADDSUB0 = 0x3, /* 3:加减法0*/
    MFU_MN_OP_ADDSUB1 = 0x4, /* 4:加减法1*/
    MFU_MN_OP_ADDSUB2 = 0x5, /* 5:加减法2*/
    MFU_MN_OP_ADDSUB3 = 0x6, /* 6:加减法3*/
    MFU_MN_OP_MUL0 = 0x7, /* 7:乘法0*/
    MFU_MN_OP_MUL1 = 0x8, /* 8:乘法1*/
    MFU_MN_OP_MUL2 = 0x9, /* 9:乘法2*/
    MFU_MN_OP_MUL3 = 0xa, /* 10:乘法3*/
    MFU_MN_OP_DIV = 0xb, /* 11:除法*/
    MFU_MN_OP_ROUND = 0xc, /* 12:取整*/
    MFU_MN_OP_SQRT = 0xd, /* 13:开方*/
    MFU_MN_OP_TRANGLE = 0xe, /* 14:三角函数*/
    MFU_MN_OP_LOG = 0xf, /* 15:对数函数*/
    MFU_MN_OP_UNARY_LOGIC = 0x10, /* 16:单目逻辑*/
    MFU_MN_OP_EXP = 0x11, /* 17:指数函数*/
    MFU_MN_OP_CMP0 = 0x12, /* 18:比较0*/
    MFU_MN_OP_CMP1 = 0x13, /* 19:比较1*/
    MFU_MN_OP_REG0 = 0x14, /* 20:十六段拟合0*/
    MFU_MN_OP_SELECT0 = 0x15, /* 21:selcet 0，三目运算*/
    MFU_MN_OP_CONST2 = 0x16 /* 22:常数输入2*/
} MFU_MN_OP;

std::ostream &operator<<(std::ostream &out, const MFU_MN_OP &c);

/**
    PRECISION
    @brief
    - INT8 : 0x0(0)  INT8
    - BF16 : 0x1(1)  BF16
    - FP32 : 0x2(2)  FP32
**/
typedef enum precision_t : uint8_t
{
    PRECISION_INT8 = 0x0, /* 0:INT8*/
    PRECISION_BF16 = 0x1, /* 1:BF16*/
    PRECISION_FP32 = 0x2 /* 2:FP32*/
} PRECISION;

std::ostream &operator<<(std::ostream &out, const PRECISION &c);

/**
    QUAN_TYPE
    @brief
    - BY_CHANNEL : 0x0(0)  分通道量化
    - BY_BATCH : 0x1(1)  分batch量化
**/
typedef enum quan_type_t : uint8_t
{
    QUAN_TYPE_BY_CHANNEL = 0x0, /* 0:分通道量化*/
    QUAN_TYPE_BY_BATCH = 0x1 /* 1:分batch量化*/
} QUAN_TYPE;

std::ostream &operator<<(std::ostream &out, const QUAN_TYPE &c);

/**
    MFU.TRANS.PERMUTE
    @brief
    - NCHW : 0x0(0)  重要
    - NCWH : 0x1(1)  重要
    - NHCW : 0x2(2)  重要
    - NHWC : 0x3(3)  重要
    - NWCH : 0x4(4)  None
    - NWHC : 0x5(5)  None
    - CNHW : 0x6(6)  None
    - CNWH : 0x7(7)  None
    - CHNW : 0x8(8)  None
    - CHWN : 0x9(9)  None
    - CWNH : 0xa(10)  None
    - CWHN : 0xb(11)  None
    - HNCW : 0xc(12)  None
    - HNWC : 0xd(13)  None
    - HCNW : 0xe(14)  None
    - HCWN : 0xf(15)  None
    - HWNC : 0x10(16)  None
    - HWCN : 0x11(17)  None
    - WNCH : 0x12(18)  None
    - WNHC : 0x13(19)  None
    - WCNH : 0x14(20)  None
    - WCHN : 0x15(21)  None
    - WHNC : 0x16(22)  None
    - WHCN : 0x17(23)  None
**/
typedef enum mfu_trans_permute_t : uint8_t
{
    MFU_TRANS_PERMUTE_NCHW = 0x0, /* 0:重要*/
    MFU_TRANS_PERMUTE_NCWH = 0x1, /* 1:重要*/
    MFU_TRANS_PERMUTE_NHCW = 0x2, /* 2:重要*/
    MFU_TRANS_PERMUTE_NHWC = 0x3, /* 3:重要*/
    MFU_TRANS_PERMUTE_NWCH = 0x4, /* 4:None*/
    MFU_TRANS_PERMUTE_NWHC = 0x5, /* 5:None*/
    MFU_TRANS_PERMUTE_CNHW = 0x6, /* 6:None*/
    MFU_TRANS_PERMUTE_CNWH = 0x7, /* 7:None*/
    MFU_TRANS_PERMUTE_CHNW = 0x8, /* 8:None*/
    MFU_TRANS_PERMUTE_CHWN = 0x9, /* 9:None*/
    MFU_TRANS_PERMUTE_CWNH = 0xa, /* 10:None*/
    MFU_TRANS_PERMUTE_CWHN = 0xb, /* 11:None*/
    MFU_TRANS_PERMUTE_HNCW = 0xc, /* 12:None*/
    MFU_TRANS_PERMUTE_HNWC = 0xd, /* 13:None*/
    MFU_TRANS_PERMUTE_HCNW = 0xe, /* 14:None*/
    MFU_TRANS_PERMUTE_HCWN = 0xf, /* 15:None*/
    MFU_TRANS_PERMUTE_HWNC = 0x10, /* 16:None*/
    MFU_TRANS_PERMUTE_HWCN = 0x11, /* 17:None*/
    MFU_TRANS_PERMUTE_WNCH = 0x12, /* 18:None*/
    MFU_TRANS_PERMUTE_WNHC = 0x13, /* 19:None*/
    MFU_TRANS_PERMUTE_WCNH = 0x14, /* 20:None*/
    MFU_TRANS_PERMUTE_WCHN = 0x15, /* 21:None*/
    MFU_TRANS_PERMUTE_WHNC = 0x16, /* 22:None*/
    MFU_TRANS_PERMUTE_WHCN = 0x17 /* 23:None*/
} MFU_TRANS_PERMUTE;

std::ostream &operator<<(std::ostream &out, const MFU_TRANS_PERMUTE &c);

/**
    STORE.ORDER
    @brief
    - NCHW : 0x0(0)  None
    - NHCW : 0x1(1)  None
**/
typedef enum store_order_t : uint8_t
{
    STORE_ORDER_NCHW = 0x0, /* 0:None*/
    STORE_ORDER_NHCW = 0x1 /* 1:None*/
} STORE_ORDER;

std::ostream &operator<<(std::ostream &out, const STORE_ORDER &c);

/**
    MFU.REDUCE.OP
    @brief
    - MAX : 0x0(0)  两者取最大值
    - MIN : 0x1(1)  两者取最小值
    - ADD : 0x2(2)  两者求和
    - SUB : 0x3(3)  左操作数减去右操作数
    - MUL : 0x4(4)  两者求其点积
**/
typedef enum mfu_reduce_op_t : uint8_t
{
    MFU_REDUCE_OP_MAX = 0x0, /* 0:两者取最大值*/
    MFU_REDUCE_OP_MIN = 0x1, /* 1:两者取最小值*/
    MFU_REDUCE_OP_ADD = 0x2, /* 2:两者求和*/
    MFU_REDUCE_OP_SUB = 0x3, /* 3:左操作数减去右操作数*/
    MFU_REDUCE_OP_MUL = 0x4 /* 4:两者求其点积*/
} MFU_REDUCE_OP;

std::ostream &operator<<(std::ostream &out, const MFU_REDUCE_OP &c);

/**
    ALIGNED
    @brief
    - DENSE : 0x0(0)  紧密存储
    - ALIGNED : 0x1(1)  对齐到64bit
**/
typedef enum aligned_t : uint8_t
{
    ALIGNED_DENSE = 0x0, /* 0:紧密存储*/
    ALIGNED_ALIGNED = 0x1 /* 1:对齐到64bit*/
} ALIGNED;

std::ostream &operator<<(std::ostream &out, const ALIGNED &c);

/**
    PRECISION_DDR
    @brief
    - INT8 : 0x0(0)  INT8
    - BF16 : 0x1(1)  BF16
    - FP32 : 0x2(2)  FP32
    - INT4 : 0x3(3)  INT4
    - INT6 : 0x4(4)  INT6
**/
typedef enum precision_ddr_t : uint8_t
{
    PRECISION_DDR_INT8 = 0x0, /* 0:INT8*/
    PRECISION_DDR_BF16 = 0x1, /* 1:BF16*/
    PRECISION_DDR_FP32 = 0x2, /* 2:FP32*/
    PRECISION_DDR_INT4 = 0x3, /* 3:INT4*/
    PRECISION_DDR_INT6 = 0x4 /* 4:INT6*/
} PRECISION_DDR;

std::ostream &operator<<(std::ostream &out, const PRECISION_DDR &c);

typedef enum mfu_mn_portin_t : uint8_t
{
    MFU_MN_PORTIN_DUMMY = 0x0,
    MFU_MN_PORTIN_VECTOR_OUT_0 = 0x1,
    MFU_MN_PORTIN_VECTOR_OUT_1 = 0x2,
    MFU_MN_PORTIN_CONST1_OUT_0 = 0x3,
    MFU_MN_PORTIN_CONST1_OUT_1 = 0x4,
    MFU_MN_PORTIN_CONST2_OUT_0 = 0x5,
    MFU_MN_PORTIN_CONST2_OUT_1 = 0x6,
    MFU_MN_PORTIN_ADDSUB0_OUT_0 = 0x7,
    MFU_MN_PORTIN_ADDSUB1_OUT_0 = 0x8,
    MFU_MN_PORTIN_ADDSUB2_OUT_0 = 0x9,
    MFU_MN_PORTIN_ADDSUB3_OUT_0 = 0xa,
    MFU_MN_PORTIN_MUL0_OUT_0 = 0xb,
    MFU_MN_PORTIN_MUL1_OUT_0 = 0xc,
    MFU_MN_PORTIN_MUL2_OUT_0 = 0xd,
    MFU_MN_PORTIN_MUL3_OUT_0 = 0xe,
    MFU_MN_PORTIN_DIV_OUT_0 = 0xf,
    MFU_MN_PORTIN_ROUND_OUT_0 = 0x10,
    MFU_MN_PORTIN_SQRT_OUT_0 = 0x11,
    MFU_MN_PORTIN_TRANGLE_OUT_0 = 0x12,
    MFU_MN_PORTIN_LOG_OUT_0 = 0x13,
    MFU_MN_PORTIN_UNARY_LOGIC_OUT_0 = 0x14,
    MFU_MN_PORTIN_EXP_OUT_0 = 0x15,
    MFU_MN_PORTIN_CMP0_OUT_0 = 0x16,
    MFU_MN_PORTIN_CMP1_OUT_0 = 0x17,
    MFU_MN_PORTIN_REG0_OUT_0 = 0x18,
    MFU_MN_PORTIN_SELECT0_OUT_0 = 0x19,
    MFU_MN_PORTIN_NOP0_OUT_0 = 0x1a,
    MFU_MN_PORTIN_NOP0_OUT_1 = 0x1b,
    MFU_MN_PORTIN_NOP0_OUT_2 = 0x1c,
    MFU_MN_PORTIN_NOP0_OUT_3 = 0x1d,
    MFU_MN_PORTIN_NOP0_OUT_4 = 0x1e,
    MFU_MN_PORTIN_NOP0_OUT_5 = 0x1f,
    MFU_MN_PORTIN_NOP0_OUT_6 = 0x20,
    MFU_MN_PORTIN_NOP0_OUT_7 = 0x21
} MFU_MN_PORTIN;

std::ostream &operator<<(std::ostream &out, const MFU_MN_PORTIN &c);

/**
    MFU.PDP.OP
    @brief
    - MIN : 0x1(1)  取最小值
    - MAX : 0x0(0)  取最大值
    - AVERAGE : 0x2(2)  取平均值
    - SUM : 0x3(3)  求和
**/
typedef enum mfu_pdp_op_t : uint8_t
{
    MFU_PDP_OP_MIN = 0x1, /* 1:取最小值*/
    MFU_PDP_OP_MAX = 0x0, /* 0:取最大值*/
    MFU_PDP_OP_AVERAGE = 0x2, /* 2:取平均值*/
    MFU_PDP_OP_SUM = 0x3 /* 3:求和*/
} MFU_PDP_OP;

std::ostream &operator<<(std::ostream &out, const MFU_PDP_OP &c);

/**
    LOAD.ORDER
    @brief
    - NCHW : 0x0(0)  None
    - RGBA : 0x1(1)  None
    - RBGA : 0x2(2)  None
    - GRBA : 0x3(3)  None
    - GBRA : 0x4(4)  None
    - BRGA : 0x5(5)  None
    - BGRA : 0x6(6)  None
    - ARGB : 0x7(7)  None
    - ARBG : 0x8(8)  None
    - AGRB : 0x9(9)  None
    - AGBR : 0xa(10)  None
    - ABRG : 0xb(11)  None
    - ABGR : 0xc(12)  None
    - RGB : 0xd(13)  None
    - RBG : 0xe(14)  None
    - GRB : 0xf(15)  None
    - GBR : 0x10(16)  None
    - BRG : 0x11(17)  None
    - BGR : 0x12(18)  None
**/
typedef enum load_order_t : uint8_t
{
    LOAD_ORDER_NCHW = 0x0, /* 0:None*/
    LOAD_ORDER_RGBA = 0x1, /* 1:None*/
    LOAD_ORDER_RBGA = 0x2, /* 2:None*/
    LOAD_ORDER_GRBA = 0x3, /* 3:None*/
    LOAD_ORDER_GBRA = 0x4, /* 4:None*/
    LOAD_ORDER_BRGA = 0x5, /* 5:None*/
    LOAD_ORDER_BGRA = 0x6, /* 6:None*/
    LOAD_ORDER_ARGB = 0x7, /* 7:None*/
    LOAD_ORDER_ARBG = 0x8, /* 8:None*/
    LOAD_ORDER_AGRB = 0x9, /* 9:None*/
    LOAD_ORDER_AGBR = 0xa, /* 10:None*/
    LOAD_ORDER_ABRG = 0xb, /* 11:None*/
    LOAD_ORDER_ABGR = 0xc, /* 12:None*/
    LOAD_ORDER_RGB = 0xd, /* 13:None*/
    LOAD_ORDER_RBG = 0xe, /* 14:None*/
    LOAD_ORDER_GRB = 0xf, /* 15:None*/
    LOAD_ORDER_GBR = 0x10, /* 16:None*/
    LOAD_ORDER_BRG = 0x11, /* 17:None*/
    LOAD_ORDER_BGR = 0x12 /* 18:None*/
} LOAD_ORDER;

std::ostream &operator<<(std::ostream &out, const LOAD_ORDER &c);

/**
    QUAN.SIGNED
    @brief
    - UNSIGNED : 0x0(0)  无符号量化
    - SIGNED : 0x1(1)  有符号量化
**/
typedef enum quan_signed_t : uint8_t
{
    QUAN_SIGNED_UNSIGNED = 0x0, /* 0:无符号量化*/
    QUAN_SIGNED_SIGNED = 0x1 /* 1:有符号量化*/
} QUAN_SIGNED;

std::ostream &operator<<(std::ostream &out, const QUAN_SIGNED &c);

/**
    SPARSIFIED
    @brief
    - DENSE : 0x0(0)  直接存储
    - SPARSIFIED : 0x1(1)  对0进行压缩编码
**/
typedef enum sparsified_t : uint8_t
{
    SPARSIFIED_DENSE = 0x0, /* 0:直接存储*/
    SPARSIFIED_SPARSIFIED = 0x1 /* 1:对0进行压缩编码*/
} SPARSIFIED;

std::ostream &operator<<(std::ostream &out, const SPARSIFIED &c);

typedef enum mfu_mn_portout_t : uint8_t
{
    MFU_MN_PORTOUT_DUMMY = 0x0,
    MFU_MN_PORTOUT_VECTOR_IN_0 = 0x1,
    MFU_MN_PORTOUT_ADDSUB0_IN_0 = 0x2,
    MFU_MN_PORTOUT_ADDSUB0_IN_1 = 0x3,
    MFU_MN_PORTOUT_ADDSUB1_IN_0 = 0x4,
    MFU_MN_PORTOUT_ADDSUB1_IN_1 = 0x5,
    MFU_MN_PORTOUT_ADDSUB2_IN_0 = 0x6,
    MFU_MN_PORTOUT_ADDSUB2_IN_1 = 0x7,
    MFU_MN_PORTOUT_ADDSUB3_IN_0 = 0x8,
    MFU_MN_PORTOUT_ADDSUB3_IN_1 = 0x9,
    MFU_MN_PORTOUT_MUL0_IN_0 = 0xa,
    MFU_MN_PORTOUT_MUL0_IN_1 = 0xb,
    MFU_MN_PORTOUT_MUL1_IN_0 = 0xc,
    MFU_MN_PORTOUT_MUL1_IN_1 = 0xd,
    MFU_MN_PORTOUT_MUL2_IN_0 = 0xe,
    MFU_MN_PORTOUT_MUL2_IN_1 = 0xf,
    MFU_MN_PORTOUT_MUL3_IN_0 = 0x10,
    MFU_MN_PORTOUT_MUL3_IN_1 = 0x11,
    MFU_MN_PORTOUT_DIV_IN_0 = 0x12,
    MFU_MN_PORTOUT_DIV_IN_1 = 0x13,
    MFU_MN_PORTOUT_ROUND_IN_0 = 0x14,
    MFU_MN_PORTOUT_SQRT_IN_0 = 0x15,
    MFU_MN_PORTOUT_TRANGLE_IN_0 = 0x16,
    MFU_MN_PORTOUT_LOG_IN_0 = 0x17,
    MFU_MN_PORTOUT_UNARY_LOGIC_IN_0 = 0x18,
    MFU_MN_PORTOUT_EXP_IN_0 = 0x19,
    MFU_MN_PORTOUT_CMP0_IN_0 = 0x1a,
    MFU_MN_PORTOUT_CMP0_IN_1 = 0x1b,
    MFU_MN_PORTOUT_CMP1_IN_0 = 0x1c,
    MFU_MN_PORTOUT_CMP1_IN_1 = 0x1d,
    MFU_MN_PORTOUT_REG0_IN_0 = 0x1e,
    MFU_MN_PORTOUT_SELECT0_IN_0 = 0x1f,
    MFU_MN_PORTOUT_SELECT0_IN_1 = 0x20,
    MFU_MN_PORTOUT_SELECT0_IN_2 = 0x21,
    MFU_MN_PORTOUT_NOP0_IN_0 = 0x22,
    MFU_MN_PORTOUT_NOP1_IN_0 = 0x23,
    MFU_MN_PORTOUT_NOP2_IN_0 = 0x24,
    MFU_MN_PORTOUT_NOP3_IN_0 = 0x25,
    MFU_MN_PORTOUT_NOP4_IN_0 = 0x26,
    MFU_MN_PORTOUT_NOP5_IN_0 = 0x27,
    MFU_MN_PORTOUT_NOP6_IN_0 = 0x28,
    MFU_MN_PORTOUT_NOP7_IN_0 = 0x29,
    MFU_MN_PORTOUT_CLEAR_IN_0 = 0x2a,
} MFU_MN_PORTOUT;

std::ostream &operator<<(std::ostream &out, const MFU_MN_PORTOUT &c);

/**
    MFU.CROP.RESIZE
    @brief
    - biliner : 0x0(0)  双线性插值
    - nearest : 0x1(1)  最邻近插值
**/
typedef enum mfu_crop_resize_t : uint8_t
{
    MFU_CROP_RESIZE_BILINER = 0x0, /* 0:双线性插值*/
    MFU_CROP_RESIZE_NEAREST = 0x1 /* 1:最邻近插值*/
} MFU_CROP_RESIZE;

std::ostream &operator<<(std::ostream &out, const MFU_CROP_RESIZE &c);

/**
    TCU_MODE
    @brief
    - CONV : 0x0(0)  CONV
    - DEPTHWISECONV : 0x1(1)  DEPTHWISECONV
    - DECONV : 0x2(2)  DECONV
    - MATMUL : 0x3(3)  MATMUL
**/
typedef enum tcu_mode_t : uint8_t
{
    TCU_MODE_CONV = 0x0, /* 0:CONV*/
    TCU_MODE_DEPTHWISECONV = 0x1, /* 1:DEPTHWISECONV*/
    TCU_MODE_DECONV = 0x2, /* 2:DECONV*/
    TCU_MODE_MATMUL = 0x3 /* 3:MATMUL*/
} TCU_MODE;

std::ostream &operator<<(std::ostream &out, const TCU_MODE &c);

/**
    OPCODE
    @brief
    - NOP : 0x0(0)  NOP
    - LI : 0x1(1)  LI
    - INTR : 0x2(2)  INTR
    - END : 0x3(3)  END
    - FENCE : 0x4(4)  FENCE
    - MMU.CONF : 0x5(5)  MMU.CONF
    - FENCE.CCR : 0x8(8)  FENCE.CCR
    - LOADIF.CONFIG : 0x10(16)  LOADIF.CONFIG
    - LOADIF : 0x11(17)  LOADIF
    - LOAD : 0x12(18)  LOAD
    - LOAD.R : 0x13(19)  LOAD.R
    - LOADIF.COMPRESS.CONF : 0x14(20)  LOADIF.COMPRESS.CONF
    - LOAD.COMPRESS.CONF : 0x15(21)  LOAD.COMPRESS.CONF
    - STORE : 0x20(32)  STORE
    - STORE.T.CONFIG : 0x21(33)  STORE.T.CONFIG
    - STORE.T : 0x22(34)  STORE.T
    - STORE.R : 0x23(35)  STORE.R
    - STORE.T.COMPRESS.CONF : 0x24(36)  STORE.T.COMPRESS.CONF
    - STORE.COMPRESS.CONF : 0x25(37)  STORE.COMPRESS.CONF
    - TCU.DM.BROADCAST : 0x41(65)  TCU.DM.BROADCAST
    - TCU.DM.CONF.IF : 0x42(66)  TCU.DM.CONF.IF
    - TCU.DM.FETCHIF : 0x43(67)  TCU.DM.FETCHIF
    - TCU.DM.CONF.W : 0x44(68)  TCU.DM.CONF.W
    - TCU.DM.FETCHW : 0x45(69)  TCU.DM.FETCHW
    - TCU.DM.CONF.OF : 0x46(70)  TCU.DM.CONF.OF
    - TCU.PU.CONF : 0x47(71)  TCU.PU.CONF
    - TCU.PU.CONF.ACT : 0x48(72)  TCU.PU.CONF.ACT
    - TCU.PU.COMPUTE : 0x49(73)  TCU.PU.COMPUTE
    - TCU.DOT.DM.IF.CONF : 0x4a(74)  TCU.DOT.DM.IF.CONF
    - TCU.DOT.DM.OF.CONF : 0x4b(75)  TCU.DOT.DM.OF.CONF
    - TCU.DOT.DM.FETCH.SRC1 : 0x4c(76)  TCU.DOT.DM.FETCH.SRC1
    - TCU.DOT.DM.FETCH.SRC2 : 0x4d(77)  TCU.DOT.DM.FETCH.SRC2
    - TCU.DM.QARG.CONF : 0x4e(78)  TCU.DM.QARG.CONF
    - TCU.PU.COMPUTE.DUMMY : 0x4f(79)  TCU.PU.COMPUTE.DUMMY
    - MFU.MN.MAP.COMPUTE : 0x81(129)  MFU.MN.MAP.COMPUTE
    - MFU.MN.VMAP.COMPUTE : 0x82(130)  MFU.MN.VMAP.COMPUTE
    - MFU.REDUCE : 0x83(131)  MFU.REDUCE
    - MFU.VREDUCE : 0x84(132)  MFU.VREDUCE
    - MFU.MN.BROADCAST.COMPUTE : 0x85(133)  MFU.MN.BROADCAST.COMPUTE
    - MFU.MN.REDUCE : 0x86(134)  MFU.MN.REDUCE
    - MFU.MN.CONF : 0x87(135)  MFU.MN.CONF
    - MFU.MNOP.CONF : 0x88(136)  MFU.MNOP.CONF
    - MFU.PDP.CONF : 0x89(137)  MFU.PDP.CONF
    - MFU.PDP.SRC.CONF : 0x8a(138)  MFU.PDP.SRC.CONF
    - MFU.PDP.REDUCE : 0x8b(139)  MFU.PDP.REDUCE
    - MFU.MN.BROADCAST.CONF : 0x8c(140)  MFU.MN.BROADCAST.CONF
    - MFU.CROP : 0x8d(141)  MFU.CROP
    - MFU.MEMSET : 0x8e(142)  MFU.MEMSET
    - MFU.MEMCPY : 0x8f(143)  MFU.MEMCPY
    - MFU.TRANS : 0x90(144)  MFU.TRANS
    - MFU.MN.CONF2 : 0x91(145)  MFU.MN.CONF2
**/
typedef enum opcode_t : uint8_t
{
    OPCODE_NOP = 0x0, /* 0:NOP*/
    OPCODE_LI = 0x1, /* 1:LI*/
    OPCODE_INTR = 0x2, /* 2:INTR*/
    OPCODE_END = 0x3, /* 3:END*/
    OPCODE_FENCE = 0x4, /* 4:FENCE*/
    OPCODE_MMU_CONF = 0x5, /* 5:MMU.CONF*/
    OPCODE_FENCE_CCR = 0x8, /* 8:FENCE.CCR*/
    OPCODE_LOADIF_CONFIG = 0x10, /* 16:LOADIF.CONFIG*/
    OPCODE_LOADIF = 0x11, /* 17:LOADIF*/
    OPCODE_LOAD = 0x12, /* 18:LOAD*/
    OPCODE_LOAD_R = 0x13, /* 19:LOAD.R*/
    OPCODE_LOADIF_COMPRESS_CONF = 0x14, /* 20:LOADIF.COMPRESS.CONF*/
    OPCODE_LOAD_COMPRESS_CONF = 0x15, /* 21:LOAD.COMPRESS.CONF*/
    OPCODE_STORE = 0x20, /* 32:STORE*/
    OPCODE_STORE_T_CONFIG = 0x21, /* 33:STORE.T.CONFIG*/
    OPCODE_STORE_T = 0x22, /* 34:STORE.T*/
    OPCODE_STORE_R = 0x23, /* 35:STORE.R*/
    OPCODE_STORE_T_COMPRESS_CONF = 0x24, /* 36:STORE.T.COMPRESS.CONF*/
    OPCODE_STORE_COMPRESS_CONF = 0x25, /* 37:STORE.COMPRESS.CONF*/
    OPCODE_TCU_DM_BROADCAST = 0x41, /* 65:TCU.DM.BROADCAST*/
    OPCODE_TCU_DM_CONF_IF = 0x42, /* 66:TCU.DM.CONF.IF*/
    OPCODE_TCU_DM_FETCHIF = 0x43, /* 67:TCU.DM.FETCHIF*/
    OPCODE_TCU_DM_CONF_W = 0x44, /* 68:TCU.DM.CONF.W*/
    OPCODE_TCU_DM_FETCHW = 0x45, /* 69:TCU.DM.FETCHW*/
    OPCODE_TCU_DM_CONF_OF = 0x46, /* 70:TCU.DM.CONF.OF*/
    OPCODE_TCU_PU_CONF = 0x47, /* 71:TCU.PU.CONF*/
    OPCODE_TCU_PU_CONF_ACT = 0x48, /* 72:TCU.PU.CONF.ACT*/
    OPCODE_TCU_PU_COMPUTE = 0x49, /* 73:TCU.PU.COMPUTE*/
    OPCODE_TCU_DOT_DM_IF_CONF = 0x4a, /* 74:TCU.DOT.DM.IF.CONF*/
    OPCODE_TCU_DOT_DM_OF_CONF = 0x4b, /* 75:TCU.DOT.DM.OF.CONF*/
    OPCODE_TCU_DOT_DM_FETCH_SRC1 = 0x4c, /* 76:TCU.DOT.DM.FETCH.SRC1*/
    OPCODE_TCU_DOT_DM_FETCH_SRC2 = 0x4d, /* 77:TCU.DOT.DM.FETCH.SRC2*/
    OPCODE_TCU_DM_QARG_CONF = 0x4e, /* 78:TCU.DM.QARG.CONF*/
    OPCODE_TCU_PU_COMPUTE_DUMMY = 0x4f, /* 79:TCU.PU.COMPUTE.DUMMY*/
    OPCODE_MFU_MN_MAP_COMPUTE = 0x81, /* 129:MFU.MN.MAP.COMPUTE*/
    OPCODE_MFU_MN_VMAP_COMPUTE = 0x82, /* 130:MFU.MN.VMAP.COMPUTE*/
    OPCODE_MFU_REDUCE = 0x83, /* 131:MFU.REDUCE*/
    OPCODE_MFU_VREDUCE = 0x84, /* 132:MFU.VREDUCE*/
    OPCODE_MFU_MN_BROADCAST_COMPUTE = 0x85, /* 133:MFU.MN.BROADCAST.COMPUTE*/
    OPCODE_MFU_MN_REDUCE = 0x86, /* 134:MFU.MN.REDUCE*/
    OPCODE_MFU_MN_CONF = 0x87, /* 135:MFU.MN.CONF*/
    OPCODE_MFU_MNOP_CONF = 0x88, /* 136:MFU.MNOP.CONF*/
    OPCODE_MFU_PDP_CONF = 0x89, /* 137:MFU.PDP.CONF*/
    OPCODE_MFU_PDP_SRC_CONF = 0x8a, /* 138:MFU.PDP.SRC.CONF*/
    OPCODE_MFU_PDP_REDUCE = 0x8b, /* 139:MFU.PDP.REDUCE*/
    OPCODE_MFU_MN_BROADCAST_CONF = 0x8c, /* 140:MFU.MN.BROADCAST.CONF*/
    OPCODE_MFU_CROP = 0x8d, /* 141:MFU.CROP*/
    OPCODE_MFU_MEMSET = 0x8e, /* 142:MFU.MEMSET*/
    OPCODE_MFU_MEMCPY = 0x8f, /* 143:MFU.MEMCPY*/
    OPCODE_MFU_TRANS = 0x90, /* 144:MFU.TRANS*/
    OPCODE_MFU_MN_CONF2 = 0x91 /* 145:MFU.MN.CONF2*/
} OPCODE;

std::ostream &operator<<(std::ostream &out, const OPCODE &c);

/**
    MFU.CROP.ALIGN
    @brief
    - none : 0x0(0)  tensorflow align_corner=false
    - corner : 0x1(1)  tensorflow align_corner=True
    - center : 0x2(2)  open cv，中心对齐
**/
typedef enum mfu_crop_align_t : uint8_t
{
    MFU_CROP_ALIGN_NONE = 0x0, /* 0:tensorflow align_corner=false*/
    MFU_CROP_ALIGN_CORNER = 0x1, /* 1:tensorflow align_corner=True*/
    MFU_CROP_ALIGN_CENTER = 0x2 /* 2:open cv，中心对齐*/
} MFU_CROP_ALIGN;

std::ostream &operator<<(std::ostream &out, const MFU_CROP_ALIGN &c);

/**
    BROADCAST
    @brief
    - SINGLE : 0x0(0)  单播
    - BROADCAST : 0x1(1)  广播
**/
typedef enum broadcast_t : uint8_t
{
    BROADCAST_SINGLE = 0x0, /* 0:单播*/
    BROADCAST_BROADCAST = 0x1 /* 1:广播*/
} BROADCAST;

std::ostream &operator<<(std::ostream &out, const BROADCAST &c);

/////////////////////////////////////
// auto generated inst pack
/**
    NOP
    @brief 无操作
    - OPCODE opcode : 8  无操作
**/
struct INST_NOP
{
    OPCODE opcode : 8; /** 无操作 **/

    void serialize(bitwriter &bw) const { bw.write<8>(opcode); }

    void deserialize(bitreader &br) { opcode = br.read<decltype(opcode), 8>(); }
};

/**
    LI
    @brief Load 一个 64bit 的立即数到指定的寄存器
    - OPCODE opcode : 8  Load 一个 64bit 的立即数到指定的寄存器
    - REG regid : 5  寄存器编号。
这个寄存器的编号只能编码32个寄存器区域。
{== TODO：是否要编码其他内容 ==}
    - uint64_t imm : 64  立即数
**/
struct INST_LI
{
    OPCODE opcode : 8; /** Load 一个 64bit 的立即数到指定的寄存器 **/
    REG regid : 5; /** 寄存器编号。
这个寄存器的编号只能编码32个寄存器区域。
{== TODO：是否要编码其他内容 ==} **/
    uint64_t imm : 64; /** 立即数 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<5>(regid);
        bw.write<64>(imm);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        regid = br.read<decltype(regid), 5>();
        imm = br.read<decltype(imm), 64>();
    }
};

/**
    INTR
    @brief 触发向DSP的软中断
    - OPCODE opcode : 8  触发向DSP的软中断
    - uint64_t intr_number : 32  中断号，用于设置inptr寄存器
    - uint64_t regmask : 32  标记哪些寄存器被用于传参，用于设置regmask寄存器
    - uint64_t ptrmask : 32
标记寄存器里的内容是指针类型还是数据，用于设置ptrmask寄存器
**/
struct INST_INTR
{
    OPCODE opcode : 8; /** 触发向DSP的软中断 **/
    uint64_t intr_number : 32; /** 中断号，用于设置inptr寄存器 **/
    uint64_t regmask : 32; /** 标记哪些寄存器被用于传参，用于设置regmask寄存器 **/
    uint64_t ptrmask : 32; /**
                            标记寄存器里的内容是指针类型还是数据，用于设置ptrmask寄存器
                            **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<32>(intr_number);
        bw.write<32>(regmask);
        bw.write<32>(ptrmask);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        intr_number = br.read<decltype(intr_number), 32>();
        regmask = br.read<decltype(regmask), 32>();
        ptrmask = br.read<decltype(ptrmask), 32>();
    }
};

/**
    END
    @brief 表示GNNE程序执行结束，GNNE向总中断控制器发起中断
    - OPCODE opcode : 8  表示GNNE程序执行结束，GNNE向总中断控制器发起中断
    - uint64_t intr_number : 32  中断号，用于设置inptr寄存器
**/
struct INST_END
{
    OPCODE opcode : 8; /** 表示GNNE程序执行结束，GNNE向总中断控制器发起中断 **/
    uint64_t intr_number : 32; /** 中断号，用于设置inptr寄存器 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<32>(intr_number);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        intr_number = br.read<decltype(intr_number), 32>();
    }
};

/**
    FENCE
    @brief 设置内存屏障。在屏障前面的指令一定都执行完成，后面的指令才会被执行
    - OPCODE opcode : 8
设置内存屏障。在屏障前面的指令一定都执行完成，后面的指令才会被执行
**/
struct INST_FENCE
{
    OPCODE opcode : 8; /**
                        设置内存屏障。在屏障前面的指令一定都执行完成，后面的指令才会被执行
                        **/

    void serialize(bitwriter &bw) const { bw.write<8>(opcode); }

    void deserialize(bitreader &br) { opcode = br.read<decltype(opcode), 8>(); }
};

/**
    MMU.CONF
    @brief 配置某条MMU规则
    - OPCODE opcode : 8  配置某条MMU规则
    - uint64_t mmu_item : 4  当前配置的MMU规则项编号，0-15
    - uint64_t start_bank : 3  当前规则开始的bank
    - MMU_CONF_WIDTH width : 2  当前配置宽度信息
    - uint64_t start_depth : 14  当前规则开始的纵坐标
    - uint64_t depth : 14
当前规则深度。这个值是相对与开始深度的差值，注意是深度而不是结束深度。
**/
struct INST_MMU_CONF
{
    OPCODE opcode : 8; /** 配置某条MMU规则 **/
    uint64_t mmu_item : 4; /** 当前配置的MMU规则项编号，0-15 **/
    uint64_t start_bank : 3; /** 当前规则开始的bank **/
    MMU_CONF_WIDTH width : 2; /** 当前配置宽度信息 **/
    uint64_t start_depth : 14; /** 当前规则开始的纵坐标 **/
    uint64_t depth : 14; /**
                                当前规则深度。这个值是相对与开始深度的差值，注意是深度而不是结束深度。
                                **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<4>(mmu_item);
        bw.write<3>(start_bank);
        bw.write<2>(width);
        bw.write<14>(start_depth);
        bw.write<14>(depth);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        mmu_item = br.read<decltype(mmu_item), 4>();
        start_bank = br.read<decltype(start_bank), 3>();
        width = br.read<decltype(width), 2>();
        start_depth = br.read<decltype(start_depth), 14>();
        depth = br.read<decltype(depth), 14>();
    }
};

/**
    FENCE.CCR
    @brief
CCR屏障，用来阻挡使用到本CCR的指令的分发，可以用来实现给CCR重新分配空间等作用
    - OPCODE opcode : 8
CCR屏障，用来阻挡使用到本CCR的指令的分发，可以用来实现给CCR重新分配空间等作用
    - uint64_t ccr : 6  CCR寄存器id
    - uint64_t mode : 1  0: wait ccr is 0
1: wait ccr is not 0 and clear ccr
**/
struct INST_FENCE_CCR
{
    OPCODE opcode : 8; /**
                        CCR屏障，用来阻挡使用到本CCR的指令的分发，可以用来实现给CCR重新分配空间等作用
                        **/
    uint64_t ccr : 6; /** CCR寄存器id **/
    uint64_t mode : 1; /** 0: wait ccr is 0
1: wait ccr is not 0 and clear ccr **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<6>(ccr);
        bw.write<1>(mode);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccr = br.read<decltype(ccr), 6>();
        mode = br.read<decltype(mode), 1>();
    }
};

/**
    LOADIF.CONFIG
    @brief 用于配置从 DDR 中加载 input featuremap 的一些基础参数
    - OPCODE opcode : 8  用于配置从 DDR 中加载 input featuremap 的一些基础参数
    - uint64_t layout_ddr_n : 16  输入 Tensor 的尺寸信息，用于计算输入地址跳转
    - uint64_t layout_ddr_c : 16  输入 Tensor 的尺寸信息，用于计算输入地址跳转
    - uint64_t layout_ddr_h : 16  输入 Tensor 的尺寸信息，用于计算输入地址跳转
    - uint64_t layout_ddr_w : 16  输入 Tensor 的尺寸信息，用于计算输入地址跳转
    - STRIDE_GLB stride_glb : 64  输出 Tensor 的尺寸信息，用于计算输出地址跳转
    - uint64_t mmu_item : 4  使用的内存映射方案
    - ADDR_GLB_8_WITH_BANK addr_qarg : 25  量化参数地址
    - QUAN_SIGNED input_signed : 1  量化数据是否是有符号数据
    - PRECISION precision_glb : 2  GLB中数据的类型
    - PRECISION_DDR precision_ddr : 3  DDR 中的数据类型
    - QUAN_TYPE quan_type : 1  量化类型，bychennel or by batch
**/
struct INST_LOADIF_CONFIG
{
    OPCODE
    opcode : 8; /** 用于配置从 DDR 中加载 input featuremap 的一些基础参数 **/
    uint64_t
        layout_ddr_n : 16; /** 输入 Tensor 的尺寸信息，用于计算输入地址跳转 **/
    uint64_t
        layout_ddr_c : 16; /** 输入 Tensor 的尺寸信息，用于计算输入地址跳转 **/
    uint64_t
        layout_ddr_h : 16; /** 输入 Tensor 的尺寸信息，用于计算输入地址跳转 **/
    uint64_t
        layout_ddr_w : 16; /** 输入 Tensor 的尺寸信息，用于计算输入地址跳转 **/
    uint64_t stride_glb : 64; /** 输出 Tensor 的尺寸信息，用于计算输出地址跳转 **/
    uint64_t mmu_item : 4; /** 使用的内存映射方案 **/
    uint64_t addr_qarg : 25; /** 量化参数地址 **/
    QUAN_SIGNED input_signed : 1; /** 量化数据是否是有符号数据 **/
    PRECISION precision_glb : 2; /** GLB中数据的类型  **/
    PRECISION_DDR precision_ddr : 3; /** DDR 中的数据类型 **/
    QUAN_TYPE quan_type : 1; /** 量化类型，bychennel or by batch **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<16>(layout_ddr_n);
        bw.write<16>(layout_ddr_c);
        bw.write<16>(layout_ddr_h);
        bw.write<16>(layout_ddr_w);
        bw.write<64>(stride_glb);
        bw.write<4>(mmu_item);
        bw.write<25>(addr_qarg);
        bw.write<1>(input_signed);
        bw.write<2>(precision_glb);
        bw.write<3>(precision_ddr);
        bw.write<1>(quan_type);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        layout_ddr_n = br.read<decltype(layout_ddr_n), 16>();
        layout_ddr_c = br.read<decltype(layout_ddr_c), 16>();
        layout_ddr_h = br.read<decltype(layout_ddr_h), 16>();
        layout_ddr_w = br.read<decltype(layout_ddr_w), 16>();
        stride_glb = br.read<decltype(stride_glb), 64>();
        mmu_item = br.read<decltype(mmu_item), 4>();
        addr_qarg = br.read<decltype(addr_qarg), 25>();
        input_signed = br.read<decltype(input_signed), 1>();
        precision_glb = br.read<decltype(precision_glb), 2>();
        precision_ddr = br.read<decltype(precision_ddr), 3>();
        quan_type = br.read<decltype(quan_type), 1>();
    }
};

/**
    LOADIF
    @brief 从 DDR 中加载张量到 GLB 中
    - OPCODE opcode : 8  从 DDR 中加载张量到 GLB 中
    - CCRCLR ccrclr : 8  None
    - CCRCLR ccrclr_qarg : 8  None
    - CCRSET ccrset : 11  数据依赖配置
    - ADDR addr_src : 32  DDR 中 张量当前数据 的起始地址。
    - ADDR_GLB_8_WITHOUT_BANK addr_dest : 21  存储到 GLB 中的地址
    - uint64_t shape_n : 16  取 Tensor 的形状的配置，用来决定实际搬运的数据
    - uint64_t shape_c : 16  取 Tensor 的形状的配置，用来决定实际搬运的数据
    - uint64_t shape_h : 16  取 Tensor 的形状的配置，用来决定实际搬运的数据
    - uint64_t shape_w : 16  取 Tensor 的形状的配置，用来决定实际搬运的数据
    - uint64_t basement : 2  DDR寻址偏移寄存器编号。
DDR访问的地址需要加上这个寄存器指示的寄存器的值
**/
struct INST_LOADIF
{
    OPCODE opcode : 8; /** 从 DDR 中加载张量到 GLB 中 **/
    uint64_t ccrclr : 8; /** None **/
    uint64_t ccrclr_qarg : 8; /** None **/
    uint64_t ccrset : 11; /** 数据依赖配置 **/
    ADDR addr_src : 32; /** DDR 中 张量当前数据 的起始地址。 **/
    uint64_t addr_dest : 21; /** 存储到 GLB 中的地址 **/
    uint64_t shape_n : 16; /** 取 Tensor 的形状的配置，用来决定实际搬运的数据 **/
    uint64_t shape_c : 16; /** 取 Tensor 的形状的配置，用来决定实际搬运的数据 **/
    uint64_t shape_h : 16; /** 取 Tensor 的形状的配置，用来决定实际搬运的数据 **/
    uint64_t shape_w : 16; /** 取 Tensor 的形状的配置，用来决定实际搬运的数据 **/
    uint64_t basement : 2; /** DDR寻址偏移寄存器编号。
DDR访问的地址需要加上这个寄存器指示的寄存器的值 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr);
        bw.write<8>(ccrclr_qarg);
        bw.write<11>(ccrset);
        bw.write<32>(addr_src);
        bw.write<21>(addr_dest);
        bw.write<16>(shape_n);
        bw.write<16>(shape_c);
        bw.write<16>(shape_h);
        bw.write<16>(shape_w);
        bw.write<2>(basement);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr = br.read<decltype(ccrclr), 8>();
        ccrclr_qarg = br.read<decltype(ccrclr_qarg), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src = br.read<decltype(addr_src), 32>();
        addr_dest = br.read<decltype(addr_dest), 21>();
        shape_n = br.read<decltype(shape_n), 16>();
        shape_c = br.read<decltype(shape_c), 16>();
        shape_h = br.read<decltype(shape_h), 16>();
        shape_w = br.read<decltype(shape_w), 16>();
        basement = br.read<decltype(basement), 2>();
    }
};

/**
    LOAD
    @brief 从 DDR 中加载一段给定长度的一维数据数据到 GLB
指定的位置上，可用来加载 weights 和其他数据
    - OPCODE opcode : 8  从 DDR 中加载一段给定长度的一维数据数据到 GLB
指定的位置上，可用来加载 weights 和其他数据
    - CCRCLR ccrclr : 8  None
    - CCRCLR ccrclr_qarg : 8  None
    - CCRSET ccrset : 11  数据依赖配置
    - ADDR addr_src : 32  DDR 中数据的地址
    - ADDR_GLB_8_WITH_BANK addr_dest : 25  GLB 中的地址
    - uint64_t length : 21  数据的长度
    - ADDR_GLB_8_WITH_BANK addr_qarg : 25  量化参数地
    - uint64_t chan_qarg : 16  每个量化参数持续作用的个数
    - uint64_t shape_c : 16
量化参数的个数，等于总长度除以量化参数持续个数，硬件需要的冗余信息
    - uint64_t basement : 2  DDR寻址偏移寄存器编号。
DDR访问的地址需要加上这个寄存器指示的寄存器的值
    - QUAN_SIGNED input_signed : 1  None
    - PRECISION precision_glb : 2  GLB中数据的类型
    - PRECISION_DDR precision_ddr : 3  DDR 中的数据类型
    - uint64_t stream : 1  是否为流式输入
**/
struct INST_LOAD
{
    OPCODE opcode : 8; /** 从 DDR 中加载一段给定长度的一维数据数据到 GLB
                        指定的位置上，可用来加载 weights 和其他数据 **/
    uint64_t ccrclr : 8; /** None **/
    uint64_t ccrclr_qarg : 8; /** None **/
    uint64_t ccrset : 11; /** 数据依赖配置 **/
    ADDR addr_src : 32; /** DDR 中数据的地址 **/
    uint64_t addr_dest : 25; /** GLB 中的地址 **/
    uint64_t length : 21; /** 数据的长度 **/
    uint64_t addr_qarg : 25; /** 量化参数地 **/
    uint64_t chan_qarg : 16; /** 每个量化参数持续作用的个数 **/
    uint64_t shape_c : 16; /**
                                   量化参数的个数，等于总长度除以量化参数持续个数，硬件需要的冗余信息
                                   **/
    uint64_t basement : 2; /** DDR寻址偏移寄存器编号。
  DDR访问的地址需要加上这个寄存器指示的寄存器的值 **/
    QUAN_SIGNED input_signed : 1; /** None **/
    PRECISION precision_glb : 2; /** GLB中数据的类型 **/
    PRECISION_DDR precision_ddr : 3; /** DDR 中的数据类型 **/
    uint64_t stream : 1; /** 是否为流式输入 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr);
        bw.write<8>(ccrclr_qarg);
        bw.write<11>(ccrset);
        bw.write<32>(addr_src);
        bw.write<25>(addr_dest);
        bw.write<21>(length);
        bw.write<25>(addr_qarg);
        bw.write<16>(chan_qarg);
        bw.write<16>(shape_c);
        bw.write<2>(basement);
        bw.write<1>(input_signed);
        bw.write<2>(precision_glb);
        bw.write<3>(precision_ddr);
        bw.write<1>(stream);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr = br.read<decltype(ccrclr), 8>();
        ccrclr_qarg = br.read<decltype(ccrclr_qarg), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src = br.read<decltype(addr_src), 32>();
        addr_dest = br.read<decltype(addr_dest), 25>();
        length = br.read<decltype(length), 21>();
        addr_qarg = br.read<decltype(addr_qarg), 25>();
        chan_qarg = br.read<decltype(chan_qarg), 16>();
        shape_c = br.read<decltype(shape_c), 16>();
        basement = br.read<decltype(basement), 2>();
        input_signed = br.read<decltype(input_signed), 1>();
        precision_glb = br.read<decltype(precision_glb), 2>();
        precision_ddr = br.read<decltype(precision_ddr), 3>();
        stream = br.read<decltype(stream), 1>();
    }
};

/**
    LOADIF.COMPRESS.CONF
    @brief 配置loadif的稀疏、压缩、参数
    - OPCODE opcode : 8  配置loadif的稀疏、压缩、参数
    - ADDR addr_bmp : 32  bitmap的地址
    - ADDR addr_code_len : 32  每一行编码之后的长度存放的地址
    - ADDR addr_block_len : 32  码块编码之后长度的存放地址
    - uint64_t code_lines : 4  每次编码的行数
    - SPARSIFIED sparsified_ddr : 1  是否稀疏
    - COMPRESSED compress_ddr : 1  是否压缩
**/
struct INST_LOADIF_COMPRESS_CONF
{
    OPCODE opcode : 8; /** 配置loadif的稀疏、压缩、参数 **/
    ADDR addr_bmp : 32; /** bitmap的地址 **/
    ADDR addr_code_len : 32; /** 每一行编码之后的长度存放的地址 **/
    ADDR addr_block_len : 32; /** 码块编码之后长度的存放地址 **/
    uint64_t code_lines : 4; /** 每次编码的行数 **/
    SPARSIFIED sparsified_ddr : 1; /** 是否稀疏 **/
    COMPRESSED compress_ddr : 1; /** 是否压缩 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<32>(addr_bmp);
        bw.write<32>(addr_code_len);
        bw.write<32>(addr_block_len);
        bw.write<4>(code_lines);
        bw.write<1>(sparsified_ddr);
        bw.write<1>(compress_ddr);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        addr_bmp = br.read<decltype(addr_bmp), 32>();
        addr_code_len = br.read<decltype(addr_code_len), 32>();
        addr_block_len = br.read<decltype(addr_block_len), 32>();
        code_lines = br.read<decltype(code_lines), 4>();
        sparsified_ddr = br.read<decltype(sparsified_ddr), 1>();
        compress_ddr = br.read<decltype(compress_ddr), 1>();
    }
};

/**
    LOAD.COMPRESS.CONF
    @brief 配置load的稀疏、压缩、参数
    - OPCODE opcode : 8  配置load的稀疏、压缩、参数
    - ADDR addr_bmp : 32  bitmap的地址
    - ADDR addr_code_len : 32  每一行编码之后的长度存放的地址
    - SPARSIFIED sparsified_ddr : 1  是否稀疏
**/
struct INST_LOAD_COMPRESS_CONF
{
    OPCODE opcode : 8; /** 配置load的稀疏、压缩、参数 **/
    ADDR addr_bmp : 32; /** bitmap的地址 **/
    ADDR addr_code_len : 32; /** 每一行编码之后的长度存放的地址 **/
    SPARSIFIED sparsified_ddr : 1; /** 是否稀疏 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<32>(addr_bmp);
        bw.write<32>(addr_code_len);
        bw.write<1>(sparsified_ddr);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        addr_bmp = br.read<decltype(addr_bmp), 32>();
        addr_code_len = br.read<decltype(addr_code_len), 32>();
        sparsified_ddr = br.read<decltype(sparsified_ddr), 1>();
    }
};

/**
    STORE
    @brief 将 GLB 中的给定长度的一维数据写回 DDR
    - OPCODE opcode : 8  将 GLB 中的给定长度的一维数据写回 DDR
    - CCRCLR ccrclr : 8  数据依赖配置
    - CCRCLR ccrclr_qarg : 8  None
    - CCRSET ccrset : 11  None
    - ADDR_GLB_8_WITH_BANK addr_src : 25  数据在 GLB 中的起始位置
    - ADDR addr_dest : 32  输出到 DDR 的位置
    - PRECISION precision_glb : 2  GLB中数据的类型
    - PRECISION_DDR precision_ddr : 3  输出数据精度
    - QUAN_SIGNED output_signed : 1  量化数据是否是有符号数据
    - uint64_t length : 21  数据长度
    - ADDR_GLB_8_WITH_BANK addr_qarg : 25  量化参数地
    - uint64_t chan_qarg : 16  每个量化参数持续作用的个数
    - uint64_t shape_c : 16
量化参数的个数，等于总长度除以量化参数持续个数，硬件需要的冗余信息
    - uint64_t clamp_hi : 16  None
    - uint64_t clamp_lo : 16  None
    - uint64_t basement : 2  DDR寻址偏移寄存器编号
**/
struct INST_STORE
{
    OPCODE opcode : 8; /** 将 GLB 中的给定长度的一维数据写回 DDR **/
    uint64_t ccrclr : 8; /** 数据依赖配置 **/
    uint64_t ccrclr_qarg : 8; /** None **/
    uint64_t ccrset : 11; /** None **/
    uint64_t addr_src : 25; /** 数据在 GLB 中的起始位置 **/
    ADDR addr_dest : 32; /** 输出到 DDR 的位置 **/
    PRECISION precision_glb : 2; /** GLB中数据的类型 **/
    PRECISION_DDR precision_ddr : 3; /** 输出数据精度 **/
    QUAN_SIGNED output_signed : 1; /** 量化数据是否是有符号数据 **/
    uint64_t length : 21; /** 数据长度 **/
    uint64_t addr_qarg : 25; /** 量化参数地 **/
    uint64_t chan_qarg : 16; /** 每个量化参数持续作用的个数 **/
    uint64_t shape_c : 16; /**
                              量化参数的个数，等于总长度除以量化参数持续个数，硬件需要的冗余信息
                              **/
    uint64_t clamp_hi : 16; /** None **/
    uint64_t clamp_lo : 16; /** None **/
    uint64_t basement : 2; /** DDR寻址偏移寄存器编号 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr);
        bw.write<8>(ccrclr_qarg);
        bw.write<11>(ccrset);
        bw.write<25>(addr_src);
        bw.write<32>(addr_dest);
        bw.write<2>(precision_glb);
        bw.write<3>(precision_ddr);
        bw.write<1>(output_signed);
        bw.write<21>(length);
        bw.write<25>(addr_qarg);
        bw.write<16>(chan_qarg);
        bw.write<16>(shape_c);
        bw.write<16>(clamp_hi);
        bw.write<16>(clamp_lo);
        bw.write<2>(basement);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr = br.read<decltype(ccrclr), 8>();
        ccrclr_qarg = br.read<decltype(ccrclr_qarg), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src = br.read<decltype(addr_src), 25>();
        addr_dest = br.read<decltype(addr_dest), 32>();
        precision_glb = br.read<decltype(precision_glb), 2>();
        precision_ddr = br.read<decltype(precision_ddr), 3>();
        output_signed = br.read<decltype(output_signed), 1>();
        length = br.read<decltype(length), 21>();
        addr_qarg = br.read<decltype(addr_qarg), 25>();
        chan_qarg = br.read<decltype(chan_qarg), 16>();
        shape_c = br.read<decltype(shape_c), 16>();
        clamp_hi = br.read<decltype(clamp_hi), 16>();
        clamp_lo = br.read<decltype(clamp_lo), 16>();
        basement = br.read<decltype(basement), 2>();
    }
};

/**
    STORE.T.CONFIG
    @brief 将 GLB 中 Tensor 的数据写回 DDR 的配置
    - OPCODE opcode : 8  将 GLB 中 Tensor 的数据写回 DDR 的配置
    - uint64_t layout_ddr_n : 16  DDR中的Tensor的尺寸信息，用用计算跳转
    - uint64_t layout_ddr_c : 16  DDR中的Tensor的尺寸信息，用用计算跳转
    - uint64_t layout_ddr_h : 16  DDR中的Tensor的尺寸信息，用用计算跳转
    - uint64_t layout_ddr_w : 16  DDR中的Tensor的尺寸信息，用用计算跳转
    - STRIDE_GLB stride_glb : 64  GLB中的Tensor的尺寸信息，用用计算跳转
    - uint64_t mmu_item : 4  使用的内存映射方案
    - ADDR_GLB_8_WITH_BANK addr_qarg : 25  量化参数地址
    - QUAN_SIGNED output_signed : 1  量化数据是否是有符号数据
    - PRECISION precision_glb : 2  GLB中数据的类型
    - PRECISION_DDR precision_ddr : 3  输出数据精度
    - QUAN_TYPE quan_type : 1  None
    - uint64_t clamp_hi : 16  None
    - uint64_t clamp_lo : 16  None
**/
struct INST_STORE_T_CONFIG
{
    OPCODE opcode : 8; /** 将 GLB 中 Tensor 的数据写回 DDR 的配置 **/
    uint64_t layout_ddr_n : 16; /** DDR中的Tensor的尺寸信息，用用计算跳转 **/
    uint64_t layout_ddr_c : 16; /** DDR中的Tensor的尺寸信息，用用计算跳转 **/
    uint64_t layout_ddr_h : 16; /** DDR中的Tensor的尺寸信息，用用计算跳转 **/
    uint64_t layout_ddr_w : 16; /** DDR中的Tensor的尺寸信息，用用计算跳转 **/
    uint64_t stride_glb : 64; /** GLB中的Tensor的尺寸信息，用用计算跳转 **/
    uint64_t mmu_item : 4; /** 使用的内存映射方案 **/
    uint64_t addr_qarg : 25; /** 量化参数地址 **/
    QUAN_SIGNED output_signed : 1; /** 量化数据是否是有符号数据 **/
    PRECISION precision_glb : 2; /** GLB中数据的类型 **/
    PRECISION_DDR precision_ddr : 3; /** 输出数据精度 **/
    QUAN_TYPE quan_type : 1; /** None **/
    uint64_t clamp_hi : 16; /** None **/
    uint64_t clamp_lo : 16; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<16>(layout_ddr_n);
        bw.write<16>(layout_ddr_c);
        bw.write<16>(layout_ddr_h);
        bw.write<16>(layout_ddr_w);
        bw.write<64>(stride_glb);
        bw.write<4>(mmu_item);
        bw.write<25>(addr_qarg);
        bw.write<1>(output_signed);
        bw.write<2>(precision_glb);
        bw.write<3>(precision_ddr);
        bw.write<1>(quan_type);
        bw.write<16>(clamp_hi);
        bw.write<16>(clamp_lo);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        layout_ddr_n = br.read<decltype(layout_ddr_n), 16>();
        layout_ddr_c = br.read<decltype(layout_ddr_c), 16>();
        layout_ddr_h = br.read<decltype(layout_ddr_h), 16>();
        layout_ddr_w = br.read<decltype(layout_ddr_w), 16>();
        stride_glb = br.read<decltype(stride_glb), 64>();
        mmu_item = br.read<decltype(mmu_item), 4>();
        addr_qarg = br.read<decltype(addr_qarg), 25>();
        output_signed = br.read<decltype(output_signed), 1>();
        precision_glb = br.read<decltype(precision_glb), 2>();
        precision_ddr = br.read<decltype(precision_ddr), 3>();
        quan_type = br.read<decltype(quan_type), 1>();
        clamp_hi = br.read<decltype(clamp_hi), 16>();
        clamp_lo = br.read<decltype(clamp_lo), 16>();
    }
};

/**
    STORE.T
    @brief 将 GLB 中 Tensor 的数据写回 DDR，并且可能发生转置或者数据类型转换
    - OPCODE opcode : 8  将 GLB 中 Tensor 的数据写回
DDR，并且可能发生转置或者数据类型转换
    - CCRCLR ccrclr : 8  数据依赖配置
    - CCRCLR ccrclr_qarg : 8  None
    - CCRSET ccrset : 11  None
    - ADDR_GLB_8_WITHOUT_BANK addr_src : 21  Tensor 在 GLB 中的起始位置
    - ADDR addr_dest : 32  输出到 DDR 的位置
    - uint64_t shape_n : 16  Slice 长度信息
    - uint64_t shape_c : 16  Slice 长度信息
    - uint64_t shape_h : 16  Slice 长度信息
    - uint64_t shape_w : 16  Slice 长度信息
    - uint64_t basement : 2  DDR寻址偏移寄存器编号
**/
struct INST_STORE_T
{
    OPCODE opcode : 8; /** 将 GLB 中 Tensor 的数据写回
                               DDR，并且可能发生转置或者数据类型转换 **/
    uint64_t ccrclr : 8; /** 数据依赖配置 **/
    uint64_t ccrclr_qarg : 8; /** None **/
    uint64_t ccrset : 11; /** None **/
    uint64_t addr_src : 21; /** Tensor 在 GLB 中的起始位置 **/
    ADDR addr_dest : 32; /** 输出到 DDR 的位置 **/
    uint64_t shape_n : 16; /** Slice 长度信息 **/
    uint64_t shape_c : 16; /** Slice 长度信息 **/
    uint64_t shape_h : 16; /** Slice 长度信息 **/
    uint64_t shape_w : 16; /** Slice 长度信息 **/
    uint64_t basement : 2; /** DDR寻址偏移寄存器编号 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr);
        bw.write<8>(ccrclr_qarg);
        bw.write<11>(ccrset);
        bw.write<21>(addr_src);
        bw.write<32>(addr_dest);
        bw.write<16>(shape_n);
        bw.write<16>(shape_c);
        bw.write<16>(shape_h);
        bw.write<16>(shape_w);
        bw.write<2>(basement);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr = br.read<decltype(ccrclr), 8>();
        ccrclr_qarg = br.read<decltype(ccrclr_qarg), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src = br.read<decltype(addr_src), 21>();
        addr_dest = br.read<decltype(addr_dest), 32>();
        shape_n = br.read<decltype(shape_n), 16>();
        shape_c = br.read<decltype(shape_c), 16>();
        shape_h = br.read<decltype(shape_h), 16>();
        shape_w = br.read<decltype(shape_w), 16>();
        basement = br.read<decltype(basement), 2>();
    }
};

/**
    STORE.T.COMPRESS.CONF
    @brief 配置store.t的稀疏、压缩、参数
    - OPCODE opcode : 8  配置store.t的稀疏、压缩、参数
    - ADDR addr_bmp : 32  bitmap的地址
    - ADDR addr_code_len : 32  每一行编码之后的长度存放的地址
    - ADDR addr_block_len : 32  码块编码之后长度的存放地址
    - uint64_t code_lines : 4  每次编码的行数
    - SPARSIFIED sparsified_ddr : 1  是否稀疏
    - COMPRESSED compress_ddr : 1  是否压缩
**/
struct INST_STORE_T_COMPRESS_CONF
{
    OPCODE opcode : 8; /** 配置store.t的稀疏、压缩、参数 **/
    ADDR addr_bmp : 32; /** bitmap的地址 **/
    ADDR addr_code_len : 32; /** 每一行编码之后的长度存放的地址 **/
    ADDR addr_block_len : 32; /** 码块编码之后长度的存放地址 **/
    uint64_t code_lines : 4; /** 每次编码的行数 **/
    SPARSIFIED sparsified_ddr : 1; /** 是否稀疏 **/
    COMPRESSED compress_ddr : 1; /** 是否压缩 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<32>(addr_bmp);
        bw.write<32>(addr_code_len);
        bw.write<32>(addr_block_len);
        bw.write<4>(code_lines);
        bw.write<1>(sparsified_ddr);
        bw.write<1>(compress_ddr);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        addr_bmp = br.read<decltype(addr_bmp), 32>();
        addr_code_len = br.read<decltype(addr_code_len), 32>();
        addr_block_len = br.read<decltype(addr_block_len), 32>();
        code_lines = br.read<decltype(code_lines), 4>();
        sparsified_ddr = br.read<decltype(sparsified_ddr), 1>();
        compress_ddr = br.read<decltype(compress_ddr), 1>();
    }
};

/**
    STORE.COMPRESS.CONF
    @brief 配置store的稀疏、压缩、参数
    - OPCODE opcode : 8  配置store的稀疏、压缩、参数
    - ADDR addr_bmp : 32  bitmap的地址
    - ADDR addr_code_len : 32  每一行编码之后的长度存放的地址
    - SPARSIFIED sparsified_ddr : 1  是否稀疏
**/
struct INST_STORE_COMPRESS_CONF
{
    OPCODE opcode : 8; /** 配置store的稀疏、压缩、参数 **/
    ADDR addr_bmp : 32; /** bitmap的地址 **/
    ADDR addr_code_len : 32; /** 每一行编码之后的长度存放的地址 **/
    SPARSIFIED sparsified_ddr : 1; /** 是否稀疏 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<32>(addr_bmp);
        bw.write<32>(addr_code_len);
        bw.write<1>(sparsified_ddr);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        addr_bmp = br.read<decltype(addr_bmp), 32>();
        addr_code_len = br.read<decltype(addr_code_len), 32>();
        sparsified_ddr = br.read<decltype(sparsified_ddr), 1>();
    }
};

/**
    TCU.DM.BROADCAST
    @brief 配置DM是否在广播模式
    - OPCODE opcode : 8  配置DM是否在广播模式
    - BROADCAST broadcast_if : 1  配置input featuremap是否在广播模式
    - BROADCAST broadcast_weight : 1  配置weight是否在广播模式
    - uint64_t psum_cascade : 1  是否为psum级联模式（用来加速1*1卷积计算） 0：否
4个TCU 1：是
**/
struct INST_TCU_DM_BROADCAST
{
    OPCODE opcode : 8; /** 配置DM是否在广播模式 **/
    BROADCAST broadcast_if : 1; /** 配置input featuremap是否在广播模式 **/
    BROADCAST broadcast_weight : 1; /** 配置weight是否在广播模式 **/
    uint64_t psum_cascade : 1; /** 是否为psum级联模式（用来加速1*1卷积计算） 0：否
                                4个TCU 1：是 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<1>(broadcast_if);
        bw.write<1>(broadcast_weight);
        bw.write<1>(psum_cascade);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        broadcast_if = br.read<decltype(broadcast_if), 1>();
        broadcast_weight = br.read<decltype(broadcast_weight), 1>();
        psum_cascade = br.read<decltype(psum_cascade), 1>();
    }
};

/**
    TCU.DM.CONF.IF
    @brief 配置 FETCHIF 的一些基础信息
    - OPCODE opcode : 8  配置 FETCHIF 的一些基础信息
    - uint64_t tcu_id : 4  选择配置第几个TCU的DM
    - STRIDE_GLB stride_input_glb : 64  GLB中大tensor的排布的信息，用于计算跳转
    - uint64_t stride_w : 16  input featuremap 在 W 方向上的 stride
    - uint64_t stride_h : 16  input featuremap 在 H 方向上的 stride
    - uint64_t padding_top : 8  input featuremap 在 HxW 平面上顶部的 zero
padding 行数
    - uint64_t padding_bottom : 8  input featuremap 在 HxW 平面下底部的 zero
padding 行数
    - uint64_t padding_left : 8  input featuremap 在 HxW 平面左边的 zero padding
列数
    - uint64_t padding_right : 8  input featuremap 在 HxW 平面右边的 zero
padding 列数
    - uint64_t input_c_per_pu : 5  每个PU计算的输入通道数，根据高度映射除出来的
    - uint64_t dilation_h : 8  input featuremap 的在 H 方向上的膨胀率
    - uint64_t transpose_if : 1  是否对输入的 input featuremap 进行转置，仅在
通道上有效
    - uint64_t mmu_item : 4  使用的内存映射方案
**/
struct INST_TCU_DM_CONF_IF
{
    OPCODE opcode : 8; /** 配置 FETCHIF 的一些基础信息 **/
    uint64_t tcu_id : 4; /** 选择配置第几个TCU的DM **/
    uint64_t
        stride_input_glb : 64; /** GLB中大tensor的排布的信息，用于计算跳转 **/
    uint64_t stride_w : 16; /** input featuremap 在 W 方向上的 stride **/
    uint64_t stride_h : 16; /** input featuremap 在 H 方向上的 stride **/
    uint64_t padding_top : 8; /** input featuremap 在 HxW 平面上顶部的 zero
                               padding 行数 **/
    uint64_t padding_bottom : 8; /** input featuremap 在 HxW 平面下底部的 zero
                                  padding 行数 **/
    uint64_t padding_left : 8; /** input featuremap 在 HxW 平面左边的 zero padding
                                列数 **/
    uint64_t padding_right : 8; /** input featuremap 在 HxW 平面右边的 zero
                                 padding 列数 **/
    uint64_t
        input_c_per_pu : 5; /** 每个PU计算的输入通道数，根据高度映射除出来的 **/
    uint64_t dilation_h : 8; /** input featuremap 的在 H 方向上的膨胀率 **/
    uint64_t transpose_if : 1; /** 是否对输入的 input featuremap 进行转置，仅在
                                通道上有效 **/
    uint64_t mmu_item : 4; /** 使用的内存映射方案 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<4>(tcu_id);
        bw.write<64>(stride_input_glb);
        bw.write<16>(stride_w);
        bw.write<16>(stride_h);
        bw.write<8>(padding_top);
        bw.write<8>(padding_bottom);
        bw.write<8>(padding_left);
        bw.write<8>(padding_right);
        bw.write<5>(input_c_per_pu);
        bw.write<8>(dilation_h);
        bw.write<1>(transpose_if);
        bw.write<4>(mmu_item);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        stride_input_glb = br.read<decltype(stride_input_glb), 64>();
        stride_w = br.read<decltype(stride_w), 16>();
        stride_h = br.read<decltype(stride_h), 16>();
        padding_top = br.read<decltype(padding_top), 8>();
        padding_bottom = br.read<decltype(padding_bottom), 8>();
        padding_left = br.read<decltype(padding_left), 8>();
        padding_right = br.read<decltype(padding_right), 8>();
        input_c_per_pu = br.read<decltype(input_c_per_pu), 5>();
        dilation_h = br.read<decltype(dilation_h), 8>();
        transpose_if = br.read<decltype(transpose_if), 1>();
        mmu_item = br.read<decltype(mmu_item), 4>();
    }
};

/**
    TCU.DM.FETCHIF
    @brief 从 GLB 中加载 input featuremap 到 PU 或者广播到多个PU中
    - OPCODE opcode : 8  从 GLB 中加载 input featuremap 到 PU 或者广播到多个PU中
    - CCRCLR ccrclr_if : 8  None
    - uint64_t tcu_id : 4  TCU ID
    - ADDR_GLB_8_WITHOUT_BANK addr_if : 21  输入slice的起始地址
    - uint64_t shape_input_n : 16  当前计算输入Slice的形状信息
    - uint64_t shape_input_c : 16  当前计算输入Slice的形状信息
    - uint64_t shape_input_h : 16  当前计算输入Slice的形状信息
    - uint64_t shape_input_w : 16  当前计算输入Slice的形状信息
**/
struct INST_TCU_DM_FETCHIF
{
    OPCODE opcode : 8; /** 从 GLB 中加载 input featuremap 到 PU 或者广播到多个PU中
                        **/
    uint64_t ccrclr_if : 8; /** None **/
    uint64_t tcu_id : 4; /** TCU ID **/
    uint64_t addr_if : 21; /** 输入slice的起始地址 **/
    uint64_t shape_input_n : 16; /** 当前计算输入Slice的形状信息 **/
    uint64_t shape_input_c : 16; /** 当前计算输入Slice的形状信息 **/
    uint64_t shape_input_h : 16; /** 当前计算输入Slice的形状信息 **/
    uint64_t shape_input_w : 16; /** 当前计算输入Slice的形状信息 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr_if);
        bw.write<4>(tcu_id);
        bw.write<21>(addr_if);
        bw.write<16>(shape_input_n);
        bw.write<16>(shape_input_c);
        bw.write<16>(shape_input_h);
        bw.write<16>(shape_input_w);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr_if = br.read<decltype(ccrclr_if), 8>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        addr_if = br.read<decltype(addr_if), 21>();
        shape_input_n = br.read<decltype(shape_input_n), 16>();
        shape_input_c = br.read<decltype(shape_input_c), 16>();
        shape_input_h = br.read<decltype(shape_input_h), 16>();
        shape_input_w = br.read<decltype(shape_input_w), 16>();
    }
};

/**
    TCU.DM.CONF.W
    @brief 配置 TCU 读取 weights 时的行为
    - OPCODE opcode : 8  配置 TCU 读取 weights 时的行为
    - uint64_t tcu_id : 4  TCU 的 ID
    - uint64_t mmu_item : 4  使用的内存映射方案
    - uint64_t load_direction : 1  是否对输入的 input featuremap
进行转置，仅在HxW通道上有效
**/
struct INST_TCU_DM_CONF_W
{
    OPCODE opcode : 8; /** 配置 TCU 读取 weights 时的行为 **/
    uint64_t tcu_id : 4; /** TCU 的 ID **/
    uint64_t mmu_item : 4; /** 使用的内存映射方案 **/
    uint64_t load_direction : 1; /** 是否对输入的 input featuremap
                                  进行转置，仅在HxW通道上有效 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<4>(tcu_id);
        bw.write<4>(mmu_item);
        bw.write<1>(load_direction);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        mmu_item = br.read<decltype(mmu_item), 4>();
        load_direction = br.read<decltype(load_direction), 1>();
    }
};

/**
    TCU.DM.FETCHW
    @brief 从 GLB 中加载 weights 到 PU 或者广播到多个PU中
    - OPCODE opcode : 8  从 GLB 中加载 weights 到 PU 或者广播到多个PU中
    - CCRCLR ccrclr_weight : 8  数据依赖配置
    - uint64_t tcu_id : 4  TCU ID
    - ADDR_GLB_8_WITHOUT_BANK addr_src : 21  GLB 中 weights
的地址,byte的偏移地址
**/
struct INST_TCU_DM_FETCHW
{
    OPCODE opcode : 8; /** 从 GLB 中加载 weights 到 PU 或者广播到多个PU中 **/
    uint64_t ccrclr_weight : 8; /** 数据依赖配置 **/
    uint64_t tcu_id : 4; /** TCU ID **/
    uint64_t addr_src : 21; /** GLB 中 weights 的地址,byte的偏移地址 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr_weight);
        bw.write<4>(tcu_id);
        bw.write<21>(addr_src);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr_weight = br.read<decltype(ccrclr_weight), 8>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        addr_src = br.read<decltype(addr_src), 21>();
    }
};

/**
    TCU.DM.CONF.OF
    @brief 配置 TCU 输出信息
    - OPCODE opcode : 8  配置 TCU 输出信息
    - uint64_t tcu_id : 4  TCU 的 ID
    - ADDR_GLB_8_WITH_BANK addr_psum : 25  中间结果的 GLB 中的地址
    - ADDR_GLB_8_WITH_BANK addr_dest : 25  输出结果的GLB中的地址
    - uint64_t shape_output_n : 16  当前计算输出Slice的形状信息
    - uint64_t shape_output_c : 16  当前计算输出Slice的形状信息
    - uint64_t shape_output_h : 16  当前计算输出Slice的形状信息
    - uint64_t shape_output_w : 16  当前计算输出Slice的形状信息
    - STRIDE_GLB stride_output_glb : 64  输出featuremap在GLB的形状信息，决定寻址
    - STRIDE_GLB stride_psum_glb : 64  输出PSUM在GLB的形状信息，决定寻址
    - uint64_t x_cut : 4
水平方向一行第一次卷积的偏移量，硬件会切掉这些数，为了在反卷积模式下和tensorflow的实现保持一致
    - PRECISION output_precision : 2  None
    - QUAN_SIGNED output_signed : 1  None
**/
struct INST_TCU_DM_CONF_OF
{
    OPCODE opcode : 8; /** 配置 TCU 输出信息 **/
    uint64_t tcu_id : 4; /** TCU 的 ID **/
    uint64_t addr_psum : 25; /** 中间结果的 GLB 中的地址 **/
    uint64_t addr_dest : 25; /** 输出结果的GLB中的地址 **/
    uint64_t shape_output_n : 16; /** 当前计算输出Slice的形状信息 **/
    uint64_t shape_output_c : 16; /** 当前计算输出Slice的形状信息 **/
    uint64_t shape_output_h : 16; /** 当前计算输出Slice的形状信息 **/
    uint64_t shape_output_w : 16; /** 当前计算输出Slice的形状信息 **/
    uint64_t
        stride_output_glb : 64; /** 输出featuremap在GLB的形状信息，决定寻址 **/
    uint64_t stride_psum_glb : 64; /** 输出PSUM在GLB的形状信息，决定寻址 **/
    uint64_t x_cut : 4; /**
                                    水平方向一行第一次卷积的偏移量，硬件会切掉这些数，为了在反卷积模式下和tensorflow的实现保持一致
                                    **/
    PRECISION output_precision : 2; /** None **/
    QUAN_SIGNED output_signed : 1; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<4>(tcu_id);
        bw.write<25>(addr_psum);
        bw.write<25>(addr_dest);
        bw.write<16>(shape_output_n);
        bw.write<16>(shape_output_c);
        bw.write<16>(shape_output_h);
        bw.write<16>(shape_output_w);
        bw.write<64>(stride_output_glb);
        bw.write<64>(stride_psum_glb);
        bw.write<4>(x_cut);
        bw.write<2>(output_precision);
        bw.write<1>(output_signed);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        addr_psum = br.read<decltype(addr_psum), 25>();
        addr_dest = br.read<decltype(addr_dest), 25>();
        shape_output_n = br.read<decltype(shape_output_n), 16>();
        shape_output_c = br.read<decltype(shape_output_c), 16>();
        shape_output_h = br.read<decltype(shape_output_h), 16>();
        shape_output_w = br.read<decltype(shape_output_w), 16>();
        stride_output_glb = br.read<decltype(stride_output_glb), 64>();
        stride_psum_glb = br.read<decltype(stride_psum_glb), 64>();
        x_cut = br.read<decltype(x_cut), 4>();
        output_precision = br.read<decltype(output_precision), 2>();
        output_signed = br.read<decltype(output_signed), 1>();
    }
};

/**
    TCU.PU.CONF
    @brief 配置给定的 TCU 的 PU 所使用的 PE 资源数量
    - OPCODE opcode : 8  配置给定的 TCU 的 PU 所使用的 PE 资源数量
    - uint64_t tcu_id : 4  TCU 的 ID
    - uint64_t pe_w : 5  TCU 的 PU 使用的 pe 列数
    - uint64_t pe_h : 5  使用的 pe 行数
    - uint64_t pe_last_w : 5  保留位置，不使用
    - uint64_t pe_last_h : 5  在不能被整除的模式的时候PE最后一次计算的激活高度
    - uint64_t kernel_h : 8  kernel 高度，如果是矩阵乘模式无效
    - uint64_t kernel_w : 8  kernel 宽度，如果是矩阵乘模式无效
    - uint64_t group : 5  分组卷积的组数，对应了TCU每个小块的宽度
    - uint64_t pu_loop_w : 16  硬件数值循环次数
每个PE列的输出 channel 数，硬件内部循环用
    - uint64_t pu_loop_h : 16  硬件水平循环次数
硬件内部信号=ceil(IF_c / input_channel_per_pu )
    - TCU_MODE mode : 2  指定PU的工作模式
**/
struct INST_TCU_PU_CONF
{
    OPCODE opcode : 8; /** 配置给定的 TCU 的 PU 所使用的 PE 资源数量 **/
    uint64_t tcu_id : 4; /** TCU 的 ID **/
    uint64_t pe_w : 5; /** TCU 的 PU 使用的 pe 列数 **/
    uint64_t pe_h : 5; /** 使用的 pe 行数 **/
    uint64_t pe_last_w : 5; /** 保留位置，不使用 **/
    uint64_t
        pe_last_h : 5; /** 在不能被整除的模式的时候PE最后一次计算的激活高度 **/
    uint64_t kernel_h : 8; /** kernel 高度，如果是矩阵乘模式无效 **/
    uint64_t kernel_w : 8; /** kernel 宽度，如果是矩阵乘模式无效 **/
    uint64_t group : 5; /** 分组卷积的组数，对应了TCU每个小块的宽度 **/
    uint64_t pu_loop_w : 16; /** 硬件数值循环次数
每个PE列的输出 channel 数，硬件内部循环用 **/
    uint64_t pu_loop_h : 16; /** 硬件水平循环次数
硬件内部信号=ceil(IF_c / input_channel_per_pu ) **/
    TCU_MODE mode : 2; /** 指定PU的工作模式 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<4>(tcu_id);
        bw.write<5>(pe_w);
        bw.write<5>(pe_h);
        bw.write<5>(pe_last_w);
        bw.write<5>(pe_last_h);
        bw.write<8>(kernel_h);
        bw.write<8>(kernel_w);
        bw.write<5>(group);
        bw.write<16>(pu_loop_w);
        bw.write<16>(pu_loop_h);
        bw.write<2>(mode);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        pe_w = br.read<decltype(pe_w), 5>();
        pe_h = br.read<decltype(pe_h), 5>();
        pe_last_w = br.read<decltype(pe_last_w), 5>();
        pe_last_h = br.read<decltype(pe_last_h), 5>();
        kernel_h = br.read<decltype(kernel_h), 8>();
        kernel_w = br.read<decltype(kernel_w), 8>();
        group = br.read<decltype(group), 5>();
        pu_loop_w = br.read<decltype(pu_loop_w), 16>();
        pu_loop_h = br.read<decltype(pu_loop_h), 16>();
        mode = br.read<decltype(mode), 2>();
    }
};

/**
    TCU.PU.CONF.ACT
    @brief 配置给定的 TCU 的 PU 使用的 fused 的两段式激活函数的参数
    - OPCODE opcode : 8  配置给定的 TCU 的 PU 使用的 fused
的两段式激活函数的参数
    - uint64_t tcu_id : 4  TCU 的 ID
    - uint64_t clamp_max : 16  上饱和数值
    - uint64_t clamp_min : 16  下饱和数值
    - ADDR_GLB_8_WITH_BANK addr_act : 25
每个输出通道激活参数（分段点，两端的scale，bias，以及量化参数）
**/
struct INST_TCU_PU_CONF_ACT
{
    OPCODE opcode : 8; /** 配置给定的 TCU 的 PU 使用的 fused
                              的两段式激活函数的参数 **/
    uint64_t tcu_id : 4; /** TCU 的 ID **/
    uint64_t clamp_max : 16; /** 上饱和数值 **/
    uint64_t clamp_min : 16; /** 下饱和数值 **/
    uint64_t addr_act : 25; /**
                              每个输出通道激活参数（分段点，两端的scale，bias，以及量化参数）
                              **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<4>(tcu_id);
        bw.write<16>(clamp_max);
        bw.write<16>(clamp_min);
        bw.write<25>(addr_act);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        clamp_max = br.read<decltype(clamp_max), 16>();
        clamp_min = br.read<decltype(clamp_min), 16>();
        addr_act = br.read<decltype(addr_act), 25>();
    }
};

/**
    TCU.PU.COMPUTE
    @brief 启动PU一次计算
    - OPCODE opcode : 8  启动PU一次计算
    - CCRCLR ccrclr_act : 8  None
    - CCRCLR ccrclr_psum : 8  None
    - CCRSET ccrset : 11  数据依赖配置
    - uint64_t tcu_id : 4  TCU 的 ID
    - uint64_t act_enable : 1 是否在这次计算完之后将结果做一次两段拟合的激活函数
    - uint64_t of_enable : 1  决定输出是psum还是output featuremap
    - uint64_t load_psum : 1  是否从 GLB 加载中间结果
    - uint64_t weight_switching : 4  是否切换 PU 的 weights
**/
struct INST_TCU_PU_COMPUTE
{
    OPCODE opcode : 8; /** 启动PU一次计算 **/
    uint64_t ccrclr_act : 8; /** None **/
    uint64_t ccrclr_psum : 8; /** None **/
    uint64_t ccrset : 11; /** 数据依赖配置 **/
    uint64_t tcu_id : 4; /** TCU 的 ID **/
    uint64_t
        act_enable : 1; /** 是否在这次计算完之后将结果做一次两段拟合的激活函数 **/
    uint64_t of_enable : 1; /** 决定输出是psum还是output featuremap **/
    uint64_t load_psum : 1; /** 是否从 GLB 加载中间结果 **/
    uint64_t weight_switching : 4; /** 是否切换 PU 的 weights **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr_act);
        bw.write<8>(ccrclr_psum);
        bw.write<11>(ccrset);
        bw.write<4>(tcu_id);
        bw.write<1>(act_enable);
        bw.write<1>(of_enable);
        bw.write<1>(load_psum);
        bw.write<4>(weight_switching);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr_act = br.read<decltype(ccrclr_act), 8>();
        ccrclr_psum = br.read<decltype(ccrclr_psum), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        act_enable = br.read<decltype(act_enable), 1>();
        of_enable = br.read<decltype(of_enable), 1>();
        load_psum = br.read<decltype(load_psum), 1>();
        weight_switching = br.read<decltype(weight_switching), 4>();
    }
};

/**
    TCU.DOT.DM.IF.CONF
    @brief None
    - OPCODE opcode : 8  None
    - uint64_t tcu_id : 4  None
    - STRIDE_GLB stride_src1_glb : 64  None
    - STRIDE_GLB stride_src2_glb : 64  None
**/
struct INST_TCU_DOT_DM_IF_CONF
{
    OPCODE opcode : 8; /** None **/
    uint64_t tcu_id : 4; /** None **/
    uint64_t stride_src1_glb : 64; /** None **/
    uint64_t stride_src2_glb : 64; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<4>(tcu_id);
        bw.write<64>(stride_src1_glb);
        bw.write<64>(stride_src2_glb);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        stride_src1_glb = br.read<decltype(stride_src1_glb), 64>();
        stride_src2_glb = br.read<decltype(stride_src2_glb), 64>();
    }
};

/**
    TCU.DOT.DM.OF.CONF
    @brief None
    - OPCODE opcode : 8  None
    - uint64_t tcu_id : 4  None
    - uint64_t stride_psum_glb : 64  None
    - ADDR_GLB_8_WITH_BANK addr_psum : 25  None
    - ADDR_GLB_8_WITH_BANK addr_dest : 25  None
    - STRIDE_GLB stride_dest_glb : 64  None
    - uint64_t shape_dest_n : 16  None
    - uint64_t shape_dest_h : 16  None
    - uint64_t shape_dest_w : 16  None
    - uint64_t shape_src1_w : 16  None
    - PRECISION output_precision : 2  None
    - QUAN_SIGNED output_signed : 1  None
**/
struct INST_TCU_DOT_DM_OF_CONF
{
    OPCODE opcode : 8; /** None **/
    uint64_t tcu_id : 4; /** None **/
    uint64_t stride_psum_glb : 64; /** None **/
    uint64_t addr_psum : 25; /** None **/
    uint64_t addr_dest : 25; /** None **/
    uint64_t stride_dest_glb : 64; /** None **/
    uint64_t shape_dest_n : 16; /** None **/
    uint64_t shape_dest_h : 16; /** None **/
    uint64_t shape_dest_w : 16; /** None **/
    uint64_t shape_src1_w : 16; /** None **/
    PRECISION output_precision : 2; /** None **/
    QUAN_SIGNED output_signed : 1; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<4>(tcu_id);
        bw.write<64>(stride_psum_glb);
        bw.write<25>(addr_psum);
        bw.write<25>(addr_dest);
        bw.write<64>(stride_dest_glb);
        bw.write<16>(shape_dest_n);
        bw.write<16>(shape_dest_h);
        bw.write<16>(shape_dest_w);
        bw.write<16>(shape_src1_w);
        bw.write<2>(output_precision);
        bw.write<1>(output_signed);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        stride_psum_glb = br.read<decltype(stride_psum_glb), 64>();
        addr_psum = br.read<decltype(addr_psum), 25>();
        addr_dest = br.read<decltype(addr_dest), 25>();
        stride_dest_glb = br.read<decltype(stride_dest_glb), 64>();
        shape_dest_n = br.read<decltype(shape_dest_n), 16>();
        shape_dest_h = br.read<decltype(shape_dest_h), 16>();
        shape_dest_w = br.read<decltype(shape_dest_w), 16>();
        shape_src1_w = br.read<decltype(shape_src1_w), 16>();
        output_precision = br.read<decltype(output_precision), 2>();
        output_signed = br.read<decltype(output_signed), 1>();
    }
};

/**
    TCU.DOT.DM.FETCH.SRC1
    @brief None
    - OPCODE opcode : 8  None
    - CCRCLR ccrclr_src1 : 8  None
    - uint64_t tcu_id : 4  None
    - ADDR_GLB_8_WITH_BANK addr_src1 : 25  None
**/
struct INST_TCU_DOT_DM_FETCH_SRC1
{
    OPCODE opcode : 8; /** None **/
    uint64_t ccrclr_src1 : 8; /** None **/
    uint64_t tcu_id : 4; /** None **/
    uint64_t addr_src1 : 25; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr_src1);
        bw.write<4>(tcu_id);
        bw.write<25>(addr_src1);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr_src1 = br.read<decltype(ccrclr_src1), 8>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        addr_src1 = br.read<decltype(addr_src1), 25>();
    }
};

/**
    TCU.DOT.DM.FETCH.SRC2
    @brief None
    - OPCODE opcode : 8  None
    - CCRCLR ccrclr_src2 : 8  None
    - uint64_t tcu_id : 4  None
    - ADDR_GLB_8_WITH_BANK addr_src2 : 25  None
**/
struct INST_TCU_DOT_DM_FETCH_SRC2
{
    OPCODE opcode : 8; /** None **/
    uint64_t ccrclr_src2 : 8; /** None **/
    uint64_t tcu_id : 4; /** None **/
    uint64_t addr_src2 : 25; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr_src2);
        bw.write<4>(tcu_id);
        bw.write<25>(addr_src2);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr_src2 = br.read<decltype(ccrclr_src2), 8>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        addr_src2 = br.read<decltype(addr_src2), 25>();
    }
};

/**
    TCU.PU.COMPUTE.DUMMY
    @brief 用来把PSUM转成output的指令
    - OPCODE opcode : 8  用来把PSUM转成output的指令
    - CCRCLR ccrclr_act : 8  None
    - CCRCLR ccrclr_psum : 8  None
    - CCRSET ccrset : 11  数据依赖配置
    - uint64_t tcu_id : 4  TCU 的 ID
    - uint64_t act_enable : 1 是否在这次计算完之后将结果做一次两段拟合的激活函数
**/
struct INST_TCU_PU_COMPUTE_DUMMY
{
    OPCODE opcode : 8; /** 用来把PSUM转成output的指令 **/
    uint64_t ccrclr_act : 8; /** None **/
    uint64_t ccrclr_psum : 8; /** None **/
    uint64_t ccrset : 11; /** 数据依赖配置 **/
    uint64_t tcu_id : 4; /** TCU 的 ID **/
    uint64_t
        act_enable : 1; /** 是否在这次计算完之后将结果做一次两段拟合的激活函数 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr_act);
        bw.write<8>(ccrclr_psum);
        bw.write<11>(ccrset);
        bw.write<4>(tcu_id);
        bw.write<1>(act_enable);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr_act = br.read<decltype(ccrclr_act), 8>();
        ccrclr_psum = br.read<decltype(ccrclr_psum), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        tcu_id = br.read<decltype(tcu_id), 4>();
        act_enable = br.read<decltype(act_enable), 1>();
    }
};

/**
    MFU.MN.MAP.COMPUTE
    @brief 配置MFU的DM模块，准备开始计算对于tensor的map运算
    - OPCODE opcode : 8  配置MFU的DM模块，准备开始计算对于tensor的map运算
    - CCRCLR ccrclr : 8  None
    - CCRSET ccrset : 11  None
    - UNION_ADDR addr_src : 32  输入向量slice地址 (GLB or DDR)
    - UNION_ADDR addr_dest : 32  输出向量地址 (GLB or DDR)
    - STRIDE_GLB stride_input_glb : 64  Tensor 的 shape
    - STRIDE_GLB stride_output_glb : 64  None
    - uint64_t shape_n : 16  Slice 长度信息
    - uint64_t shape_c : 16  Slice 长度信息
    - uint64_t shape_h : 16  Slice 长度信息
    - uint64_t shape_w : 16  Slice 长度信息
    - uint64_t basement_src1 : 2  None
    - uint64_t basement_dest : 2  None
**/
struct INST_MFU_MN_MAP_COMPUTE
{
    OPCODE opcode : 8; /** 配置MFU的DM模块，准备开始计算对于tensor的map运算 **/
    uint64_t ccrclr : 8; /** None **/
    uint64_t ccrset : 11; /** None **/
    uint64_t addr_src : 32; /** 输入向量slice地址 (GLB or DDR) **/
    uint64_t addr_dest : 32; /** 输出向量地址 (GLB or DDR) **/
    uint64_t stride_input_glb : 64; /** Tensor 的 shape **/
    uint64_t stride_output_glb : 64; /** None **/
    uint64_t shape_n : 16; /** Slice 长度信息 **/
    uint64_t shape_c : 16; /** Slice 长度信息 **/
    uint64_t shape_h : 16; /** Slice 长度信息 **/
    uint64_t shape_w : 16; /** Slice 长度信息 **/
    uint64_t basement_src1 : 2; /** None **/
    uint64_t basement_dest : 2; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr);
        bw.write<11>(ccrset);
        bw.write<32>(addr_src);
        bw.write<32>(addr_dest);
        bw.write<64>(stride_input_glb);
        bw.write<64>(stride_output_glb);
        bw.write<16>(shape_n);
        bw.write<16>(shape_c);
        bw.write<16>(shape_h);
        bw.write<16>(shape_w);
        bw.write<2>(basement_src1);
        bw.write<2>(basement_dest);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr = br.read<decltype(ccrclr), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src = br.read<decltype(addr_src), 32>();
        addr_dest = br.read<decltype(addr_dest), 32>();
        stride_input_glb = br.read<decltype(stride_input_glb), 64>();
        stride_output_glb = br.read<decltype(stride_output_glb), 64>();
        shape_n = br.read<decltype(shape_n), 16>();
        shape_c = br.read<decltype(shape_c), 16>();
        shape_h = br.read<decltype(shape_h), 16>();
        shape_w = br.read<decltype(shape_w), 16>();
        basement_src1 = br.read<decltype(basement_src1), 2>();
        basement_dest = br.read<decltype(basement_dest), 2>();
    }
};

/**
    MFU.MN.VMAP.COMPUTE
    @brief 配置MFU的DM模块，准备开始计算vector的map运算
    - OPCODE opcode : 8  配置MFU的DM模块，准备开始计算vector的map运算
    - CCRCLR ccrclr : 8  None
    - CCRSET ccrset : 11  None
    - UNION_ADDR addr_src : 32  输入向量地址 (GLB or DDR)
    - ADDR addr_dest : 32  输出向量地址 (GLB or DDR)
    - uint64_t length : 32  向量长度
    - uint64_t basement_src : 2  None
    - uint64_t basement_dest : 2  None
**/
struct INST_MFU_MN_VMAP_COMPUTE
{
    OPCODE opcode : 8; /** 配置MFU的DM模块，准备开始计算vector的map运算 **/
    uint64_t ccrclr : 8; /** None **/
    uint64_t ccrset : 11; /** None **/
    uint64_t addr_src : 32; /** 输入向量地址 (GLB or DDR) **/
    ADDR addr_dest : 32; /** 输出向量地址 (GLB or DDR) **/
    uint64_t length : 32; /** 向量长度 **/
    uint64_t basement_src : 2; /** None **/
    uint64_t basement_dest : 2; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr);
        bw.write<11>(ccrset);
        bw.write<32>(addr_src);
        bw.write<32>(addr_dest);
        bw.write<32>(length);
        bw.write<2>(basement_src);
        bw.write<2>(basement_dest);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr = br.read<decltype(ccrclr), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src = br.read<decltype(addr_src), 32>();
        addr_dest = br.read<decltype(addr_dest), 32>();
        length = br.read<decltype(length), 32>();
        basement_src = br.read<decltype(basement_src), 2>();
        basement_dest = br.read<decltype(basement_dest), 2>();
    }
};

/**
    MFU.REDUCE
    @brief 计算reduce，这个指令相当于bypass meshnet的reduce的短版本
    - OPCODE opcode : 8  计算reduce，这个指令相当于bypass
meshnet的reduce的短版本
    - CCRCLR ccrclr : 8  None
    - CCRSET ccrset : 11  None
    - UNION_ADDR addr_src : 32  输入向量地址 (GLB or DDR)
    - UNION_ADDR addr_dest : 32  输出到 GLB 或者 DDR 的位置
    - uint64_t init_value : 16  need_init 为 True 时有效，此时上面的 a1 为
init_value
    - STRIDE_GLB stride_input_glb : 64  Tensor 的 shape
    - uint64_t shape_n : 16  Slice 长度信息
    - uint64_t shape_c : 16  Slice 长度信息
    - uint64_t shape_h : 16  Slice 长度信息
    - uint64_t shape_w : 16  Slice 长度信息
    - MFU_REDUCE_OP op : 3  参与的运算 Scalar Binary Operator（含义见上方）
    - MFU_REDUCE_DIM dimension : 2  None
    - uint64_t basement_src : 2  是否是栈变量
    - uint64_t basement_dest : 2  是否是栈变量
**/
struct INST_MFU_REDUCE
{
    OPCODE opcode : 8; /** 计算reduce，这个指令相当于bypass
                              meshnet的reduce的短版本 **/
    uint64_t ccrclr : 8; /** None **/
    uint64_t ccrset : 11; /** None **/
    uint64_t addr_src : 32; /** 输入向量地址 (GLB or DDR) **/
    uint64_t addr_dest : 32; /** 输出到 GLB 或者 DDR 的位置 **/
    uint64_t init_value : 16; /** need_init 为 True 时有效，此时上面的 a1 为
                               init_value **/
    uint64_t stride_input_glb : 64; /** Tensor 的 shape **/
    uint64_t shape_n : 16; /** Slice 长度信息 **/
    uint64_t shape_c : 16; /** Slice 长度信息 **/
    uint64_t shape_h : 16; /** Slice 长度信息 **/
    uint64_t shape_w : 16; /** Slice 长度信息 **/
    MFU_REDUCE_OP op : 3; /** 参与的运算 Scalar Binary Operator（含义见上方） **/
    MFU_REDUCE_DIM dimension : 2; /** None **/
    uint64_t basement_src : 2; /** 是否是栈变量 **/
    uint64_t basement_dest : 2; /** 是否是栈变量 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr);
        bw.write<11>(ccrset);
        bw.write<32>(addr_src);
        bw.write<32>(addr_dest);
        bw.write<16>(init_value);
        bw.write<64>(stride_input_glb);
        bw.write<16>(shape_n);
        bw.write<16>(shape_c);
        bw.write<16>(shape_h);
        bw.write<16>(shape_w);
        bw.write<3>(op);
        bw.write<2>(dimension);
        bw.write<2>(basement_src);
        bw.write<2>(basement_dest);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr = br.read<decltype(ccrclr), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src = br.read<decltype(addr_src), 32>();
        addr_dest = br.read<decltype(addr_dest), 32>();
        init_value = br.read<decltype(init_value), 16>();
        stride_input_glb = br.read<decltype(stride_input_glb), 64>();
        shape_n = br.read<decltype(shape_n), 16>();
        shape_c = br.read<decltype(shape_c), 16>();
        shape_h = br.read<decltype(shape_h), 16>();
        shape_w = br.read<decltype(shape_w), 16>();
        op = br.read<decltype(op), 3>();
        dimension = br.read<decltype(dimension), 2>();
        basement_src = br.read<decltype(basement_src), 2>();
        basement_dest = br.read<decltype(basement_dest), 2>();
    }
};

/**
    MFU.VREDUCE
    @brief 计算reduce，这个指令相当于bypass meshnet的reduce的短版本
    - OPCODE opcode : 8  计算reduce，这个指令相当于bypass
meshnet的reduce的短版本
    - CCRCLR ccrclr : 8  None
    - CCRSET ccrset : 11  None
    - UNION_ADDR addr_src : 32  输入向量地址 (GLB or DDR)
    - UNION_ADDR addr_dest : 32  输出到 GLB 或者 DDR 的位置
    - uint64_t init_value : 16  need_init 为 True 时有效，此时上面的 a1 为
init_value
    - uint64_t length : 32  向量输出长度（总长度是length*reduce_length）
    - uint64_t reduce_length : 16  每这么多个点输出一个点
    - MFU_REDUCE_OP op : 3  参与的运算 Scalar Binary Operator（含义见上方）
    - uint64_t basement_src : 2  是否是栈变量
    - uint64_t basement_dest : 2  是否是栈变量
**/
struct INST_MFU_VREDUCE
{
    OPCODE opcode : 8; /** 计算reduce，这个指令相当于bypass
                              meshnet的reduce的短版本 **/
    uint64_t ccrclr : 8; /** None **/
    uint64_t ccrset : 11; /** None **/
    uint64_t addr_src : 32; /** 输入向量地址 (GLB or DDR) **/
    uint64_t addr_dest : 32; /** 输出到 GLB 或者 DDR 的位置 **/
    uint64_t init_value : 16; /** need_init 为 True 时有效，此时上面的 a1 为
                               init_value **/
    uint64_t length : 32; /** 向量输出长度（总长度是length*reduce_length） **/
    uint64_t reduce_length : 16; /** 每这么多个点输出一个点 **/
    MFU_REDUCE_OP op : 3; /** 参与的运算 Scalar Binary Operator（含义见上方） **/
    uint64_t basement_src : 2; /** 是否是栈变量 **/
    uint64_t basement_dest : 2; /** 是否是栈变量 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr);
        bw.write<11>(ccrset);
        bw.write<32>(addr_src);
        bw.write<32>(addr_dest);
        bw.write<16>(init_value);
        bw.write<32>(length);
        bw.write<16>(reduce_length);
        bw.write<3>(op);
        bw.write<2>(basement_src);
        bw.write<2>(basement_dest);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr = br.read<decltype(ccrclr), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src = br.read<decltype(addr_src), 32>();
        addr_dest = br.read<decltype(addr_dest), 32>();
        init_value = br.read<decltype(init_value), 16>();
        length = br.read<decltype(length), 32>();
        reduce_length = br.read<decltype(reduce_length), 16>();
        op = br.read<decltype(op), 3>();
        basement_src = br.read<decltype(basement_src), 2>();
        basement_dest = br.read<decltype(basement_dest), 2>();
    }
};

/**
    MFU.MN.BROADCAST.COMPUTE
    @brief 配置MFU的DM模块，准备开始计算
    - OPCODE opcode : 8  配置MFU的DM模块，准备开始计算
    - CCRCLR ccrclr_src1 : 8  None
    - CCRCLR ccrclr_src2 : 8  None
    - CCRSET ccrset : 11  None
    - UNION_ADDR addr_src1 : 32  向量输入1地址
    - UNION_ADDR addr_src2 : 32  向量输入2地址
    - UNION_ADDR addr_dest : 32  向量输出地址
    - uint64_t len_src1 : 32  向量输入1长度
    - uint64_t len_src2 : 32  向量输入2长度
    - uint64_t len_dest : 32
输出向量长度（超过了之后就会停止，和src1，src2的总长度成倍数，实现在左面广播）
    - uint64_t basement_src1 : 2  None
    - uint64_t basement_src2 : 2  None
    - uint64_t basement_dest : 2  None
**/
struct INST_MFU_MN_BROADCAST_COMPUTE
{
    OPCODE opcode : 8; /** 配置MFU的DM模块，准备开始计算 **/
    uint64_t ccrclr_src1 : 8; /** None **/
    uint64_t ccrclr_src2 : 8; /** None **/
    uint64_t ccrset : 11; /** None **/
    uint64_t addr_src1 : 32; /** 向量输入1地址 **/
    uint64_t addr_src2 : 32; /** 向量输入2地址 **/
    uint64_t addr_dest : 32; /** 向量输出地址 **/
    uint64_t len_src1 : 32; /** 向量输入1长度 **/
    uint64_t len_src2 : 32; /** 向量输入2长度 **/
    uint64_t len_dest : 32; /**
                                 输出向量长度（超过了之后就会停止，和src1，src2的总长度成倍数，实现在左面广播）
                                 **/
    uint64_t basement_src1 : 2; /** None **/
    uint64_t basement_src2 : 2; /** None **/
    uint64_t basement_dest : 2; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr_src1);
        bw.write<8>(ccrclr_src2);
        bw.write<11>(ccrset);
        bw.write<32>(addr_src1);
        bw.write<32>(addr_src2);
        bw.write<32>(addr_dest);
        bw.write<32>(len_src1);
        bw.write<32>(len_src2);
        bw.write<32>(len_dest);
        bw.write<2>(basement_src1);
        bw.write<2>(basement_src2);
        bw.write<2>(basement_dest);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr_src1 = br.read<decltype(ccrclr_src1), 8>();
        ccrclr_src2 = br.read<decltype(ccrclr_src2), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src1 = br.read<decltype(addr_src1), 32>();
        addr_src2 = br.read<decltype(addr_src2), 32>();
        addr_dest = br.read<decltype(addr_dest), 32>();
        len_src1 = br.read<decltype(len_src1), 32>();
        len_src2 = br.read<decltype(len_src2), 32>();
        len_dest = br.read<decltype(len_dest), 32>();
        basement_src1 = br.read<decltype(basement_src1), 2>();
        basement_src2 = br.read<decltype(basement_src2), 2>();
        basement_dest = br.read<decltype(basement_dest), 2>();
    }
};

/**
    MFU.MN.REDUCE
    @brief 这条指令用于配置Fuse在Meshnet后面的Reduce
    - OPCODE opcode : 8  这条指令用于配置Fuse在Meshnet后面的Reduce
    - uint64_t init_value : 16  need_init 为 True 时有效，此时上面的 a1 为
init_value
    - uint64_t recude_length : 16  有多少个元素参与一次reduce
    - uint64_t length : 29  reduce元素的次数
    - MFU_REDUCE_OP op : 3  参与的运算 Scalar Binary Operator
**/
struct INST_MFU_MN_REDUCE
{
    OPCODE opcode : 8; /** 这条指令用于配置Fuse在Meshnet后面的Reduce **/
    uint64_t init_value : 16; /** need_init 为 True 时有效，此时上面的 a1 为
                               init_value **/
    uint64_t recude_length : 16; /** 有多少个元素参与一次reduce **/
    uint64_t length : 29; /** reduce元素的次数 **/
    MFU_REDUCE_OP op : 3; /** 参与的运算 Scalar Binary Operator **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<16>(init_value);
        bw.write<16>(recude_length);
        bw.write<29>(length);
        bw.write<3>(op);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        init_value = br.read<decltype(init_value), 16>();
        recude_length = br.read<decltype(recude_length), 16>();
        length = br.read<decltype(length), 29>();
        op = br.read<decltype(op), 3>();
    }
};

struct INST_MFU_MN_CONF
{
    OPCODE opcode : 8; /**
                               配置互联网络。我们认为某个原件编号为0就是与网络断开连接并且gate掉。整个网络连接初始值都是0
                               **/
    MFU_MN_PORTOUT out1 : 6; /** VECTOR_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out2 : 6; /** VECTOR_OUT_1连接的端口 **/
    MFU_MN_PORTOUT out3 : 6; /** CONST1_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out4 : 6; /** CONST1_OUT_1连接的端口 **/
    MFU_MN_PORTOUT out5 : 6; /** CONST2_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out6 : 6; /** CONST2_OUT_1连接的端口 **/
    MFU_MN_PORTOUT out7 : 6; /** ADDSUB0_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out8 : 6; /** ADDSUB1_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out9 : 6; /** ADDSUB2_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out10 : 6; /** ADDSUB3_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out11 : 6; /** MUL0_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out12 : 6; /** MUL1_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out13 : 6; /** MUL2_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out14 : 6; /** MUL3_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out15 : 6; /** DIV_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out16 : 6; /** ROUND_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out17 : 6; /** SQRT_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out18 : 6; /** TRANGLE_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out19 : 6; /** LOG_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out20 : 6; /** UNARY_LOGIC_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out21 : 6; /** EXP_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out22 : 6; /** CMP0_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out23 : 6; /** CMP1_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out24 : 6; /** REG0_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out25 : 6; /** SELECT0_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out26 : 6; /** NOP0_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out27 : 6; /** NOP1_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out28 : 6; /** NOP2_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out29 : 6; /** NOP3_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out30 : 6; /** NOP4_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out31 : 6; /** NOP5_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out32 : 6; /** NOP6_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out33 : 6; /** NOP7_OUT_0连接的端口 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<6>(out1);
        bw.write<6>(out2);
        bw.write<6>(out3);
        bw.write<6>(out4);
        bw.write<6>(out5);
        bw.write<6>(out6);
        bw.write<6>(out7);
        bw.write<6>(out8);
        bw.write<6>(out9);
        bw.write<6>(out10);
        bw.write<6>(out11);
        bw.write<6>(out12);
        bw.write<6>(out13);
        bw.write<6>(out14);
        bw.write<6>(out15);
        bw.write<6>(out16);
        bw.write<6>(out17);
        bw.write<6>(out18);
        bw.write<6>(out19);
        bw.write<6>(out20);
        bw.write<6>(out21);
        bw.write<6>(out22);
        bw.write<6>(out23);
        bw.write<6>(out24);
        bw.write<6>(out25);
        bw.write<6>(out26);
        bw.write<6>(out27);
        bw.write<6>(out28);
        bw.write<6>(out29);
        bw.write<6>(out30);
        bw.write<6>(out31);
        bw.write<6>(out32);
        bw.write<6>(out33);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        out1 = br.read<decltype(out1), 6>();
        out2 = br.read<decltype(out2), 6>();
        out3 = br.read<decltype(out3), 6>();
        out4 = br.read<decltype(out4), 6>();
        out5 = br.read<decltype(out5), 6>();
        out6 = br.read<decltype(out6), 6>();
        out7 = br.read<decltype(out7), 6>();
        out8 = br.read<decltype(out8), 6>();
        out9 = br.read<decltype(out9), 6>();
        out10 = br.read<decltype(out10), 6>();
        out11 = br.read<decltype(out11), 6>();
        out12 = br.read<decltype(out12), 6>();
        out13 = br.read<decltype(out13), 6>();
        out14 = br.read<decltype(out14), 6>();
        out15 = br.read<decltype(out15), 6>();
        out16 = br.read<decltype(out16), 6>();
        out17 = br.read<decltype(out17), 6>();
        out18 = br.read<decltype(out18), 6>();
        out19 = br.read<decltype(out19), 6>();
        out20 = br.read<decltype(out20), 6>();
        out21 = br.read<decltype(out21), 6>();
        out22 = br.read<decltype(out22), 6>();
        out23 = br.read<decltype(out23), 6>();
        out24 = br.read<decltype(out24), 6>();
        out25 = br.read<decltype(out25), 6>();
        out26 = br.read<decltype(out26), 6>();
        out27 = br.read<decltype(out27), 6>();
        out28 = br.read<decltype(out28), 6>();
        out29 = br.read<decltype(out29), 6>();
        out30 = br.read<decltype(out30), 6>();
        out31 = br.read<decltype(out31), 6>();
        out32 = br.read<decltype(out32), 6>();
        out33 = br.read<decltype(out33), 6>();
    }
};

/**
    MFU.MNOP.CONF
    @brief
这条指令用来配置原件的内部寄存器，比如group的操作的选通，或者是拟合运算的拟合表的配置
    - OPCODE opcode : 8
这条指令用来配置原件的内部寄存器，比如group的操作的选通，或者是拟合运算的拟合表的配置
    - uint64_t mode : 2  模式。
0：配置的是各种原件的标志bit
1：配置拟合模块0的参数地址
2：配置拟合模块1的参数地址
    - uint64_t val : 32  配置
**/
struct INST_MFU_MNOP_CONF
{
    OPCODE opcode : 8; /**
                        这条指令用来配置原件的内部寄存器，比如group的操作的选通，或者是拟合运算的拟合表的配置
                        **/
    uint64_t mode : 2; /** 模式。
0：配置的是各种原件的标志bit
1：配置拟合模块0的参数地址
2：配置拟合模块1的参数地址 **/
    uint64_t val : 32; /** 配置 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<2>(mode);
        bw.write<32>(val);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        mode = br.read<decltype(mode), 2>();
        val = br.read<decltype(val), 32>();
    }
};

/**
    MFU.PDP.CONF
    @brief 主要用于配置 PDP 模块计算中共用的一些全局配置
    - OPCODE opcode : 8  主要用于配置 PDP 模块计算中共用的一些全局配置
    - uint64_t multiple_channels : 8  是否在多个 channel
上同时进行一样窗口和窗口移动规则。
    - STRIDE_GLB stride_dest_glb : 64  None
**/
struct INST_MFU_PDP_CONF
{
    OPCODE opcode : 8; /** 主要用于配置 PDP 模块计算中共用的一些全局配置 **/
    uint64_t multiple_channels : 8; /** 是否在多个 channel
                                     上同时进行一样窗口和窗口移动规则。 **/
    uint64_t stride_dest_glb : 64; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(multiple_channels);
        bw.write<64>(stride_dest_glb);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        multiple_channels = br.read<decltype(multiple_channels), 8>();
        stride_dest_glb = br.read<decltype(stride_dest_glb), 64>();
    }
};

/**
    MFU.PDP.SRC.CONF
    @brief  主要用来配置待操作的 Tensor 的位置
    - OPCODE opcode : 8   主要用来配置待操作的 Tensor 的位置
    - STRIDE_GLB stride_glb : 64  Tensor 的 shape
**/
struct INST_MFU_PDP_SRC_CONF
{
    OPCODE opcode : 8; /**  主要用来配置待操作的 Tensor 的位置 **/
    uint64_t stride_glb : 64; /** Tensor 的 shape **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<64>(stride_glb);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        stride_glb = br.read<decltype(stride_glb), 64>();
    }
};

/**
    MFU.PDP.REDUCE
    @brief 进行PDP运算
    - OPCODE opcode : 8  进行PDP运算
    - CCRCLR ccrclr : 8  None
    - CCRSET ccrset : 11  None
    - ADDR_GLB_8_WITH_BANK addr_src : 25  输入向量地址 (GLB or
DDR)（slice起始地址）
    - ADDR_GLB_8_WITH_BANK addr_dest : 25  输出位置
    - uint64_t window_w : 5  2D 窗口的宽
    - uint64_t window_h : 5  2D 窗口的高
    - uint64_t active_h : 5  PDP阵列硬件使能的高度（需要是window_h的整数倍）
    - uint64_t shape_n : 16  None
    - uint64_t shape_c : 16  None
    - uint64_t shape_h : 16  None
    - uint64_t shape_w : 16  None
    - uint64_t count_w : 16  W 维度上做 N 次 ReduceWindow 操作
    - uint64_t count_h : 16  H 维度上做 N 次 ReduceWindow操作
    - uint64_t stride_w : 8  W 维度上每次取窗口的下标增加量
    - uint64_t stride_h : 8  H 维度上每次取窗口的下标增加量
    - uint64_t padding_top : 8  在 2D Tensor 的顶部加的 Zero padding 行数
    - uint64_t padding_bottom : 8  在 2D Tensor 的底部加的 Zero padding 行数
    - uint64_t padding_left : 8  在 2D Tensor 的左边加的 Zero padding 行数
    - uint64_t padding_right : 8  在 2D Tensor 的右边加的 Zero padding 行数
    - uint64_t pe_last_h : 8  来表明最后一次计算需要使能多少个pdp单元
    - MFU_PDP_OP computation : 2  我们只支持 Map/Reduce 二元算子中的 Max/Min/Avg
三种
    - uint64_t quantized : 1  是否是量化计算
**/
struct INST_MFU_PDP_REDUCE
{
    OPCODE opcode : 8; /** 进行PDP运算 **/
    uint64_t ccrclr : 8; /** None **/
    uint64_t ccrset : 11; /** None **/
    uint64_t addr_src : 25; /** 输入向量地址 (GLB or DDR)（slice起始地址） **/
    uint64_t addr_dest : 25; /** 输出位置 **/
    uint64_t window_w : 5; /** 2D 窗口的宽 **/
    uint64_t window_h : 5; /** 2D 窗口的高 **/
    uint64_t active_h : 5; /** PDP阵列硬件使能的高度（需要是window_h的整数倍） **/
    uint64_t shape_n : 16; /** None **/
    uint64_t shape_c : 16; /** None **/
    uint64_t shape_h : 16; /** None **/
    uint64_t shape_w : 16; /** None **/
    uint64_t count_w : 16; /** W 维度上做 N 次 ReduceWindow 操作 **/
    uint64_t count_h : 16; /** H 维度上做 N 次 ReduceWindow操作 **/
    uint64_t stride_w : 8; /** W 维度上每次取窗口的下标增加量 **/
    uint64_t stride_h : 8; /** H 维度上每次取窗口的下标增加量 **/
    uint64_t padding_top : 8; /** 在 2D Tensor 的顶部加的 Zero padding 行数 **/
    uint64_t padding_bottom : 8; /** 在 2D Tensor 的底部加的 Zero padding 行数 **/
    uint64_t padding_left : 8; /** 在 2D Tensor 的左边加的 Zero padding 行数 **/
    uint64_t padding_right : 8; /** 在 2D Tensor 的右边加的 Zero padding 行数 **/
    uint64_t pe_last_h : 8; /** 来表明最后一次计算需要使能多少个pdp单元 **/
    MFU_PDP_OP computation : 2; /** 我们只支持 Map/Reduce 二元算子中的 Max/Min/Avg
                                 三种 **/
    uint64_t quantized : 1; /** 是否是量化计算 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr);
        bw.write<11>(ccrset);
        bw.write<25>(addr_src);
        bw.write<25>(addr_dest);
        bw.write<5>(window_w);
        bw.write<5>(window_h);
        bw.write<5>(active_h);
        bw.write<16>(shape_n);
        bw.write<16>(shape_c);
        bw.write<16>(shape_h);
        bw.write<16>(shape_w);
        bw.write<16>(count_w);
        bw.write<16>(count_h);
        bw.write<8>(stride_w);
        bw.write<8>(stride_h);
        bw.write<8>(padding_top);
        bw.write<8>(padding_bottom);
        bw.write<8>(padding_left);
        bw.write<8>(padding_right);
        bw.write<8>(pe_last_h);
        bw.write<2>(computation);
        bw.write<1>(quantized);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr = br.read<decltype(ccrclr), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src = br.read<decltype(addr_src), 25>();
        addr_dest = br.read<decltype(addr_dest), 25>();
        window_w = br.read<decltype(window_w), 5>();
        window_h = br.read<decltype(window_h), 5>();
        active_h = br.read<decltype(active_h), 5>();
        shape_n = br.read<decltype(shape_n), 16>();
        shape_c = br.read<decltype(shape_c), 16>();
        shape_h = br.read<decltype(shape_h), 16>();
        shape_w = br.read<decltype(shape_w), 16>();
        count_w = br.read<decltype(count_w), 16>();
        count_h = br.read<decltype(count_h), 16>();
        stride_w = br.read<decltype(stride_w), 8>();
        stride_h = br.read<decltype(stride_h), 8>();
        padding_top = br.read<decltype(padding_top), 8>();
        padding_bottom = br.read<decltype(padding_bottom), 8>();
        padding_left = br.read<decltype(padding_left), 8>();
        padding_right = br.read<decltype(padding_right), 8>();
        pe_last_h = br.read<decltype(pe_last_h), 8>();
        computation = br.read<decltype(computation), 2>();
        quantized = br.read<decltype(quantized), 1>();
    }
};

/**
    MFU.MN.BROADCAST.CONF
    @brief None
    - OPCODE opcode : 8  None
    - uint64_t slice_src1 : 16  向量输入1 slice长度
    - uint64_t slice_src2 : 16  向量输入2 slice长度
    - uint64_t repeats_src1 : 16  向量输入1每个元素重复的次数（在右面广播）
    - uint64_t repeats_src2 : 16  向量输入2每个元素重复的次数（在右面广播）
    - uint64_t slice_repeats_src1 : 16  向量输入1 slice次数（实现在中间广播）
    - uint64_t slice_repeats_src2 : 16  向量输入2 slice次数（实现在中间广播）
    - STRIDE_GLB stride_src1_glb : 64  None
    - uint64_t shape_src1_n : 16  None
    - uint64_t shape_src1_c : 16  None
    - uint64_t shape_src1_h : 16  None
    - uint64_t shape_src1_w : 16  None
    - STRIDE_GLB stride_src2_glb : 64  None
    - uint64_t shape_src2_n : 16  None
    - uint64_t shape_src2_c : 16  None
    - uint64_t shape_src2_h : 16  None
    - uint64_t shape_src2_w : 16  None
    - uint64_t const1 : 16  常量1
    - uint64_t const2 : 16  常量2
    - uint64_t const3 : 16  常量3
    - uint64_t const4 : 16  常量4
**/
struct INST_MFU_MN_BROADCAST_CONF
{
    OPCODE opcode : 8; /** None **/
    uint64_t slice_src1 : 16; /** 向量输入1 slice长度 **/
    uint64_t slice_src2 : 16; /** 向量输入2 slice长度 **/
    uint64_t repeats_src1 : 16; /** 向量输入1每个元素重复的次数（在右面广播） **/
    uint64_t repeats_src2 : 16; /** 向量输入2每个元素重复的次数（在右面广播） **/
    uint64_t
        slice_repeats_src1 : 16; /** 向量输入1 slice次数（实现在中间广播） **/
    uint64_t
        slice_repeats_src2 : 16; /** 向量输入2 slice次数（实现在中间广播） **/
    uint64_t stride_src1_glb : 64; /** None **/
    uint64_t shape_src1_n : 16; /** None **/
    uint64_t shape_src1_c : 16; /** None **/
    uint64_t shape_src1_h : 16; /** None **/
    uint64_t shape_src1_w : 16; /** None **/
    uint64_t stride_src2_glb : 64; /** None **/
    uint64_t shape_src2_n : 16; /** None **/
    uint64_t shape_src2_c : 16; /** None **/
    uint64_t shape_src2_h : 16; /** None **/
    uint64_t shape_src2_w : 16; /** None **/
    uint64_t const1 : 16; /** 常量1 **/
    uint64_t const2 : 16; /** 常量2 **/
    uint64_t const3 : 16; /** 常量3 **/
    uint64_t const4 : 16; /** 常量4 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<16>(slice_src1);
        bw.write<16>(slice_src2);
        bw.write<16>(repeats_src1);
        bw.write<16>(repeats_src2);
        bw.write<16>(slice_repeats_src1);
        bw.write<16>(slice_repeats_src2);
        bw.write<64>(stride_src1_glb);
        bw.write<16>(shape_src1_n);
        bw.write<16>(shape_src1_c);
        bw.write<16>(shape_src1_h);
        bw.write<16>(shape_src1_w);
        bw.write<64>(stride_src2_glb);
        bw.write<16>(shape_src2_n);
        bw.write<16>(shape_src2_c);
        bw.write<16>(shape_src2_h);
        bw.write<16>(shape_src2_w);
        bw.write<16>(const1);
        bw.write<16>(const2);
        bw.write<16>(const3);
        bw.write<16>(const4);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        slice_src1 = br.read<decltype(slice_src1), 16>();
        slice_src2 = br.read<decltype(slice_src2), 16>();
        repeats_src1 = br.read<decltype(repeats_src1), 16>();
        repeats_src2 = br.read<decltype(repeats_src2), 16>();
        slice_repeats_src1 = br.read<decltype(slice_repeats_src1), 16>();
        slice_repeats_src2 = br.read<decltype(slice_repeats_src2), 16>();
        stride_src1_glb = br.read<decltype(stride_src1_glb), 64>();
        shape_src1_n = br.read<decltype(shape_src1_n), 16>();
        shape_src1_c = br.read<decltype(shape_src1_c), 16>();
        shape_src1_h = br.read<decltype(shape_src1_h), 16>();
        shape_src1_w = br.read<decltype(shape_src1_w), 16>();
        stride_src2_glb = br.read<decltype(stride_src2_glb), 64>();
        shape_src2_n = br.read<decltype(shape_src2_n), 16>();
        shape_src2_c = br.read<decltype(shape_src2_c), 16>();
        shape_src2_h = br.read<decltype(shape_src2_h), 16>();
        shape_src2_w = br.read<decltype(shape_src2_w), 16>();
        const1 = br.read<decltype(const1), 16>();
        const2 = br.read<decltype(const2), 16>();
        const3 = br.read<decltype(const3), 16>();
        const4 = br.read<decltype(const4), 16>();
    }
};

/**
    MFU.CROP
    @brief None
    - OPCODE opcode : 8  None
    - CCRCLR ccrclr : 8  None
    - CCRSET ccrset : 11  None
    - ADDR_GLB_8_WITH_BANK addr_src : 25  None
    - ADDR_GLB_8_WITH_BANK addr_dest : 25  None
    - ADDR_GLB_8_WITH_BANK addr_bbox : 25  None
    - uint64_t reserve : 16  None
    - uint64_t shape_src_c : 16  None
    - uint64_t shape_src_h : 16  None
    - uint64_t shape_src_w : 16  None
    - STRIDE_GLB stride_src_glb : 64  None
    - STRIDE_GLB stride_dest_glb : 64  None
    - uint64_t roi_amount : 16  None
    - uint64_t dest_h : 16  None
    - uint64_t dest_w : 16  None
    - BF24 step_h : 24  None
    - BF24 step_w : 24  None
    - MFU_CROP_ALIGN align_method : 2  None
    - MFU_CROP_RESIZE resize_method : 1  None
**/
struct INST_MFU_CROP
{
    OPCODE opcode : 8; /** None **/
    uint64_t ccrclr : 8; /** None **/
    uint64_t ccrset : 11; /** None **/
    uint64_t addr_src : 25; /** None **/
    uint64_t addr_dest : 25; /** None **/
    uint64_t addr_bbox : 25; /** None **/
    uint64_t reserve : 16; /** None **/
    uint64_t shape_src_c : 16; /** None **/
    uint64_t shape_src_h : 16; /** None **/
    uint64_t shape_src_w : 16; /** None **/
    uint64_t stride_src_glb : 64; /** None **/
    uint64_t stride_dest_glb : 64; /** None **/
    uint64_t roi_amount : 16; /** None **/
    uint64_t dest_h : 16; /** None **/
    uint64_t dest_w : 16; /** None **/
    BF24 step_h : 24; /** None **/
    BF24 step_w : 24; /** None **/
    MFU_CROP_ALIGN align_method : 2; /** None **/
    MFU_CROP_RESIZE resize_method : 1; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr);
        bw.write<11>(ccrset);
        bw.write<25>(addr_src);
        bw.write<25>(addr_dest);
        bw.write<25>(addr_bbox);
        bw.write<16>(reserve);
        bw.write<16>(shape_src_c);
        bw.write<16>(shape_src_h);
        bw.write<16>(shape_src_w);
        bw.write<64>(stride_src_glb);
        bw.write<64>(stride_dest_glb);
        bw.write<16>(roi_amount);
        bw.write<16>(dest_h);
        bw.write<16>(dest_w);
        bw.write<24>(step_h);
        bw.write<24>(step_w);
        bw.write<2>(align_method);
        bw.write<1>(resize_method);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr = br.read<decltype(ccrclr), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src = br.read<decltype(addr_src), 25>();
        addr_dest = br.read<decltype(addr_dest), 25>();
        addr_bbox = br.read<decltype(addr_bbox), 25>();
        reserve = br.read<decltype(reserve), 16>();
        shape_src_c = br.read<decltype(shape_src_c), 16>();
        shape_src_h = br.read<decltype(shape_src_h), 16>();
        shape_src_w = br.read<decltype(shape_src_w), 16>();
        stride_src_glb = br.read<decltype(stride_src_glb), 64>();
        stride_dest_glb = br.read<decltype(stride_dest_glb), 64>();
        roi_amount = br.read<decltype(roi_amount), 16>();
        dest_h = br.read<decltype(dest_h), 16>();
        dest_w = br.read<decltype(dest_w), 16>();
        step_h = br.read<decltype(step_h), 24>();
        step_w = br.read<decltype(step_w), 24>();
        align_method = br.read<decltype(align_method), 2>();
        resize_method = br.read<decltype(resize_method), 1>();
    }
};

/**
    MFU.MEMSET
    @brief None
    - OPCODE opcode : 8  None
    - CCRSET ccrset : 11  None
    - ADDR_GLB_8_WITH_BANK addr_dest : 25  None
    - uint64_t imm : 16  None
    - uint64_t len : 20  None
**/
struct INST_MFU_MEMSET
{
    OPCODE opcode : 8; /** None **/
    uint64_t ccrset : 11; /** None **/
    uint64_t addr_dest : 25; /** None **/
    uint64_t imm : 16; /** None **/
    uint64_t len : 20; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<11>(ccrset);
        bw.write<25>(addr_dest);
        bw.write<16>(imm);
        bw.write<20>(len);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_dest = br.read<decltype(addr_dest), 25>();
        imm = br.read<decltype(imm), 16>();
        len = br.read<decltype(len), 20>();
    }
};

/**
    MFU.MEMCPY
    @brief None
    - OPCODE opcode : 8  None
    - CCRCLR ccrclr : 8  None
    - CCRSET ccrset : 11  None
    - ADDR_GLB_8_WITH_BANK addr_src : 25  None
    - ADDR_GLB_8_WITH_BANK addr_dest : 25  None
    - STRIDE_GLB stride_src_glb : 64  None
    - STRIDE_GLB stride_dest_glb : 64  None
    - uint64_t shape_n : 16  None
    - uint64_t shape_c : 16  None
    - uint64_t shape_h : 16  None
    - uint64_t shape_w : 16  None
    - PRECISION precision_glb : 2  None
**/
struct INST_MFU_MEMCPY
{
    OPCODE opcode : 8; /** None **/
    uint64_t ccrclr : 8; /** None **/
    uint64_t ccrset : 11; /** None **/
    uint64_t addr_src : 25; /** None **/
    uint64_t addr_dest : 25; /** None **/
    uint64_t stride_src_glb : 64; /** None **/
    uint64_t stride_dest_glb : 64; /** None **/
    uint64_t shape_n : 16; /** None **/
    uint64_t shape_c : 16; /** None **/
    uint64_t shape_h : 16; /** None **/
    uint64_t shape_w : 16; /** None **/
    PRECISION precision_glb : 2; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr);
        bw.write<11>(ccrset);
        bw.write<25>(addr_src);
        bw.write<25>(addr_dest);
        bw.write<64>(stride_src_glb);
        bw.write<64>(stride_dest_glb);
        bw.write<16>(shape_n);
        bw.write<16>(shape_c);
        bw.write<16>(shape_h);
        bw.write<16>(shape_w);
        bw.write<2>(precision_glb);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr = br.read<decltype(ccrclr), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src = br.read<decltype(addr_src), 25>();
        addr_dest = br.read<decltype(addr_dest), 25>();
        stride_src_glb = br.read<decltype(stride_src_glb), 64>();
        stride_dest_glb = br.read<decltype(stride_dest_glb), 64>();
        shape_n = br.read<decltype(shape_n), 16>();
        shape_c = br.read<decltype(shape_c), 16>();
        shape_h = br.read<decltype(shape_h), 16>();
        shape_w = br.read<decltype(shape_w), 16>();
        precision_glb = br.read<decltype(precision_glb), 2>();
    }
};

/**
    MFU.TRANS
    @brief None
    - OPCODE opcode : 8  None
    - CCRCLR ccrclr : 8  None
    - CCRSET ccrset : 11  None
    - ADDR_GLB_8_WITH_BANK addr_src : 25  None
    - ADDR_GLB_8_WITH_BANK addr_dest : 25  None
    - STRIDE_GLB stride_src_glb : 64  None
    - STRIDE_GLB stride_dest_glb : 64  None
    - uint64_t shape_n : 16  None
    - uint64_t shape_c : 16  None
    - uint64_t shape_h : 16  None
    - uint64_t shape_w : 16  None
    - PRECISION precision_glb : 2  None
    - MFU_TRANS_PERMUTE permute : 5  None
**/
struct INST_MFU_TRANS
{
    OPCODE opcode : 8; /** None **/
    uint64_t ccrclr : 8; /** None **/
    uint64_t ccrset : 11; /** None **/
    uint64_t addr_src : 25; /** None **/
    uint64_t addr_dest : 25; /** None **/
    uint64_t stride_src_glb : 64; /** None **/
    uint64_t stride_dest_glb : 64; /** None **/
    uint64_t shape_n : 16; /** None **/
    uint64_t shape_c : 16; /** None **/
    uint64_t shape_h : 16; /** None **/
    uint64_t shape_w : 16; /** None **/
    PRECISION precision_glb : 2; /** None **/
    MFU_TRANS_PERMUTE permute : 5; /** None **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<8>(ccrclr);
        bw.write<11>(ccrset);
        bw.write<25>(addr_src);
        bw.write<25>(addr_dest);
        bw.write<64>(stride_src_glb);
        bw.write<64>(stride_dest_glb);
        bw.write<16>(shape_n);
        bw.write<16>(shape_c);
        bw.write<16>(shape_h);
        bw.write<16>(shape_w);
        bw.write<2>(precision_glb);
        bw.write<5>(permute);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        ccrclr = br.read<decltype(ccrclr), 8>();
        ccrset = br.read<decltype(ccrset), 11>();
        addr_src = br.read<decltype(addr_src), 25>();
        addr_dest = br.read<decltype(addr_dest), 25>();
        stride_src_glb = br.read<decltype(stride_src_glb), 64>();
        stride_dest_glb = br.read<decltype(stride_dest_glb), 64>();
        shape_n = br.read<decltype(shape_n), 16>();
        shape_c = br.read<decltype(shape_c), 16>();
        shape_h = br.read<decltype(shape_h), 16>();
        shape_w = br.read<decltype(shape_w), 16>();
        precision_glb = br.read<decltype(precision_glb), 2>();
        permute = br.read<decltype(permute), 5>();
    }
};

struct INST_MFU_MN_CONF2
{
    OPCODE opcode : 8; /** 这条指令用来反向配置meshnet连接关系 **/
    MFU_MN_PORTIN in1 : 6; /** VECTOR_IN_0连接的端口 **/
    MFU_MN_PORTIN in2 : 6; /** ADDSUB0_IN_0连接的端口 **/
    MFU_MN_PORTIN in3 : 6; /** ADDSUB0_IN_1连接的端口 **/
    MFU_MN_PORTIN in4 : 6; /** ADDSUB1_IN_0连接的端口 **/
    MFU_MN_PORTIN in5 : 6; /** ADDSUB1_IN_1连接的端口 **/
    MFU_MN_PORTIN in6 : 6; /** ADDSUB2_IN_0连接的端口 **/
    MFU_MN_PORTIN in7 : 6; /** ADDSUB2_IN_1连接的端口 **/
    MFU_MN_PORTIN in8 : 6; /** ADDSUB3_IN_0连接的端口 **/
    MFU_MN_PORTIN in9 : 6; /** ADDSUB3_IN_1连接的端口 **/
    MFU_MN_PORTIN in10 : 6; /** MUL0_IN_0连接的端口 **/
    MFU_MN_PORTIN in11 : 6; /** MUL0_IN_1连接的端口 **/
    MFU_MN_PORTIN in12 : 6; /** MUL1_IN_0连接的端口 **/
    MFU_MN_PORTIN in13 : 6; /** MUL1_IN_1连接的端口 **/
    MFU_MN_PORTIN in14 : 6; /** MUL2_IN_0连接的端口 **/
    MFU_MN_PORTIN in15 : 6; /** MUL2_IN_1连接的端口 **/
    MFU_MN_PORTIN in16 : 6; /** MUL3_IN_0连接的端口 **/
    MFU_MN_PORTIN in17 : 6; /** MUL3_IN_1连接的端口 **/
    MFU_MN_PORTIN in18 : 6; /** DIV_IN_0连接的端口 **/
    MFU_MN_PORTIN in19 : 6; /** DIV_IN_1连接的端口 **/
    MFU_MN_PORTIN in20 : 6; /** ROUND_IN_0连接的端口 **/
    MFU_MN_PORTIN in21 : 6; /** SQRT_IN_0连接的端口 **/
    MFU_MN_PORTIN in22 : 6; /** TRANGLE_IN_0连接的端口 **/
    MFU_MN_PORTIN in23 : 6; /** LOG_IN_0连接的端口 **/
    MFU_MN_PORTIN in24 : 6; /** UNARY_LOGIC_IN_0连接的端口 **/
    MFU_MN_PORTIN in25 : 6; /** EXP_IN_0连接的端口 **/
    MFU_MN_PORTIN in26 : 6; /** CMP0_IN_0连接的端口 **/
    MFU_MN_PORTIN in27 : 6; /** CMP0_IN_1连接的端口 **/
    MFU_MN_PORTIN in28 : 6; /** CMP1_IN_0连接的端口 **/
    MFU_MN_PORTIN in29 : 6; /** CMP1_IN_1连接的端口 **/
    MFU_MN_PORTIN in30 : 6; /** REG0_IN_0连接的端口 **/
    MFU_MN_PORTIN in31 : 6; /** SELECT0_IN_0连接的端口 **/
    MFU_MN_PORTIN in32 : 6; /** SELECT0_IN_1连接的端口 **/
    MFU_MN_PORTIN in33 : 6; /** SELECT0_IN_2连接的端口 **/
    MFU_MN_PORTIN in34 : 6; /** NOP0_IN_0连接的端口 **/
    MFU_MN_PORTIN in35 : 6; /** NOP1_IN_0连接的端口 **/
    MFU_MN_PORTIN in36 : 6; /** NOP2_IN_0连接的端口 **/
    MFU_MN_PORTIN in37 : 6; /** NOP3_IN_0连接的端口 **/
    MFU_MN_PORTIN in38 : 6; /** NOP4_IN_0连接的端口 **/
    MFU_MN_PORTIN in39 : 6; /** NOP5_IN_0连接的端口 **/
    MFU_MN_PORTIN in40 : 6; /** NOP6_IN_0连接的端口 **/
    MFU_MN_PORTIN in41 : 6; /** NOP7_IN_0连接的端口 **/
    MFU_MN_PORTIN in42 : 6; /** CLEAR0_IN_0连接的端口 **/

    void serialize(bitwriter &bw) const
    {
        bw.write<8>(opcode);
        bw.write<6>(in1);
        bw.write<6>(in2);
        bw.write<6>(in3);
        bw.write<6>(in4);
        bw.write<6>(in5);
        bw.write<6>(in6);
        bw.write<6>(in7);
        bw.write<6>(in8);
        bw.write<6>(in9);
        bw.write<6>(in10);
        bw.write<6>(in11);
        bw.write<6>(in12);
        bw.write<6>(in13);
        bw.write<6>(in14);
        bw.write<6>(in15);
        bw.write<6>(in16);
        bw.write<6>(in17);
        bw.write<6>(in18);
        bw.write<6>(in19);
        bw.write<6>(in20);
        bw.write<6>(in21);
        bw.write<6>(in22);
        bw.write<6>(in23);
        bw.write<6>(in24);
        bw.write<6>(in25);
        bw.write<6>(in26);
        bw.write<6>(in27);
        bw.write<6>(in28);
        bw.write<6>(in29);
        bw.write<6>(in30);
        bw.write<6>(in31);
        bw.write<6>(in32);
        bw.write<6>(in33);
        bw.write<6>(in34);
        bw.write<6>(in35);
        bw.write<6>(in36);
        bw.write<6>(in37);
        bw.write<6>(in38);
        bw.write<6>(in39);
        bw.write<6>(in40);
        bw.write<6>(in41);
        bw.write<6>(in42);
    }

    void deserialize(bitreader &br)
    {
        opcode = br.read<decltype(opcode), 8>();
        in1 = br.read<decltype(in1), 6>();
        in2 = br.read<decltype(in2), 6>();
        in3 = br.read<decltype(in3), 6>();
        in4 = br.read<decltype(in4), 6>();
        in5 = br.read<decltype(in5), 6>();
        in6 = br.read<decltype(in6), 6>();
        in7 = br.read<decltype(in7), 6>();
        in8 = br.read<decltype(in8), 6>();
        in9 = br.read<decltype(in9), 6>();
        in10 = br.read<decltype(in10), 6>();
        in11 = br.read<decltype(in11), 6>();
        in12 = br.read<decltype(in12), 6>();
        in13 = br.read<decltype(in13), 6>();
        in14 = br.read<decltype(in14), 6>();
        in15 = br.read<decltype(in15), 6>();
        in16 = br.read<decltype(in16), 6>();
        in17 = br.read<decltype(in17), 6>();
        in18 = br.read<decltype(in18), 6>();
        in19 = br.read<decltype(in19), 6>();
        in20 = br.read<decltype(in20), 6>();
        in21 = br.read<decltype(in21), 6>();
        in22 = br.read<decltype(in22), 6>();
        in23 = br.read<decltype(in23), 6>();
        in24 = br.read<decltype(in24), 6>();
        in25 = br.read<decltype(in25), 6>();
        in26 = br.read<decltype(in26), 6>();
        in27 = br.read<decltype(in27), 6>();
        in28 = br.read<decltype(in28), 6>();
        in29 = br.read<decltype(in29), 6>();
        in30 = br.read<decltype(in30), 6>();
        in31 = br.read<decltype(in31), 6>();
        in32 = br.read<decltype(in32), 6>();
        in33 = br.read<decltype(in33), 6>();
        in34 = br.read<decltype(in34), 6>();
        in35 = br.read<decltype(in35), 6>();
        in36 = br.read<decltype(in36), 6>();
        in37 = br.read<decltype(in37), 6>();
        in38 = br.read<decltype(in38), 6>();
        in39 = br.read<decltype(in39), 6>();
        in40 = br.read<decltype(in40), 6>();
        in41 = br.read<decltype(in41), 6>();
        in42 = br.read<decltype(in42), 6>();
    }
};

////////////////////////////
class gnne_instruction;
class inst_nop;
class inst_li;
class inst_intr;
class inst_end;
class inst_fence;
class inst_mmu_conf;
class inst_fence_ccr;
class inst_loadif_config;
class inst_loadif;
class inst_load;
class inst_loadif_compress_conf;
class inst_load_compress_conf;
class inst_store;
class inst_store_t_config;
class inst_store_t;
class inst_store_t_compress_conf;
class inst_store_compress_conf;
class inst_tcu_dm_broadcast;
class inst_tcu_dm_conf_if;
class inst_tcu_dm_fetchif;
class inst_tcu_dm_conf_w;
class inst_tcu_dm_fetchw;
class inst_tcu_dm_conf_of;
class inst_tcu_pu_conf;
class inst_tcu_pu_conf_act;
class inst_tcu_pu_compute;
class inst_tcu_dot_dm_if_conf;
class inst_tcu_dot_dm_of_conf;
class inst_tcu_dot_dm_fetch_src1;
class inst_tcu_dot_dm_fetch_src2;
class inst_tcu_pu_compute_dummy;
class inst_mfu_mn_map_compute;
class inst_mfu_mn_vmap_compute;
class inst_mfu_reduce;
class inst_mfu_vreduce;
class inst_mfu_mn_broadcast_compute;
class inst_mfu_mn_reduce;
class inst_mfu_mn_conf;
class inst_mfu_mnop_conf;
class inst_mfu_pdp_conf;
class inst_mfu_pdp_src_conf;
class inst_mfu_pdp_reduce;
class inst_mfu_mn_broadcast_conf;
class inst_mfu_crop;
class inst_mfu_memset;
class inst_mfu_memcpy;
class inst_mfu_trans;
class inst_mfu_mn_conf2;
class gnne_instruction
{
public:
    virtual ~gnne_instruction() = default;
    virtual void serialize(binary_writer &writer) const = 0;
    virtual void to_string(std::ostream &out) const = 0;
    [[nodiscard]] virtual OPCODE opcode() const = 0;
    [[nodiscard]] std::string to_string() const;
};

std::ostream &operator<<(std::ostream &out, const gnne_instruction &c);

//////////////////////////////////
// auto generated instructions

/**
    NOP
    @brief 无操作
    - OPCODE opcode 无操作
**/
class inst_nop : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 无操作 **/
public:
    inst_nop();

    explicit inst_nop(const INST_NOP &ref);

    [[nodiscard]] struct INST_NOP to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 无操作
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 无操作
  */
    void opcode(const OPCODE &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_nop &c);

/**
    LI
    @brief Load 一个 64bit 的立即数到指定的寄存器
    - OPCODE opcode Load 一个 64bit 的立即数到指定的寄存器
    - REG regid 寄存器编号。
这个寄存器的编号只能编码32个寄存器区域。
{== TODO：是否要编码其他内容 ==}
    - uint64_t imm 立即数
**/
class inst_li : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** Load 一个 64bit 的立即数到指定的寄存器 **/
    REG regid_ = REG(0); /** 寄存器编号。
这个寄存器的编号只能编码32个寄存器区域。
{== TODO：是否要编码其他内容 ==} **/
    uint64_t imm_ = uint64_t(0); /** 立即数 **/
public:
    inst_li(REG regid = REG(0), uint64_t imm = uint64_t(0));

    explicit inst_li(const INST_LI &ref);

    [[nodiscard]] struct INST_LI to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode Load 一个 64bit 的立即数到指定的寄存器
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode Load 一个 64bit 的立即数到指定的寄存器
  */
    void opcode(const OPCODE &value);
    /**
      REG regid 寄存器编号。
这个寄存器的编号只能编码32个寄存器区域。
{== TODO：是否要编码其他内容 ==}
  */
    [[nodiscard]] REG regid() const;
    /**
      REG regid 寄存器编号。
这个寄存器的编号只能编码32个寄存器区域。
{== TODO：是否要编码其他内容 ==}
  */
    void regid(const REG &value);
    /**
      uint64_t imm 立即数
  */
    [[nodiscard]] uint64_t imm() const;
    /**
      uint64_t imm 立即数
  */
    void imm(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_li &c);

/**
    INTR
    @brief 触发向DSP的软中断
    - OPCODE opcode 触发向DSP的软中断
    - uint64_t intr_number 中断号，用于设置inptr寄存器
    - uint64_t regmask 标记哪些寄存器被用于传参，用于设置regmask寄存器
    - uint64_t ptrmask
标记寄存器里的内容是指针类型还是数据，用于设置ptrmask寄存器
**/
class inst_intr : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 触发向DSP的软中断 **/
    uint64_t intr_number_ = uint64_t(0); /** 中断号，用于设置inptr寄存器 **/
    uint64_t regmask_ = uint64_t(0); /** 标记哪些寄存器被用于传参，用于设置regmask寄存器 **/
    uint64_t ptrmask_ = uint64_t(
        0); /** 标记寄存器里的内容是指针类型还是数据，用于设置ptrmask寄存器 **/
public:
    inst_intr(uint64_t intr_number = uint64_t(0), uint64_t regmask = uint64_t(0),
        uint64_t ptrmask = uint64_t(0));

    explicit inst_intr(const INST_INTR &ref);

    [[nodiscard]] struct INST_INTR to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 触发向DSP的软中断
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 触发向DSP的软中断
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t intr_number 中断号，用于设置inptr寄存器
  */
    [[nodiscard]] uint64_t intr_number() const;
    /**
      uint64_t intr_number 中断号，用于设置inptr寄存器
  */
    void intr_number(const uint64_t &value);
    /**
      uint64_t regmask 标记哪些寄存器被用于传参，用于设置regmask寄存器
  */
    [[nodiscard]] uint64_t regmask() const;
    /**
      uint64_t regmask 标记哪些寄存器被用于传参，用于设置regmask寄存器
  */
    void regmask(const uint64_t &value);
    /**
      uint64_t ptrmask
     标记寄存器里的内容是指针类型还是数据，用于设置ptrmask寄存器
  */
    [[nodiscard]] uint64_t ptrmask() const;
    /**
      uint64_t ptrmask
     标记寄存器里的内容是指针类型还是数据，用于设置ptrmask寄存器
  */
    void ptrmask(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_intr &c);

/**
    END
    @brief 表示GNNE程序执行结束，GNNE向总中断控制器发起中断
    - OPCODE opcode 表示GNNE程序执行结束，GNNE向总中断控制器发起中断
    - uint64_t intr_number 中断号，用于设置inptr寄存器
**/
class inst_end : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 表示GNNE程序执行结束，GNNE向总中断控制器发起中断 **/
    uint64_t intr_number_ = uint64_t(0); /** 中断号，用于设置inptr寄存器 **/
public:
    inst_end(uint64_t intr_number = uint64_t(0));

    explicit inst_end(const INST_END &ref);

    [[nodiscard]] struct INST_END to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 表示GNNE程序执行结束，GNNE向总中断控制器发起中断
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 表示GNNE程序执行结束，GNNE向总中断控制器发起中断
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t intr_number 中断号，用于设置inptr寄存器
  */
    [[nodiscard]] uint64_t intr_number() const;
    /**
      uint64_t intr_number 中断号，用于设置inptr寄存器
  */
    void intr_number(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_end &c);

/**
    FENCE
    @brief 设置内存屏障。在屏障前面的指令一定都执行完成，后面的指令才会被执行
    - OPCODE opcode
设置内存屏障。在屏障前面的指令一定都执行完成，后面的指令才会被执行
**/
class inst_fence : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(
        0); /** 设置内存屏障。在屏障前面的指令一定都执行完成，后面的指令才会被执行
             **/
public:
    inst_fence();

    explicit inst_fence(const INST_FENCE &ref);

    [[nodiscard]] struct INST_FENCE to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode
     设置内存屏障。在屏障前面的指令一定都执行完成，后面的指令才会被执行
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode
     设置内存屏障。在屏障前面的指令一定都执行完成，后面的指令才会被执行
  */
    void opcode(const OPCODE &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_fence &c);

/**
    MMU.CONF
    @brief 配置某条MMU规则
    - OPCODE opcode 配置某条MMU规则
    - uint64_t mmu_item 当前配置的MMU规则项编号，0-15
    - uint64_t start_bank 当前规则开始的bank
    - MMU_CONF_WIDTH width 当前配置宽度信息
    - uint64_t start_depth 当前规则开始的纵坐标
    - uint64_t depth
当前规则深度。这个值是相对与开始深度的差值，注意是深度而不是结束深度。
**/
class inst_mmu_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置某条MMU规则 **/
    uint64_t mmu_item_ = uint64_t(0); /** 当前配置的MMU规则项编号，0-15 **/
    uint64_t start_bank_ = uint64_t(0); /** 当前规则开始的bank **/
    MMU_CONF_WIDTH width_ = MMU_CONF_WIDTH(0); /** 当前配置宽度信息 **/
    uint64_t start_depth_ = uint64_t(0); /** 当前规则开始的纵坐标 **/
    uint64_t depth_ = uint64_t(0); /**
                                          当前规则深度。这个值是相对与开始深度的差值，注意是深度而不是结束深度。
                                          **/
public:
    inst_mmu_conf(uint64_t mmu_item = uint64_t(0),
        uint64_t start_bank = uint64_t(0),
        MMU_CONF_WIDTH width = MMU_CONF_WIDTH(0),
        uint64_t start_depth = uint64_t(0),
        uint64_t depth = uint64_t(0));

    explicit inst_mmu_conf(const INST_MMU_CONF &ref);

    [[nodiscard]] struct INST_MMU_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置某条MMU规则
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置某条MMU规则
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t mmu_item 当前配置的MMU规则项编号，0-15
  */
    [[nodiscard]] uint64_t mmu_item() const;
    /**
      uint64_t mmu_item 当前配置的MMU规则项编号，0-15
  */
    void mmu_item(const uint64_t &value);
    /**
      uint64_t start_bank 当前规则开始的bank
  */
    [[nodiscard]] uint64_t start_bank() const;
    /**
      uint64_t start_bank 当前规则开始的bank
  */
    void start_bank(const uint64_t &value);
    /**
      MMU_CONF_WIDTH width 当前配置宽度信息
  */
    [[nodiscard]] MMU_CONF_WIDTH width() const;
    /**
      MMU_CONF_WIDTH width 当前配置宽度信息
  */
    void width(const MMU_CONF_WIDTH &value);
    /**
      uint64_t start_depth 当前规则开始的纵坐标
  */
    [[nodiscard]] uint64_t start_depth() const;
    /**
      uint64_t start_depth 当前规则开始的纵坐标
  */
    void start_depth(const uint64_t &value);
    /**
      uint64_t depth
     当前规则深度。这个值是相对与开始深度的差值，注意是深度而不是结束深度。
  */
    [[nodiscard]] uint64_t depth() const;
    /**
      uint64_t depth
     当前规则深度。这个值是相对与开始深度的差值，注意是深度而不是结束深度。
  */
    void depth(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mmu_conf &c);

/**
    FENCE.CCR
    @brief
CCR屏障，用来阻挡使用到本CCR的指令的分发，可以用来实现给CCR重新分配空间等作用
    - OPCODE opcode
CCR屏障，用来阻挡使用到本CCR的指令的分发，可以用来实现给CCR重新分配空间等作用
    - uint64_t ccr CCR寄存器id
    - uint64_t mode 0: wait ccr is 0
1: wait ccr is not 0 and clear ccr
**/
class inst_fence_ccr : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /**
                                   CCR屏障，用来阻挡使用到本CCR的指令的分发，可以用来实现给CCR重新分配空间等作用
                                   **/
    uint64_t ccr_ = uint64_t(0); /** CCR寄存器id **/
    uint64_t mode_ = uint64_t(0); /** 0: wait ccr is 0
1: wait ccr is not 0 and clear ccr **/
public:
    inst_fence_ccr(uint64_t ccr = uint64_t(0), uint64_t mode = uint64_t(0));

    explicit inst_fence_ccr(const INST_FENCE_CCR &ref);

    [[nodiscard]] struct INST_FENCE_CCR to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode
     CCR屏障，用来阻挡使用到本CCR的指令的分发，可以用来实现给CCR重新分配空间等作用
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode
     CCR屏障，用来阻挡使用到本CCR的指令的分发，可以用来实现给CCR重新分配空间等作用
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t ccr CCR寄存器id
  */
    [[nodiscard]] uint64_t ccr() const;
    /**
      uint64_t ccr CCR寄存器id
  */
    void ccr(const uint64_t &value);
    /**
      uint64_t mode 0: wait ccr is 0
1: wait ccr is not 0 and clear ccr
  */
    [[nodiscard]] uint64_t mode() const;
    /**
      uint64_t mode 0: wait ccr is 0
1: wait ccr is not 0 and clear ccr
  */
    void mode(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_fence_ccr &c);

/**
    LOADIF.CONFIG
    @brief 用于配置从 DDR 中加载 input featuremap 的一些基础参数
    - OPCODE opcode 用于配置从 DDR 中加载 input featuremap 的一些基础参数
    - uint64_t layout_ddr_n 输入 Tensor 的尺寸信息，用于计算输入地址跳转
    - uint64_t layout_ddr_c 输入 Tensor 的尺寸信息，用于计算输入地址跳转
    - uint64_t layout_ddr_h 输入 Tensor 的尺寸信息，用于计算输入地址跳转
    - uint64_t layout_ddr_w 输入 Tensor 的尺寸信息，用于计算输入地址跳转
    - STRIDE_GLB stride_glb 输出 Tensor 的尺寸信息，用于计算输出地址跳转
    - uint64_t mmu_item 使用的内存映射方案
    - ADDR_GLB_8_WITH_BANK addr_qarg 量化参数地址
    - QUAN_SIGNED input_signed 量化数据是否是有符号数据
    - PRECISION precision_glb GLB中数据的类型
    - PRECISION_DDR precision_ddr DDR 中的数据类型
    - QUAN_TYPE quan_type 量化类型，bychennel or by batch
**/
class inst_loadif_config : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 用于配置从 DDR 中加载 input featuremap 的一些基础参数 **/
    uint64_t layout_ddr_n_ = uint64_t(0); /** 输入 Tensor 的尺寸信息，用于计算输入地址跳转 **/
    uint64_t layout_ddr_c_ = uint64_t(0); /** 输入 Tensor 的尺寸信息，用于计算输入地址跳转 **/
    uint64_t layout_ddr_h_ = uint64_t(0); /** 输入 Tensor 的尺寸信息，用于计算输入地址跳转 **/
    uint64_t layout_ddr_w_ = uint64_t(0); /** 输入 Tensor 的尺寸信息，用于计算输入地址跳转 **/
    STRIDE_GLB stride_glb_ = STRIDE_GLB(0); /** 输出 Tensor 的尺寸信息，用于计算输出地址跳转 **/
    uint64_t mmu_item_ = uint64_t(0); /** 使用的内存映射方案 **/
    ADDR_GLB_8_WITH_BANK addr_qarg_ = ADDR_GLB_8_WITH_BANK(0); /** 量化参数地址 **/
    QUAN_SIGNED input_signed_ = QUAN_SIGNED(0); /** 量化数据是否是有符号数据 **/
    PRECISION precision_glb_ = PRECISION(0); /** GLB中数据的类型  **/
    PRECISION_DDR precision_ddr_ = PRECISION_DDR(0); /** DDR 中的数据类型 **/
    QUAN_TYPE quan_type_ = QUAN_TYPE(0); /** 量化类型，bychennel or by batch **/
public:
    inst_loadif_config(uint64_t layout_ddr_n = uint64_t(0),
        uint64_t layout_ddr_c = uint64_t(0),
        uint64_t layout_ddr_h = uint64_t(0),
        uint64_t layout_ddr_w = uint64_t(0),
        STRIDE_GLB stride_glb = STRIDE_GLB(0),
        uint64_t mmu_item = uint64_t(0),
        ADDR_GLB_8_WITH_BANK addr_qarg = ADDR_GLB_8_WITH_BANK(0),
        QUAN_SIGNED input_signed = QUAN_SIGNED(0),
        PRECISION precision_glb = PRECISION(0),
        PRECISION_DDR precision_ddr = PRECISION_DDR(0),
        QUAN_TYPE quan_type = QUAN_TYPE(0));

    explicit inst_loadif_config(const INST_LOADIF_CONFIG &ref);

    [[nodiscard]] struct INST_LOADIF_CONFIG to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 用于配置从 DDR 中加载 input featuremap 的一些基础参数
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 用于配置从 DDR 中加载 input featuremap 的一些基础参数
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t layout_ddr_n 输入 Tensor 的尺寸信息，用于计算输入地址跳转
  */
    [[nodiscard]] uint64_t layout_ddr_n() const;
    /**
      uint64_t layout_ddr_n 输入 Tensor 的尺寸信息，用于计算输入地址跳转
  */
    void layout_ddr_n(const uint64_t &value);
    /**
      uint64_t layout_ddr_c 输入 Tensor 的尺寸信息，用于计算输入地址跳转
  */
    [[nodiscard]] uint64_t layout_ddr_c() const;
    /**
      uint64_t layout_ddr_c 输入 Tensor 的尺寸信息，用于计算输入地址跳转
  */
    void layout_ddr_c(const uint64_t &value);
    /**
      uint64_t layout_ddr_h 输入 Tensor 的尺寸信息，用于计算输入地址跳转
  */
    [[nodiscard]] uint64_t layout_ddr_h() const;
    /**
      uint64_t layout_ddr_h 输入 Tensor 的尺寸信息，用于计算输入地址跳转
  */
    void layout_ddr_h(const uint64_t &value);
    /**
      uint64_t layout_ddr_w 输入 Tensor 的尺寸信息，用于计算输入地址跳转
  */
    [[nodiscard]] uint64_t layout_ddr_w() const;
    /**
      uint64_t layout_ddr_w 输入 Tensor 的尺寸信息，用于计算输入地址跳转
  */
    void layout_ddr_w(const uint64_t &value);
    /**
      STRIDE_GLB stride_glb 输出 Tensor 的尺寸信息，用于计算输出地址跳转
  */
    [[nodiscard]] STRIDE_GLB stride_glb() const;
    /**
      STRIDE_GLB stride_glb 输出 Tensor 的尺寸信息，用于计算输出地址跳转
  */
    void stride_glb(const STRIDE_GLB &value);
    /**
      uint64_t mmu_item 使用的内存映射方案
  */
    [[nodiscard]] uint64_t mmu_item() const;
    /**
      uint64_t mmu_item 使用的内存映射方案
  */
    void mmu_item(const uint64_t &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_qarg 量化参数地址
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_qarg() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_qarg 量化参数地址
  */
    void addr_qarg(const ADDR_GLB_8_WITH_BANK &value);
    /**
      QUAN_SIGNED input_signed 量化数据是否是有符号数据
  */
    [[nodiscard]] QUAN_SIGNED input_signed() const;
    /**
      QUAN_SIGNED input_signed 量化数据是否是有符号数据
  */
    void input_signed(const QUAN_SIGNED &value);
    /**
      PRECISION precision_glb GLB中数据的类型
  */
    [[nodiscard]] PRECISION precision_glb() const;
    /**
      PRECISION precision_glb GLB中数据的类型
  */
    void precision_glb(const PRECISION &value);
    /**
      PRECISION_DDR precision_ddr DDR 中的数据类型
  */
    [[nodiscard]] PRECISION_DDR precision_ddr() const;
    /**
      PRECISION_DDR precision_ddr DDR 中的数据类型
  */
    void precision_ddr(const PRECISION_DDR &value);
    /**
      QUAN_TYPE quan_type 量化类型，bychennel or by batch
  */
    [[nodiscard]] QUAN_TYPE quan_type() const;
    /**
      QUAN_TYPE quan_type 量化类型，bychennel or by batch
  */
    void quan_type(const QUAN_TYPE &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_loadif_config &c);

/**
    LOADIF
    @brief 从 DDR 中加载张量到 GLB 中
    - OPCODE opcode 从 DDR 中加载张量到 GLB 中
    - CCRCLR ccrclr None
    - CCRCLR ccrclr_qarg None
    - CCRSET ccrset 数据依赖配置
    - ADDR addr_src DDR 中 张量当前数据 的起始地址。
    - ADDR_GLB_8_WITHOUT_BANK addr_dest 存储到 GLB 中的地址
    - uint64_t shape_n 取 Tensor 的形状的配置，用来决定实际搬运的数据
    - uint64_t shape_c 取 Tensor 的形状的配置，用来决定实际搬运的数据
    - uint64_t shape_h 取 Tensor 的形状的配置，用来决定实际搬运的数据
    - uint64_t shape_w 取 Tensor 的形状的配置，用来决定实际搬运的数据
    - uint64_t basement DDR寻址偏移寄存器编号。
DDR访问的地址需要加上这个寄存器指示的寄存器的值
**/
class inst_loadif : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 从 DDR 中加载张量到 GLB 中 **/
    CCRCLR ccrclr_ = CCRCLR(0); /** None **/
    CCRCLR ccrclr_qarg_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** 数据依赖配置 **/
    ADDR addr_src_ = ADDR(0); /** DDR 中 张量当前数据 的起始地址。 **/
    ADDR_GLB_8_WITHOUT_BANK addr_dest_ = ADDR_GLB_8_WITHOUT_BANK(0); /** 存储到 GLB 中的地址 **/
    uint64_t shape_n_ = uint64_t(0); /** 取 Tensor 的形状的配置，用来决定实际搬运的数据 **/
    uint64_t shape_c_ = uint64_t(0); /** 取 Tensor 的形状的配置，用来决定实际搬运的数据 **/
    uint64_t shape_h_ = uint64_t(0); /** 取 Tensor 的形状的配置，用来决定实际搬运的数据 **/
    uint64_t shape_w_ = uint64_t(0); /** 取 Tensor 的形状的配置，用来决定实际搬运的数据 **/
    uint64_t basement_ = uint64_t(0); /** DDR寻址偏移寄存器编号。
DDR访问的地址需要加上这个寄存器指示的寄存器的值 **/
public:
    inst_loadif(CCRCLR ccrclr = CCRCLR(0), CCRCLR ccrclr_qarg = CCRCLR(0),
        CCRSET ccrset = CCRSET(0), ADDR addr_src = ADDR(0),
        ADDR_GLB_8_WITHOUT_BANK addr_dest = ADDR_GLB_8_WITHOUT_BANK(0),
        uint64_t shape_n = uint64_t(0), uint64_t shape_c = uint64_t(0),
        uint64_t shape_h = uint64_t(0), uint64_t shape_w = uint64_t(0),
        uint64_t basement = uint64_t(0));

    explicit inst_loadif(const INST_LOADIF &ref);

    [[nodiscard]] struct INST_LOADIF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 从 DDR 中加载张量到 GLB 中
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 从 DDR 中加载张量到 GLB 中
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr None
  */
    [[nodiscard]] CCRCLR ccrclr() const;
    /**
      CCRCLR ccrclr None
  */
    void ccrclr(const CCRCLR &value);
    /**
      CCRCLR ccrclr_qarg None
  */
    [[nodiscard]] CCRCLR ccrclr_qarg() const;
    /**
      CCRCLR ccrclr_qarg None
  */
    void ccrclr_qarg(const CCRCLR &value);
    /**
      CCRSET ccrset 数据依赖配置
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset 数据依赖配置
  */
    void ccrset(const CCRSET &value);
    /**
      ADDR addr_src DDR 中 张量当前数据 的起始地址。
  */
    [[nodiscard]] ADDR addr_src() const;
    /**
      ADDR addr_src DDR 中 张量当前数据 的起始地址。
  */
    void addr_src(const ADDR &value);
    /**
      ADDR_GLB_8_WITHOUT_BANK addr_dest 存储到 GLB 中的地址
  */
    [[nodiscard]] ADDR_GLB_8_WITHOUT_BANK addr_dest() const;
    /**
      ADDR_GLB_8_WITHOUT_BANK addr_dest 存储到 GLB 中的地址
  */
    void addr_dest(const ADDR_GLB_8_WITHOUT_BANK &value);
    /**
      uint64_t shape_n 取 Tensor 的形状的配置，用来决定实际搬运的数据
  */
    [[nodiscard]] uint64_t shape_n() const;
    /**
      uint64_t shape_n 取 Tensor 的形状的配置，用来决定实际搬运的数据
  */
    void shape_n(const uint64_t &value);
    /**
      uint64_t shape_c 取 Tensor 的形状的配置，用来决定实际搬运的数据
  */
    [[nodiscard]] uint64_t shape_c() const;
    /**
      uint64_t shape_c 取 Tensor 的形状的配置，用来决定实际搬运的数据
  */
    void shape_c(const uint64_t &value);
    /**
      uint64_t shape_h 取 Tensor 的形状的配置，用来决定实际搬运的数据
  */
    [[nodiscard]] uint64_t shape_h() const;
    /**
      uint64_t shape_h 取 Tensor 的形状的配置，用来决定实际搬运的数据
  */
    void shape_h(const uint64_t &value);
    /**
      uint64_t shape_w 取 Tensor 的形状的配置，用来决定实际搬运的数据
  */
    [[nodiscard]] uint64_t shape_w() const;
    /**
      uint64_t shape_w 取 Tensor 的形状的配置，用来决定实际搬运的数据
  */
    void shape_w(const uint64_t &value);
    /**
      uint64_t basement DDR寻址偏移寄存器编号。
DDR访问的地址需要加上这个寄存器指示的寄存器的值
  */
    [[nodiscard]] uint64_t basement() const;
    /**
      uint64_t basement DDR寻址偏移寄存器编号。
DDR访问的地址需要加上这个寄存器指示的寄存器的值
  */
    void basement(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_loadif &c);

/**
    LOAD
    @brief 从 DDR 中加载一段给定长度的一维数据数据到 GLB
指定的位置上，可用来加载 weights 和其他数据
    - OPCODE opcode 从 DDR 中加载一段给定长度的一维数据数据到 GLB
指定的位置上，可用来加载 weights 和其他数据
    - CCRCLR ccrclr None
    - CCRCLR ccrclr_qarg None
    - CCRSET ccrset 数据依赖配置
    - ADDR addr_src DDR 中数据的地址
    - ADDR_GLB_8_WITH_BANK addr_dest GLB 中的地址
    - uint64_t length 数据的长度
    - ADDR_GLB_8_WITH_BANK addr_qarg 量化参数地
    - uint64_t chan_qarg 每个量化参数持续作用的个数
    - uint64_t shape_c
量化参数的个数，等于总长度除以量化参数持续个数，硬件需要的冗余信息
    - uint64_t basement DDR寻址偏移寄存器编号。
DDR访问的地址需要加上这个寄存器指示的寄存器的值
    - QUAN_SIGNED input_signed None
    - PRECISION precision_glb GLB中数据的类型
    - PRECISION_DDR precision_ddr DDR 中的数据类型
    - uint64_t stream 是否为流式输入
**/
class inst_load : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 从 DDR 中加载一段给定长度的一维数据数据到 GLB
                                 指定的位置上，可用来加载 weights 和其他数据 **/
    CCRCLR ccrclr_ = CCRCLR(0); /** None **/
    CCRCLR ccrclr_qarg_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** 数据依赖配置 **/
    ADDR addr_src_ = ADDR(0); /** DDR 中数据的地址 **/
    ADDR_GLB_8_WITH_BANK addr_dest_ = ADDR_GLB_8_WITH_BANK(0); /** GLB 中的地址 **/
    uint64_t length_ = uint64_t(0); /** 数据的长度 **/
    ADDR_GLB_8_WITH_BANK addr_qarg_ = ADDR_GLB_8_WITH_BANK(0); /** 量化参数地 **/
    uint64_t chan_qarg_ = uint64_t(0); /** 每个量化参数持续作用的个数 **/
    uint64_t shape_c_ = uint64_t(
        0); /** 量化参数的个数，等于总长度除以量化参数持续个数，硬件需要的冗余信息
             **/
    uint64_t basement_ = uint64_t(0); /** DDR寻址偏移寄存器编号。
DDR访问的地址需要加上这个寄存器指示的寄存器的值 **/
    QUAN_SIGNED input_signed_ = QUAN_SIGNED(0); /** None **/
    PRECISION precision_glb_ = PRECISION(0); /** GLB中数据的类型 **/
    PRECISION_DDR precision_ddr_ = PRECISION_DDR(0); /** DDR 中的数据类型 **/
    uint64_t stream_ = uint64_t(0); /** 是否为流式输入 **/
public:
    inst_load(CCRCLR ccrclr = CCRCLR(0), CCRCLR ccrclr_qarg = CCRCLR(0),
        CCRSET ccrset = CCRSET(0), ADDR addr_src = ADDR(0),
        ADDR_GLB_8_WITH_BANK addr_dest = ADDR_GLB_8_WITH_BANK(0),
        uint64_t length = uint64_t(0),
        ADDR_GLB_8_WITH_BANK addr_qarg = ADDR_GLB_8_WITH_BANK(0),
        uint64_t chan_qarg = uint64_t(0), uint64_t shape_c = uint64_t(0),
        uint64_t basement = uint64_t(0),
        QUAN_SIGNED input_signed = QUAN_SIGNED(0),
        PRECISION precision_glb = PRECISION(0),
        PRECISION_DDR precision_ddr = PRECISION_DDR(0),
        uint64_t stream = uint64_t(0));

    explicit inst_load(const INST_LOAD &ref);

    [[nodiscard]] struct INST_LOAD to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 从 DDR 中加载一段给定长度的一维数据数据到 GLB
     指定的位置上，可用来加载 weights 和其他数据
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 从 DDR 中加载一段给定长度的一维数据数据到 GLB
     指定的位置上，可用来加载 weights 和其他数据
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr None
  */
    [[nodiscard]] CCRCLR ccrclr() const;
    /**
      CCRCLR ccrclr None
  */
    void ccrclr(const CCRCLR &value);
    /**
      CCRCLR ccrclr_qarg None
  */
    [[nodiscard]] CCRCLR ccrclr_qarg() const;
    /**
      CCRCLR ccrclr_qarg None
  */
    void ccrclr_qarg(const CCRCLR &value);
    /**
      CCRSET ccrset 数据依赖配置
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset 数据依赖配置
  */
    void ccrset(const CCRSET &value);
    /**
      ADDR addr_src DDR 中数据的地址
  */
    [[nodiscard]] ADDR addr_src() const;
    /**
      ADDR addr_src DDR 中数据的地址
  */
    void addr_src(const ADDR &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_dest GLB 中的地址
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_dest() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_dest GLB 中的地址
  */
    void addr_dest(const ADDR_GLB_8_WITH_BANK &value);
    /**
      uint64_t length 数据的长度
  */
    [[nodiscard]] uint64_t length() const;
    /**
      uint64_t length 数据的长度
  */
    void length(const uint64_t &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_qarg 量化参数地
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_qarg() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_qarg 量化参数地
  */
    void addr_qarg(const ADDR_GLB_8_WITH_BANK &value);
    /**
      uint64_t chan_qarg 每个量化参数持续作用的个数
  */
    [[nodiscard]] uint64_t chan_qarg() const;
    /**
      uint64_t chan_qarg 每个量化参数持续作用的个数
  */
    void chan_qarg(const uint64_t &value);
    /**
      uint64_t shape_c
     量化参数的个数，等于总长度除以量化参数持续个数，硬件需要的冗余信息
  */
    [[nodiscard]] uint64_t shape_c() const;
    /**
      uint64_t shape_c
     量化参数的个数，等于总长度除以量化参数持续个数，硬件需要的冗余信息
  */
    void shape_c(const uint64_t &value);
    /**
      uint64_t basement DDR寻址偏移寄存器编号。
DDR访问的地址需要加上这个寄存器指示的寄存器的值
  */
    [[nodiscard]] uint64_t basement() const;
    /**
      uint64_t basement DDR寻址偏移寄存器编号。
DDR访问的地址需要加上这个寄存器指示的寄存器的值
  */
    void basement(const uint64_t &value);
    /**
      QUAN_SIGNED input_signed None
  */
    [[nodiscard]] QUAN_SIGNED input_signed() const;
    /**
      QUAN_SIGNED input_signed None
  */
    void input_signed(const QUAN_SIGNED &value);
    /**
      PRECISION precision_glb GLB中数据的类型
  */
    [[nodiscard]] PRECISION precision_glb() const;
    /**
      PRECISION precision_glb GLB中数据的类型
  */
    void precision_glb(const PRECISION &value);
    /**
      PRECISION_DDR precision_ddr DDR 中的数据类型
  */
    [[nodiscard]] PRECISION_DDR precision_ddr() const;
    /**
      PRECISION_DDR precision_ddr DDR 中的数据类型
  */
    void precision_ddr(const PRECISION_DDR &value);
    /**
      uint64_t stream 是否为流式输入
  */
    [[nodiscard]] uint64_t stream() const;
    /**
      uint64_t stream 是否为流式输入
  */
    void stream(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_load &c);

/**
    LOADIF.COMPRESS.CONF
    @brief 配置loadif的稀疏、压缩、参数
    - OPCODE opcode 配置loadif的稀疏、压缩、参数
    - ADDR addr_bmp bitmap的地址
    - ADDR addr_code_len 每一行编码之后的长度存放的地址
    - ADDR addr_block_len 码块编码之后长度的存放地址
    - uint64_t code_lines 每次编码的行数
    - SPARSIFIED sparsified_ddr 是否稀疏
    - COMPRESSED compress_ddr 是否压缩
**/
class inst_loadif_compress_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置loadif的稀疏、压缩、参数 **/
    ADDR addr_bmp_ = ADDR(0); /** bitmap的地址 **/
    ADDR addr_code_len_ = ADDR(0); /** 每一行编码之后的长度存放的地址 **/
    ADDR addr_block_len_ = ADDR(0); /** 码块编码之后长度的存放地址 **/
    uint64_t code_lines_ = uint64_t(0); /** 每次编码的行数 **/
    SPARSIFIED sparsified_ddr_ = SPARSIFIED(0); /** 是否稀疏 **/
    COMPRESSED compress_ddr_ = COMPRESSED(0); /** 是否压缩 **/
public:
    inst_loadif_compress_conf(ADDR addr_bmp = ADDR(0),
        ADDR addr_code_len = ADDR(0),
        ADDR addr_block_len = ADDR(0),
        uint64_t code_lines = uint64_t(0),
        SPARSIFIED sparsified_ddr = SPARSIFIED(0),
        COMPRESSED compress_ddr = COMPRESSED(0));

    explicit inst_loadif_compress_conf(const INST_LOADIF_COMPRESS_CONF &ref);

    [[nodiscard]] struct INST_LOADIF_COMPRESS_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置loadif的稀疏、压缩、参数
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置loadif的稀疏、压缩、参数
  */
    void opcode(const OPCODE &value);
    /**
      ADDR addr_bmp bitmap的地址
  */
    [[nodiscard]] ADDR addr_bmp() const;
    /**
      ADDR addr_bmp bitmap的地址
  */
    void addr_bmp(const ADDR &value);
    /**
      ADDR addr_code_len 每一行编码之后的长度存放的地址
  */
    [[nodiscard]] ADDR addr_code_len() const;
    /**
      ADDR addr_code_len 每一行编码之后的长度存放的地址
  */
    void addr_code_len(const ADDR &value);
    /**
      ADDR addr_block_len 码块编码之后长度的存放地址
  */
    [[nodiscard]] ADDR addr_block_len() const;
    /**
      ADDR addr_block_len 码块编码之后长度的存放地址
  */
    void addr_block_len(const ADDR &value);
    /**
      uint64_t code_lines 每次编码的行数
  */
    [[nodiscard]] uint64_t code_lines() const;
    /**
      uint64_t code_lines 每次编码的行数
  */
    void code_lines(const uint64_t &value);
    /**
      SPARSIFIED sparsified_ddr 是否稀疏
  */
    [[nodiscard]] SPARSIFIED sparsified_ddr() const;
    /**
      SPARSIFIED sparsified_ddr 是否稀疏
  */
    void sparsified_ddr(const SPARSIFIED &value);
    /**
      COMPRESSED compress_ddr 是否压缩
  */
    [[nodiscard]] COMPRESSED compress_ddr() const;
    /**
      COMPRESSED compress_ddr 是否压缩
  */
    void compress_ddr(const COMPRESSED &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_loadif_compress_conf &c);

/**
    LOAD.COMPRESS.CONF
    @brief 配置load的稀疏、压缩、参数
    - OPCODE opcode 配置load的稀疏、压缩、参数
    - ADDR addr_bmp bitmap的地址
    - ADDR addr_code_len 每一行编码之后的长度存放的地址
    - SPARSIFIED sparsified_ddr 是否稀疏
**/
class inst_load_compress_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置load的稀疏、压缩、参数 **/
    ADDR addr_bmp_ = ADDR(0); /** bitmap的地址 **/
    ADDR addr_code_len_ = ADDR(0); /** 每一行编码之后的长度存放的地址 **/
    SPARSIFIED sparsified_ddr_ = SPARSIFIED(0); /** 是否稀疏 **/
public:
    inst_load_compress_conf(ADDR addr_bmp = ADDR(0), ADDR addr_code_len = ADDR(0),
        SPARSIFIED sparsified_ddr = SPARSIFIED(0));

    explicit inst_load_compress_conf(const INST_LOAD_COMPRESS_CONF &ref);

    [[nodiscard]] struct INST_LOAD_COMPRESS_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置load的稀疏、压缩、参数
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置load的稀疏、压缩、参数
  */
    void opcode(const OPCODE &value);
    /**
      ADDR addr_bmp bitmap的地址
  */
    [[nodiscard]] ADDR addr_bmp() const;
    /**
      ADDR addr_bmp bitmap的地址
  */
    void addr_bmp(const ADDR &value);
    /**
      ADDR addr_code_len 每一行编码之后的长度存放的地址
  */
    [[nodiscard]] ADDR addr_code_len() const;
    /**
      ADDR addr_code_len 每一行编码之后的长度存放的地址
  */
    void addr_code_len(const ADDR &value);
    /**
      SPARSIFIED sparsified_ddr 是否稀疏
  */
    [[nodiscard]] SPARSIFIED sparsified_ddr() const;
    /**
      SPARSIFIED sparsified_ddr 是否稀疏
  */
    void sparsified_ddr(const SPARSIFIED &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_load_compress_conf &c);

/**
    STORE
    @brief 将 GLB 中的给定长度的一维数据写回 DDR
    - OPCODE opcode 将 GLB 中的给定长度的一维数据写回 DDR
    - CCRCLR ccrclr 数据依赖配置
    - CCRCLR ccrclr_qarg None
    - CCRSET ccrset None
    - ADDR_GLB_8_WITH_BANK addr_src 数据在 GLB 中的起始位置
    - ADDR addr_dest 输出到 DDR 的位置
    - PRECISION precision_glb GLB中数据的类型
    - PRECISION_DDR precision_ddr 输出数据精度
    - QUAN_SIGNED output_signed 量化数据是否是有符号数据
    - uint64_t length 数据长度
    - ADDR_GLB_8_WITH_BANK addr_qarg 量化参数地
    - uint64_t chan_qarg 每个量化参数持续作用的个数
    - uint64_t shape_c
量化参数的个数，等于总长度除以量化参数持续个数，硬件需要的冗余信息
    - uint64_t clamp_hi None
    - uint64_t clamp_lo None
    - uint64_t basement DDR寻址偏移寄存器编号
**/
class inst_store : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 将 GLB 中的给定长度的一维数据写回 DDR **/
    CCRCLR ccrclr_ = CCRCLR(0); /** 数据依赖配置 **/
    CCRCLR ccrclr_qarg_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_src_ = ADDR_GLB_8_WITH_BANK(0); /** 数据在 GLB 中的起始位置 **/
    ADDR addr_dest_ = ADDR(0); /** 输出到 DDR 的位置 **/
    PRECISION precision_glb_ = PRECISION(0); /** GLB中数据的类型 **/
    PRECISION_DDR precision_ddr_ = PRECISION_DDR(0); /** 输出数据精度 **/
    QUAN_SIGNED output_signed_ = QUAN_SIGNED(0); /** 量化数据是否是有符号数据 **/
    uint64_t length_ = uint64_t(0); /** 数据长度 **/
    ADDR_GLB_8_WITH_BANK addr_qarg_ = ADDR_GLB_8_WITH_BANK(0); /** 量化参数地 **/
    uint64_t chan_qarg_ = uint64_t(0); /** 每个量化参数持续作用的个数 **/
    uint64_t shape_c_ = uint64_t(
        0); /** 量化参数的个数，等于总长度除以量化参数持续个数，硬件需要的冗余信息
             **/
    uint64_t clamp_hi_ = uint64_t(0); /** None **/
    uint64_t clamp_lo_ = uint64_t(0); /** None **/
    uint64_t basement_ = uint64_t(0); /** DDR寻址偏移寄存器编号 **/
public:
    inst_store(CCRCLR ccrclr = CCRCLR(0), CCRCLR ccrclr_qarg = CCRCLR(0),
        CCRSET ccrset = CCRSET(0),
        ADDR_GLB_8_WITH_BANK addr_src = ADDR_GLB_8_WITH_BANK(0),
        ADDR addr_dest = ADDR(0), PRECISION precision_glb = PRECISION(0),
        PRECISION_DDR precision_ddr = PRECISION_DDR(0),
        QUAN_SIGNED output_signed = QUAN_SIGNED(0),
        uint64_t length = uint64_t(0),
        ADDR_GLB_8_WITH_BANK addr_qarg = ADDR_GLB_8_WITH_BANK(0),
        uint64_t chan_qarg = uint64_t(0), uint64_t shape_c = uint64_t(0),
        uint64_t clamp_hi = uint64_t(0), uint64_t clamp_lo = uint64_t(0),
        uint64_t basement = uint64_t(0));

    explicit inst_store(const INST_STORE &ref);

    [[nodiscard]] struct INST_STORE to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 将 GLB 中的给定长度的一维数据写回 DDR
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 将 GLB 中的给定长度的一维数据写回 DDR
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr 数据依赖配置
  */
    [[nodiscard]] CCRCLR ccrclr() const;
    /**
      CCRCLR ccrclr 数据依赖配置
  */
    void ccrclr(const CCRCLR &value);
    /**
      CCRCLR ccrclr_qarg None
  */
    [[nodiscard]] CCRCLR ccrclr_qarg() const;
    /**
      CCRCLR ccrclr_qarg None
  */
    void ccrclr_qarg(const CCRCLR &value);
    /**
      CCRSET ccrset None
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset None
  */
    void ccrset(const CCRSET &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_src 数据在 GLB 中的起始位置
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_src() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_src 数据在 GLB 中的起始位置
  */
    void addr_src(const ADDR_GLB_8_WITH_BANK &value);
    /**
      ADDR addr_dest 输出到 DDR 的位置
  */
    [[nodiscard]] ADDR addr_dest() const;
    /**
      ADDR addr_dest 输出到 DDR 的位置
  */
    void addr_dest(const ADDR &value);
    /**
      PRECISION precision_glb GLB中数据的类型
  */
    [[nodiscard]] PRECISION precision_glb() const;
    /**
      PRECISION precision_glb GLB中数据的类型
  */
    void precision_glb(const PRECISION &value);
    /**
      PRECISION_DDR precision_ddr 输出数据精度
  */
    [[nodiscard]] PRECISION_DDR precision_ddr() const;
    /**
      PRECISION_DDR precision_ddr 输出数据精度
  */
    void precision_ddr(const PRECISION_DDR &value);
    /**
      QUAN_SIGNED output_signed 量化数据是否是有符号数据
  */
    [[nodiscard]] QUAN_SIGNED output_signed() const;
    /**
      QUAN_SIGNED output_signed 量化数据是否是有符号数据
  */
    void output_signed(const QUAN_SIGNED &value);
    /**
      uint64_t length 数据长度
  */
    [[nodiscard]] uint64_t length() const;
    /**
      uint64_t length 数据长度
  */
    void length(const uint64_t &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_qarg 量化参数地
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_qarg() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_qarg 量化参数地
  */
    void addr_qarg(const ADDR_GLB_8_WITH_BANK &value);
    /**
      uint64_t chan_qarg 每个量化参数持续作用的个数
  */
    [[nodiscard]] uint64_t chan_qarg() const;
    /**
      uint64_t chan_qarg 每个量化参数持续作用的个数
  */
    void chan_qarg(const uint64_t &value);
    /**
      uint64_t shape_c
     量化参数的个数，等于总长度除以量化参数持续个数，硬件需要的冗余信息
  */
    [[nodiscard]] uint64_t shape_c() const;
    /**
      uint64_t shape_c
     量化参数的个数，等于总长度除以量化参数持续个数，硬件需要的冗余信息
  */
    void shape_c(const uint64_t &value);
    /**
      uint64_t clamp_hi None
  */
    [[nodiscard]] uint64_t clamp_hi() const;
    /**
      uint64_t clamp_hi None
  */
    void clamp_hi(const uint64_t &value);
    /**
      uint64_t clamp_lo None
  */
    [[nodiscard]] uint64_t clamp_lo() const;
    /**
      uint64_t clamp_lo None
  */
    void clamp_lo(const uint64_t &value);
    /**
      uint64_t basement DDR寻址偏移寄存器编号
  */
    [[nodiscard]] uint64_t basement() const;
    /**
      uint64_t basement DDR寻址偏移寄存器编号
  */
    void basement(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_store &c);

/**
    STORE.T.CONFIG
    @brief 将 GLB 中 Tensor 的数据写回 DDR 的配置
    - OPCODE opcode 将 GLB 中 Tensor 的数据写回 DDR 的配置
    - uint64_t layout_ddr_n DDR中的Tensor的尺寸信息，用用计算跳转
    - uint64_t layout_ddr_c DDR中的Tensor的尺寸信息，用用计算跳转
    - uint64_t layout_ddr_h DDR中的Tensor的尺寸信息，用用计算跳转
    - uint64_t layout_ddr_w DDR中的Tensor的尺寸信息，用用计算跳转
    - STRIDE_GLB stride_glb GLB中的Tensor的尺寸信息，用用计算跳转
    - uint64_t mmu_item 使用的内存映射方案
    - ADDR_GLB_8_WITH_BANK addr_qarg 量化参数地址
    - QUAN_SIGNED output_signed 量化数据是否是有符号数据
    - PRECISION precision_glb GLB中数据的类型
    - PRECISION_DDR precision_ddr 输出数据精度
    - QUAN_TYPE quan_type None
    - uint64_t clamp_hi None
    - uint64_t clamp_lo None
**/
class inst_store_t_config : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 将 GLB 中 Tensor 的数据写回 DDR 的配置 **/
    uint64_t layout_ddr_n_ = uint64_t(0); /** DDR中的Tensor的尺寸信息，用用计算跳转 **/
    uint64_t layout_ddr_c_ = uint64_t(0); /** DDR中的Tensor的尺寸信息，用用计算跳转 **/
    uint64_t layout_ddr_h_ = uint64_t(0); /** DDR中的Tensor的尺寸信息，用用计算跳转 **/
    uint64_t layout_ddr_w_ = uint64_t(0); /** DDR中的Tensor的尺寸信息，用用计算跳转 **/
    STRIDE_GLB stride_glb_ = STRIDE_GLB(0); /** GLB中的Tensor的尺寸信息，用用计算跳转 **/
    uint64_t mmu_item_ = uint64_t(0); /** 使用的内存映射方案 **/
    ADDR_GLB_8_WITH_BANK addr_qarg_ = ADDR_GLB_8_WITH_BANK(0); /** 量化参数地址 **/
    QUAN_SIGNED output_signed_ = QUAN_SIGNED(0); /** 量化数据是否是有符号数据 **/
    PRECISION precision_glb_ = PRECISION(0); /** GLB中数据的类型 **/
    PRECISION_DDR precision_ddr_ = PRECISION_DDR(0); /** 输出数据精度 **/
    QUAN_TYPE quan_type_ = QUAN_TYPE(0); /** None **/
    uint64_t clamp_hi_ = uint64_t(0); /** None **/
    uint64_t clamp_lo_ = uint64_t(0); /** None **/
public:
    inst_store_t_config(
        uint64_t layout_ddr_n = uint64_t(0), uint64_t layout_ddr_c = uint64_t(0),
        uint64_t layout_ddr_h = uint64_t(0), uint64_t layout_ddr_w = uint64_t(0),
        STRIDE_GLB stride_glb = STRIDE_GLB(0), uint64_t mmu_item = uint64_t(0),
        ADDR_GLB_8_WITH_BANK addr_qarg = ADDR_GLB_8_WITH_BANK(0),
        QUAN_SIGNED output_signed = QUAN_SIGNED(0),
        PRECISION precision_glb = PRECISION(0),
        PRECISION_DDR precision_ddr = PRECISION_DDR(0),
        QUAN_TYPE quan_type = QUAN_TYPE(0), uint64_t clamp_hi = uint64_t(0),
        uint64_t clamp_lo = uint64_t(0));

    explicit inst_store_t_config(const INST_STORE_T_CONFIG &ref);

    [[nodiscard]] struct INST_STORE_T_CONFIG to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 将 GLB 中 Tensor 的数据写回 DDR 的配置
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 将 GLB 中 Tensor 的数据写回 DDR 的配置
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t layout_ddr_n DDR中的Tensor的尺寸信息，用用计算跳转
  */
    [[nodiscard]] uint64_t layout_ddr_n() const;
    /**
      uint64_t layout_ddr_n DDR中的Tensor的尺寸信息，用用计算跳转
  */
    void layout_ddr_n(const uint64_t &value);
    /**
      uint64_t layout_ddr_c DDR中的Tensor的尺寸信息，用用计算跳转
  */
    [[nodiscard]] uint64_t layout_ddr_c() const;
    /**
      uint64_t layout_ddr_c DDR中的Tensor的尺寸信息，用用计算跳转
  */
    void layout_ddr_c(const uint64_t &value);
    /**
      uint64_t layout_ddr_h DDR中的Tensor的尺寸信息，用用计算跳转
  */
    [[nodiscard]] uint64_t layout_ddr_h() const;
    /**
      uint64_t layout_ddr_h DDR中的Tensor的尺寸信息，用用计算跳转
  */
    void layout_ddr_h(const uint64_t &value);
    /**
      uint64_t layout_ddr_w DDR中的Tensor的尺寸信息，用用计算跳转
  */
    [[nodiscard]] uint64_t layout_ddr_w() const;
    /**
      uint64_t layout_ddr_w DDR中的Tensor的尺寸信息，用用计算跳转
  */
    void layout_ddr_w(const uint64_t &value);
    /**
      STRIDE_GLB stride_glb GLB中的Tensor的尺寸信息，用用计算跳转
  */
    [[nodiscard]] STRIDE_GLB stride_glb() const;
    /**
      STRIDE_GLB stride_glb GLB中的Tensor的尺寸信息，用用计算跳转
  */
    void stride_glb(const STRIDE_GLB &value);
    /**
      uint64_t mmu_item 使用的内存映射方案
  */
    [[nodiscard]] uint64_t mmu_item() const;
    /**
      uint64_t mmu_item 使用的内存映射方案
  */
    void mmu_item(const uint64_t &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_qarg 量化参数地址
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_qarg() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_qarg 量化参数地址
  */
    void addr_qarg(const ADDR_GLB_8_WITH_BANK &value);
    /**
      QUAN_SIGNED output_signed 量化数据是否是有符号数据
  */
    [[nodiscard]] QUAN_SIGNED output_signed() const;
    /**
      QUAN_SIGNED output_signed 量化数据是否是有符号数据
  */
    void output_signed(const QUAN_SIGNED &value);
    /**
      PRECISION precision_glb GLB中数据的类型
  */
    [[nodiscard]] PRECISION precision_glb() const;
    /**
      PRECISION precision_glb GLB中数据的类型
  */
    void precision_glb(const PRECISION &value);
    /**
      PRECISION_DDR precision_ddr 输出数据精度
  */
    [[nodiscard]] PRECISION_DDR precision_ddr() const;
    /**
      PRECISION_DDR precision_ddr 输出数据精度
  */
    void precision_ddr(const PRECISION_DDR &value);
    /**
      QUAN_TYPE quan_type None
  */
    [[nodiscard]] QUAN_TYPE quan_type() const;
    /**
      QUAN_TYPE quan_type None
  */
    void quan_type(const QUAN_TYPE &value);
    /**
      uint64_t clamp_hi None
  */
    [[nodiscard]] uint64_t clamp_hi() const;
    /**
      uint64_t clamp_hi None
  */
    void clamp_hi(const uint64_t &value);
    /**
      uint64_t clamp_lo None
  */
    [[nodiscard]] uint64_t clamp_lo() const;
    /**
      uint64_t clamp_lo None
  */
    void clamp_lo(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_store_t_config &c);

/**
    STORE.T
    @brief 将 GLB 中 Tensor 的数据写回 DDR，并且可能发生转置或者数据类型转换
    - OPCODE opcode 将 GLB 中 Tensor 的数据写回
DDR，并且可能发生转置或者数据类型转换
    - CCRCLR ccrclr 数据依赖配置
    - CCRCLR ccrclr_qarg None
    - CCRSET ccrset None
    - ADDR_GLB_8_WITHOUT_BANK addr_src Tensor 在 GLB 中的起始位置
    - ADDR addr_dest 输出到 DDR 的位置
    - uint64_t shape_n Slice 长度信息
    - uint64_t shape_c Slice 长度信息
    - uint64_t shape_h Slice 长度信息
    - uint64_t shape_w Slice 长度信息
    - uint64_t basement DDR寻址偏移寄存器编号
**/
class inst_store_t : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 将 GLB 中 Tensor 的数据写回
                                      DDR，并且可能发生转置或者数据类型转换 **/
    CCRCLR ccrclr_ = CCRCLR(0); /** 数据依赖配置 **/
    CCRCLR ccrclr_qarg_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** None **/
    ADDR_GLB_8_WITHOUT_BANK addr_src_ = ADDR_GLB_8_WITHOUT_BANK(0); /** Tensor 在 GLB 中的起始位置 **/
    ADDR addr_dest_ = ADDR(0); /** 输出到 DDR 的位置 **/
    uint64_t shape_n_ = uint64_t(0); /** Slice 长度信息 **/
    uint64_t shape_c_ = uint64_t(0); /** Slice 长度信息 **/
    uint64_t shape_h_ = uint64_t(0); /** Slice 长度信息 **/
    uint64_t shape_w_ = uint64_t(0); /** Slice 长度信息 **/
    uint64_t basement_ = uint64_t(0); /** DDR寻址偏移寄存器编号 **/
public:
    inst_store_t(CCRCLR ccrclr = CCRCLR(0), CCRCLR ccrclr_qarg = CCRCLR(0),
        CCRSET ccrset = CCRSET(0),
        ADDR_GLB_8_WITHOUT_BANK addr_src = ADDR_GLB_8_WITHOUT_BANK(0),
        ADDR addr_dest = ADDR(0), uint64_t shape_n = uint64_t(0),
        uint64_t shape_c = uint64_t(0), uint64_t shape_h = uint64_t(0),
        uint64_t shape_w = uint64_t(0), uint64_t basement = uint64_t(0));

    explicit inst_store_t(const INST_STORE_T &ref);

    [[nodiscard]] struct INST_STORE_T to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 将 GLB 中 Tensor 的数据写回
     DDR，并且可能发生转置或者数据类型转换
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 将 GLB 中 Tensor 的数据写回
     DDR，并且可能发生转置或者数据类型转换
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr 数据依赖配置
  */
    [[nodiscard]] CCRCLR ccrclr() const;
    /**
      CCRCLR ccrclr 数据依赖配置
  */
    void ccrclr(const CCRCLR &value);
    /**
      CCRCLR ccrclr_qarg None
  */
    [[nodiscard]] CCRCLR ccrclr_qarg() const;
    /**
      CCRCLR ccrclr_qarg None
  */
    void ccrclr_qarg(const CCRCLR &value);
    /**
      CCRSET ccrset None
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset None
  */
    void ccrset(const CCRSET &value);
    /**
      ADDR_GLB_8_WITHOUT_BANK addr_src Tensor 在 GLB 中的起始位置
  */
    [[nodiscard]] ADDR_GLB_8_WITHOUT_BANK addr_src() const;
    /**
      ADDR_GLB_8_WITHOUT_BANK addr_src Tensor 在 GLB 中的起始位置
  */
    void addr_src(const ADDR_GLB_8_WITHOUT_BANK &value);
    /**
      ADDR addr_dest 输出到 DDR 的位置
  */
    [[nodiscard]] ADDR addr_dest() const;
    /**
      ADDR addr_dest 输出到 DDR 的位置
  */
    void addr_dest(const ADDR &value);
    /**
      uint64_t shape_n Slice 长度信息
  */
    [[nodiscard]] uint64_t shape_n() const;
    /**
      uint64_t shape_n Slice 长度信息
  */
    void shape_n(const uint64_t &value);
    /**
      uint64_t shape_c Slice 长度信息
  */
    [[nodiscard]] uint64_t shape_c() const;
    /**
      uint64_t shape_c Slice 长度信息
  */
    void shape_c(const uint64_t &value);
    /**
      uint64_t shape_h Slice 长度信息
  */
    [[nodiscard]] uint64_t shape_h() const;
    /**
      uint64_t shape_h Slice 长度信息
  */
    void shape_h(const uint64_t &value);
    /**
      uint64_t shape_w Slice 长度信息
  */
    [[nodiscard]] uint64_t shape_w() const;
    /**
      uint64_t shape_w Slice 长度信息
  */
    void shape_w(const uint64_t &value);
    /**
      uint64_t basement DDR寻址偏移寄存器编号
  */
    [[nodiscard]] uint64_t basement() const;
    /**
      uint64_t basement DDR寻址偏移寄存器编号
  */
    void basement(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_store_t &c);

/**
    STORE.T.COMPRESS.CONF
    @brief 配置store.t的稀疏、压缩、参数
    - OPCODE opcode 配置store.t的稀疏、压缩、参数
    - ADDR addr_bmp bitmap的地址
    - ADDR addr_code_len 每一行编码之后的长度存放的地址
    - ADDR addr_block_len 码块编码之后长度的存放地址
    - uint64_t code_lines 每次编码的行数
    - SPARSIFIED sparsified_ddr 是否稀疏
    - COMPRESSED compress_ddr 是否压缩
**/
class inst_store_t_compress_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置store.t的稀疏、压缩、参数 **/
    ADDR addr_bmp_ = ADDR(0); /** bitmap的地址 **/
    ADDR addr_code_len_ = ADDR(0); /** 每一行编码之后的长度存放的地址 **/
    ADDR addr_block_len_ = ADDR(0); /** 码块编码之后长度的存放地址 **/
    uint64_t code_lines_ = uint64_t(0); /** 每次编码的行数 **/
    SPARSIFIED sparsified_ddr_ = SPARSIFIED(0); /** 是否稀疏 **/
    COMPRESSED compress_ddr_ = COMPRESSED(0); /** 是否压缩 **/
public:
    inst_store_t_compress_conf(ADDR addr_bmp = ADDR(0),
        ADDR addr_code_len = ADDR(0),
        ADDR addr_block_len = ADDR(0),
        uint64_t code_lines = uint64_t(0),
        SPARSIFIED sparsified_ddr = SPARSIFIED(0),
        COMPRESSED compress_ddr = COMPRESSED(0));

    explicit inst_store_t_compress_conf(const INST_STORE_T_COMPRESS_CONF &ref);

    [[nodiscard]] struct INST_STORE_T_COMPRESS_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置store.t的稀疏、压缩、参数
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置store.t的稀疏、压缩、参数
  */
    void opcode(const OPCODE &value);
    /**
      ADDR addr_bmp bitmap的地址
  */
    [[nodiscard]] ADDR addr_bmp() const;
    /**
      ADDR addr_bmp bitmap的地址
  */
    void addr_bmp(const ADDR &value);
    /**
      ADDR addr_code_len 每一行编码之后的长度存放的地址
  */
    [[nodiscard]] ADDR addr_code_len() const;
    /**
      ADDR addr_code_len 每一行编码之后的长度存放的地址
  */
    void addr_code_len(const ADDR &value);
    /**
      ADDR addr_block_len 码块编码之后长度的存放地址
  */
    [[nodiscard]] ADDR addr_block_len() const;
    /**
      ADDR addr_block_len 码块编码之后长度的存放地址
  */
    void addr_block_len(const ADDR &value);
    /**
      uint64_t code_lines 每次编码的行数
  */
    [[nodiscard]] uint64_t code_lines() const;
    /**
      uint64_t code_lines 每次编码的行数
  */
    void code_lines(const uint64_t &value);
    /**
      SPARSIFIED sparsified_ddr 是否稀疏
  */
    [[nodiscard]] SPARSIFIED sparsified_ddr() const;
    /**
      SPARSIFIED sparsified_ddr 是否稀疏
  */
    void sparsified_ddr(const SPARSIFIED &value);
    /**
      COMPRESSED compress_ddr 是否压缩
  */
    [[nodiscard]] COMPRESSED compress_ddr() const;
    /**
      COMPRESSED compress_ddr 是否压缩
  */
    void compress_ddr(const COMPRESSED &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out,
    const inst_store_t_compress_conf &c);

/**
    STORE.COMPRESS.CONF
    @brief 配置store的稀疏、压缩、参数
    - OPCODE opcode 配置store的稀疏、压缩、参数
    - ADDR addr_bmp bitmap的地址
    - ADDR addr_code_len 每一行编码之后的长度存放的地址
    - SPARSIFIED sparsified_ddr 是否稀疏
**/
class inst_store_compress_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置store的稀疏、压缩、参数 **/
    ADDR addr_bmp_ = ADDR(0); /** bitmap的地址 **/
    ADDR addr_code_len_ = ADDR(0); /** 每一行编码之后的长度存放的地址 **/
    SPARSIFIED sparsified_ddr_ = SPARSIFIED(0); /** 是否稀疏 **/
public:
    inst_store_compress_conf(ADDR addr_bmp = ADDR(0),
        ADDR addr_code_len = ADDR(0),
        SPARSIFIED sparsified_ddr = SPARSIFIED(0));

    explicit inst_store_compress_conf(const INST_STORE_COMPRESS_CONF &ref);

    [[nodiscard]] struct INST_STORE_COMPRESS_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置store的稀疏、压缩、参数
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置store的稀疏、压缩、参数
  */
    void opcode(const OPCODE &value);
    /**
      ADDR addr_bmp bitmap的地址
  */
    [[nodiscard]] ADDR addr_bmp() const;
    /**
      ADDR addr_bmp bitmap的地址
  */
    void addr_bmp(const ADDR &value);
    /**
      ADDR addr_code_len 每一行编码之后的长度存放的地址
  */
    [[nodiscard]] ADDR addr_code_len() const;
    /**
      ADDR addr_code_len 每一行编码之后的长度存放的地址
  */
    void addr_code_len(const ADDR &value);
    /**
      SPARSIFIED sparsified_ddr 是否稀疏
  */
    [[nodiscard]] SPARSIFIED sparsified_ddr() const;
    /**
      SPARSIFIED sparsified_ddr 是否稀疏
  */
    void sparsified_ddr(const SPARSIFIED &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_store_compress_conf &c);

/**
    TCU.DM.BROADCAST
    @brief 配置DM是否在广播模式
    - OPCODE opcode 配置DM是否在广播模式
    - BROADCAST broadcast_if 配置input featuremap是否在广播模式
    - BROADCAST broadcast_weight 配置weight是否在广播模式
    - uint64_t psum_cascade 是否为psum级联模式（用来加速1*1卷积计算） 0：否
4个TCU 1：是
**/
class inst_tcu_dm_broadcast : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置DM是否在广播模式 **/
    BROADCAST broadcast_if_ = BROADCAST(0); /** 配置input featuremap是否在广播模式 **/
    BROADCAST broadcast_weight_ = BROADCAST(0); /** 配置weight是否在广播模式 **/
    uint64_t psum_cascade_ = uint64_t(
        0); /** 是否为psum级联模式（用来加速1*1卷积计算） 0：否 4个TCU 1：是 **/
public:
    inst_tcu_dm_broadcast(BROADCAST broadcast_if = BROADCAST(0),
        BROADCAST broadcast_weight = BROADCAST(0),
        uint64_t psum_cascade = uint64_t(0));

    explicit inst_tcu_dm_broadcast(const INST_TCU_DM_BROADCAST &ref);

    [[nodiscard]] struct INST_TCU_DM_BROADCAST to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置DM是否在广播模式
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置DM是否在广播模式
  */
    void opcode(const OPCODE &value);
    /**
      BROADCAST broadcast_if 配置input featuremap是否在广播模式
  */
    [[nodiscard]] BROADCAST broadcast_if() const;
    /**
      BROADCAST broadcast_if 配置input featuremap是否在广播模式
  */
    void broadcast_if(const BROADCAST &value);
    /**
      BROADCAST broadcast_weight 配置weight是否在广播模式
  */
    [[nodiscard]] BROADCAST broadcast_weight() const;
    /**
      BROADCAST broadcast_weight 配置weight是否在广播模式
  */
    void broadcast_weight(const BROADCAST &value);
    /**
      uint64_t psum_cascade 是否为psum级联模式（用来加速1*1卷积计算） 0：否
     4个TCU 1：是
  */
    [[nodiscard]] uint64_t psum_cascade() const;
    /**
      uint64_t psum_cascade 是否为psum级联模式（用来加速1*1卷积计算） 0：否
     4个TCU 1：是
  */
    void psum_cascade(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_tcu_dm_broadcast &c);

/**
    TCU.DM.CONF.IF
    @brief 配置 FETCHIF 的一些基础信息
    - OPCODE opcode 配置 FETCHIF 的一些基础信息
    - uint64_t tcu_id 选择配置第几个TCU的DM
    - STRIDE_GLB stride_input_glb GLB中大tensor的排布的信息，用于计算跳转
    - uint64_t stride_w input featuremap 在 W 方向上的 stride
    - uint64_t stride_h input featuremap 在 H 方向上的 stride
    - uint64_t padding_top input featuremap 在 HxW 平面上顶部的 zero padding
行数
    - uint64_t padding_bottom input featuremap 在 HxW 平面下底部的 zero padding
行数
    - uint64_t padding_left input featuremap 在 HxW 平面左边的 zero padding 列数
    - uint64_t padding_right input featuremap 在 HxW 平面右边的 zero padding
列数
    - uint64_t input_c_per_pu 每个PU计算的输入通道数，根据高度映射除出来的
    - uint64_t dilation_h input featuremap 的在 H 方向上的膨胀率
    - uint64_t transpose_if 是否对输入的 input featuremap 进行转置，仅在
通道上有效
    - uint64_t mmu_item 使用的内存映射方案
**/
class inst_tcu_dm_conf_if : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置 FETCHIF 的一些基础信息 **/
    uint64_t tcu_id_ = uint64_t(0); /** 选择配置第几个TCU的DM **/
    STRIDE_GLB stride_input_glb_ = STRIDE_GLB(0); /** GLB中大tensor的排布的信息，用于计算跳转 **/
    uint64_t stride_w_ = uint64_t(0); /** input featuremap 在 W 方向上的 stride **/
    uint64_t stride_h_ = uint64_t(0); /** input featuremap 在 H 方向上的 stride **/
    uint64_t padding_top_ = uint64_t(
        0); /** input featuremap 在 HxW 平面上顶部的 zero padding 行数 **/
    uint64_t padding_bottom_ = uint64_t(
        0); /** input featuremap 在 HxW 平面下底部的 zero padding 行数 **/
    uint64_t padding_left_ = uint64_t(0); /** input featuremap 在 HxW 平面左边的 zero padding 列数 **/
    uint64_t padding_right_ = uint64_t(0); /** input featuremap 在 HxW 平面右边的 zero padding 列数 **/
    uint64_t input_c_per_pu_ = uint64_t(0); /** 每个PU计算的输入通道数，根据高度映射除出来的 **/
    uint64_t dilation_h_ = uint64_t(0); /** input featuremap 的在 H 方向上的膨胀率 **/
    uint64_t transpose_if_ = uint64_t(
        0); /** 是否对输入的 input featuremap 进行转置，仅在 通道上有效 **/
    uint64_t mmu_item_ = uint64_t(0); /** 使用的内存映射方案 **/
public:
    inst_tcu_dm_conf_if(
        uint64_t tcu_id = uint64_t(0),
        STRIDE_GLB stride_input_glb = STRIDE_GLB(0),
        uint64_t stride_w = uint64_t(0), uint64_t stride_h = uint64_t(0),
        uint64_t padding_top = uint64_t(0), uint64_t padding_bottom = uint64_t(0),
        uint64_t padding_left = uint64_t(0), uint64_t padding_right = uint64_t(0),
        uint64_t input_c_per_pu = uint64_t(0), uint64_t dilation_h = uint64_t(0),
        uint64_t transpose_if = uint64_t(0), uint64_t mmu_item = uint64_t(0));

    explicit inst_tcu_dm_conf_if(const INST_TCU_DM_CONF_IF &ref);

    [[nodiscard]] struct INST_TCU_DM_CONF_IF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置 FETCHIF 的一些基础信息
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置 FETCHIF 的一些基础信息
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t tcu_id 选择配置第几个TCU的DM
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id 选择配置第几个TCU的DM
  */
    void tcu_id(const uint64_t &value);
    /**
      STRIDE_GLB stride_input_glb GLB中大tensor的排布的信息，用于计算跳转
  */
    [[nodiscard]] STRIDE_GLB stride_input_glb() const;
    /**
      STRIDE_GLB stride_input_glb GLB中大tensor的排布的信息，用于计算跳转
  */
    void stride_input_glb(const STRIDE_GLB &value);
    /**
      uint64_t stride_w input featuremap 在 W 方向上的 stride
  */
    [[nodiscard]] uint64_t stride_w() const;
    /**
      uint64_t stride_w input featuremap 在 W 方向上的 stride
  */
    void stride_w(const uint64_t &value);
    /**
      uint64_t stride_h input featuremap 在 H 方向上的 stride
  */
    [[nodiscard]] uint64_t stride_h() const;
    /**
      uint64_t stride_h input featuremap 在 H 方向上的 stride
  */
    void stride_h(const uint64_t &value);
    /**
      uint64_t padding_top input featuremap 在 HxW 平面上顶部的 zero padding
     行数
  */
    [[nodiscard]] uint64_t padding_top() const;
    /**
      uint64_t padding_top input featuremap 在 HxW 平面上顶部的 zero padding
     行数
  */
    void padding_top(const uint64_t &value);
    /**
      uint64_t padding_bottom input featuremap 在 HxW 平面下底部的 zero padding
     行数
  */
    [[nodiscard]] uint64_t padding_bottom() const;
    /**
      uint64_t padding_bottom input featuremap 在 HxW 平面下底部的 zero padding
     行数
  */
    void padding_bottom(const uint64_t &value);
    /**
      uint64_t padding_left input featuremap 在 HxW 平面左边的 zero padding 列数
  */
    [[nodiscard]] uint64_t padding_left() const;
    /**
      uint64_t padding_left input featuremap 在 HxW 平面左边的 zero padding 列数
  */
    void padding_left(const uint64_t &value);
    /**
      uint64_t padding_right input featuremap 在 HxW 平面右边的 zero padding
     列数
  */
    [[nodiscard]] uint64_t padding_right() const;
    /**
      uint64_t padding_right input featuremap 在 HxW 平面右边的 zero padding
     列数
  */
    void padding_right(const uint64_t &value);
    /**
      uint64_t input_c_per_pu 每个PU计算的输入通道数，根据高度映射除出来的
  */
    [[nodiscard]] uint64_t input_c_per_pu() const;
    /**
      uint64_t input_c_per_pu 每个PU计算的输入通道数，根据高度映射除出来的
  */
    void input_c_per_pu(const uint64_t &value);
    /**
      uint64_t dilation_h input featuremap 的在 H 方向上的膨胀率
  */
    [[nodiscard]] uint64_t dilation_h() const;
    /**
      uint64_t dilation_h input featuremap 的在 H 方向上的膨胀率
  */
    void dilation_h(const uint64_t &value);
    /**
      uint64_t transpose_if 是否对输入的 input featuremap 进行转置，仅在
     通道上有效
  */
    [[nodiscard]] uint64_t transpose_if() const;
    /**
      uint64_t transpose_if 是否对输入的 input featuremap 进行转置，仅在
     通道上有效
  */
    void transpose_if(const uint64_t &value);
    /**
      uint64_t mmu_item 使用的内存映射方案
  */
    [[nodiscard]] uint64_t mmu_item() const;
    /**
      uint64_t mmu_item 使用的内存映射方案
  */
    void mmu_item(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_tcu_dm_conf_if &c);

/**
    TCU.DM.FETCHIF
    @brief 从 GLB 中加载 input featuremap 到 PU 或者广播到多个PU中
    - OPCODE opcode 从 GLB 中加载 input featuremap 到 PU 或者广播到多个PU中
    - CCRCLR ccrclr_if None
    - uint64_t tcu_id TCU ID
    - ADDR_GLB_8_WITHOUT_BANK addr_if 输入slice的起始地址
    - uint64_t shape_input_n 当前计算输入Slice的形状信息
    - uint64_t shape_input_c 当前计算输入Slice的形状信息
    - uint64_t shape_input_h 当前计算输入Slice的形状信息
    - uint64_t shape_input_w 当前计算输入Slice的形状信息
**/
class inst_tcu_dm_fetchif : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 从 GLB 中加载 input featuremap 到 PU 或者广播到多个PU中 **/
    CCRCLR ccrclr_if_ = CCRCLR(0); /** None **/
    uint64_t tcu_id_ = uint64_t(0); /** TCU ID **/
    ADDR_GLB_8_WITHOUT_BANK addr_if_ = ADDR_GLB_8_WITHOUT_BANK(0); /** 输入slice的起始地址 **/
    uint64_t shape_input_n_ = uint64_t(0); /** 当前计算输入Slice的形状信息 **/
    uint64_t shape_input_c_ = uint64_t(0); /** 当前计算输入Slice的形状信息 **/
    uint64_t shape_input_h_ = uint64_t(0); /** 当前计算输入Slice的形状信息 **/
    uint64_t shape_input_w_ = uint64_t(0); /** 当前计算输入Slice的形状信息 **/
public:
    inst_tcu_dm_fetchif(
        CCRCLR ccrclr_if = CCRCLR(0), uint64_t tcu_id = uint64_t(0),
        ADDR_GLB_8_WITHOUT_BANK addr_if = ADDR_GLB_8_WITHOUT_BANK(0),
        uint64_t shape_input_n = uint64_t(0),
        uint64_t shape_input_c = uint64_t(0),
        uint64_t shape_input_h = uint64_t(0),
        uint64_t shape_input_w = uint64_t(0));

    explicit inst_tcu_dm_fetchif(const INST_TCU_DM_FETCHIF &ref);

    [[nodiscard]] struct INST_TCU_DM_FETCHIF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 从 GLB 中加载 input featuremap 到 PU 或者广播到多个PU中
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 从 GLB 中加载 input featuremap 到 PU 或者广播到多个PU中
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr_if None
  */
    [[nodiscard]] CCRCLR ccrclr_if() const;
    /**
      CCRCLR ccrclr_if None
  */
    void ccrclr_if(const CCRCLR &value);
    /**
      uint64_t tcu_id TCU ID
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id TCU ID
  */
    void tcu_id(const uint64_t &value);
    /**
      ADDR_GLB_8_WITHOUT_BANK addr_if 输入slice的起始地址
  */
    [[nodiscard]] ADDR_GLB_8_WITHOUT_BANK addr_if() const;
    /**
      ADDR_GLB_8_WITHOUT_BANK addr_if 输入slice的起始地址
  */
    void addr_if(const ADDR_GLB_8_WITHOUT_BANK &value);
    /**
      uint64_t shape_input_n 当前计算输入Slice的形状信息
  */
    [[nodiscard]] uint64_t shape_input_n() const;
    /**
      uint64_t shape_input_n 当前计算输入Slice的形状信息
  */
    void shape_input_n(const uint64_t &value);
    /**
      uint64_t shape_input_c 当前计算输入Slice的形状信息
  */
    [[nodiscard]] uint64_t shape_input_c() const;
    /**
      uint64_t shape_input_c 当前计算输入Slice的形状信息
  */
    void shape_input_c(const uint64_t &value);
    /**
      uint64_t shape_input_h 当前计算输入Slice的形状信息
  */
    [[nodiscard]] uint64_t shape_input_h() const;
    /**
      uint64_t shape_input_h 当前计算输入Slice的形状信息
  */
    void shape_input_h(const uint64_t &value);
    /**
      uint64_t shape_input_w 当前计算输入Slice的形状信息
  */
    [[nodiscard]] uint64_t shape_input_w() const;
    /**
      uint64_t shape_input_w 当前计算输入Slice的形状信息
  */
    void shape_input_w(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_tcu_dm_fetchif &c);

/**
    TCU.DM.CONF.W
    @brief 配置 TCU 读取 weights 时的行为
    - OPCODE opcode 配置 TCU 读取 weights 时的行为
    - uint64_t tcu_id TCU 的 ID
    - uint64_t mmu_item 使用的内存映射方案
    - uint64_t load_direction 是否对输入的 input featuremap
进行转置，仅在HxW通道上有效
**/
class inst_tcu_dm_conf_w : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置 TCU 读取 weights 时的行为 **/
    uint64_t tcu_id_ = uint64_t(0); /** TCU 的 ID **/
    uint64_t mmu_item_ = uint64_t(0); /** 使用的内存映射方案 **/
    uint64_t load_direction_ = uint64_t(
        0); /** 是否对输入的 input featuremap 进行转置，仅在HxW通道上有效 **/
public:
    inst_tcu_dm_conf_w(uint64_t tcu_id = uint64_t(0),
        uint64_t mmu_item = uint64_t(0),
        uint64_t load_direction = uint64_t(0));

    explicit inst_tcu_dm_conf_w(const INST_TCU_DM_CONF_W &ref);

    [[nodiscard]] struct INST_TCU_DM_CONF_W to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置 TCU 读取 weights 时的行为
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置 TCU 读取 weights 时的行为
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t tcu_id TCU 的 ID
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id TCU 的 ID
  */
    void tcu_id(const uint64_t &value);
    /**
      uint64_t mmu_item 使用的内存映射方案
  */
    [[nodiscard]] uint64_t mmu_item() const;
    /**
      uint64_t mmu_item 使用的内存映射方案
  */
    void mmu_item(const uint64_t &value);
    /**
      uint64_t load_direction 是否对输入的 input featuremap
     进行转置，仅在HxW通道上有效
  */
    [[nodiscard]] uint64_t load_direction() const;
    /**
      uint64_t load_direction 是否对输入的 input featuremap
     进行转置，仅在HxW通道上有效
  */
    void load_direction(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_tcu_dm_conf_w &c);

/**
    TCU.DM.FETCHW
    @brief 从 GLB 中加载 weights 到 PU 或者广播到多个PU中
    - OPCODE opcode 从 GLB 中加载 weights 到 PU 或者广播到多个PU中
    - CCRCLR ccrclr_weight 数据依赖配置
    - uint64_t tcu_id TCU ID
    - ADDR_GLB_8_WITHOUT_BANK addr_src GLB 中 weights 的地址,byte的偏移地址
**/
class inst_tcu_dm_fetchw : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 从 GLB 中加载 weights 到 PU 或者广播到多个PU中 **/
    CCRCLR ccrclr_weight_ = CCRCLR(0); /** 数据依赖配置 **/
    uint64_t tcu_id_ = uint64_t(0); /** TCU ID **/
    ADDR_GLB_8_WITHOUT_BANK addr_src_ = ADDR_GLB_8_WITHOUT_BANK(0); /** GLB 中 weights 的地址,byte的偏移地址 **/
public:
    inst_tcu_dm_fetchw(
        CCRCLR ccrclr_weight = CCRCLR(0), uint64_t tcu_id = uint64_t(0),
        ADDR_GLB_8_WITHOUT_BANK addr_src = ADDR_GLB_8_WITHOUT_BANK(0));

    explicit inst_tcu_dm_fetchw(const INST_TCU_DM_FETCHW &ref);

    [[nodiscard]] struct INST_TCU_DM_FETCHW to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 从 GLB 中加载 weights 到 PU 或者广播到多个PU中
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 从 GLB 中加载 weights 到 PU 或者广播到多个PU中
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr_weight 数据依赖配置
  */
    [[nodiscard]] CCRCLR ccrclr_weight() const;
    /**
      CCRCLR ccrclr_weight 数据依赖配置
  */
    void ccrclr_weight(const CCRCLR &value);
    /**
      uint64_t tcu_id TCU ID
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id TCU ID
  */
    void tcu_id(const uint64_t &value);
    /**
      ADDR_GLB_8_WITHOUT_BANK addr_src GLB 中 weights 的地址,byte的偏移地址
  */
    [[nodiscard]] ADDR_GLB_8_WITHOUT_BANK addr_src() const;
    /**
      ADDR_GLB_8_WITHOUT_BANK addr_src GLB 中 weights 的地址,byte的偏移地址
  */
    void addr_src(const ADDR_GLB_8_WITHOUT_BANK &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_tcu_dm_fetchw &c);

/**
    TCU.DM.CONF.OF
    @brief 配置 TCU 输出信息
    - OPCODE opcode 配置 TCU 输出信息
    - uint64_t tcu_id TCU 的 ID
    - ADDR_GLB_8_WITH_BANK addr_psum 中间结果的 GLB 中的地址
    - ADDR_GLB_8_WITH_BANK addr_dest 输出结果的GLB中的地址
    - uint64_t shape_output_n 当前计算输出Slice的形状信息
    - uint64_t shape_output_c 当前计算输出Slice的形状信息
    - uint64_t shape_output_h 当前计算输出Slice的形状信息
    - uint64_t shape_output_w 当前计算输出Slice的形状信息
    - STRIDE_GLB stride_output_glb 输出featuremap在GLB的形状信息，决定寻址
    - STRIDE_GLB stride_psum_glb 输出PSUM在GLB的形状信息，决定寻址
    - uint64_t x_cut
水平方向一行第一次卷积的偏移量，硬件会切掉这些数，为了在反卷积模式下和tensorflow的实现保持一致
    - PRECISION output_precision None
    - QUAN_SIGNED output_signed None
**/
class inst_tcu_dm_conf_of : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置 TCU 输出信息 **/
    uint64_t tcu_id_ = uint64_t(0); /** TCU 的 ID **/
    ADDR_GLB_8_WITH_BANK addr_psum_ = ADDR_GLB_8_WITH_BANK(0); /** 中间结果的 GLB 中的地址 **/
    ADDR_GLB_8_WITH_BANK addr_dest_ = ADDR_GLB_8_WITH_BANK(0); /** 输出结果的GLB中的地址 **/
    uint64_t shape_output_n_ = uint64_t(0); /** 当前计算输出Slice的形状信息 **/
    uint64_t shape_output_c_ = uint64_t(0); /** 当前计算输出Slice的形状信息 **/
    uint64_t shape_output_h_ = uint64_t(0); /** 当前计算输出Slice的形状信息 **/
    uint64_t shape_output_w_ = uint64_t(0); /** 当前计算输出Slice的形状信息 **/
    STRIDE_GLB stride_output_glb_ = STRIDE_GLB(0); /** 输出featuremap在GLB的形状信息，决定寻址 **/
    STRIDE_GLB stride_psum_glb_ = STRIDE_GLB(0); /** 输出PSUM在GLB的形状信息，决定寻址 **/
    uint64_t x_cut_ = uint64_t(0); /**
                                                  水平方向一行第一次卷积的偏移量，硬件会切掉这些数，为了在反卷积模式下和tensorflow的实现保持一致
                                                  **/
    PRECISION output_precision_ = PRECISION(0); /** None **/
    QUAN_SIGNED output_signed_ = QUAN_SIGNED(0); /** None **/
public:
    inst_tcu_dm_conf_of(uint64_t tcu_id = uint64_t(0),
        ADDR_GLB_8_WITH_BANK addr_psum = ADDR_GLB_8_WITH_BANK(0),
        ADDR_GLB_8_WITH_BANK addr_dest = ADDR_GLB_8_WITH_BANK(0),
        uint64_t shape_output_n = uint64_t(0),
        uint64_t shape_output_c = uint64_t(0),
        uint64_t shape_output_h = uint64_t(0),
        uint64_t shape_output_w = uint64_t(0),
        STRIDE_GLB stride_output_glb = STRIDE_GLB(0),
        STRIDE_GLB stride_psum_glb = STRIDE_GLB(0),
        uint64_t x_cut = uint64_t(0),
        PRECISION output_precision = PRECISION(0),
        QUAN_SIGNED output_signed = QUAN_SIGNED(0));

    explicit inst_tcu_dm_conf_of(const INST_TCU_DM_CONF_OF &ref);

    [[nodiscard]] struct INST_TCU_DM_CONF_OF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置 TCU 输出信息
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置 TCU 输出信息
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t tcu_id TCU 的 ID
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id TCU 的 ID
  */
    void tcu_id(const uint64_t &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_psum 中间结果的 GLB 中的地址
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_psum() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_psum 中间结果的 GLB 中的地址
  */
    void addr_psum(const ADDR_GLB_8_WITH_BANK &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_dest 输出结果的GLB中的地址
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_dest() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_dest 输出结果的GLB中的地址
  */
    void addr_dest(const ADDR_GLB_8_WITH_BANK &value);
    /**
      uint64_t shape_output_n 当前计算输出Slice的形状信息
  */
    [[nodiscard]] uint64_t shape_output_n() const;
    /**
      uint64_t shape_output_n 当前计算输出Slice的形状信息
  */
    void shape_output_n(const uint64_t &value);
    /**
      uint64_t shape_output_c 当前计算输出Slice的形状信息
  */
    [[nodiscard]] uint64_t shape_output_c() const;
    /**
      uint64_t shape_output_c 当前计算输出Slice的形状信息
  */
    void shape_output_c(const uint64_t &value);
    /**
      uint64_t shape_output_h 当前计算输出Slice的形状信息
  */
    [[nodiscard]] uint64_t shape_output_h() const;
    /**
      uint64_t shape_output_h 当前计算输出Slice的形状信息
  */
    void shape_output_h(const uint64_t &value);
    /**
      uint64_t shape_output_w 当前计算输出Slice的形状信息
  */
    [[nodiscard]] uint64_t shape_output_w() const;
    /**
      uint64_t shape_output_w 当前计算输出Slice的形状信息
  */
    void shape_output_w(const uint64_t &value);
    /**
      STRIDE_GLB stride_output_glb 输出featuremap在GLB的形状信息，决定寻址
  */
    [[nodiscard]] STRIDE_GLB stride_output_glb() const;
    /**
      STRIDE_GLB stride_output_glb 输出featuremap在GLB的形状信息，决定寻址
  */
    void stride_output_glb(const STRIDE_GLB &value);
    /**
      STRIDE_GLB stride_psum_glb 输出PSUM在GLB的形状信息，决定寻址
  */
    [[nodiscard]] STRIDE_GLB stride_psum_glb() const;
    /**
      STRIDE_GLB stride_psum_glb 输出PSUM在GLB的形状信息，决定寻址
  */
    void stride_psum_glb(const STRIDE_GLB &value);
    /**
      uint64_t x_cut
     水平方向一行第一次卷积的偏移量，硬件会切掉这些数，为了在反卷积模式下和tensorflow的实现保持一致
  */
    [[nodiscard]] uint64_t x_cut() const;
    /**
      uint64_t x_cut
     水平方向一行第一次卷积的偏移量，硬件会切掉这些数，为了在反卷积模式下和tensorflow的实现保持一致
  */
    void x_cut(const uint64_t &value);
    /**
      PRECISION output_precision None
  */
    [[nodiscard]] PRECISION output_precision() const;
    /**
      PRECISION output_precision None
  */
    void output_precision(const PRECISION &value);
    /**
      QUAN_SIGNED output_signed None
  */
    [[nodiscard]] QUAN_SIGNED output_signed() const;
    /**
      QUAN_SIGNED output_signed None
  */
    void output_signed(const QUAN_SIGNED &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_tcu_dm_conf_of &c);

/**
    TCU.PU.CONF
    @brief 配置给定的 TCU 的 PU 所使用的 PE 资源数量
    - OPCODE opcode 配置给定的 TCU 的 PU 所使用的 PE 资源数量
    - uint64_t tcu_id TCU 的 ID
    - uint64_t pe_w TCU 的 PU 使用的 pe 列数
    - uint64_t pe_h 使用的 pe 行数
    - uint64_t pe_last_w 保留位置，不使用
    - uint64_t pe_last_h 在不能被整除的模式的时候PE最后一次计算的激活高度
    - uint64_t kernel_h kernel 高度，如果是矩阵乘模式无效
    - uint64_t kernel_w kernel 宽度，如果是矩阵乘模式无效
    - uint64_t group 分组卷积的组数，对应了TCU每个小块的宽度
    - uint64_t pu_loop_w 硬件数值循环次数
每个PE列的输出 channel 数，硬件内部循环用
    - uint64_t pu_loop_h 硬件水平循环次数
硬件内部信号=ceil(IF_c / input_channel_per_pu )
    - TCU_MODE mode 指定PU的工作模式
**/
class inst_tcu_pu_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置给定的 TCU 的 PU 所使用的 PE 资源数量 **/
    uint64_t tcu_id_ = uint64_t(0); /** TCU 的 ID **/
    uint64_t pe_w_ = uint64_t(0); /** TCU 的 PU 使用的 pe 列数 **/
    uint64_t pe_h_ = uint64_t(0); /** 使用的 pe 行数 **/
    uint64_t pe_last_w_ = uint64_t(0); /** 保留位置，不使用 **/
    uint64_t pe_last_h_ = uint64_t(0); /** 在不能被整除的模式的时候PE最后一次计算的激活高度 **/
    uint64_t kernel_h_ = uint64_t(0); /** kernel 高度，如果是矩阵乘模式无效 **/
    uint64_t kernel_w_ = uint64_t(0); /** kernel 宽度，如果是矩阵乘模式无效 **/
    uint64_t group_ = uint64_t(0); /** 分组卷积的组数，对应了TCU每个小块的宽度 **/
    uint64_t pu_loop_w_ = uint64_t(0); /** 硬件数值循环次数
每个PE列的输出 channel 数，硬件内部循环用 **/
    uint64_t pu_loop_h_ = uint64_t(0); /** 硬件水平循环次数
硬件内部信号=ceil(IF_c / input_channel_per_pu ) **/
    TCU_MODE mode_ = TCU_MODE(0); /** 指定PU的工作模式 **/
public:
    inst_tcu_pu_conf(
        uint64_t tcu_id = uint64_t(0), uint64_t pe_w = uint64_t(0),
        uint64_t pe_h = uint64_t(0), uint64_t pe_last_w = uint64_t(0),
        uint64_t pe_last_h = uint64_t(0), uint64_t kernel_h = uint64_t(0),
        uint64_t kernel_w = uint64_t(0), uint64_t group = uint64_t(0),
        uint64_t pu_loop_w = uint64_t(0), uint64_t pu_loop_h = uint64_t(0),
        TCU_MODE mode = TCU_MODE(0));

    explicit inst_tcu_pu_conf(const INST_TCU_PU_CONF &ref);

    [[nodiscard]] struct INST_TCU_PU_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置给定的 TCU 的 PU 所使用的 PE 资源数量
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置给定的 TCU 的 PU 所使用的 PE 资源数量
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t tcu_id TCU 的 ID
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id TCU 的 ID
  */
    void tcu_id(const uint64_t &value);
    /**
      uint64_t pe_w TCU 的 PU 使用的 pe 列数
  */
    [[nodiscard]] uint64_t pe_w() const;
    /**
      uint64_t pe_w TCU 的 PU 使用的 pe 列数
  */
    void pe_w(const uint64_t &value);
    /**
      uint64_t pe_h 使用的 pe 行数
  */
    [[nodiscard]] uint64_t pe_h() const;
    /**
      uint64_t pe_h 使用的 pe 行数
  */
    void pe_h(const uint64_t &value);
    /**
      uint64_t pe_last_w 保留位置，不使用
  */
    [[nodiscard]] uint64_t pe_last_w() const;
    /**
      uint64_t pe_last_w 保留位置，不使用
  */
    void pe_last_w(const uint64_t &value);
    /**
      uint64_t pe_last_h 在不能被整除的模式的时候PE最后一次计算的激活高度
  */
    [[nodiscard]] uint64_t pe_last_h() const;
    /**
      uint64_t pe_last_h 在不能被整除的模式的时候PE最后一次计算的激活高度
  */
    void pe_last_h(const uint64_t &value);
    /**
      uint64_t kernel_h kernel 高度，如果是矩阵乘模式无效
  */
    [[nodiscard]] uint64_t kernel_h() const;
    /**
      uint64_t kernel_h kernel 高度，如果是矩阵乘模式无效
  */
    void kernel_h(const uint64_t &value);
    /**
      uint64_t kernel_w kernel 宽度，如果是矩阵乘模式无效
  */
    [[nodiscard]] uint64_t kernel_w() const;
    /**
      uint64_t kernel_w kernel 宽度，如果是矩阵乘模式无效
  */
    void kernel_w(const uint64_t &value);
    /**
      uint64_t group 分组卷积的组数，对应了TCU每个小块的宽度
  */
    [[nodiscard]] uint64_t group() const;
    /**
      uint64_t group 分组卷积的组数，对应了TCU每个小块的宽度
  */
    void group(const uint64_t &value);
    /**
      uint64_t pu_loop_w 硬件数值循环次数
每个PE列的输出 channel 数，硬件内部循环用
  */
    [[nodiscard]] uint64_t pu_loop_w() const;
    /**
      uint64_t pu_loop_w 硬件数值循环次数
每个PE列的输出 channel 数，硬件内部循环用
  */
    void pu_loop_w(const uint64_t &value);
    /**
      uint64_t pu_loop_h 硬件水平循环次数
硬件内部信号=ceil(IF_c / input_channel_per_pu )
  */
    [[nodiscard]] uint64_t pu_loop_h() const;
    /**
      uint64_t pu_loop_h 硬件水平循环次数
硬件内部信号=ceil(IF_c / input_channel_per_pu )
  */
    void pu_loop_h(const uint64_t &value);
    /**
      TCU_MODE mode 指定PU的工作模式
  */
    [[nodiscard]] TCU_MODE mode() const;
    /**
      TCU_MODE mode 指定PU的工作模式
  */
    void mode(const TCU_MODE &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_tcu_pu_conf &c);

/**
    TCU.PU.CONF.ACT
    @brief 配置给定的 TCU 的 PU 使用的 fused 的两段式激活函数的参数
    - OPCODE opcode 配置给定的 TCU 的 PU 使用的 fused 的两段式激活函数的参数
    - uint64_t tcu_id TCU 的 ID
    - uint64_t clamp_max 上饱和数值
    - uint64_t clamp_min 下饱和数值
    - ADDR_GLB_8_WITH_BANK addr_act
每个输出通道激活参数（分段点，两端的scale，bias，以及量化参数）
**/
class inst_tcu_pu_conf_act : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(
        0); /** 配置给定的 TCU 的 PU 使用的 fused 的两段式激活函数的参数 **/
    uint64_t tcu_id_ = uint64_t(0); /** TCU 的 ID **/
    uint64_t clamp_max_ = uint64_t(0); /** 上饱和数值 **/
    uint64_t clamp_min_ = uint64_t(0); /** 下饱和数值 **/
    ADDR_GLB_8_WITH_BANK addr_act_ = ADDR_GLB_8_WITH_BANK(
        0); /** 每个输出通道激活参数（分段点，两端的scale，bias，以及量化参数）
             **/
public:
    inst_tcu_pu_conf_act(uint64_t tcu_id = uint64_t(0),
        uint64_t clamp_max = uint64_t(0),
        uint64_t clamp_min = uint64_t(0),
        ADDR_GLB_8_WITH_BANK addr_act = ADDR_GLB_8_WITH_BANK(0));

    explicit inst_tcu_pu_conf_act(const INST_TCU_PU_CONF_ACT &ref);

    [[nodiscard]] struct INST_TCU_PU_CONF_ACT to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置给定的 TCU 的 PU 使用的 fused 的两段式激活函数的参数
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置给定的 TCU 的 PU 使用的 fused 的两段式激活函数的参数
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t tcu_id TCU 的 ID
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id TCU 的 ID
  */
    void tcu_id(const uint64_t &value);
    /**
      uint64_t clamp_max 上饱和数值
  */
    [[nodiscard]] uint64_t clamp_max() const;
    /**
      uint64_t clamp_max 上饱和数值
  */
    void clamp_max(const uint64_t &value);
    /**
      uint64_t clamp_min 下饱和数值
  */
    [[nodiscard]] uint64_t clamp_min() const;
    /**
      uint64_t clamp_min 下饱和数值
  */
    void clamp_min(const uint64_t &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_act
     每个输出通道激活参数（分段点，两端的scale，bias，以及量化参数）
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_act() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_act
     每个输出通道激活参数（分段点，两端的scale，bias，以及量化参数）
  */
    void addr_act(const ADDR_GLB_8_WITH_BANK &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_tcu_pu_conf_act &c);

/**
    TCU.PU.COMPUTE
    @brief 启动PU一次计算
    - OPCODE opcode 启动PU一次计算
    - CCRCLR ccrclr_act None
    - CCRCLR ccrclr_psum None
    - CCRSET ccrset 数据依赖配置
    - uint64_t tcu_id TCU 的 ID
    - uint64_t act_enable 是否在这次计算完之后将结果做一次两段拟合的激活函数
    - uint64_t of_enable 决定输出是psum还是output featuremap
    - uint64_t load_psum 是否从 GLB 加载中间结果
    - uint64_t weight_switching 是否切换 PU 的 weights
**/
class inst_tcu_pu_compute : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 启动PU一次计算 **/
    CCRCLR ccrclr_act_ = CCRCLR(0); /** None **/
    CCRCLR ccrclr_psum_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** 数据依赖配置 **/
    uint64_t tcu_id_ = uint64_t(0); /** TCU 的 ID **/
    uint64_t act_enable_ = uint64_t(0); /** 是否在这次计算完之后将结果做一次两段拟合的激活函数 **/
    uint64_t of_enable_ = uint64_t(0); /** 决定输出是psum还是output featuremap **/
    uint64_t load_psum_ = uint64_t(0); /** 是否从 GLB 加载中间结果 **/
    uint64_t weight_switching_ = uint64_t(0); /** 是否切换 PU 的 weights **/
public:
    inst_tcu_pu_compute(CCRCLR ccrclr_act = CCRCLR(0),
        CCRCLR ccrclr_psum = CCRCLR(0), CCRSET ccrset = CCRSET(0),
        uint64_t tcu_id = uint64_t(0),
        uint64_t act_enable = uint64_t(0),
        uint64_t of_enable = uint64_t(0),
        uint64_t load_psum = uint64_t(0),
        uint64_t weight_switching = uint64_t(0));

    explicit inst_tcu_pu_compute(const INST_TCU_PU_COMPUTE &ref);

    [[nodiscard]] struct INST_TCU_PU_COMPUTE to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 启动PU一次计算
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 启动PU一次计算
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr_act None
  */
    [[nodiscard]] CCRCLR ccrclr_act() const;
    /**
      CCRCLR ccrclr_act None
  */
    void ccrclr_act(const CCRCLR &value);
    /**
      CCRCLR ccrclr_psum None
  */
    [[nodiscard]] CCRCLR ccrclr_psum() const;
    /**
      CCRCLR ccrclr_psum None
  */
    void ccrclr_psum(const CCRCLR &value);
    /**
      CCRSET ccrset 数据依赖配置
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset 数据依赖配置
  */
    void ccrset(const CCRSET &value);
    /**
      uint64_t tcu_id TCU 的 ID
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id TCU 的 ID
  */
    void tcu_id(const uint64_t &value);
    /**
      uint64_t act_enable 是否在这次计算完之后将结果做一次两段拟合的激活函数
  */
    [[nodiscard]] uint64_t act_enable() const;
    /**
      uint64_t act_enable 是否在这次计算完之后将结果做一次两段拟合的激活函数
  */
    void act_enable(const uint64_t &value);
    /**
      uint64_t of_enable 决定输出是psum还是output featuremap
  */
    [[nodiscard]] uint64_t of_enable() const;
    /**
      uint64_t of_enable 决定输出是psum还是output featuremap
  */
    void of_enable(const uint64_t &value);
    /**
      uint64_t load_psum 是否从 GLB 加载中间结果
  */
    [[nodiscard]] uint64_t load_psum() const;
    /**
      uint64_t load_psum 是否从 GLB 加载中间结果
  */
    void load_psum(const uint64_t &value);
    /**
      uint64_t weight_switching 是否切换 PU 的 weights
  */
    [[nodiscard]] uint64_t weight_switching() const;
    /**
      uint64_t weight_switching 是否切换 PU 的 weights
  */
    void weight_switching(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_tcu_pu_compute &c);

/**
    TCU.DOT.DM.IF.CONF
    @brief None
    - OPCODE opcode None
    - uint64_t tcu_id None
    - STRIDE_GLB stride_src1_glb None
    - STRIDE_GLB stride_src2_glb None
**/
class inst_tcu_dot_dm_if_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** None **/
    uint64_t tcu_id_ = uint64_t(0); /** None **/
    STRIDE_GLB stride_src1_glb_ = STRIDE_GLB(0); /** None **/
    STRIDE_GLB stride_src2_glb_ = STRIDE_GLB(0); /** None **/
public:
    inst_tcu_dot_dm_if_conf(uint64_t tcu_id = uint64_t(0),
        STRIDE_GLB stride_src1_glb = STRIDE_GLB(0),
        STRIDE_GLB stride_src2_glb = STRIDE_GLB(0));

    explicit inst_tcu_dot_dm_if_conf(const INST_TCU_DOT_DM_IF_CONF &ref);

    [[nodiscard]] struct INST_TCU_DOT_DM_IF_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode None
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode None
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t tcu_id None
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id None
  */
    void tcu_id(const uint64_t &value);
    /**
      STRIDE_GLB stride_src1_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_src1_glb() const;
    /**
      STRIDE_GLB stride_src1_glb None
  */
    void stride_src1_glb(const STRIDE_GLB &value);
    /**
      STRIDE_GLB stride_src2_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_src2_glb() const;
    /**
      STRIDE_GLB stride_src2_glb None
  */
    void stride_src2_glb(const STRIDE_GLB &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_tcu_dot_dm_if_conf &c);

/**
    TCU.DOT.DM.OF.CONF
    @brief None
    - OPCODE opcode None
    - uint64_t tcu_id None
    - uint64_t stride_psum_glb None
    - ADDR_GLB_8_WITH_BANK addr_psum None
    - ADDR_GLB_8_WITH_BANK addr_dest None
    - STRIDE_GLB stride_dest_glb None
    - uint64_t shape_dest_n None
    - uint64_t shape_dest_h None
    - uint64_t shape_dest_w None
    - uint64_t shape_src1_w None
    - PRECISION output_precision None
    - QUAN_SIGNED output_signed None
**/
class inst_tcu_dot_dm_of_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** None **/
    uint64_t tcu_id_ = uint64_t(0); /** None **/
    uint64_t stride_psum_glb_ = uint64_t(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_psum_ = ADDR_GLB_8_WITH_BANK(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_dest_ = ADDR_GLB_8_WITH_BANK(0); /** None **/
    STRIDE_GLB stride_dest_glb_ = STRIDE_GLB(0); /** None **/
    uint64_t shape_dest_n_ = uint64_t(0); /** None **/
    uint64_t shape_dest_h_ = uint64_t(0); /** None **/
    uint64_t shape_dest_w_ = uint64_t(0); /** None **/
    uint64_t shape_src1_w_ = uint64_t(0); /** None **/
    PRECISION output_precision_ = PRECISION(0); /** None **/
    QUAN_SIGNED output_signed_ = QUAN_SIGNED(0); /** None **/
public:
    inst_tcu_dot_dm_of_conf(
        uint64_t tcu_id = uint64_t(0), uint64_t stride_psum_glb = uint64_t(0),
        ADDR_GLB_8_WITH_BANK addr_psum = ADDR_GLB_8_WITH_BANK(0),
        ADDR_GLB_8_WITH_BANK addr_dest = ADDR_GLB_8_WITH_BANK(0),
        STRIDE_GLB stride_dest_glb = STRIDE_GLB(0),
        uint64_t shape_dest_n = uint64_t(0), uint64_t shape_dest_h = uint64_t(0),
        uint64_t shape_dest_w = uint64_t(0), uint64_t shape_src1_w = uint64_t(0),
        PRECISION output_precision = PRECISION(0),
        QUAN_SIGNED output_signed = QUAN_SIGNED(0));

    explicit inst_tcu_dot_dm_of_conf(const INST_TCU_DOT_DM_OF_CONF &ref);

    [[nodiscard]] struct INST_TCU_DOT_DM_OF_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode None
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode None
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t tcu_id None
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id None
  */
    void tcu_id(const uint64_t &value);
    /**
      uint64_t stride_psum_glb None
  */
    [[nodiscard]] uint64_t stride_psum_glb() const;
    /**
      uint64_t stride_psum_glb None
  */
    void stride_psum_glb(const uint64_t &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_psum None
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_psum() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_psum None
  */
    void addr_psum(const ADDR_GLB_8_WITH_BANK &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_dest None
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_dest() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_dest None
  */
    void addr_dest(const ADDR_GLB_8_WITH_BANK &value);
    /**
      STRIDE_GLB stride_dest_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_dest_glb() const;
    /**
      STRIDE_GLB stride_dest_glb None
  */
    void stride_dest_glb(const STRIDE_GLB &value);
    /**
      uint64_t shape_dest_n None
  */
    [[nodiscard]] uint64_t shape_dest_n() const;
    /**
      uint64_t shape_dest_n None
  */
    void shape_dest_n(const uint64_t &value);
    /**
      uint64_t shape_dest_h None
  */
    [[nodiscard]] uint64_t shape_dest_h() const;
    /**
      uint64_t shape_dest_h None
  */
    void shape_dest_h(const uint64_t &value);
    /**
      uint64_t shape_dest_w None
  */
    [[nodiscard]] uint64_t shape_dest_w() const;
    /**
      uint64_t shape_dest_w None
  */
    void shape_dest_w(const uint64_t &value);
    /**
      uint64_t shape_src1_w None
  */
    [[nodiscard]] uint64_t shape_src1_w() const;
    /**
      uint64_t shape_src1_w None
  */
    void shape_src1_w(const uint64_t &value);
    /**
      PRECISION output_precision None
  */
    [[nodiscard]] PRECISION output_precision() const;
    /**
      PRECISION output_precision None
  */
    void output_precision(const PRECISION &value);
    /**
      QUAN_SIGNED output_signed None
  */
    [[nodiscard]] QUAN_SIGNED output_signed() const;
    /**
      QUAN_SIGNED output_signed None
  */
    void output_signed(const QUAN_SIGNED &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_tcu_dot_dm_of_conf &c);

/**
    TCU.DOT.DM.FETCH.SRC1
    @brief None
    - OPCODE opcode None
    - CCRCLR ccrclr_src1 None
    - uint64_t tcu_id None
    - ADDR_GLB_8_WITH_BANK addr_src1 None
**/
class inst_tcu_dot_dm_fetch_src1 : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** None **/
    CCRCLR ccrclr_src1_ = CCRCLR(0); /** None **/
    uint64_t tcu_id_ = uint64_t(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_src1_ = ADDR_GLB_8_WITH_BANK(0); /** None **/
public:
    inst_tcu_dot_dm_fetch_src1(
        CCRCLR ccrclr_src1 = CCRCLR(0), uint64_t tcu_id = uint64_t(0),
        ADDR_GLB_8_WITH_BANK addr_src1 = ADDR_GLB_8_WITH_BANK(0));

    explicit inst_tcu_dot_dm_fetch_src1(const INST_TCU_DOT_DM_FETCH_SRC1 &ref);

    [[nodiscard]] struct INST_TCU_DOT_DM_FETCH_SRC1 to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode None
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode None
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr_src1 None
  */
    [[nodiscard]] CCRCLR ccrclr_src1() const;
    /**
      CCRCLR ccrclr_src1 None
  */
    void ccrclr_src1(const CCRCLR &value);
    /**
      uint64_t tcu_id None
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id None
  */
    void tcu_id(const uint64_t &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_src1 None
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_src1() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_src1 None
  */
    void addr_src1(const ADDR_GLB_8_WITH_BANK &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out,
    const inst_tcu_dot_dm_fetch_src1 &c);

/**
    TCU.DOT.DM.FETCH.SRC2
    @brief None
    - OPCODE opcode None
    - CCRCLR ccrclr_src2 None
    - uint64_t tcu_id None
    - ADDR_GLB_8_WITH_BANK addr_src2 None
**/
class inst_tcu_dot_dm_fetch_src2 : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** None **/
    CCRCLR ccrclr_src2_ = CCRCLR(0); /** None **/
    uint64_t tcu_id_ = uint64_t(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_src2_ = ADDR_GLB_8_WITH_BANK(0); /** None **/
public:
    inst_tcu_dot_dm_fetch_src2(
        CCRCLR ccrclr_src2 = CCRCLR(0), uint64_t tcu_id = uint64_t(0),
        ADDR_GLB_8_WITH_BANK addr_src2 = ADDR_GLB_8_WITH_BANK(0));

    explicit inst_tcu_dot_dm_fetch_src2(const INST_TCU_DOT_DM_FETCH_SRC2 &ref);

    [[nodiscard]] struct INST_TCU_DOT_DM_FETCH_SRC2 to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode None
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode None
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr_src2 None
  */
    [[nodiscard]] CCRCLR ccrclr_src2() const;
    /**
      CCRCLR ccrclr_src2 None
  */
    void ccrclr_src2(const CCRCLR &value);
    /**
      uint64_t tcu_id None
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id None
  */
    void tcu_id(const uint64_t &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_src2 None
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_src2() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_src2 None
  */
    void addr_src2(const ADDR_GLB_8_WITH_BANK &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out,
    const inst_tcu_dot_dm_fetch_src2 &c);

/**
    TCU.PU.COMPUTE.DUMMY
    @brief 用来把PSUM转成output的指令
    - OPCODE opcode 用来把PSUM转成output的指令
    - CCRCLR ccrclr_act None
    - CCRCLR ccrclr_psum None
    - CCRSET ccrset 数据依赖配置
    - uint64_t tcu_id TCU 的 ID
    - uint64_t act_enable 是否在这次计算完之后将结果做一次两段拟合的激活函数
**/
class inst_tcu_pu_compute_dummy : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 用来把PSUM转成output的指令 **/
    CCRCLR ccrclr_act_ = CCRCLR(0); /** None **/
    CCRCLR ccrclr_psum_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** 数据依赖配置 **/
    uint64_t tcu_id_ = uint64_t(0); /** TCU 的 ID **/
    uint64_t act_enable_ = uint64_t(0); /** 是否在这次计算完之后将结果做一次两段拟合的激活函数 **/
public:
    inst_tcu_pu_compute_dummy(CCRCLR ccrclr_act = CCRCLR(0),
        CCRCLR ccrclr_psum = CCRCLR(0),
        CCRSET ccrset = CCRSET(0),
        uint64_t tcu_id = uint64_t(0),
        uint64_t act_enable = uint64_t(0));

    explicit inst_tcu_pu_compute_dummy(const INST_TCU_PU_COMPUTE_DUMMY &ref);

    [[nodiscard]] struct INST_TCU_PU_COMPUTE_DUMMY to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 用来把PSUM转成output的指令
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 用来把PSUM转成output的指令
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr_act None
  */
    [[nodiscard]] CCRCLR ccrclr_act() const;
    /**
      CCRCLR ccrclr_act None
  */
    void ccrclr_act(const CCRCLR &value);
    /**
      CCRCLR ccrclr_psum None
  */
    [[nodiscard]] CCRCLR ccrclr_psum() const;
    /**
      CCRCLR ccrclr_psum None
  */
    void ccrclr_psum(const CCRCLR &value);
    /**
      CCRSET ccrset 数据依赖配置
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset 数据依赖配置
  */
    void ccrset(const CCRSET &value);
    /**
      uint64_t tcu_id TCU 的 ID
  */
    [[nodiscard]] uint64_t tcu_id() const;
    /**
      uint64_t tcu_id TCU 的 ID
  */
    void tcu_id(const uint64_t &value);
    /**
      uint64_t act_enable 是否在这次计算完之后将结果做一次两段拟合的激活函数
  */
    [[nodiscard]] uint64_t act_enable() const;
    /**
      uint64_t act_enable 是否在这次计算完之后将结果做一次两段拟合的激活函数
  */
    void act_enable(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_tcu_pu_compute_dummy &c);

/**
    MFU.MN.MAP.COMPUTE
    @brief 配置MFU的DM模块，准备开始计算对于tensor的map运算
    - OPCODE opcode 配置MFU的DM模块，准备开始计算对于tensor的map运算
    - CCRCLR ccrclr None
    - CCRSET ccrset None
    - UNION_ADDR addr_src 输入向量slice地址 (GLB or DDR)
    - UNION_ADDR addr_dest 输出向量地址 (GLB or DDR)
    - STRIDE_GLB stride_input_glb Tensor 的 shape
    - STRIDE_GLB stride_output_glb None
    - uint64_t shape_n Slice 长度信息
    - uint64_t shape_c Slice 长度信息
    - uint64_t shape_h Slice 长度信息
    - uint64_t shape_w Slice 长度信息
    - uint64_t basement_src1 None
    - uint64_t basement_dest None
**/
class inst_mfu_mn_map_compute : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置MFU的DM模块，准备开始计算对于tensor的map运算 **/
    CCRCLR ccrclr_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** None **/
    UNION_ADDR addr_src_ = UNION_ADDR(0); /** 输入向量slice地址 (GLB or DDR) **/
    UNION_ADDR addr_dest_ = UNION_ADDR(0); /** 输出向量地址 (GLB or DDR) **/
    STRIDE_GLB stride_input_glb_ = STRIDE_GLB(0); /** Tensor 的 shape **/
    STRIDE_GLB stride_output_glb_ = STRIDE_GLB(0); /** None **/
    uint64_t shape_n_ = uint64_t(0); /** Slice 长度信息 **/
    uint64_t shape_c_ = uint64_t(0); /** Slice 长度信息 **/
    uint64_t shape_h_ = uint64_t(0); /** Slice 长度信息 **/
    uint64_t shape_w_ = uint64_t(0); /** Slice 长度信息 **/
    uint64_t basement_src1_ = uint64_t(0); /** None **/
    uint64_t basement_dest_ = uint64_t(0); /** None **/
public:
    inst_mfu_mn_map_compute(CCRCLR ccrclr = CCRCLR(0), CCRSET ccrset = CCRSET(0),
        UNION_ADDR addr_src = UNION_ADDR(0),
        UNION_ADDR addr_dest = UNION_ADDR(0),
        STRIDE_GLB stride_input_glb = STRIDE_GLB(0),
        STRIDE_GLB stride_output_glb = STRIDE_GLB(0),
        uint64_t shape_n = uint64_t(0),
        uint64_t shape_c = uint64_t(0),
        uint64_t shape_h = uint64_t(0),
        uint64_t shape_w = uint64_t(0),
        uint64_t basement_src1 = uint64_t(0),
        uint64_t basement_dest = uint64_t(0));

    explicit inst_mfu_mn_map_compute(const INST_MFU_MN_MAP_COMPUTE &ref);

    [[nodiscard]] struct INST_MFU_MN_MAP_COMPUTE to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置MFU的DM模块，准备开始计算对于tensor的map运算
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置MFU的DM模块，准备开始计算对于tensor的map运算
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr None
  */
    [[nodiscard]] CCRCLR ccrclr() const;
    /**
      CCRCLR ccrclr None
  */
    void ccrclr(const CCRCLR &value);
    /**
      CCRSET ccrset None
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset None
  */
    void ccrset(const CCRSET &value);
    /**
      UNION_ADDR addr_src 输入向量slice地址 (GLB or DDR)
  */
    [[nodiscard]] UNION_ADDR addr_src() const;
    /**
      UNION_ADDR addr_src 输入向量slice地址 (GLB or DDR)
  */
    void addr_src(const UNION_ADDR &value);
    /**
      UNION_ADDR addr_dest 输出向量地址 (GLB or DDR)
  */
    [[nodiscard]] UNION_ADDR addr_dest() const;
    /**
      UNION_ADDR addr_dest 输出向量地址 (GLB or DDR)
  */
    void addr_dest(const UNION_ADDR &value);
    /**
      STRIDE_GLB stride_input_glb Tensor 的 shape
  */
    [[nodiscard]] STRIDE_GLB stride_input_glb() const;
    /**
      STRIDE_GLB stride_input_glb Tensor 的 shape
  */
    void stride_input_glb(const STRIDE_GLB &value);
    /**
      STRIDE_GLB stride_output_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_output_glb() const;
    /**
      STRIDE_GLB stride_output_glb None
  */
    void stride_output_glb(const STRIDE_GLB &value);
    /**
      uint64_t shape_n Slice 长度信息
  */
    [[nodiscard]] uint64_t shape_n() const;
    /**
      uint64_t shape_n Slice 长度信息
  */
    void shape_n(const uint64_t &value);
    /**
      uint64_t shape_c Slice 长度信息
  */
    [[nodiscard]] uint64_t shape_c() const;
    /**
      uint64_t shape_c Slice 长度信息
  */
    void shape_c(const uint64_t &value);
    /**
      uint64_t shape_h Slice 长度信息
  */
    [[nodiscard]] uint64_t shape_h() const;
    /**
      uint64_t shape_h Slice 长度信息
  */
    void shape_h(const uint64_t &value);
    /**
      uint64_t shape_w Slice 长度信息
  */
    [[nodiscard]] uint64_t shape_w() const;
    /**
      uint64_t shape_w Slice 长度信息
  */
    void shape_w(const uint64_t &value);
    /**
      uint64_t basement_src1 None
  */
    [[nodiscard]] uint64_t basement_src1() const;
    /**
      uint64_t basement_src1 None
  */
    void basement_src1(const uint64_t &value);
    /**
      uint64_t basement_dest None
  */
    [[nodiscard]] uint64_t basement_dest() const;
    /**
      uint64_t basement_dest None
  */
    void basement_dest(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_mn_map_compute &c);

/**
    MFU.MN.VMAP.COMPUTE
    @brief 配置MFU的DM模块，准备开始计算vector的map运算
    - OPCODE opcode 配置MFU的DM模块，准备开始计算vector的map运算
    - CCRCLR ccrclr None
    - CCRSET ccrset None
    - UNION_ADDR addr_src 输入向量地址 (GLB or DDR)
    - ADDR addr_dest 输出向量地址 (GLB or DDR)
    - uint64_t length 向量长度
    - uint64_t basement_src None
    - uint64_t basement_dest None
**/
class inst_mfu_mn_vmap_compute : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置MFU的DM模块，准备开始计算vector的map运算 **/
    CCRCLR ccrclr_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** None **/
    UNION_ADDR addr_src_ = UNION_ADDR(0); /** 输入向量地址 (GLB or DDR) **/
    ADDR addr_dest_ = ADDR(0); /** 输出向量地址 (GLB or DDR) **/
    uint64_t length_ = uint64_t(0); /** 向量长度 **/
    uint64_t basement_src_ = uint64_t(0); /** None **/
    uint64_t basement_dest_ = uint64_t(0); /** None **/
public:
    inst_mfu_mn_vmap_compute(CCRCLR ccrclr = CCRCLR(0), CCRSET ccrset = CCRSET(0),
        UNION_ADDR addr_src = UNION_ADDR(0),
        ADDR addr_dest = ADDR(0),
        uint64_t length = uint64_t(0),
        uint64_t basement_src = uint64_t(0),
        uint64_t basement_dest = uint64_t(0));

    explicit inst_mfu_mn_vmap_compute(const INST_MFU_MN_VMAP_COMPUTE &ref);

    [[nodiscard]] struct INST_MFU_MN_VMAP_COMPUTE to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置MFU的DM模块，准备开始计算vector的map运算
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置MFU的DM模块，准备开始计算vector的map运算
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr None
  */
    [[nodiscard]] CCRCLR ccrclr() const;
    /**
      CCRCLR ccrclr None
  */
    void ccrclr(const CCRCLR &value);
    /**
      CCRSET ccrset None
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset None
  */
    void ccrset(const CCRSET &value);
    /**
      UNION_ADDR addr_src 输入向量地址 (GLB or DDR)
  */
    [[nodiscard]] UNION_ADDR addr_src() const;
    /**
      UNION_ADDR addr_src 输入向量地址 (GLB or DDR)
  */
    void addr_src(const UNION_ADDR &value);
    /**
      ADDR addr_dest 输出向量地址 (GLB or DDR)
  */
    [[nodiscard]] ADDR addr_dest() const;
    /**
      ADDR addr_dest 输出向量地址 (GLB or DDR)
  */
    void addr_dest(const ADDR &value);
    /**
      uint64_t length 向量长度
  */
    [[nodiscard]] uint64_t length() const;
    /**
      uint64_t length 向量长度
  */
    void length(const uint64_t &value);
    /**
      uint64_t basement_src None
  */
    [[nodiscard]] uint64_t basement_src() const;
    /**
      uint64_t basement_src None
  */
    void basement_src(const uint64_t &value);
    /**
      uint64_t basement_dest None
  */
    [[nodiscard]] uint64_t basement_dest() const;
    /**
      uint64_t basement_dest None
  */
    void basement_dest(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_mn_vmap_compute &c);

/**
    MFU.REDUCE
    @brief 计算reduce，这个指令相当于bypass meshnet的reduce的短版本
    - OPCODE opcode 计算reduce，这个指令相当于bypass meshnet的reduce的短版本
    - CCRCLR ccrclr None
    - CCRSET ccrset None
    - UNION_ADDR addr_src 输入向量地址 (GLB or DDR)
    - UNION_ADDR addr_dest 输出到 GLB 或者 DDR 的位置
    - uint64_t init_value need_init 为 True 时有效，此时上面的 a1 为 init_value
    - STRIDE_GLB stride_input_glb Tensor 的 shape
    - uint64_t shape_n Slice 长度信息
    - uint64_t shape_c Slice 长度信息
    - uint64_t shape_h Slice 长度信息
    - uint64_t shape_w Slice 长度信息
    - MFU_REDUCE_OP op 参与的运算 Scalar Binary Operator（含义见上方）
    - MFU_REDUCE_DIM dimension None
    - uint64_t basement_src 是否是栈变量
    - uint64_t basement_dest 是否是栈变量
**/
class inst_mfu_reduce : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(
        0); /** 计算reduce，这个指令相当于bypass meshnet的reduce的短版本 **/
    CCRCLR ccrclr_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** None **/
    UNION_ADDR addr_src_ = UNION_ADDR(0); /** 输入向量地址 (GLB or DDR) **/
    UNION_ADDR addr_dest_ = UNION_ADDR(0); /** 输出到 GLB 或者 DDR 的位置 **/
    uint64_t init_value_ = uint64_t(0); /** need_init 为 True 时有效，此时上面的 a1 为 init_value **/
    STRIDE_GLB stride_input_glb_ = STRIDE_GLB(0); /** Tensor 的 shape **/
    uint64_t shape_n_ = uint64_t(0); /** Slice 长度信息 **/
    uint64_t shape_c_ = uint64_t(0); /** Slice 长度信息 **/
    uint64_t shape_h_ = uint64_t(0); /** Slice 长度信息 **/
    uint64_t shape_w_ = uint64_t(0); /** Slice 长度信息 **/
    MFU_REDUCE_OP op_ = MFU_REDUCE_OP(0); /** 参与的运算 Scalar Binary Operator（含义见上方） **/
    MFU_REDUCE_DIM dimension_ = MFU_REDUCE_DIM(0); /** None **/
    uint64_t basement_src_ = uint64_t(0); /** 是否是栈变量 **/
    uint64_t basement_dest_ = uint64_t(0); /** 是否是栈变量 **/
public:
    inst_mfu_reduce(CCRCLR ccrclr = CCRCLR(0), CCRSET ccrset = CCRSET(0),
        UNION_ADDR addr_src = UNION_ADDR(0),
        UNION_ADDR addr_dest = UNION_ADDR(0),
        uint64_t init_value = uint64_t(0),
        STRIDE_GLB stride_input_glb = STRIDE_GLB(0),
        uint64_t shape_n = uint64_t(0),
        uint64_t shape_c = uint64_t(0),
        uint64_t shape_h = uint64_t(0),
        uint64_t shape_w = uint64_t(0),
        MFU_REDUCE_OP op = MFU_REDUCE_OP(0),
        MFU_REDUCE_DIM dimension = MFU_REDUCE_DIM(0),
        uint64_t basement_src = uint64_t(0),
        uint64_t basement_dest = uint64_t(0));

    explicit inst_mfu_reduce(const INST_MFU_REDUCE &ref);

    [[nodiscard]] struct INST_MFU_REDUCE to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 计算reduce，这个指令相当于bypass meshnet的reduce的短版本
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 计算reduce，这个指令相当于bypass meshnet的reduce的短版本
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr None
  */
    [[nodiscard]] CCRCLR ccrclr() const;
    /**
      CCRCLR ccrclr None
  */
    void ccrclr(const CCRCLR &value);
    /**
      CCRSET ccrset None
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset None
  */
    void ccrset(const CCRSET &value);
    /**
      UNION_ADDR addr_src 输入向量地址 (GLB or DDR)
  */
    [[nodiscard]] UNION_ADDR addr_src() const;
    /**
      UNION_ADDR addr_src 输入向量地址 (GLB or DDR)
  */
    void addr_src(const UNION_ADDR &value);
    /**
      UNION_ADDR addr_dest 输出到 GLB 或者 DDR 的位置
  */
    [[nodiscard]] UNION_ADDR addr_dest() const;
    /**
      UNION_ADDR addr_dest 输出到 GLB 或者 DDR 的位置
  */
    void addr_dest(const UNION_ADDR &value);
    /**
      uint64_t init_value need_init 为 True 时有效，此时上面的 a1 为 init_value
  */
    [[nodiscard]] uint64_t init_value() const;
    /**
      uint64_t init_value need_init 为 True 时有效，此时上面的 a1 为 init_value
  */
    void init_value(const uint64_t &value);
    /**
      STRIDE_GLB stride_input_glb Tensor 的 shape
  */
    [[nodiscard]] STRIDE_GLB stride_input_glb() const;
    /**
      STRIDE_GLB stride_input_glb Tensor 的 shape
  */
    void stride_input_glb(const STRIDE_GLB &value);
    /**
      uint64_t shape_n Slice 长度信息
  */
    [[nodiscard]] uint64_t shape_n() const;
    /**
      uint64_t shape_n Slice 长度信息
  */
    void shape_n(const uint64_t &value);
    /**
      uint64_t shape_c Slice 长度信息
  */
    [[nodiscard]] uint64_t shape_c() const;
    /**
      uint64_t shape_c Slice 长度信息
  */
    void shape_c(const uint64_t &value);
    /**
      uint64_t shape_h Slice 长度信息
  */
    [[nodiscard]] uint64_t shape_h() const;
    /**
      uint64_t shape_h Slice 长度信息
  */
    void shape_h(const uint64_t &value);
    /**
      uint64_t shape_w Slice 长度信息
  */
    [[nodiscard]] uint64_t shape_w() const;
    /**
      uint64_t shape_w Slice 长度信息
  */
    void shape_w(const uint64_t &value);
    /**
      MFU_REDUCE_OP op 参与的运算 Scalar Binary Operator（含义见上方）
  */
    [[nodiscard]] MFU_REDUCE_OP op() const;
    /**
      MFU_REDUCE_OP op 参与的运算 Scalar Binary Operator（含义见上方）
  */
    void op(const MFU_REDUCE_OP &value);
    /**
      MFU_REDUCE_DIM dimension None
  */
    [[nodiscard]] MFU_REDUCE_DIM dimension() const;
    /**
      MFU_REDUCE_DIM dimension None
  */
    void dimension(const MFU_REDUCE_DIM &value);
    /**
      uint64_t basement_src 是否是栈变量
  */
    [[nodiscard]] uint64_t basement_src() const;
    /**
      uint64_t basement_src 是否是栈变量
  */
    void basement_src(const uint64_t &value);
    /**
      uint64_t basement_dest 是否是栈变量
  */
    [[nodiscard]] uint64_t basement_dest() const;
    /**
      uint64_t basement_dest 是否是栈变量
  */
    void basement_dest(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_reduce &c);

/**
    MFU.VREDUCE
    @brief 计算reduce，这个指令相当于bypass meshnet的reduce的短版本
    - OPCODE opcode 计算reduce，这个指令相当于bypass meshnet的reduce的短版本
    - CCRCLR ccrclr None
    - CCRSET ccrset None
    - UNION_ADDR addr_src 输入向量地址 (GLB or DDR)
    - UNION_ADDR addr_dest 输出到 GLB 或者 DDR 的位置
    - uint64_t init_value need_init 为 True 时有效，此时上面的 a1 为 init_value
    - uint64_t length 向量输出长度（总长度是length*reduce_length）
    - uint64_t reduce_length 每这么多个点输出一个点
    - MFU_REDUCE_OP op 参与的运算 Scalar Binary Operator（含义见上方）
    - uint64_t basement_src 是否是栈变量
    - uint64_t basement_dest 是否是栈变量
**/
class inst_mfu_vreduce : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(
        0); /** 计算reduce，这个指令相当于bypass meshnet的reduce的短版本 **/
    CCRCLR ccrclr_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** None **/
    UNION_ADDR addr_src_ = UNION_ADDR(0); /** 输入向量地址 (GLB or DDR) **/
    UNION_ADDR addr_dest_ = UNION_ADDR(0); /** 输出到 GLB 或者 DDR 的位置 **/
    uint64_t init_value_ = uint64_t(0); /** need_init 为 True 时有效，此时上面的 a1 为 init_value **/
    uint64_t length_ = uint64_t(0); /** 向量输出长度（总长度是length*reduce_length） **/
    uint64_t reduce_length_ = uint64_t(0); /** 每这么多个点输出一个点 **/
    MFU_REDUCE_OP op_ = MFU_REDUCE_OP(0); /** 参与的运算 Scalar Binary Operator（含义见上方） **/
    uint64_t basement_src_ = uint64_t(0); /** 是否是栈变量 **/
    uint64_t basement_dest_ = uint64_t(0); /** 是否是栈变量 **/
public:
    inst_mfu_vreduce(CCRCLR ccrclr = CCRCLR(0), CCRSET ccrset = CCRSET(0),
        UNION_ADDR addr_src = UNION_ADDR(0),
        UNION_ADDR addr_dest = UNION_ADDR(0),
        uint64_t init_value = uint64_t(0),
        uint64_t length = uint64_t(0),
        uint64_t reduce_length = uint64_t(0),
        MFU_REDUCE_OP op = MFU_REDUCE_OP(0),
        uint64_t basement_src = uint64_t(0),
        uint64_t basement_dest = uint64_t(0));

    explicit inst_mfu_vreduce(const INST_MFU_VREDUCE &ref);

    [[nodiscard]] struct INST_MFU_VREDUCE to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 计算reduce，这个指令相当于bypass meshnet的reduce的短版本
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 计算reduce，这个指令相当于bypass meshnet的reduce的短版本
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr None
  */
    [[nodiscard]] CCRCLR ccrclr() const;
    /**
      CCRCLR ccrclr None
  */
    void ccrclr(const CCRCLR &value);
    /**
      CCRSET ccrset None
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset None
  */
    void ccrset(const CCRSET &value);
    /**
      UNION_ADDR addr_src 输入向量地址 (GLB or DDR)
  */
    [[nodiscard]] UNION_ADDR addr_src() const;
    /**
      UNION_ADDR addr_src 输入向量地址 (GLB or DDR)
  */
    void addr_src(const UNION_ADDR &value);
    /**
      UNION_ADDR addr_dest 输出到 GLB 或者 DDR 的位置
  */
    [[nodiscard]] UNION_ADDR addr_dest() const;
    /**
      UNION_ADDR addr_dest 输出到 GLB 或者 DDR 的位置
  */
    void addr_dest(const UNION_ADDR &value);
    /**
      uint64_t init_value need_init 为 True 时有效，此时上面的 a1 为 init_value
  */
    [[nodiscard]] uint64_t init_value() const;
    /**
      uint64_t init_value need_init 为 True 时有效，此时上面的 a1 为 init_value
  */
    void init_value(const uint64_t &value);
    /**
      uint64_t length 向量输出长度（总长度是length*reduce_length）
  */
    [[nodiscard]] uint64_t length() const;
    /**
      uint64_t length 向量输出长度（总长度是length*reduce_length）
  */
    void length(const uint64_t &value);
    /**
      uint64_t reduce_length 每这么多个点输出一个点
  */
    [[nodiscard]] uint64_t reduce_length() const;
    /**
      uint64_t reduce_length 每这么多个点输出一个点
  */
    void reduce_length(const uint64_t &value);
    /**
      MFU_REDUCE_OP op 参与的运算 Scalar Binary Operator（含义见上方）
  */
    [[nodiscard]] MFU_REDUCE_OP op() const;
    /**
      MFU_REDUCE_OP op 参与的运算 Scalar Binary Operator（含义见上方）
  */
    void op(const MFU_REDUCE_OP &value);
    /**
      uint64_t basement_src 是否是栈变量
  */
    [[nodiscard]] uint64_t basement_src() const;
    /**
      uint64_t basement_src 是否是栈变量
  */
    void basement_src(const uint64_t &value);
    /**
      uint64_t basement_dest 是否是栈变量
  */
    [[nodiscard]] uint64_t basement_dest() const;
    /**
      uint64_t basement_dest 是否是栈变量
  */
    void basement_dest(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_vreduce &c);

/**
    MFU.MN.BROADCAST.COMPUTE
    @brief 配置MFU的DM模块，准备开始计算
    - OPCODE opcode 配置MFU的DM模块，准备开始计算
    - CCRCLR ccrclr_src1 None
    - CCRCLR ccrclr_src2 None
    - CCRSET ccrset None
    - UNION_ADDR addr_src1 向量输入1地址
    - UNION_ADDR addr_src2 向量输入2地址
    - UNION_ADDR addr_dest 向量输出地址
    - uint64_t len_src1 向量输入1长度
    - uint64_t len_src2 向量输入2长度
    - uint64_t len_dest
输出向量长度（超过了之后就会停止，和src1，src2的总长度成倍数，实现在左面广播）
    - uint64_t basement_src1 None
    - uint64_t basement_src2 None
    - uint64_t basement_dest None
**/
class inst_mfu_mn_broadcast_compute : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 配置MFU的DM模块，准备开始计算 **/
    CCRCLR ccrclr_src1_ = CCRCLR(0); /** None **/
    CCRCLR ccrclr_src2_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** None **/
    UNION_ADDR addr_src1_ = UNION_ADDR(0); /** 向量输入1地址 **/
    UNION_ADDR addr_src2_ = UNION_ADDR(0); /** 向量输入2地址 **/
    UNION_ADDR addr_dest_ = UNION_ADDR(0); /** 向量输出地址 **/
    uint64_t len_src1_ = uint64_t(0); /** 向量输入1长度 **/
    uint64_t len_src2_ = uint64_t(0); /** 向量输入2长度 **/
    uint64_t len_dest_ = uint64_t(0); /**
                                            输出向量长度（超过了之后就会停止，和src1，src2的总长度成倍数，实现在左面广播）
                                            **/
    uint64_t basement_src1_ = uint64_t(0); /** None **/
    uint64_t basement_src2_ = uint64_t(0); /** None **/
    uint64_t basement_dest_ = uint64_t(0); /** None **/
public:
    inst_mfu_mn_broadcast_compute(
        CCRCLR ccrclr_src1 = CCRCLR(0), CCRCLR ccrclr_src2 = CCRCLR(0),
        CCRSET ccrset = CCRSET(0), UNION_ADDR addr_src1 = UNION_ADDR(0),
        UNION_ADDR addr_src2 = UNION_ADDR(0),
        UNION_ADDR addr_dest = UNION_ADDR(0), uint64_t len_src1 = uint64_t(0),
        uint64_t len_src2 = uint64_t(0), uint64_t len_dest = uint64_t(0),
        uint64_t basement_src1 = uint64_t(0),
        uint64_t basement_src2 = uint64_t(0),
        uint64_t basement_dest = uint64_t(0));

    explicit inst_mfu_mn_broadcast_compute(
        const INST_MFU_MN_BROADCAST_COMPUTE &ref);

    [[nodiscard]] struct INST_MFU_MN_BROADCAST_COMPUTE to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 配置MFU的DM模块，准备开始计算
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 配置MFU的DM模块，准备开始计算
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr_src1 None
  */
    [[nodiscard]] CCRCLR ccrclr_src1() const;
    /**
      CCRCLR ccrclr_src1 None
  */
    void ccrclr_src1(const CCRCLR &value);
    /**
      CCRCLR ccrclr_src2 None
  */
    [[nodiscard]] CCRCLR ccrclr_src2() const;
    /**
      CCRCLR ccrclr_src2 None
  */
    void ccrclr_src2(const CCRCLR &value);
    /**
      CCRSET ccrset None
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset None
  */
    void ccrset(const CCRSET &value);
    /**
      UNION_ADDR addr_src1 向量输入1地址
  */
    [[nodiscard]] UNION_ADDR addr_src1() const;
    /**
      UNION_ADDR addr_src1 向量输入1地址
  */
    void addr_src1(const UNION_ADDR &value);
    /**
      UNION_ADDR addr_src2 向量输入2地址
  */
    [[nodiscard]] UNION_ADDR addr_src2() const;
    /**
      UNION_ADDR addr_src2 向量输入2地址
  */
    void addr_src2(const UNION_ADDR &value);
    /**
      UNION_ADDR addr_dest 向量输出地址
  */
    [[nodiscard]] UNION_ADDR addr_dest() const;
    /**
      UNION_ADDR addr_dest 向量输出地址
  */
    void addr_dest(const UNION_ADDR &value);
    /**
      uint64_t len_src1 向量输入1长度
  */
    [[nodiscard]] uint64_t len_src1() const;
    /**
      uint64_t len_src1 向量输入1长度
  */
    void len_src1(const uint64_t &value);
    /**
      uint64_t len_src2 向量输入2长度
  */
    [[nodiscard]] uint64_t len_src2() const;
    /**
      uint64_t len_src2 向量输入2长度
  */
    void len_src2(const uint64_t &value);
    /**
      uint64_t len_dest
     输出向量长度（超过了之后就会停止，和src1，src2的总长度成倍数，实现在左面广播）
  */
    [[nodiscard]] uint64_t len_dest() const;
    /**
      uint64_t len_dest
     输出向量长度（超过了之后就会停止，和src1，src2的总长度成倍数，实现在左面广播）
  */
    void len_dest(const uint64_t &value);
    /**
      uint64_t basement_src1 None
  */
    [[nodiscard]] uint64_t basement_src1() const;
    /**
      uint64_t basement_src1 None
  */
    void basement_src1(const uint64_t &value);
    /**
      uint64_t basement_src2 None
  */
    [[nodiscard]] uint64_t basement_src2() const;
    /**
      uint64_t basement_src2 None
  */
    void basement_src2(const uint64_t &value);
    /**
      uint64_t basement_dest None
  */
    [[nodiscard]] uint64_t basement_dest() const;
    /**
      uint64_t basement_dest None
  */
    void basement_dest(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out,
    const inst_mfu_mn_broadcast_compute &c);

/**
    MFU.MN.REDUCE
    @brief 这条指令用于配置Fuse在Meshnet后面的Reduce
    - OPCODE opcode 这条指令用于配置Fuse在Meshnet后面的Reduce
    - uint64_t init_value need_init 为 True 时有效，此时上面的 a1 为 init_value
    - uint64_t recude_length 有多少个元素参与一次reduce
    - uint64_t length reduce元素的次数
    - MFU_REDUCE_OP op 参与的运算 Scalar Binary Operator
**/
class inst_mfu_mn_reduce : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 这条指令用于配置Fuse在Meshnet后面的Reduce **/
    uint64_t init_value_ = uint64_t(0); /** need_init 为 True 时有效，此时上面的 a1 为 init_value **/
    uint64_t recude_length_ = uint64_t(0); /** 有多少个元素参与一次reduce **/
    uint64_t length_ = uint64_t(0); /** reduce元素的次数 **/
    MFU_REDUCE_OP op_ = MFU_REDUCE_OP(0); /** 参与的运算 Scalar Binary Operator **/
public:
    inst_mfu_mn_reduce(uint64_t init_value = uint64_t(0),
        uint64_t recude_length = uint64_t(0),
        uint64_t length = uint64_t(0),
        MFU_REDUCE_OP op = MFU_REDUCE_OP(0));

    explicit inst_mfu_mn_reduce(const INST_MFU_MN_REDUCE &ref);

    [[nodiscard]] struct INST_MFU_MN_REDUCE to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 这条指令用于配置Fuse在Meshnet后面的Reduce
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 这条指令用于配置Fuse在Meshnet后面的Reduce
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t init_value need_init 为 True 时有效，此时上面的 a1 为 init_value
  */
    [[nodiscard]] uint64_t init_value() const;
    /**
      uint64_t init_value need_init 为 True 时有效，此时上面的 a1 为 init_value
  */
    void init_value(const uint64_t &value);
    /**
      uint64_t recude_length 有多少个元素参与一次reduce
  */
    [[nodiscard]] uint64_t recude_length() const;
    /**
      uint64_t recude_length 有多少个元素参与一次reduce
  */
    void recude_length(const uint64_t &value);
    /**
      uint64_t length reduce元素的次数
  */
    [[nodiscard]] uint64_t length() const;
    /**
      uint64_t length reduce元素的次数
  */
    void length(const uint64_t &value);
    /**
      MFU_REDUCE_OP op 参与的运算 Scalar Binary Operator
  */
    [[nodiscard]] MFU_REDUCE_OP op() const;
    /**
      MFU_REDUCE_OP op 参与的运算 Scalar Binary Operator
  */
    void op(const MFU_REDUCE_OP &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_mn_reduce &c);

class inst_mfu_mn_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /**
                                               配置互联网络。我们认为某个原件编号为0就是与网络断开连接并且gate掉。整个网络连接初始值都是0
                                               **/
    MFU_MN_PORTOUT out1_ = MFU_MN_PORTOUT(0); /** VECTOR_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out2_ = MFU_MN_PORTOUT(0); /** VECTOR_OUT_1连接的端口 **/
    MFU_MN_PORTOUT out3_ = MFU_MN_PORTOUT(0); /** CONST1_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out4_ = MFU_MN_PORTOUT(0); /** CONST1_OUT_1连接的端口 **/
    MFU_MN_PORTOUT out5_ = MFU_MN_PORTOUT(0); /** CONST2_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out6_ = MFU_MN_PORTOUT(0); /** CONST2_OUT_1连接的端口 **/
    MFU_MN_PORTOUT out7_ = MFU_MN_PORTOUT(0); /** ADDSUB0_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out8_ = MFU_MN_PORTOUT(0); /** ADDSUB1_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out9_ = MFU_MN_PORTOUT(0); /** ADDSUB2_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out10_ = MFU_MN_PORTOUT(0); /** ADDSUB3_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out11_ = MFU_MN_PORTOUT(0); /** MUL0_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out12_ = MFU_MN_PORTOUT(0); /** MUL1_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out13_ = MFU_MN_PORTOUT(0); /** MUL2_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out14_ = MFU_MN_PORTOUT(0); /** MUL3_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out15_ = MFU_MN_PORTOUT(0); /** DIV_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out16_ = MFU_MN_PORTOUT(0); /** ROUND_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out17_ = MFU_MN_PORTOUT(0); /** SQRT_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out18_ = MFU_MN_PORTOUT(0); /** TRANGLE_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out19_ = MFU_MN_PORTOUT(0); /** LOG_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out20_ = MFU_MN_PORTOUT(0); /** UNARY_LOGIC_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out21_ = MFU_MN_PORTOUT(0); /** EXP_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out22_ = MFU_MN_PORTOUT(0); /** CMP0_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out23_ = MFU_MN_PORTOUT(0); /** CMP1_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out24_ = MFU_MN_PORTOUT(0); /** REG0_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out25_ = MFU_MN_PORTOUT(0); /** SELECT0_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out26_ = MFU_MN_PORTOUT(0); /** NOP0_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out27_ = MFU_MN_PORTOUT(0); /** NOP1_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out28_ = MFU_MN_PORTOUT(0); /** NOP2_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out29_ = MFU_MN_PORTOUT(0); /** NOP3_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out30_ = MFU_MN_PORTOUT(0); /** NOP4_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out31_ = MFU_MN_PORTOUT(0); /** NOP5_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out32_ = MFU_MN_PORTOUT(0); /** NOP6_OUT_0连接的端口 **/
    MFU_MN_PORTOUT out33_ = MFU_MN_PORTOUT(0); /** NOP7_OUT_0连接的端口 **/
public:
    inst_mfu_mn_conf(MFU_MN_PORTOUT out1 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out2 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out3 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out4 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out5 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out6 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out7 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out8 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out9 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out10 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out11 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out12 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out13 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out14 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out15 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out16 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out17 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out18 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out19 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out20 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out21 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out22 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out23 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out24 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out25 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out26 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out27 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out28 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out29 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out30 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out31 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out32 = MFU_MN_PORTOUT(0),
        MFU_MN_PORTOUT out33 = MFU_MN_PORTOUT(0));

    explicit inst_mfu_mn_conf(const INST_MFU_MN_CONF &ref);

    [[nodiscard]] struct INST_MFU_MN_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode
     配置互联网络。我们认为某个原件编号为0就是与网络断开连接并且gate掉。整个网络连接初始值都是0
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode
     配置互联网络。我们认为某个原件编号为0就是与网络断开连接并且gate掉。整个网络连接初始值都是0
  */
    void opcode(const OPCODE &value);
    /**
      MFU_MN_PORTOUT out1 VECTOR_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out1() const;
    /**
      MFU_MN_PORTOUT out1 VECTOR_OUT_0连接的端口
  */
    void out1(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out2 VECTOR_OUT_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out2() const;
    /**
      MFU_MN_PORTOUT out2 VECTOR_OUT_1连接的端口
  */
    void out2(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out3 CONST1_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out3() const;
    /**
      MFU_MN_PORTOUT out3 CONST1_OUT_0连接的端口
  */
    void out3(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out4 CONST1_OUT_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out4() const;
    /**
      MFU_MN_PORTOUT out4 CONST1_OUT_1连接的端口
  */
    void out4(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out5 CONST2_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out5() const;
    /**
      MFU_MN_PORTOUT out5 CONST2_OUT_0连接的端口
  */
    void out5(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out6 CONST2_OUT_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out6() const;
    /**
      MFU_MN_PORTOUT out6 CONST2_OUT_1连接的端口
  */
    void out6(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out7 ADDSUB0_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out7() const;
    /**
      MFU_MN_PORTOUT out7 ADDSUB0_OUT_0连接的端口
  */
    void out7(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out8 ADDSUB1_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out8() const;
    /**
      MFU_MN_PORTOUT out8 ADDSUB1_OUT_0连接的端口
  */
    void out8(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out9 ADDSUB2_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out9() const;
    /**
      MFU_MN_PORTOUT out9 ADDSUB2_OUT_0连接的端口
  */
    void out9(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out10 ADDSUB3_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out10() const;
    /**
      MFU_MN_PORTOUT out10 ADDSUB3_OUT_0连接的端口
  */
    void out10(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out11 MUL0_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out11() const;
    /**
      MFU_MN_PORTOUT out11 MUL0_OUT_0连接的端口
  */
    void out11(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out12 MUL1_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out12() const;
    /**
      MFU_MN_PORTOUT out12 MUL1_OUT_0连接的端口
  */
    void out12(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out13 MUL2_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out13() const;
    /**
      MFU_MN_PORTOUT out13 MUL2_OUT_0连接的端口
  */
    void out13(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out14 MUL3_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out14() const;
    /**
      MFU_MN_PORTOUT out14 MUL3_OUT_0连接的端口
  */
    void out14(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out15 DIV_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out15() const;
    /**
      MFU_MN_PORTOUT out15 DIV_OUT_0连接的端口
  */
    void out15(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out16 ROUND_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out16() const;
    /**
      MFU_MN_PORTOUT out16 ROUND_OUT_0连接的端口
  */
    void out16(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out17 SQRT_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out17() const;
    /**
      MFU_MN_PORTOUT out17 SQRT_OUT_0连接的端口
  */
    void out17(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out18 TRANGLE_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out18() const;
    /**
      MFU_MN_PORTOUT out18 TRANGLE_OUT_0连接的端口
  */
    void out18(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out19 LOG_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out19() const;
    /**
      MFU_MN_PORTOUT out19 LOG_OUT_0连接的端口
  */
    void out19(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out20 UNARY_LOGIC_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out20() const;
    /**
      MFU_MN_PORTOUT out20 UNARY_LOGIC_OUT_0连接的端口
  */
    void out20(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out21 EXP_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out21() const;
    /**
      MFU_MN_PORTOUT out21 EXP_OUT_0连接的端口
  */
    void out21(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out22 CMP0_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out22() const;
    /**
      MFU_MN_PORTOUT out22 CMP0_OUT_0连接的端口
  */
    void out22(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out23 CMP1_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out23() const;
    /**
      MFU_MN_PORTOUT out23 CMP1_OUT_0连接的端口
  */
    void out23(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out24 REG0_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out24() const;
    /**
      MFU_MN_PORTOUT out24 REG0_OUT_0连接的端口
  */
    void out24(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out25 SELECT0_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out25() const;
    /**
      MFU_MN_PORTOUT out25 SELECT0_OUT_0连接的端口
  */
    void out25(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out26 NOP0_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out26() const;
    /**
      MFU_MN_PORTOUT out26 NOP0_OUT_0连接的端口
  */
    void out26(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out27 NOP1_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out27() const;
    /**
      MFU_MN_PORTOUT out27 NOP1_OUT_0连接的端口
  */
    void out27(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out28 NOP2_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out28() const;
    /**
      MFU_MN_PORTOUT out28 NOP2_OUT_0连接的端口
  */
    void out28(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out29 NOP3_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out29() const;
    /**
      MFU_MN_PORTOUT out29 NOP3_OUT_0连接的端口
  */
    void out29(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out30 NOP4_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out30() const;
    /**
      MFU_MN_PORTOUT out26 NOP4_OUT_0连接的端口
  */
    void out30(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out31 NOP5_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out31() const;
    /**
      MFU_MN_PORTOUT out31 NOP5_OUT_0连接的端口
  */
    void out31(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out32 NOP6_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out32() const;
    /**
      MFU_MN_PORTOUT out32 NOP6_OUT_0连接的端口
  */
    void out32(const MFU_MN_PORTOUT &value);
    /**
      MFU_MN_PORTOUT out33 NOP7_OUT_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTOUT out33() const;
    /**
      MFU_MN_PORTOUT out33 NOP7_OUT_0连接的端口
  */
    void out33(const MFU_MN_PORTOUT &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_mn_conf &c);

/**
    MFU.MNOP.CONF
    @brief
这条指令用来配置原件的内部寄存器，比如group的操作的选通，或者是拟合运算的拟合表的配置
    - OPCODE opcode
这条指令用来配置原件的内部寄存器，比如group的操作的选通，或者是拟合运算的拟合表的配置
    - uint64_t mode 模式。
0：配置的是各种原件的标志bit
1：配置拟合模块0的参数地址
2：配置拟合模块1的参数地址
    - uint64_t val 配置
**/
class inst_mfu_mnop_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /**
                                   这条指令用来配置原件的内部寄存器，比如group的操作的选通，或者是拟合运算的拟合表的配置
                                   **/
    uint64_t mode_ = uint64_t(0); /** 模式。
0：配置的是各种原件的标志bit
1：配置拟合模块0的参数地址
2：配置拟合模块1的参数地址 **/
    uint64_t val_ = uint64_t(0); /** 配置 **/
public:
    inst_mfu_mnop_conf(uint64_t mode = uint64_t(0), uint64_t val = uint64_t(0));

    explicit inst_mfu_mnop_conf(const INST_MFU_MNOP_CONF &ref);

    [[nodiscard]] struct INST_MFU_MNOP_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode
     这条指令用来配置原件的内部寄存器，比如group的操作的选通，或者是拟合运算的拟合表的配置
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode
     这条指令用来配置原件的内部寄存器，比如group的操作的选通，或者是拟合运算的拟合表的配置
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t mode 模式。
0：配置的是各种原件的标志bit
1：配置拟合模块0的参数地址
2：配置拟合模块1的参数地址
  */
    [[nodiscard]] uint64_t mode() const;
    /**
      uint64_t mode 模式。
0：配置的是各种原件的标志bit
1：配置拟合模块0的参数地址
2：配置拟合模块1的参数地址
  */
    void mode(const uint64_t &value);
    /**
      uint64_t val 配置
  */
    [[nodiscard]] uint64_t val() const;
    /**
      uint64_t val 配置
  */
    void val(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_mnop_conf &c);

/**
    MFU.PDP.CONF
    @brief 主要用于配置 PDP 模块计算中共用的一些全局配置
    - OPCODE opcode 主要用于配置 PDP 模块计算中共用的一些全局配置
    - uint64_t multiple_channels 是否在多个 channel
上同时进行一样窗口和窗口移动规则。
    - STRIDE_GLB stride_dest_glb None
**/
class inst_mfu_pdp_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 主要用于配置 PDP 模块计算中共用的一些全局配置 **/
    uint64_t multiple_channels_ = uint64_t(0); /** 是否在多个 channel 上同时进行一样窗口和窗口移动规则。 **/
    STRIDE_GLB stride_dest_glb_ = STRIDE_GLB(0); /** None **/
public:
    inst_mfu_pdp_conf(uint64_t multiple_channels = uint64_t(0),
        STRIDE_GLB stride_dest_glb = STRIDE_GLB(0));

    explicit inst_mfu_pdp_conf(const INST_MFU_PDP_CONF &ref);

    [[nodiscard]] struct INST_MFU_PDP_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 主要用于配置 PDP 模块计算中共用的一些全局配置
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 主要用于配置 PDP 模块计算中共用的一些全局配置
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t multiple_channels 是否在多个 channel
     上同时进行一样窗口和窗口移动规则。
  */
    [[nodiscard]] uint64_t multiple_channels() const;
    /**
      uint64_t multiple_channels 是否在多个 channel
     上同时进行一样窗口和窗口移动规则。
  */
    void multiple_channels(const uint64_t &value);
    /**
      STRIDE_GLB stride_dest_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_dest_glb() const;
    /**
      STRIDE_GLB stride_dest_glb None
  */
    void stride_dest_glb(const STRIDE_GLB &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_pdp_conf &c);

/**
    MFU.PDP.SRC.CONF
    @brief  主要用来配置待操作的 Tensor 的位置
    - OPCODE opcode  主要用来配置待操作的 Tensor 的位置
    - STRIDE_GLB stride_glb Tensor 的 shape
**/
class inst_mfu_pdp_src_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /**  主要用来配置待操作的 Tensor 的位置 **/
    STRIDE_GLB stride_glb_ = STRIDE_GLB(0); /** Tensor 的 shape **/
public:
    inst_mfu_pdp_src_conf(STRIDE_GLB stride_glb = STRIDE_GLB(0));

    explicit inst_mfu_pdp_src_conf(const INST_MFU_PDP_SRC_CONF &ref);

    [[nodiscard]] struct INST_MFU_PDP_SRC_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode  主要用来配置待操作的 Tensor 的位置
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode  主要用来配置待操作的 Tensor 的位置
  */
    void opcode(const OPCODE &value);
    /**
      STRIDE_GLB stride_glb Tensor 的 shape
  */
    [[nodiscard]] STRIDE_GLB stride_glb() const;
    /**
      STRIDE_GLB stride_glb Tensor 的 shape
  */
    void stride_glb(const STRIDE_GLB &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_pdp_src_conf &c);

/**
    MFU.PDP.REDUCE
    @brief 进行PDP运算
    - OPCODE opcode 进行PDP运算
    - CCRCLR ccrclr None
    - CCRSET ccrset None
    - ADDR_GLB_8_WITH_BANK addr_src 输入向量地址 (GLB or DDR)（slice起始地址）
    - ADDR_GLB_8_WITH_BANK addr_dest 输出位置
    - uint64_t window_w 2D 窗口的宽
    - uint64_t window_h 2D 窗口的高
    - uint64_t active_h PDP阵列硬件使能的高度（需要是window_h的整数倍）
    - uint64_t shape_n None
    - uint64_t shape_c None
    - uint64_t shape_h None
    - uint64_t shape_w None
    - uint64_t count_w W 维度上做 N 次 ReduceWindow 操作
    - uint64_t count_h H 维度上做 N 次 ReduceWindow操作
    - uint64_t stride_w W 维度上每次取窗口的下标增加量
    - uint64_t stride_h H 维度上每次取窗口的下标增加量
    - uint64_t padding_top 在 2D Tensor 的顶部加的 Zero padding 行数
    - uint64_t padding_bottom 在 2D Tensor 的底部加的 Zero padding 行数
    - uint64_t padding_left 在 2D Tensor 的左边加的 Zero padding 行数
    - uint64_t padding_right 在 2D Tensor 的右边加的 Zero padding 行数
    - uint64_t pe_last_h 来表明最后一次计算需要使能多少个pdp单元
    - MFU_PDP_OP computation 我们只支持 Map/Reduce 二元算子中的 Max/Min/Avg 三种
    - uint64_t quantized 是否是量化计算
**/
class inst_mfu_pdp_reduce : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 进行PDP运算 **/
    CCRCLR ccrclr_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_src_ = ADDR_GLB_8_WITH_BANK(
        0); /** 输入向量地址 (GLB or DDR)（slice起始地址） **/
    ADDR_GLB_8_WITH_BANK addr_dest_ = ADDR_GLB_8_WITH_BANK(0); /** 输出位置 **/
    uint64_t window_w_ = uint64_t(0); /** 2D 窗口的宽 **/
    uint64_t window_h_ = uint64_t(0); /** 2D 窗口的高 **/
    uint64_t active_h_ = uint64_t(0); /** PDP阵列硬件使能的高度（需要是window_h的整数倍） **/
    uint64_t shape_n_ = uint64_t(0); /** None **/
    uint64_t shape_c_ = uint64_t(0); /** None **/
    uint64_t shape_h_ = uint64_t(0); /** None **/
    uint64_t shape_w_ = uint64_t(0); /** None **/
    uint64_t count_w_ = uint64_t(0); /** W 维度上做 N 次 ReduceWindow 操作 **/
    uint64_t count_h_ = uint64_t(0); /** H 维度上做 N 次 ReduceWindow操作 **/
    uint64_t stride_w_ = uint64_t(0); /** W 维度上每次取窗口的下标增加量 **/
    uint64_t stride_h_ = uint64_t(0); /** H 维度上每次取窗口的下标增加量 **/
    uint64_t padding_top_ = uint64_t(0); /** 在 2D Tensor 的顶部加的 Zero padding 行数 **/
    uint64_t padding_bottom_ = uint64_t(0); /** 在 2D Tensor 的底部加的 Zero padding 行数 **/
    uint64_t padding_left_ = uint64_t(0); /** 在 2D Tensor 的左边加的 Zero padding 行数 **/
    uint64_t padding_right_ = uint64_t(0); /** 在 2D Tensor 的右边加的 Zero padding 行数 **/
    uint64_t pe_last_h_ = uint64_t(0); /** 来表明最后一次计算需要使能多少个pdp单元 **/
    MFU_PDP_OP computation_ = MFU_PDP_OP(0); /** 我们只支持 Map/Reduce 二元算子中的 Max/Min/Avg 三种 **/
    uint64_t quantized_ = uint64_t(0); /** 是否是量化计算 **/
public:
    inst_mfu_pdp_reduce(
        CCRCLR ccrclr = CCRCLR(0), CCRSET ccrset = CCRSET(0),
        ADDR_GLB_8_WITH_BANK addr_src = ADDR_GLB_8_WITH_BANK(0),
        ADDR_GLB_8_WITH_BANK addr_dest = ADDR_GLB_8_WITH_BANK(0),
        uint64_t window_w = uint64_t(0), uint64_t window_h = uint64_t(0),
        uint64_t active_h = uint64_t(0), uint64_t shape_n = uint64_t(0),
        uint64_t shape_c = uint64_t(0), uint64_t shape_h = uint64_t(0),
        uint64_t shape_w = uint64_t(0), uint64_t count_w = uint64_t(0),
        uint64_t count_h = uint64_t(0), uint64_t stride_w = uint64_t(0),
        uint64_t stride_h = uint64_t(0), uint64_t padding_top = uint64_t(0),
        uint64_t padding_bottom = uint64_t(0),
        uint64_t padding_left = uint64_t(0), uint64_t padding_right = uint64_t(0),
        uint64_t pe_last_h = uint64_t(0), MFU_PDP_OP computation = MFU_PDP_OP(0),
        uint64_t quantized = uint64_t(0));

    explicit inst_mfu_pdp_reduce(const INST_MFU_PDP_REDUCE &ref);

    [[nodiscard]] struct INST_MFU_PDP_REDUCE to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 进行PDP运算
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 进行PDP运算
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr None
  */
    [[nodiscard]] CCRCLR ccrclr() const;
    /**
      CCRCLR ccrclr None
  */
    void ccrclr(const CCRCLR &value);
    /**
      CCRSET ccrset None
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset None
  */
    void ccrset(const CCRSET &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_src 输入向量地址 (GLB or DDR)（slice起始地址）
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_src() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_src 输入向量地址 (GLB or DDR)（slice起始地址）
  */
    void addr_src(const ADDR_GLB_8_WITH_BANK &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_dest 输出位置
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_dest() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_dest 输出位置
  */
    void addr_dest(const ADDR_GLB_8_WITH_BANK &value);
    /**
      uint64_t window_w 2D 窗口的宽
  */
    [[nodiscard]] uint64_t window_w() const;
    /**
      uint64_t window_w 2D 窗口的宽
  */
    void window_w(const uint64_t &value);
    /**
      uint64_t window_h 2D 窗口的高
  */
    [[nodiscard]] uint64_t window_h() const;
    /**
      uint64_t window_h 2D 窗口的高
  */
    void window_h(const uint64_t &value);
    /**
      uint64_t active_h PDP阵列硬件使能的高度（需要是window_h的整数倍）
  */
    [[nodiscard]] uint64_t active_h() const;
    /**
      uint64_t active_h PDP阵列硬件使能的高度（需要是window_h的整数倍）
  */
    void active_h(const uint64_t &value);
    /**
      uint64_t shape_n None
  */
    [[nodiscard]] uint64_t shape_n() const;
    /**
      uint64_t shape_n None
  */
    void shape_n(const uint64_t &value);
    /**
      uint64_t shape_c None
  */
    [[nodiscard]] uint64_t shape_c() const;
    /**
      uint64_t shape_c None
  */
    void shape_c(const uint64_t &value);
    /**
      uint64_t shape_h None
  */
    [[nodiscard]] uint64_t shape_h() const;
    /**
      uint64_t shape_h None
  */
    void shape_h(const uint64_t &value);
    /**
      uint64_t shape_w None
  */
    [[nodiscard]] uint64_t shape_w() const;
    /**
      uint64_t shape_w None
  */
    void shape_w(const uint64_t &value);
    /**
      uint64_t count_w W 维度上做 N 次 ReduceWindow 操作
  */
    [[nodiscard]] uint64_t count_w() const;
    /**
      uint64_t count_w W 维度上做 N 次 ReduceWindow 操作
  */
    void count_w(const uint64_t &value);
    /**
      uint64_t count_h H 维度上做 N 次 ReduceWindow操作
  */
    [[nodiscard]] uint64_t count_h() const;
    /**
      uint64_t count_h H 维度上做 N 次 ReduceWindow操作
  */
    void count_h(const uint64_t &value);
    /**
      uint64_t stride_w W 维度上每次取窗口的下标增加量
  */
    [[nodiscard]] uint64_t stride_w() const;
    /**
      uint64_t stride_w W 维度上每次取窗口的下标增加量
  */
    void stride_w(const uint64_t &value);
    /**
      uint64_t stride_h H 维度上每次取窗口的下标增加量
  */
    [[nodiscard]] uint64_t stride_h() const;
    /**
      uint64_t stride_h H 维度上每次取窗口的下标增加量
  */
    void stride_h(const uint64_t &value);
    /**
      uint64_t padding_top 在 2D Tensor 的顶部加的 Zero padding 行数
  */
    [[nodiscard]] uint64_t padding_top() const;
    /**
      uint64_t padding_top 在 2D Tensor 的顶部加的 Zero padding 行数
  */
    void padding_top(const uint64_t &value);
    /**
      uint64_t padding_bottom 在 2D Tensor 的底部加的 Zero padding 行数
  */
    [[nodiscard]] uint64_t padding_bottom() const;
    /**
      uint64_t padding_bottom 在 2D Tensor 的底部加的 Zero padding 行数
  */
    void padding_bottom(const uint64_t &value);
    /**
      uint64_t padding_left 在 2D Tensor 的左边加的 Zero padding 行数
  */
    [[nodiscard]] uint64_t padding_left() const;
    /**
      uint64_t padding_left 在 2D Tensor 的左边加的 Zero padding 行数
  */
    void padding_left(const uint64_t &value);
    /**
      uint64_t padding_right 在 2D Tensor 的右边加的 Zero padding 行数
  */
    [[nodiscard]] uint64_t padding_right() const;
    /**
      uint64_t padding_right 在 2D Tensor 的右边加的 Zero padding 行数
  */
    void padding_right(const uint64_t &value);
    /**
      uint64_t pe_last_h 来表明最后一次计算需要使能多少个pdp单元
  */
    [[nodiscard]] uint64_t pe_last_h() const;
    /**
      uint64_t pe_last_h 来表明最后一次计算需要使能多少个pdp单元
  */
    void pe_last_h(const uint64_t &value);
    /**
      MFU_PDP_OP computation 我们只支持 Map/Reduce 二元算子中的 Max/Min/Avg 三种
  */
    [[nodiscard]] MFU_PDP_OP computation() const;
    /**
      MFU_PDP_OP computation 我们只支持 Map/Reduce 二元算子中的 Max/Min/Avg 三种
  */
    void computation(const MFU_PDP_OP &value);
    /**
      uint64_t quantized 是否是量化计算
  */
    [[nodiscard]] uint64_t quantized() const;
    /**
      uint64_t quantized 是否是量化计算
  */
    void quantized(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_pdp_reduce &c);

/**
    MFU.MN.BROADCAST.CONF
    @brief None
    - OPCODE opcode None
    - uint64_t slice_src1 向量输入1 slice长度
    - uint64_t slice_src2 向量输入2 slice长度
    - uint64_t repeats_src1 向量输入1每个元素重复的次数（在右面广播）
    - uint64_t repeats_src2 向量输入2每个元素重复的次数（在右面广播）
    - uint64_t slice_repeats_src1 向量输入1 slice次数（实现在中间广播）
    - uint64_t slice_repeats_src2 向量输入2 slice次数（实现在中间广播）
    - STRIDE_GLB stride_src1_glb None
    - uint64_t shape_src1_n None
    - uint64_t shape_src1_c None
    - uint64_t shape_src1_h None
    - uint64_t shape_src1_w None
    - STRIDE_GLB stride_src2_glb None
    - uint64_t shape_src2_n None
    - uint64_t shape_src2_c None
    - uint64_t shape_src2_h None
    - uint64_t shape_src2_w None
    - uint64_t const1 常量1
    - uint64_t const2 常量2
    - uint64_t const3 常量3
    - uint64_t const4 常量4
**/
class inst_mfu_mn_broadcast_conf : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** None **/
    uint64_t slice_src1_ = uint64_t(0); /** 向量输入1 slice长度 **/
    uint64_t slice_src2_ = uint64_t(0); /** 向量输入2 slice长度 **/
    uint64_t repeats_src1_ = uint64_t(0); /** 向量输入1每个元素重复的次数（在右面广播） **/
    uint64_t repeats_src2_ = uint64_t(0); /** 向量输入2每个元素重复的次数（在右面广播） **/
    uint64_t slice_repeats_src1_ = uint64_t(0); /** 向量输入1 slice次数（实现在中间广播） **/
    uint64_t slice_repeats_src2_ = uint64_t(0); /** 向量输入2 slice次数（实现在中间广播） **/
    STRIDE_GLB stride_src1_glb_ = STRIDE_GLB(0); /** None **/
    uint64_t shape_src1_n_ = uint64_t(0); /** None **/
    uint64_t shape_src1_c_ = uint64_t(0); /** None **/
    uint64_t shape_src1_h_ = uint64_t(0); /** None **/
    uint64_t shape_src1_w_ = uint64_t(0); /** None **/
    STRIDE_GLB stride_src2_glb_ = STRIDE_GLB(0); /** None **/
    uint64_t shape_src2_n_ = uint64_t(0); /** None **/
    uint64_t shape_src2_c_ = uint64_t(0); /** None **/
    uint64_t shape_src2_h_ = uint64_t(0); /** None **/
    uint64_t shape_src2_w_ = uint64_t(0); /** None **/
    uint64_t const1_ = uint64_t(0); /** 常量1 **/
    uint64_t const2_ = uint64_t(0); /** 常量2 **/
    uint64_t const3_ = uint64_t(0); /** 常量3 **/
    uint64_t const4_ = uint64_t(0); /** 常量4 **/
public:
    inst_mfu_mn_broadcast_conf(
        uint64_t slice_src1 = uint64_t(0), uint64_t slice_src2 = uint64_t(0),
        uint64_t repeats_src1 = uint64_t(0), uint64_t repeats_src2 = uint64_t(0),
        uint64_t slice_repeats_src1 = uint64_t(0),
        uint64_t slice_repeats_src2 = uint64_t(0),
        STRIDE_GLB stride_src1_glb = STRIDE_GLB(0),
        uint64_t shape_src1_n = uint64_t(0), uint64_t shape_src1_c = uint64_t(0),
        uint64_t shape_src1_h = uint64_t(0), uint64_t shape_src1_w = uint64_t(0),
        STRIDE_GLB stride_src2_glb = STRIDE_GLB(0),
        uint64_t shape_src2_n = uint64_t(0), uint64_t shape_src2_c = uint64_t(0),
        uint64_t shape_src2_h = uint64_t(0), uint64_t shape_src2_w = uint64_t(0),
        uint64_t const1 = uint64_t(0), uint64_t const2 = uint64_t(0),
        uint64_t const3 = uint64_t(0), uint64_t const4 = uint64_t(0));

    explicit inst_mfu_mn_broadcast_conf(const INST_MFU_MN_BROADCAST_CONF &ref);

    [[nodiscard]] struct INST_MFU_MN_BROADCAST_CONF to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode None
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode None
  */
    void opcode(const OPCODE &value);
    /**
      uint64_t slice_src1 向量输入1 slice长度
  */
    [[nodiscard]] uint64_t slice_src1() const;
    /**
      uint64_t slice_src1 向量输入1 slice长度
  */
    void slice_src1(const uint64_t &value);
    /**
      uint64_t slice_src2 向量输入2 slice长度
  */
    [[nodiscard]] uint64_t slice_src2() const;
    /**
      uint64_t slice_src2 向量输入2 slice长度
  */
    void slice_src2(const uint64_t &value);
    /**
      uint64_t repeats_src1 向量输入1每个元素重复的次数（在右面广播）
  */
    [[nodiscard]] uint64_t repeats_src1() const;
    /**
      uint64_t repeats_src1 向量输入1每个元素重复的次数（在右面广播）
  */
    void repeats_src1(const uint64_t &value);
    /**
      uint64_t repeats_src2 向量输入2每个元素重复的次数（在右面广播）
  */
    [[nodiscard]] uint64_t repeats_src2() const;
    /**
      uint64_t repeats_src2 向量输入2每个元素重复的次数（在右面广播）
  */
    void repeats_src2(const uint64_t &value);
    /**
      uint64_t slice_repeats_src1 向量输入1 slice次数（实现在中间广播）
  */
    [[nodiscard]] uint64_t slice_repeats_src1() const;
    /**
      uint64_t slice_repeats_src1 向量输入1 slice次数（实现在中间广播）
  */
    void slice_repeats_src1(const uint64_t &value);
    /**
      uint64_t slice_repeats_src2 向量输入2 slice次数（实现在中间广播）
  */
    [[nodiscard]] uint64_t slice_repeats_src2() const;
    /**
      uint64_t slice_repeats_src2 向量输入2 slice次数（实现在中间广播）
  */
    void slice_repeats_src2(const uint64_t &value);
    /**
      STRIDE_GLB stride_src1_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_src1_glb() const;
    /**
      STRIDE_GLB stride_src1_glb None
  */
    void stride_src1_glb(const STRIDE_GLB &value);
    /**
      uint64_t shape_src1_n None
  */
    [[nodiscard]] uint64_t shape_src1_n() const;
    /**
      uint64_t shape_src1_n None
  */
    void shape_src1_n(const uint64_t &value);
    /**
      uint64_t shape_src1_c None
  */
    [[nodiscard]] uint64_t shape_src1_c() const;
    /**
      uint64_t shape_src1_c None
  */
    void shape_src1_c(const uint64_t &value);
    /**
      uint64_t shape_src1_h None
  */
    [[nodiscard]] uint64_t shape_src1_h() const;
    /**
      uint64_t shape_src1_h None
  */
    void shape_src1_h(const uint64_t &value);
    /**
      uint64_t shape_src1_w None
  */
    [[nodiscard]] uint64_t shape_src1_w() const;
    /**
      uint64_t shape_src1_w None
  */
    void shape_src1_w(const uint64_t &value);
    /**
      STRIDE_GLB stride_src2_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_src2_glb() const;
    /**
      STRIDE_GLB stride_src2_glb None
  */
    void stride_src2_glb(const STRIDE_GLB &value);
    /**
      uint64_t shape_src2_n None
  */
    [[nodiscard]] uint64_t shape_src2_n() const;
    /**
      uint64_t shape_src2_n None
  */
    void shape_src2_n(const uint64_t &value);
    /**
      uint64_t shape_src2_c None
  */
    [[nodiscard]] uint64_t shape_src2_c() const;
    /**
      uint64_t shape_src2_c None
  */
    void shape_src2_c(const uint64_t &value);
    /**
      uint64_t shape_src2_h None
  */
    [[nodiscard]] uint64_t shape_src2_h() const;
    /**
      uint64_t shape_src2_h None
  */
    void shape_src2_h(const uint64_t &value);
    /**
      uint64_t shape_src2_w None
  */
    [[nodiscard]] uint64_t shape_src2_w() const;
    /**
      uint64_t shape_src2_w None
  */
    void shape_src2_w(const uint64_t &value);
    /**
      uint64_t const1 常量1
  */
    [[nodiscard]] uint64_t const1() const;
    /**
      uint64_t const1 常量1
  */
    void const1(const uint64_t &value);
    /**
      uint64_t const2 常量2
  */
    [[nodiscard]] uint64_t const2() const;
    /**
      uint64_t const2 常量2
  */
    void const2(const uint64_t &value);
    /**
      uint64_t const3 常量3
  */
    [[nodiscard]] uint64_t const3() const;
    /**
      uint64_t const3 常量3
  */
    void const3(const uint64_t &value);
    /**
      uint64_t const4 常量4
  */
    [[nodiscard]] uint64_t const4() const;
    /**
      uint64_t const4 常量4
  */
    void const4(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out,
    const inst_mfu_mn_broadcast_conf &c);

/**
    MFU.CROP
    @brief None
    - OPCODE opcode None
    - CCRCLR ccrclr None
    - CCRSET ccrset None
    - ADDR_GLB_8_WITH_BANK addr_src None
    - ADDR_GLB_8_WITH_BANK addr_dest None
    - ADDR_GLB_8_WITH_BANK addr_bbox None
    - uint64_t reserve None
    - uint64_t shape_src_c None
    - uint64_t shape_src_h None
    - uint64_t shape_src_w None
    - STRIDE_GLB stride_src_glb None
    - STRIDE_GLB stride_dest_glb None
    - uint64_t roi_amount None
    - uint64_t dest_h None
    - uint64_t dest_w None
    - BF24 step_h None
    - BF24 step_w None
    - MFU_CROP_ALIGN align_method None
    - MFU_CROP_RESIZE resize_method None
**/
class inst_mfu_crop : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** None **/
    CCRCLR ccrclr_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_src_ = ADDR_GLB_8_WITH_BANK(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_dest_ = ADDR_GLB_8_WITH_BANK(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_bbox_ = ADDR_GLB_8_WITH_BANK(0); /** None **/
    uint64_t reserve_ = uint64_t(0); /** None **/
    uint64_t shape_src_c_ = uint64_t(0); /** None **/
    uint64_t shape_src_h_ = uint64_t(0); /** None **/
    uint64_t shape_src_w_ = uint64_t(0); /** None **/
    STRIDE_GLB stride_src_glb_ = STRIDE_GLB(0); /** None **/
    STRIDE_GLB stride_dest_glb_ = STRIDE_GLB(0); /** None **/
    uint64_t roi_amount_ = uint64_t(0); /** None **/
    uint64_t dest_h_ = uint64_t(0); /** None **/
    uint64_t dest_w_ = uint64_t(0); /** None **/
    BF24 step_h_ = BF24(0); /** None **/
    BF24 step_w_ = BF24(0); /** None **/
    MFU_CROP_ALIGN align_method_ = MFU_CROP_ALIGN(0); /** None **/
    MFU_CROP_RESIZE resize_method_ = MFU_CROP_RESIZE(0); /** None **/
public:
    inst_mfu_crop(CCRCLR ccrclr = CCRCLR(0), CCRSET ccrset = CCRSET(0),
        ADDR_GLB_8_WITH_BANK addr_src = ADDR_GLB_8_WITH_BANK(0),
        ADDR_GLB_8_WITH_BANK addr_dest = ADDR_GLB_8_WITH_BANK(0),
        ADDR_GLB_8_WITH_BANK addr_bbox = ADDR_GLB_8_WITH_BANK(0),
        uint64_t reserve = uint64_t(0),
        uint64_t shape_src_c = uint64_t(0),
        uint64_t shape_src_h = uint64_t(0),
        uint64_t shape_src_w = uint64_t(0),
        STRIDE_GLB stride_src_glb = STRIDE_GLB(0),
        STRIDE_GLB stride_dest_glb = STRIDE_GLB(0),
        uint64_t roi_amount = uint64_t(0),
        uint64_t dest_h = uint64_t(0), uint64_t dest_w = uint64_t(0),
        BF24 step_h = BF24(0), BF24 step_w = BF24(0),
        MFU_CROP_ALIGN align_method = MFU_CROP_ALIGN(0),
        MFU_CROP_RESIZE resize_method = MFU_CROP_RESIZE(0));

    explicit inst_mfu_crop(const INST_MFU_CROP &ref);

    [[nodiscard]] struct INST_MFU_CROP to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode None
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode None
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr None
  */
    [[nodiscard]] CCRCLR ccrclr() const;
    /**
      CCRCLR ccrclr None
  */
    void ccrclr(const CCRCLR &value);
    /**
      CCRSET ccrset None
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset None
  */
    void ccrset(const CCRSET &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_src None
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_src() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_src None
  */
    void addr_src(const ADDR_GLB_8_WITH_BANK &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_dest None
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_dest() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_dest None
  */
    void addr_dest(const ADDR_GLB_8_WITH_BANK &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_bbox None
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_bbox() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_bbox None
  */
    void addr_bbox(const ADDR_GLB_8_WITH_BANK &value);
    /**
      uint64_t reserve None
  */
    [[nodiscard]] uint64_t reserve() const;
    /**
      uint64_t reserve None
  */
    void reserve(const uint64_t &value);
    /**
      uint64_t shape_src_c None
  */
    [[nodiscard]] uint64_t shape_src_c() const;
    /**
      uint64_t shape_src_c None
  */
    void shape_src_c(const uint64_t &value);
    /**
      uint64_t shape_src_h None
  */
    [[nodiscard]] uint64_t shape_src_h() const;
    /**
      uint64_t shape_src_h None
  */
    void shape_src_h(const uint64_t &value);
    /**
      uint64_t shape_src_w None
  */
    [[nodiscard]] uint64_t shape_src_w() const;
    /**
      uint64_t shape_src_w None
  */
    void shape_src_w(const uint64_t &value);
    /**
      STRIDE_GLB stride_src_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_src_glb() const;
    /**
      STRIDE_GLB stride_src_glb None
  */
    void stride_src_glb(const STRIDE_GLB &value);
    /**
      STRIDE_GLB stride_dest_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_dest_glb() const;
    /**
      STRIDE_GLB stride_dest_glb None
  */
    void stride_dest_glb(const STRIDE_GLB &value);
    /**
      uint64_t roi_amount None
  */
    [[nodiscard]] uint64_t roi_amount() const;
    /**
      uint64_t roi_amount None
  */
    void roi_amount(const uint64_t &value);
    /**
      uint64_t dest_h None
  */
    [[nodiscard]] uint64_t dest_h() const;
    /**
      uint64_t dest_h None
  */
    void dest_h(const uint64_t &value);
    /**
      uint64_t dest_w None
  */
    [[nodiscard]] uint64_t dest_w() const;
    /**
      uint64_t dest_w None
  */
    void dest_w(const uint64_t &value);
    /**
      BF24 step_h None
  */
    [[nodiscard]] BF24 step_h() const;
    /**
      BF24 step_h None
  */
    void step_h(const BF24 &value);
    /**
      BF24 step_w None
  */
    [[nodiscard]] BF24 step_w() const;
    /**
      BF24 step_w None
  */
    void step_w(const BF24 &value);
    /**
      MFU_CROP_ALIGN align_method None
  */
    [[nodiscard]] MFU_CROP_ALIGN align_method() const;
    /**
      MFU_CROP_ALIGN align_method None
  */
    void align_method(const MFU_CROP_ALIGN &value);
    /**
      MFU_CROP_RESIZE resize_method None
  */
    [[nodiscard]] MFU_CROP_RESIZE resize_method() const;
    /**
      MFU_CROP_RESIZE resize_method None
  */
    void resize_method(const MFU_CROP_RESIZE &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_crop &c);

/**
    MFU.MEMSET
    @brief None
    - OPCODE opcode None
    - CCRSET ccrset None
    - ADDR_GLB_8_WITH_BANK addr_dest None
    - uint64_t imm None
    - uint64_t len None
**/
class inst_mfu_memset : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_dest_ = ADDR_GLB_8_WITH_BANK(0); /** None **/
    uint64_t imm_ = uint64_t(0); /** None **/
    uint64_t len_ = uint64_t(0); /** None **/
public:
    inst_mfu_memset(CCRSET ccrset = CCRSET(0),
        ADDR_GLB_8_WITH_BANK addr_dest = ADDR_GLB_8_WITH_BANK(0),
        uint64_t imm = uint64_t(0), uint64_t len = uint64_t(0));

    explicit inst_mfu_memset(const INST_MFU_MEMSET &ref);

    [[nodiscard]] struct INST_MFU_MEMSET to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode None
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode None
  */
    void opcode(const OPCODE &value);
    /**
      CCRSET ccrset None
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset None
  */
    void ccrset(const CCRSET &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_dest None
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_dest() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_dest None
  */
    void addr_dest(const ADDR_GLB_8_WITH_BANK &value);
    /**
      uint64_t imm None
  */
    [[nodiscard]] uint64_t imm() const;
    /**
      uint64_t imm None
  */
    void imm(const uint64_t &value);
    /**
      uint64_t len None
  */
    [[nodiscard]] uint64_t len() const;
    /**
      uint64_t len None
  */
    void len(const uint64_t &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_memset &c);

/**
    MFU.MEMCPY
    @brief None
    - OPCODE opcode None
    - CCRCLR ccrclr None
    - CCRSET ccrset None
    - ADDR_GLB_8_WITH_BANK addr_src None
    - ADDR_GLB_8_WITH_BANK addr_dest None
    - STRIDE_GLB stride_src_glb None
    - STRIDE_GLB stride_dest_glb None
    - uint64_t shape_n None
    - uint64_t shape_c None
    - uint64_t shape_h None
    - uint64_t shape_w None
    - PRECISION precision_glb None
**/
class inst_mfu_memcpy : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** None **/
    CCRCLR ccrclr_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_src_ = ADDR_GLB_8_WITH_BANK(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_dest_ = ADDR_GLB_8_WITH_BANK(0); /** None **/
    STRIDE_GLB stride_src_glb_ = STRIDE_GLB(0); /** None **/
    STRIDE_GLB stride_dest_glb_ = STRIDE_GLB(0); /** None **/
    uint64_t shape_n_ = uint64_t(0); /** None **/
    uint64_t shape_c_ = uint64_t(0); /** None **/
    uint64_t shape_h_ = uint64_t(0); /** None **/
    uint64_t shape_w_ = uint64_t(0); /** None **/
    PRECISION precision_glb_ = PRECISION(0); /** None **/
public:
    inst_mfu_memcpy(CCRCLR ccrclr = CCRCLR(0), CCRSET ccrset = CCRSET(0),
        ADDR_GLB_8_WITH_BANK addr_src = ADDR_GLB_8_WITH_BANK(0),
        ADDR_GLB_8_WITH_BANK addr_dest = ADDR_GLB_8_WITH_BANK(0),
        STRIDE_GLB stride_src_glb = STRIDE_GLB(0),
        STRIDE_GLB stride_dest_glb = STRIDE_GLB(0),
        uint64_t shape_n = uint64_t(0),
        uint64_t shape_c = uint64_t(0),
        uint64_t shape_h = uint64_t(0),
        uint64_t shape_w = uint64_t(0),
        PRECISION precision_glb = PRECISION(0));

    explicit inst_mfu_memcpy(const INST_MFU_MEMCPY &ref);

    [[nodiscard]] struct INST_MFU_MEMCPY to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode None
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode None
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr None
  */
    [[nodiscard]] CCRCLR ccrclr() const;
    /**
      CCRCLR ccrclr None
  */
    void ccrclr(const CCRCLR &value);
    /**
      CCRSET ccrset None
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset None
  */
    void ccrset(const CCRSET &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_src None
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_src() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_src None
  */
    void addr_src(const ADDR_GLB_8_WITH_BANK &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_dest None
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_dest() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_dest None
  */
    void addr_dest(const ADDR_GLB_8_WITH_BANK &value);
    /**
      STRIDE_GLB stride_src_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_src_glb() const;
    /**
      STRIDE_GLB stride_src_glb None
  */
    void stride_src_glb(const STRIDE_GLB &value);
    /**
      STRIDE_GLB stride_dest_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_dest_glb() const;
    /**
      STRIDE_GLB stride_dest_glb None
  */
    void stride_dest_glb(const STRIDE_GLB &value);
    /**
      uint64_t shape_n None
  */
    [[nodiscard]] uint64_t shape_n() const;
    /**
      uint64_t shape_n None
  */
    void shape_n(const uint64_t &value);
    /**
      uint64_t shape_c None
  */
    [[nodiscard]] uint64_t shape_c() const;
    /**
      uint64_t shape_c None
  */
    void shape_c(const uint64_t &value);
    /**
      uint64_t shape_h None
  */
    [[nodiscard]] uint64_t shape_h() const;
    /**
      uint64_t shape_h None
  */
    void shape_h(const uint64_t &value);
    /**
      uint64_t shape_w None
  */
    [[nodiscard]] uint64_t shape_w() const;
    /**
      uint64_t shape_w None
  */
    void shape_w(const uint64_t &value);
    /**
      PRECISION precision_glb None
  */
    [[nodiscard]] PRECISION precision_glb() const;
    /**
      PRECISION precision_glb None
  */
    void precision_glb(const PRECISION &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_memcpy &c);

/**
    MFU.TRANS
    @brief None
    - OPCODE opcode None
    - CCRCLR ccrclr None
    - CCRSET ccrset None
    - ADDR_GLB_8_WITH_BANK addr_src None
    - ADDR_GLB_8_WITH_BANK addr_dest None
    - STRIDE_GLB stride_src_glb None
    - STRIDE_GLB stride_dest_glb None
    - uint64_t shape_n None
    - uint64_t shape_c None
    - uint64_t shape_h None
    - uint64_t shape_w None
    - PRECISION precision_glb None
    - MFU_TRANS_PERMUTE permute None
**/
class inst_mfu_trans : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** None **/
    CCRCLR ccrclr_ = CCRCLR(0); /** None **/
    CCRSET ccrset_ = CCRSET(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_src_ = ADDR_GLB_8_WITH_BANK(0); /** None **/
    ADDR_GLB_8_WITH_BANK addr_dest_ = ADDR_GLB_8_WITH_BANK(0); /** None **/
    STRIDE_GLB stride_src_glb_ = STRIDE_GLB(0); /** None **/
    STRIDE_GLB stride_dest_glb_ = STRIDE_GLB(0); /** None **/
    uint64_t shape_n_ = uint64_t(0); /** None **/
    uint64_t shape_c_ = uint64_t(0); /** None **/
    uint64_t shape_h_ = uint64_t(0); /** None **/
    uint64_t shape_w_ = uint64_t(0); /** None **/
    PRECISION precision_glb_ = PRECISION(0); /** None **/
    MFU_TRANS_PERMUTE permute_ = MFU_TRANS_PERMUTE(0); /** None **/
public:
    inst_mfu_trans(CCRCLR ccrclr = CCRCLR(0), CCRSET ccrset = CCRSET(0),
        ADDR_GLB_8_WITH_BANK addr_src = ADDR_GLB_8_WITH_BANK(0),
        ADDR_GLB_8_WITH_BANK addr_dest = ADDR_GLB_8_WITH_BANK(0),
        STRIDE_GLB stride_src_glb = STRIDE_GLB(0),
        STRIDE_GLB stride_dest_glb = STRIDE_GLB(0),
        uint64_t shape_n = uint64_t(0), uint64_t shape_c = uint64_t(0),
        uint64_t shape_h = uint64_t(0), uint64_t shape_w = uint64_t(0),
        PRECISION precision_glb = PRECISION(0),
        MFU_TRANS_PERMUTE permute = MFU_TRANS_PERMUTE(0));

    explicit inst_mfu_trans(const INST_MFU_TRANS &ref);

    [[nodiscard]] struct INST_MFU_TRANS to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode None
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode None
  */
    void opcode(const OPCODE &value);
    /**
      CCRCLR ccrclr None
  */
    [[nodiscard]] CCRCLR ccrclr() const;
    /**
      CCRCLR ccrclr None
  */
    void ccrclr(const CCRCLR &value);
    /**
      CCRSET ccrset None
  */
    [[nodiscard]] CCRSET ccrset() const;
    /**
      CCRSET ccrset None
  */
    void ccrset(const CCRSET &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_src None
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_src() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_src None
  */
    void addr_src(const ADDR_GLB_8_WITH_BANK &value);
    /**
      ADDR_GLB_8_WITH_BANK addr_dest None
  */
    [[nodiscard]] ADDR_GLB_8_WITH_BANK addr_dest() const;
    /**
      ADDR_GLB_8_WITH_BANK addr_dest None
  */
    void addr_dest(const ADDR_GLB_8_WITH_BANK &value);
    /**
      STRIDE_GLB stride_src_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_src_glb() const;
    /**
      STRIDE_GLB stride_src_glb None
  */
    void stride_src_glb(const STRIDE_GLB &value);
    /**
      STRIDE_GLB stride_dest_glb None
  */
    [[nodiscard]] STRIDE_GLB stride_dest_glb() const;
    /**
      STRIDE_GLB stride_dest_glb None
  */
    void stride_dest_glb(const STRIDE_GLB &value);
    /**
      uint64_t shape_n None
  */
    [[nodiscard]] uint64_t shape_n() const;
    /**
      uint64_t shape_n None
  */
    void shape_n(const uint64_t &value);
    /**
      uint64_t shape_c None
  */
    [[nodiscard]] uint64_t shape_c() const;
    /**
      uint64_t shape_c None
  */
    void shape_c(const uint64_t &value);
    /**
      uint64_t shape_h None
  */
    [[nodiscard]] uint64_t shape_h() const;
    /**
      uint64_t shape_h None
  */
    void shape_h(const uint64_t &value);
    /**
      uint64_t shape_w None
  */
    [[nodiscard]] uint64_t shape_w() const;
    /**
      uint64_t shape_w None
  */
    void shape_w(const uint64_t &value);
    /**
      PRECISION precision_glb None
  */
    [[nodiscard]] PRECISION precision_glb() const;
    /**
      PRECISION precision_glb None
  */
    void precision_glb(const PRECISION &value);
    /**
      MFU_TRANS_PERMUTE permute None
  */
    [[nodiscard]] MFU_TRANS_PERMUTE permute() const;
    /**
      MFU_TRANS_PERMUTE permute None
  */
    void permute(const MFU_TRANS_PERMUTE &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_trans &c);

class inst_mfu_mn_conf2 : public gnne_instruction
{
    OPCODE opcode_ = OPCODE(0); /** 这条指令用来反向配置meshnet连接关系 **/
    MFU_MN_PORTIN in1_ = MFU_MN_PORTIN(0); /** VECTOR_IN_0连接的端口 **/
    MFU_MN_PORTIN in2_ = MFU_MN_PORTIN(0); /** ADDSUB0_IN_0连接的端口 **/
    MFU_MN_PORTIN in3_ = MFU_MN_PORTIN(0); /** ADDSUB0_IN_1连接的端口 **/
    MFU_MN_PORTIN in4_ = MFU_MN_PORTIN(0); /** ADDSUB1_IN_0连接的端口 **/
    MFU_MN_PORTIN in5_ = MFU_MN_PORTIN(0); /** ADDSUB1_IN_1连接的端口 **/
    MFU_MN_PORTIN in6_ = MFU_MN_PORTIN(0); /** ADDSUB2_IN_0连接的端口 **/
    MFU_MN_PORTIN in7_ = MFU_MN_PORTIN(0); /** ADDSUB2_IN_1连接的端口 **/
    MFU_MN_PORTIN in8_ = MFU_MN_PORTIN(0); /** ADDSUB3_IN_0连接的端口 **/
    MFU_MN_PORTIN in9_ = MFU_MN_PORTIN(0); /** ADDSUB3_IN_1连接的端口 **/
    MFU_MN_PORTIN in10_ = MFU_MN_PORTIN(0); /** MUL0_IN_0连接的端口 **/
    MFU_MN_PORTIN in11_ = MFU_MN_PORTIN(0); /** MUL0_IN_1连接的端口 **/
    MFU_MN_PORTIN in12_ = MFU_MN_PORTIN(0); /** MUL1_IN_0连接的端口 **/
    MFU_MN_PORTIN in13_ = MFU_MN_PORTIN(0); /** MUL1_IN_1连接的端口 **/
    MFU_MN_PORTIN in14_ = MFU_MN_PORTIN(0); /** MUL2_IN_0连接的端口 **/
    MFU_MN_PORTIN in15_ = MFU_MN_PORTIN(0); /** MUL2_IN_1连接的端口 **/
    MFU_MN_PORTIN in16_ = MFU_MN_PORTIN(0); /** MUL3_IN_0连接的端口 **/
    MFU_MN_PORTIN in17_ = MFU_MN_PORTIN(0); /** MUL3_IN_1连接的端口 **/
    MFU_MN_PORTIN in18_ = MFU_MN_PORTIN(0); /** DIV_IN_0连接的端口 **/
    MFU_MN_PORTIN in19_ = MFU_MN_PORTIN(0); /** DIV_IN_1连接的端口 **/
    MFU_MN_PORTIN in20_ = MFU_MN_PORTIN(0); /** ROUND_IN_0连接的端口 **/
    MFU_MN_PORTIN in21_ = MFU_MN_PORTIN(0); /** SQRT_IN_0连接的端口 **/
    MFU_MN_PORTIN in22_ = MFU_MN_PORTIN(0); /** TRANGLE_IN_0连接的端口 **/
    MFU_MN_PORTIN in23_ = MFU_MN_PORTIN(0); /** LOG_IN_0连接的端口 **/
    MFU_MN_PORTIN in24_ = MFU_MN_PORTIN(0); /** UNARY_LOGIC_IN_0连接的端口 **/
    MFU_MN_PORTIN in25_ = MFU_MN_PORTIN(0); /** EXP_IN_0连接的端口 **/
    MFU_MN_PORTIN in26_ = MFU_MN_PORTIN(0); /** CMP0_IN_0连接的端口 **/
    MFU_MN_PORTIN in27_ = MFU_MN_PORTIN(0); /** CMP0_IN_1连接的端口 **/
    MFU_MN_PORTIN in28_ = MFU_MN_PORTIN(0); /** CMP1_IN_0连接的端口 **/
    MFU_MN_PORTIN in29_ = MFU_MN_PORTIN(0); /** CMP1_IN_1连接的端口 **/
    MFU_MN_PORTIN in30_ = MFU_MN_PORTIN(0); /** REG0_IN_0连接的端口 **/
    MFU_MN_PORTIN in31_ = MFU_MN_PORTIN(0); /** SELECT0_IN_0连接的端口 **/
    MFU_MN_PORTIN in32_ = MFU_MN_PORTIN(0); /** SELECT0_IN_1连接的端口 **/
    MFU_MN_PORTIN in33_ = MFU_MN_PORTIN(0); /** SELECT0_IN_2连接的端口 **/
    MFU_MN_PORTIN in34_ = MFU_MN_PORTIN(0); /** NOP0_IN_0连接的端口 **/
    MFU_MN_PORTIN in35_ = MFU_MN_PORTIN(0); /** NOP1_IN_0连接的端口 **/
    MFU_MN_PORTIN in36_ = MFU_MN_PORTIN(0); /** NOP2_IN_0连接的端口 **/
    MFU_MN_PORTIN in37_ = MFU_MN_PORTIN(0); /** NOP3_IN_0连接的端口 **/
    MFU_MN_PORTIN in38_ = MFU_MN_PORTIN(0); /** NOP4_IN_0连接的端口 **/
    MFU_MN_PORTIN in39_ = MFU_MN_PORTIN(0); /** NOP5_IN_0连接的端口 **/
    MFU_MN_PORTIN in40_ = MFU_MN_PORTIN(0); /** NOP6_IN_0连接的端口 **/
    MFU_MN_PORTIN in41_ = MFU_MN_PORTIN(0); /** NOP7_IN_0连接的端口 **/
    MFU_MN_PORTIN in42_ = MFU_MN_PORTIN(0); /** CLEAR0_IN_0连接的端口 **/

public:
    inst_mfu_mn_conf2(MFU_MN_PORTIN in1 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in2 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in3 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in4 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in5 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in6 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in7 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in8 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in9 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in10 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in11 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in12 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in13 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in14 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in15 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in16 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in17 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in18 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in19 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in20 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in21 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in22 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in23 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in24 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in25 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in26 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in27 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in28 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in29 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in30 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in31 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in32 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in33 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in34 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in35 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in36 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in37 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in38 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in39 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in40 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in41 = MFU_MN_PORTIN(0),
        MFU_MN_PORTIN in42 = MFU_MN_PORTIN(0));

    explicit inst_mfu_mn_conf2(const INST_MFU_MN_CONF2 &ref);

    [[nodiscard]] struct INST_MFU_MN_CONF2 to_struct() const;

    void serialize(binary_writer &writer) const override;

    // getter and setters
    /**
      OPCODE opcode 这条指令用来反向配置meshnet连接关系
  */
    [[nodiscard]] OPCODE opcode() const override;
    /**
      OPCODE opcode 这条指令用来反向配置meshnet连接关系
  */
    void opcode(const OPCODE &value);
    /**
      MFU_MN_PORTIN in1 VECTOR_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in1() const;
    /**
      MFU_MN_PORTIN in1 VECTOR_IN_0连接的端口
  */
    void in1(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in2 ADDSUB0_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in2() const;
    /**
      MFU_MN_PORTIN in2 ADDSUB0_IN_0连接的端口
  */
    void in2(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in3 ADDSUB0_IN_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in3() const;
    /**
      MFU_MN_PORTIN in3 ADDSUB0_IN_1连接的端口
  */
    void in3(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in4 ADDSUB1_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in4() const;
    /**
      MFU_MN_PORTIN in4 ADDSUB1_IN_0连接的端口
  */
    void in4(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in5 ADDSUB1_IN_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in5() const;
    /**
      MFU_MN_PORTIN in5 ADDSUB1_IN_1连接的端口
  */
    void in5(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in6 ADDSUB2_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in6() const;
    /**
      MFU_MN_PORTIN in6 ADDSUB2_IN_0连接的端口
  */
    void in6(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in7 ADDSUB2_IN_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in7() const;
    /**
      MFU_MN_PORTIN in7 ADDSUB2_IN_1连接的端口
  */
    void in7(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in8 ADDSUB3_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in8() const;
    /**
      MFU_MN_PORTIN in8 ADDSUB3_IN_0连接的端口
  */
    void in8(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in9 ADDSUB3_IN_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in9() const;
    /**
      MFU_MN_PORTIN in9 ADDSUB3_IN_1连接的端口
  */
    void in9(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in10 MUL0_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in10() const;
    /**
      MFU_MN_PORTIN in10 MUL0_IN_0连接的端口
  */
    void in10(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in11 MUL0_IN_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in11() const;
    /**
      MFU_MN_PORTIN in11 MUL0_IN_1连接的端口
  */
    void in11(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in12 MUL1_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in12() const;
    /**
      MFU_MN_PORTIN in12 MUL1_IN_0连接的端口
  */
    void in12(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in13 MUL1_IN_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in13() const;
    /**
      MFU_MN_PORTIN in13 MUL1_IN_1连接的端口
  */
    void in13(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in14 MUL2_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in14() const;
    /**
      MFU_MN_PORTIN in14 MUL2_IN_0连接的端口
  */
    void in14(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in15 MUL2_IN_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in15() const;
    /**
      MFU_MN_PORTIN in15 MUL2_IN_1连接的端口
  */
    void in15(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in16 MUL3_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in16() const;
    /**
      MFU_MN_PORTIN in16 MUL3_IN_0连接的端口
  */
    void in16(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in17 MUL3_IN_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in17() const;
    /**
      MFU_MN_PORTIN in17 MUL3_IN_1连接的端口
  */
    void in17(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in18 DIV_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in18() const;
    /**
      MFU_MN_PORTIN in18 DIV_IN_0连接的端口
  */
    void in18(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in19 DIV_IN_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in19() const;
    /**
      MFU_MN_PORTIN in19 DIV_IN_1连接的端口
  */
    void in19(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in20 ROUND_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in20() const;
    /**
      MFU_MN_PORTIN in20 ROUND_IN_0连接的端口
  */
    void in20(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in21 SQRT_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in21() const;
    /**
      MFU_MN_PORTIN in21 SQRT_IN_0连接的端口
  */
    void in21(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in22 TRANGLE_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in22() const;
    /**
      MFU_MN_PORTIN in22 TRANGLE_IN_0连接的端口
  */
    void in22(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in23 LOG_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in23() const;
    /**
      MFU_MN_PORTIN in23 LOG_IN_0连接的端口
  */
    void in23(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in24 UNARY_LOGIC_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in24() const;
    /**
      MFU_MN_PORTIN in24 UNARY_LOGIC_IN_0连接的端口
  */
    void in24(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in25 EXP_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in25() const;
    /**
      MFU_MN_PORTIN in25 EXP_IN_0连接的端口
  */
    void in25(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in26 CMP0_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in26() const;
    /**
      MFU_MN_PORTIN in26 CMP0_IN_0连接的端口
  */
    void in26(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in27 CMP0_IN_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in27() const;
    /**
      MFU_MN_PORTIN in27 CMP0_IN_1连接的端口
  */
    void in27(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in28 CMP1_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in28() const;
    /**
      MFU_MN_PORTIN in28 CMP1_IN_0连接的端口
  */
    void in28(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in29 CMP1_IN_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in29() const;
    /**
      MFU_MN_PORTIN in29 CMP1_IN_1连接的端口
  */
    void in29(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in30 REG0_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in30() const;
    /**
      MFU_MN_PORTIN in30 REG0_IN_0连接的端口
  */
    void in30(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in31 SELECT0_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in31() const;
    /**
      MFU_MN_PORTIN in31 SELECT0_IN_0连接的端口
  */
    void in31(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in32 SELECT0_IN_1连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in32() const;
    /**
      MFU_MN_PORTIN in32 SELECT0_IN_1连接的端口
  */
    void in32(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in33 SELECT0_IN_2连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in33() const;
    /**
      MFU_MN_PORTIN in33 SELECT0_IN_2连接的端口
  */
    void in33(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in34 NOP0_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in34() const;
    /**
      MFU_MN_PORTIN in34 NOP0_IN_0连接的端口
  */
    void in34(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in35 NOP1_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in35() const;
    /**
      MFU_MN_PORTIN in35 NOP1_IN_0连接的端口
  */
    void in35(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in36 NOP2_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in36() const;
    /**
      MFU_MN_PORTIN in36 NOP2_IN_0连接的端口
  */
    void in36(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in37 NOP3_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in37() const;
    /**
      MFU_MN_PORTIN in37 NOP3_IN_0连接的端口
  */
    void in37(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in38 NOP4_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in38() const;
    /**
      MFU_MN_PORTIN in38 NOP4_IN_0连接的端口
  */
    void in38(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in39 NOP5_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in39() const;
    /**
      MFU_MN_PORTIN in39 NOP5_IN_0连接的端口
  */
    void in39(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in40 NOP6_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in40() const;
    /**
      MFU_MN_PORTIN in40 NOP6_IN_0连接的端口
  */
    void in40(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in41 NOP7_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in41() const;
    /**
      MFU_MN_PORTIN in41 NOP7_IN_0连接的端口
  */
    void in41(const MFU_MN_PORTIN &value);
    /**
      MFU_MN_PORTIN in42 CLEAR0_IN_0连接的端口
  */
    [[nodiscard]] MFU_MN_PORTIN in42() const;
    /**
      MFU_MN_PORTIN in42 CLEAR0_IN_0连接的端口
  */
    void in42(const MFU_MN_PORTIN &value);

    void to_string(std::ostream &out) const override;
};

std::ostream &operator<<(std::ostream &out, const inst_mfu_mn_conf2 &c);

////////////////////////////////////

class gnne_visitor
{
public:
    virtual void visit(uint8_t *pc) = 0;
    virtual std::unique_ptr<inst_nop> visit(std::unique_ptr<inst_nop> inst) = 0;
    virtual std::unique_ptr<inst_li> visit(std::unique_ptr<inst_li> inst) = 0;
    virtual std::unique_ptr<inst_intr> visit(std::unique_ptr<inst_intr> inst) = 0;
    virtual std::unique_ptr<inst_end> visit(std::unique_ptr<inst_end> inst) = 0;
    virtual std::unique_ptr<inst_fence>
    visit(std::unique_ptr<inst_fence> inst) = 0;
    virtual std::unique_ptr<inst_mmu_conf>
    visit(std::unique_ptr<inst_mmu_conf> inst) = 0;
    virtual std::unique_ptr<inst_fence_ccr>
    visit(std::unique_ptr<inst_fence_ccr> inst) = 0;
    virtual std::unique_ptr<inst_loadif_config>
    visit(std::unique_ptr<inst_loadif_config> inst) = 0;
    virtual std::unique_ptr<inst_loadif>
    visit(std::unique_ptr<inst_loadif> inst) = 0;
    virtual std::unique_ptr<inst_load> visit(std::unique_ptr<inst_load> inst) = 0;
    virtual std::unique_ptr<inst_loadif_compress_conf>
    visit(std::unique_ptr<inst_loadif_compress_conf> inst) = 0;
    virtual std::unique_ptr<inst_load_compress_conf>
    visit(std::unique_ptr<inst_load_compress_conf> inst) = 0;
    virtual std::unique_ptr<inst_store>
    visit(std::unique_ptr<inst_store> inst) = 0;
    virtual std::unique_ptr<inst_store_t_config>
    visit(std::unique_ptr<inst_store_t_config> inst) = 0;
    virtual std::unique_ptr<inst_store_t>
    visit(std::unique_ptr<inst_store_t> inst) = 0;
    virtual std::unique_ptr<inst_store_t_compress_conf>
    visit(std::unique_ptr<inst_store_t_compress_conf> inst) = 0;
    virtual std::unique_ptr<inst_store_compress_conf>
    visit(std::unique_ptr<inst_store_compress_conf> inst) = 0;
    virtual std::unique_ptr<inst_tcu_dm_broadcast>
    visit(std::unique_ptr<inst_tcu_dm_broadcast> inst) = 0;
    virtual std::unique_ptr<inst_tcu_dm_conf_if>
    visit(std::unique_ptr<inst_tcu_dm_conf_if> inst) = 0;
    virtual std::unique_ptr<inst_tcu_dm_fetchif>
    visit(std::unique_ptr<inst_tcu_dm_fetchif> inst) = 0;
    virtual std::unique_ptr<inst_tcu_dm_conf_w>
    visit(std::unique_ptr<inst_tcu_dm_conf_w> inst) = 0;
    virtual std::unique_ptr<inst_tcu_dm_fetchw>
    visit(std::unique_ptr<inst_tcu_dm_fetchw> inst) = 0;
    virtual std::unique_ptr<inst_tcu_dm_conf_of>
    visit(std::unique_ptr<inst_tcu_dm_conf_of> inst) = 0;
    virtual std::unique_ptr<inst_tcu_pu_conf>
    visit(std::unique_ptr<inst_tcu_pu_conf> inst) = 0;
    virtual std::unique_ptr<inst_tcu_pu_conf_act>
    visit(std::unique_ptr<inst_tcu_pu_conf_act> inst) = 0;
    virtual std::unique_ptr<inst_tcu_pu_compute>
    visit(std::unique_ptr<inst_tcu_pu_compute> inst) = 0;
    virtual std::unique_ptr<inst_tcu_dot_dm_if_conf>
    visit(std::unique_ptr<inst_tcu_dot_dm_if_conf> inst) = 0;
    virtual std::unique_ptr<inst_tcu_dot_dm_of_conf>
    visit(std::unique_ptr<inst_tcu_dot_dm_of_conf> inst) = 0;
    virtual std::unique_ptr<inst_tcu_dot_dm_fetch_src1>
    visit(std::unique_ptr<inst_tcu_dot_dm_fetch_src1> inst) = 0;
    virtual std::unique_ptr<inst_tcu_dot_dm_fetch_src2>
    visit(std::unique_ptr<inst_tcu_dot_dm_fetch_src2> inst) = 0;
    virtual std::unique_ptr<inst_tcu_pu_compute_dummy>
    visit(std::unique_ptr<inst_tcu_pu_compute_dummy> inst) = 0;
    virtual std::unique_ptr<inst_mfu_mn_map_compute>
    visit(std::unique_ptr<inst_mfu_mn_map_compute> inst) = 0;
    virtual std::unique_ptr<inst_mfu_mn_vmap_compute>
    visit(std::unique_ptr<inst_mfu_mn_vmap_compute> inst) = 0;
    virtual std::unique_ptr<inst_mfu_reduce>
    visit(std::unique_ptr<inst_mfu_reduce> inst) = 0;
    virtual std::unique_ptr<inst_mfu_vreduce>
    visit(std::unique_ptr<inst_mfu_vreduce> inst) = 0;
    virtual std::unique_ptr<inst_mfu_mn_broadcast_compute>
    visit(std::unique_ptr<inst_mfu_mn_broadcast_compute> inst) = 0;
    virtual std::unique_ptr<inst_mfu_mn_reduce>
    visit(std::unique_ptr<inst_mfu_mn_reduce> inst) = 0;
    virtual std::unique_ptr<inst_mfu_mn_conf>
    visit(std::unique_ptr<inst_mfu_mn_conf> inst) = 0;
    virtual std::unique_ptr<inst_mfu_mnop_conf>
    visit(std::unique_ptr<inst_mfu_mnop_conf> inst) = 0;
    virtual std::unique_ptr<inst_mfu_pdp_conf>
    visit(std::unique_ptr<inst_mfu_pdp_conf> inst) = 0;
    virtual std::unique_ptr<inst_mfu_pdp_src_conf>
    visit(std::unique_ptr<inst_mfu_pdp_src_conf> inst) = 0;
    virtual std::unique_ptr<inst_mfu_pdp_reduce>
    visit(std::unique_ptr<inst_mfu_pdp_reduce> inst) = 0;
    virtual std::unique_ptr<inst_mfu_mn_broadcast_conf>
    visit(std::unique_ptr<inst_mfu_mn_broadcast_conf> inst) = 0;
    virtual std::unique_ptr<inst_mfu_crop>
    visit(std::unique_ptr<inst_mfu_crop> inst) = 0;
    virtual std::unique_ptr<inst_mfu_memset>
    visit(std::unique_ptr<inst_mfu_memset> inst) = 0;
    virtual std::unique_ptr<inst_mfu_memcpy>
    visit(std::unique_ptr<inst_mfu_memcpy> inst) = 0;
    virtual std::unique_ptr<inst_mfu_trans>
    visit(std::unique_ptr<inst_mfu_trans> inst) = 0;
    virtual std::unique_ptr<inst_mfu_mn_conf2>
    visit(std::unique_ptr<inst_mfu_mn_conf2> inst) = 0;
};

std::unique_ptr<gnne_instruction> decode(uint8_t *pc, uint8_t **next_pc,
    gnne_visitor *visitor);

template <class T, typename... Args>
inline std::shared_ptr<gnne_instruction> make_instructions(Args &&... args)
{
    return static_cast<std::shared_ptr<gnne_instruction>>(
        std::make_shared<T>(std::forward<Args>(args)...));
}

} // namespace nncase::runtime::k230::isa
