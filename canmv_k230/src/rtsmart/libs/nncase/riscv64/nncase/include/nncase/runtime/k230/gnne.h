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
#ifndef _GNNE_H_
#define _GNNE_H_

#include <stdint.h>

#undef L2_BASE_ADDR
#define L2_BASE_ADDR 0x80000000

#undef GNNE_BASE_ADDR
#define GNNE_BASE_ADDR 0x80400000
#define GNNE_ICACHE_CFG_OFFSET 0xf0

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum _gnne_status
    {
        GNNE_STATUS_IDLE,
        GNNE_STATUS_RUNNING,
        GNNE_STATUS_PENDING,
        GNNE_STATUS_ERROR
    } gnne_status_t;

    typedef enum _gnne_reset_status
    {
        GNNE_RESET_STATUS_NORMAL,
        GNNE_RESET_STATUS_FLUSHING_BUS_INTERFACE,
        GNNE_RESET_STATUS_INITIALIZING_SUB_MODULE,
        GNNE_RESET_STATUS_TURNING_ON_CLK
    } gnne_reset_status_t;

    typedef enum _gnne_exception
    {
        GNNE_EXCEPTION_OK,
        GNNE_EXCEPTION_ILLEGAL_INSTRUCTION,
        GNNE_EXCEPTION_CCR_ERROR,
        GNNE_EXCEPTION_TIME_OUT_ERROR
    } gnne_exception_t;

    typedef enum
#ifdef __cplusplus
        : uint64_t
#endif
    {
        GNNE_CTRL_ENABLE_SET = (0x0001ULL << 32) | (0x0001),
        GNNE_CTRL_ENABLE_CLEAR = (0x0001ULL << 32) | (0x0000),
        GNNE_CTRL_CPU_INTR_CLEAR = (0x0001ULL << 34) | (0x0001ULL << 2),
        GNNE_CTRL_DEBUG_MODE_SET = (0x0001ULL << 35) | (0x0001ULL << 3),
        GNNE_CTRL_CG_OFF_SET = (0x0001ULL << 36) | (0x0001ULL << 4),
        GNNE_CTRL_CG_OFF_CLEAR = (0x0001ULL << 36) | (0x0000),
        GNNE_CTRL_CPU_RESUME_MODE_0 = (0x0001ULL << 39) | (0x0001ULL << 38) | (0x0001ULL << 37) | (0x0001ULL << 5),
        GNNE_CTRL_CPU_RESUME_MODE_1 = (0x0001ULL << 39) | (0x0001ULL << 38) | (0x0001ULL << 37) | (0x0001ULL << 6) | (0x0001ULL << 5),
        GNNE_CTRL_CPU_RESUME_MODE_2 = (0x0001ULL << 39) | (0x0001ULL << 38) | (0x0001ULL << 37) | (0x0001ULL << 7) | (0x0001ULL << 5),
        GNNE_CTRL_CPU_RESUME_MODE_3 = (0x0001ULL << 39) | (0x0001ULL << 38) | (0x0001ULL << 37) | (0x0001ULL << 7) | (0x0001ULL << 6) | (0x0001ULL << 5)
    } gnne_ctrl_function_t;

    typedef union
    {
        struct icache
        {
            uint64_t gb_pre_pc_byte_len : 32;
            uint64_t gb_pos_pc_byte_len : 32;
            uint64_t gb_fet_len_when_miss : 32;
            uint64_t gb_fet_len_when_pre : 32;
        } bits;
        uint64_t data[2];
    } gnne_icache_cfg;

    typedef union
    {
        struct gnne_pc
        {
            uint64_t start_pc_addr_reg : 32;
            uint64_t end_pc_addr_reg : 32;
            uint64_t breakpoint_pc_addr_reg : 32;
            uint64_t reserved : 32;
        } bits;
        uint64_t data[2];
    } gnne_pc_cfg;

    typedef union
    {
        struct ctrl
        {
            uint64_t reserved0 : 64;
            uint64_t gnne_enable : 1;
            uint64_t reserved1 : 1;
            uint64_t cpu_intr_clr : 1;
            uint64_t debug_mode_en : 1;
            uint64_t gnne_cg_off : 1;
            uint64_t gb_cpu_resume_en : 1;
            uint64_t cpu_resume_mode : 2;
            uint64_t reserved2 : 24;
            uint64_t gnne_enable_wmask : 1;
            uint64_t reserved3 : 1;
            uint64_t cpu_intr_clr_wmask : 1;
            uint64_t debug_mode_en_wmask : 1;
            uint64_t gnne_cg_off_wmask : 1;
            uint64_t cpu_resume_wmask : 1;
            uint64_t cpu_resume_mode_wmask : 2;
            uint64_t reserved4 : 24;
        } bits;
        uint64_t data[2];
    } gnne_ctrl;

    typedef union
    {
        struct status
        {
            uint64_t load_que_satus : 1;
            uint64_t store_que_status : 1;
            uint64_t dm_que_status : 1;
            uint64_t pu_que_status : 1;
            uint64_t mfu_que_status : 1;
            uint64_t load_module_status : 1;
            uint64_t store_module_status : 1;
            uint64_t dm_module_status : 1;
            uint64_t pu_module_status : 1;
            uint64_t mfu_module_status : 1;
            uint64_t version : 4;
            uint64_t kpu_work_status : 2;
            uint64_t noc_rx_status : 1;
            uint64_t noc_tx_status : 1;
            uint64_t reserved0 : 1;
            uint64_t store_biu_status : 1;
            uint64_t load_biu_status : 1;
            uint64_t all_biu_status : 1;
            uint64_t exception_status : 2;
            uint64_t reset_status : 2;
            uint64_t axi_bresp_error : 1;
            uint64_t ai2d_que_no_empty : 1;
            uint64_t ai2d_busy : 1;
            uint64_t reserved1 : 2;
            uint64_t intr_status : 1;
            uint64_t intr_num : 32;
            uint64_t dm_w_status : 1;
            uint64_t dm_if_status : 1;
            uint64_t dm_psum_status : 1;
            uint64_t dm_act_status : 1;
            uint64_t pu_pe_status : 1;
            uint64_t pu_dw_status : 1;
            uint64_t pu_act0_status : 1;
            uint64_t pu_act1_status : 1;
            uint64_t reserved2 : 56;
        } bits;
        uint64_t data[2];
    } gnne_status;

    typedef union
    {
        struct dec_ld_st_mfu_pc
        {
            uint64_t dec_pc : 32;
            uint64_t load_pc : 32;
            uint64_t store_pc : 32;
            uint64_t mfu_pc : 32;
        } bits;
        uint64_t data[2];
    } gnne_dec_ld_st_mfu_pc;

    typedef union
    {
        struct pu_pc_
        {
            uint64_t pu_pc : 32;
            uint64_t dw_pc : 32;
            uint64_t act0_pc : 32;
            uint64_t act1_pc : 32;
        } bits;
        uint64_t data[2];
    } gnne_pu_pc;

    typedef union
    {
        struct dm_pc
        {
            uint64_t dm_w_pc : 32;
            uint64_t dm_if_pc : 32;
            uint64_t dm_psum_pc : 32;
            uint64_t dm_act_pc : 32;
        } bits;
        uint64_t data[2];
    } gnne_dm_pc;
    typedef union
    {
        struct ccr_status
        {
            uint64_t ccr0 : 4;
            uint64_t ccr1 : 4;
            uint64_t ccr2 : 4;
            uint64_t ccr3 : 4;
            uint64_t ccr4 : 4;
            uint64_t ccr5 : 4;
            uint64_t ccr6 : 4;
            uint64_t ccr7 : 4;
            uint64_t ccr8 : 4;
            uint64_t ccr9 : 4;
            uint64_t ccr10 : 4;
            uint64_t ccr11 : 4;
            uint64_t ccr12 : 4;
            uint64_t ccr13 : 4;
            uint64_t ccr14 : 4;
            uint64_t ccr15 : 4;
            uint64_t ccr16 : 4;
            uint64_t ccr17 : 4;
            uint64_t ccr18 : 4;
            uint64_t ccr19 : 4;
            uint64_t ccr20 : 4;
            uint64_t ccr21 : 4;
            uint64_t ccr22 : 4;
            uint64_t ccr23 : 4;
            uint64_t ccr24 : 4;
            uint64_t ccr25 : 4;
            uint64_t ccr26 : 4;
            uint64_t ccr27 : 4;
            uint64_t ccr28 : 4;
            uint64_t ccr29 : 4;
            uint64_t ccr30 : 4;
            uint64_t ccr31 : 4;
        } bits;
        uint64_t data[2];
    } gnne_ccr_status;

    typedef union
    {
        struct ai2d_pc
        {
            uint64_t ai2d_pc_addr : 32;
            uint64_t reserved0 : 32;
            uint64_t reserved1 : 64;
        } bits;
        uint64_t data[2];
    } gnne_ai2d_pc;

    typedef union
    {
        struct time_out
        {
            uint64_t time_out_value : 32;
            uint64_t reserved0 : 32;
            uint64_t reserved1 : 64;
        } bits;
        uint64_t data[2];
    } gnne_time_out;

    typedef union
    {
        struct clk_gate
        {
            uint64_t gb_disable_dm_act_cg : 1;
            uint64_t gb_disable_dm_w_cg : 1;
            uint64_t gb_disable_dm_of_cg : 1;
            uint64_t gb_disable_dm_if_cg : 1;
            uint64_t gb_disable_act_cg : 1;
            uint64_t gb_disable_dw_cg : 1;
            uint64_t reserved0 : 58;
            uint64_t reserved1 : 64;
        } bits;
        uint64_t data[2];
    } gnne_clk_gate_switch;

    typedef struct _gnne_reg_file
    {
        gnne_icache_cfg icache_cfg;
        gnne_pc_cfg pc_cfg;
        uint64_t reserved0[2];
        gnne_ctrl ctrl;
        gnne_status status;
        gnne_dec_ld_st_mfu_pc dec_ld_st_mfu_pc;
        gnne_pu_pc pu_pc;
        gnne_dm_pc dm_pc;
        gnne_ccr_status ccr_status;
        gnne_ai2d_pc ai2d_pc;
        gnne_time_out time_out;
        uint64_t reserved1[12];
        gnne_clk_gate_switch clk_gate_switch;
    } gnne_reg_file_t;

#if !defined(__linux__)
    void flush_cache();
    void clean_cache();
    void invalidate_cache();
#endif
    void gnne_set_base(volatile void *addr);
    int gnne_enable(uint64_t pc_start, uint64_t pc_end, uint64_t pc_breakpoint);
    void gnne_init();
    void gnne_disable();
    void gnne_clear_cpu_intr();
    int gnne_resume(gnne_ctrl_function_t resume_mode, uint64_t pc_start);
    gnne_status gnne_get_status();
    void gnne_dump_status();
    void gnne_dump_pc();
    void gnne_dump_ccr();
    uint64_t gnne_get_pc();
    gnne_ctrl gnne_get_ctrl();
    gnne_dec_ld_st_mfu_pc gnne_get_dec_ld_st_mfu_pc();
    gnne_pu_pc gnne_get_pu_pc();
    gnne_dm_pc gnne_get_dm_pc();
    gnne_ccr_status gnne_get_ccr_status();

    uint32_t gnne_get_time_out();
    void gnne_set_time_out(uint32_t val);

    // ai2d
    uint32_t gnne_get_ai2d_pc();
    void gnne_set_ai2d_pc(uint32_t val);

#ifdef __cplusplus
}
#endif

#endif
