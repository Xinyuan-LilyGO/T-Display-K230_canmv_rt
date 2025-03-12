#ifndef BUILDING_RUNTIME
#include <filesystem>
#endif
#include <nncase/runtime/k230/gnne_tile_utils.h>

namespace nncase
{
namespace F
{
    namespace k230
    {
        using namespace nncase::runtime;
        using namespace nncase::runtime::k230;
        // using namespace nncase::ir::transforms;

#define AI2D_BASE_ADDR_PAGE_ALIGNED 0x80400000
#define AI2D_BASE_OFFSET 0x0c00
#define AI2D_MMAP_SIZE 0x1000

        typedef enum AI2D_INTR
        {
            AI2D_INTR_OK,
            AI2D_INTR_TIME_OUT,
            AI2D_INTR_EXCEPTION,
        } ai2d_intr_t;

#if defined(BUILDING_RUNTIME)
        struct addr
        {
            volatile void *addr;
            volatile void *paddr;
#if defined(__linux__)
            volatile void *vaddr;
            size_t mmap_size;
#endif
        };
#endif

        class ai2d_builder
        {
        private:
            ai2d_datatype_t ai2d_dtype_;
            ai2d_crop_param_t crop_param_;
            ai2d_shift_param_t shift_param_;
            ai2d_pad_param_t pad_param_;
            ai2d_resize_param_t resize_param_;
            ai2d_affine_param_t affine_param_;
            std::vector<std::vector<uint32_t>> regs_;
            std::vector<uint32_t> split_pos_;
            ai2d_config config_;

            result<void> check_config();

            int32_t input_c_;
            int32_t output_c_;
            int32_t input_h_;
            int32_t input_w_;
            int32_t output_h_;
            int32_t output_w_;

#if defined(BUILDING_RUNTIME)
            struct addr ai2d_addr_;
#if !defined(__linux__)
            volatile bool ai2d_done_;
#else
            int ai2d_fd_;
            int mem_fd_;
#endif
            void ai2d_set_base(volatile void *addr);
            volatile uint8_t *ai2d_get_base();
            uint16_t ai2d_get_intr_num();
            void ai2d_set_time_out(uint32_t val);
#endif

        public:
            ai2d_builder(dims_t &input_shape, dims_t &output_shape, ai2d_datatype_t ai2d_dtype,
                ai2d_crop_param_t crop_param, ai2d_shift_param_t shift_param, ai2d_pad_param_t pad_param,
                ai2d_resize_param_t resize_param, ai2d_affine_param_t affine_param);
            static std::unique_ptr<ai2d_builder> create(dims_t &input_shape, dims_t &output_shape, ai2d_datatype_t &ai2d_dtype,
                ai2d_crop_param_t &crop_param, ai2d_shift_param_t &shift_param, ai2d_pad_param_t &pad_param,
                ai2d_resize_param_t &resize_param, ai2d_affine_param_t &affine_param);
            ~ai2d_builder();

#if defined(BUILDING_RUNTIME)
            void ai2d_clear_cpu_intr();
#endif
            dims_t input_shape_, output_shape_;
            // runtime::k230::span_writer input_writer_, reg_writer_;

            bool dump_asm_;
#ifndef BUILDING_RUNTIME
            std::filesystem::path dump_dir_;
#endif

            result<void> build_schedule();
            void dump_asm(bool write_all);
            result<void> dump_gmodel();
            result<void> invoke(runtime_tensor &input, runtime_tensor &output);
        };
    }
}
}