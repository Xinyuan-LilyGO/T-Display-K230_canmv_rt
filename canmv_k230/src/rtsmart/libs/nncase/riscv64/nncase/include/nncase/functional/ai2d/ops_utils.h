#pragma once
#include <array>
#include <functional>
#include <nncase/runtime/datatypes.h>
#include <nncase/runtime/k230/error.h>
// #include <nncase/runtime/k230/fp24.h>
// #include <nncase/runtime/k230/gnne_instructions.h>
// #include <nncase/runtime/k230/runtime_types.h>
#include <nncase/runtime/runtime_op_utility.h>
#include <nncase/runtime/runtime_tensor.h>
#include <string>
#include <utility>

namespace nncase
{
namespace F
{
    namespace k230
    {
        enum class slot_t : uint8_t
        {
            in_shape = 0x1,
            in_stride = 0x2,
            in_bytes = 0x3,
            out_shape = 0x4,
            out_stride = 0x5,
            out_bytes = 0x6
        };

        struct range_t
        {
            size_t begin;
            size_t end;

            template <typename T>
            range_t(T a)
                : begin(0), end(a)
            {
            }

            template <typename T>
            range_t(T a, T b)
                : begin(a), end(b)
            {
            }

            friend std::ostream &operator<<(std::ostream &os, const range_t &range)
            {
                os << std::to_string(range.begin) << std::string(" ") << std::to_string(range.end);
                return os;
            }
        };

        template <typename T, bool ByteAligned = true>
        struct slot_decs
        {
            using datatype = T;
            const static bool byte_aligned = ByteAligned;
            size_t bits_offset;
            size_t total_bits;
            range_t range;
        };

        // template <typename From, typename To>
        // struct is_explicit_convertiable : std::bool_constant<std::is_constructible_v<To, From> && !std::is_convertible_v<From, To>>
        // {
        // };

        // template <typename From, typename To>
        // constexpr bool is_explicit_convertiable_v = is_explicit_convertiable<From, To>::value;

        template <typename WriteDtype, typename T>
        inline void overwrite(const uint8_t *dest, size_t offset, T &value)
        {
            WriteDtype *ptr = reinterpret_cast<WriteDtype *>(const_cast<uint8_t *>(dest + offset));
            *ptr = (WriteDtype)value;
        }

        template <typename WriteDtype>
        inline void overwrite(const uint8_t *dest, size_t offset, const dims_t &value)
        {
            for (size_t i = 0; i < value.size(); i++)
            {
                overwrite<WriteDtype>(dest, offset + i * sizeof(WriteDtype), value[i]);
            }
        }

        template <typename T>
        inline void overwrite(datatype_t dt, const uint8_t *dest, size_t offset, const T &value)
        {
            if (dt->typecode() == dt_uint32)
            {
                overwrite<uint32_t>(dest, offset, value);
            }
        }

        /**
 * @brief write bits
 *
 * @param total_bits the value field total bits
 * @param writed_bits current writed bits , for recursive
 * @param bits_offest current bits offset in the one byte
 */
        inline void bits_writer(uint8_t *dest, size_t total_bits, size_t writed_bits, size_t bits_offest, const uint64_t value)
        {
            // inner var
            int write_bits_ = std::min(total_bits - writed_bits, (size_t)8);
            int over_flow_ = std::max(write_bits_ + (int)bits_offest - 8, 0);
            // used var
            int cur_bits = write_bits_ - over_flow_;
            uint8_t byte = ((value >> writed_bits) & (0xff >> (8 - cur_bits))) << bits_offest;
            uint8_t mask = (0xff >> (8 - bits_offest)) | (0xff << (cur_bits + bits_offest));
            int carry = (bits_offest + cur_bits) / 8;
            int mod = (bits_offest + cur_bits) % 8;
            // assgin value

            *dest = byte | (*dest & mask);
            if (total_bits > writed_bits + cur_bits)
            {
                bits_writer(dest + carry, total_bits, writed_bits + cur_bits, mod, value);
            }
        }

        // template <typename Datatype, typename T, class = std::enable_if_t<std::is_integral_v<T>>>
        // inline void write_value(const slot_decs<Datatype, true> &slot, uint8_t *dest, const T value)
        // {
        //     Datatype *ptr = reinterpret_cast<Datatype *>(dest + slot.bits_offset / 8);
        //     *ptr = static_cast<Datatype>(value);
        // }

        // template <typename Datatype, typename T, class = std::enable_if_t<std::is_integral_v<T>>>
        // inline void write_value(const slot_decs<Datatype, false> &slot, uint8_t *dest, const T &value)
        // {
        //     // bits write int value
        //     size_t byte_offset = slot.bits_offset / 8;
        //     size_t mod = slot.bits_offset - (byte_offset * 8);
        //     bits_writer(dest + byte_offset, slot.total_bits, 0, mod, (uint64_t)value);
        // }

        // template <typename Datatype, bool Aligned>
        // inline void write_value(const slot_decs<Datatype, Aligned> &slot, uint8_t *dest, const fp24 &value)
        // {
        //     write_value<Datatype>(slot, dest, value.value);
        // }

        // template <typename Datatype, bool Aligned, typename T, class = std::enable_if_t<!std::is_integral_v<T>>>
        // inline void write_value(const slot_decs<Datatype, Aligned> &slot, uint8_t *dest, const T &value)
        // {
        //     // bits write int value
        //     if constexpr (std::is_convertible_v<T, uint64_t> || is_explicit_convertiable_v<T, uint64_t>)
        //     {
        //         write_value<Datatype>(slot, dest, (uint64_t)value);
        //     }
        //     else
        //     {
        //         static_assert(sizeof(T) == 0, "current datatype is not integral and can't cast to uint64_t, please check !");
        //     }
        // }

        /**
 * @brief when write vectorlike, dispatch aligned inner loop
 */
        template <typename Datatype, bool Aligned,
            template <typename, size_t> class Container, typename T, size_t N>
        inline void write_value(const slot_decs<Datatype, Aligned> &slot, uint8_t *dest, const Container<T, N> &value)
        {
            for (size_t i = slot.range.begin; i < slot.range.end; i++)
            {
                // NOTE the inst write offset must start from 0.
                write_value<Datatype>(slot, dest + (i - slot.range.begin) * (slot.total_bits / 8), value[i]);
            }
        }

        // template <typename... TArgs, typename Callable, size_t... Is>
        // inline void foreach_slots(const std::tuple<TArgs...> &slots, Callable &&fn, std::index_sequence<Is...>)
        // {
        //     (fn(std::get<Is>(slots)), ...);
        // }

        template <typename... TArgs, typename Callable>
        inline void foreach_slots(const std::tuple<TArgs...> &slots, Callable &&fn)
        {
            foreach_slots(slots, fn, std::make_index_sequence<sizeof...(TArgs)> {});
        }

        template <typename T>
        inline auto get_callback(uint8_t *dest, const T &value)
        {
            return [dest, &value](auto slot) {
                write_value(slot, dest, value);
            };
        }

        /**
 * @brief compute fold shape, for fast load and store
 *
 * @param shape
 * @param split
 * @return dims_t
 */
        inline dims_t compute_shape_fold(dims_t shape, int split)
        {
            dims_t fold_shape = shape;
            if (split == -1)
                return fold_shape;
            for (int i = split + 1; i < int(shape.size()) - 1; i++)
            {
                fold_shape[i + 1] *= fold_shape[i];
                fold_shape[i] = 1;
            }
            return fold_shape;
        }

        inline dims_t get_default_strides(const dims_t &shape)
        {
            dims_t strides = runtime::get_default_strides(shape);
            // remove the zero strides.
            for (int i = shape.size() - 1; i >= 0; i--)
            {
                if (strides[i] == 0)
                    strides[i] = (i == int(shape.size()) - 1) ? 1 : (strides[i + 1] * shape[i + 1]);
            }
            return strides;
        }

        inline dims_t compute_ddr_stride(const dims_t &shape, datatype_t ddr_dtype)
        {
            // NOTE if the inst use ddr data,the STRIDE_GLB actual is tensor.shape
            size_t bytes = runtime::get_bytes(ddr_dtype);
            dims_t strides = runtime::get_default_strides(shape);
            // remove the zero strides.
            for (int i = shape.size() - 1; i >= 0; i--)
            {
                if (strides[i] == 0)
                    strides[i] = (i == int(shape.size()) - 1) ? 1 : (strides[i + 1] * shape[i + 1]);
            }
            std::for_each(strides.begin(), strides.end(), [&bytes](size_t &x) { x *= bytes; });
            return strides;
        }

        template <typename ShapeT>
        inline ShapeT align_broadcast_shape(const ShapeT &src, const ShapeT &dest)
        {
            assert(src.size() == dest.size());
            auto it = std::find_if(src.begin(), src.end(), [](size_t dim) { return dim != 1; });
            if (it == src.end())
            {
                return ShapeT(dest.size(), 1);
            }
            else
            {
                auto begin = std::distance(src.begin(), it);
                auto sub_len = src.size() - begin;
                for (int32_t i = dest.size() - sub_len; i >= 0; i--)
                {
                    bool found = true;
                    for (size_t j = 0; j < sub_len; j++)
                    {
                        auto dim = src.at(begin + j);
                        if (dim != 1 && dim != dest.at(i + j))
                        {
                            found = false;
                            break;
                        }
                    }

                    if (found)
                    {
                        ShapeT ret(i, 1);
                        for (size_t j = 0; j < sub_len; j++)
                            ret.push_back(src.at(begin + j));
                        auto surfix = dest.size() - ret.size();
                        for (size_t j = 0; j < surfix; j++)
                            ret.push_back(1);
                        return ret;
                    }
                }
            }

            throw std::runtime_error("Unexpected broadcast");
        }

        template <typename T = size_t>
        inline T compute_size(const dims_t &shape)
        {
            size_t size = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<size_t>());
            return static_cast<T>(size);
        }

        /**
 * @brief 把一个shape按split axis 分为两部分, 外面是主重复次数, 里面是block的size
 *
 * @param shape
 * @param split_axis
 * @return std::tuple<size_t, size_t>
 */
        inline std::tuple<size_t, size_t> compute_size(const dims_t &shape, int split_axis)
        {
            size_t inner = std::accumulate(shape.begin() + split_axis, shape.end(), 1, std::multiplies<size_t> {});
            size_t outter = std::accumulate(shape.begin(), shape.begin() + split_axis, 1, std::multiplies<size_t> {});
            return std::make_tuple(outter, inner);
        }

        template <typename T = size_t>
        inline T compute_bytes(size_t size, datatype_t dtype)
        {
            size_t bytes = size * runtime::get_bytes(dtype);
            return static_cast<T>(bytes);
        }

        inline size_t compute_once_size(const dims_t &shape, size_t split_axis)
        {
            return std::accumulate(shape.begin() + split_axis, shape.end(), 1, std::multiplies<size_t> {});
        }

        // template <datatype_t Dtype = dt_bfloat16>
        // gsl::span<to_cpp_type_t<Dtype>> runtime_tensor_view(nncase::runtime::runtime_tensor &t)
        // {
        //     auto map = std::move(nncase::runtime::hrt::map(t, nncase::runtime::hrt::map_read).unwrap_or_throw());
        //     return map.buffer().as_span<to_cpp_type_t<Dtype>>();
        // }

        // template <typename Dtype>
        // gsl::span<Dtype> runtime_tensor_view(nncase::runtime::runtime_tensor &t)
        // {
        //     auto map = std::move(nncase::runtime::hrt::map(t, nncase::runtime::hrt::map_read).unwrap_or_throw());
        //     return map.buffer().as_span<Dtype>();
        // }

        inline value_range<float> fixup_range(value_range<float> range, bool symmetric = false)
        {
            if (symmetric)
            {
                auto r = std::max({ std::abs(range.min), std::abs(range.max), 0.01f });
                return { -r, r };
            }
            else
            {
                if (range.min < -1e3)
                    range.min = -1e3;
                if (range.max > 1e3)
                    range.max = 1e3;
                auto r = range.max - range.min;
                if (r == 0)
                    r = 0.1f;
                else if (r < 0.01f)
                    r = 0.01f;
                range.max = range.min + r;

                if (range.max < 0)
                    range.max = 0;
                if (range.min > 0)
                    range.min = 0;
            }

            return range;
        }

        inline quant_param_t get_quant_param(value_range<float> range, int32_t bits = 8)
        {
            range = fixup_range(range);
            auto Q_max = (float)pow(2, bits) - 1;
            auto Q_min = bits % 2 == 0 ? 0 : -(float)pow(2, bits);
            auto scale = (range.max - range.min) / (Q_max - Q_min);
            auto bias = roundf((range.max * Q_min - range.min * Q_max) / (range.max - range.min));
            return quant_param_t { static_cast<int32_t>(bias), (float)scale };
        }

        inline size_t offset(const dims_t &strides, size_t b)
        {
            return b * strides[0];
        }

        inline size_t offset(const dims_t &strides, size_t b, size_t c)
        {
            return offset(strides, b) + c * strides[1];
        }

        inline size_t offset(const dims_t &strides, size_t b, size_t c, size_t h)
        {
            return offset(strides, b, c) + h * strides[2];
        }

        inline size_t offset(const dims_t &strides, size_t b, size_t c, size_t h, size_t w)
        {
            return offset(strides, b, c, h) + w * strides[3];
        }
    }
}
}