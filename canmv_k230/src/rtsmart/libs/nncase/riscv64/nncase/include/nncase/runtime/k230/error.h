/* Copyright 2019-2020 Canaan Inc.
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
#include <nncase/runtime/error.h>

BEGIN_NS_NNCASE_RT_K230

enum class nncase_k230_errc
{
    k230_illegal_instruction = 0x01
};

NNCASE_MODULES_K230_API const std::error_category &nncase_k230_category() noexcept;
NNCASE_MODULES_K230_API std::error_condition make_error_condition(nncase_k230_errc code);

END_NS_NNCASE_RT_K230

namespace std
{
template <>
struct is_error_condition_enum<nncase::runtime::k230::nncase_k230_errc> : true_type
{
};
}
