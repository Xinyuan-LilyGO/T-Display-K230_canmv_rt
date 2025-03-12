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
#include <nncase/runtime/runtime_module.h>
#include <nncase/runtime/runtime_tensor.h>

BEGIN_NS_NNCASE_RT_K230

#define K230_SEC_TEXT ".text"
#define K230_SEC_DSP_TEXT ".dsp.text"
#define K230_DSP_BASEMENT 2

NNCASE_INLINE_VAR constexpr module_kind_t k230_module_type = to_module_kind("k230");

NNCASE_MODULES_K230_API result<std::unique_ptr<runtime_module>> create_k230_runtime_module();
NNCASE_MODULES_K230_API result<std::vector<std::pair<std::string, runtime_module::custom_call_type>>> create_k230_custom_calls();

END_NS_NNCASE_RT_K230
