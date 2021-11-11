/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013, 2014 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <assert.h>

#include "py/emit.h"

#if MICROPY_ENABLE_COMPILER

#include <string.h>
#include "py/nativeglue.h"

void mp_emit_common_init(mp_emit_common_t *emit, qstr source_file) {
    mp_map_init(&emit->qstr_map, 1);
    mp_map_lookup(&emit->qstr_map, MP_OBJ_NEW_QSTR(source_file), MP_MAP_LOOKUP_ADD_IF_NOT_FOUND)->value = MP_OBJ_NEW_SMALL_INT(0);
}

void mp_emit_common_start_pass(mp_emit_common_t *emit, pass_kind_t pass) {
    emit->pass = pass;
    #if MICROPY_PERSISTENT_CODE
    if (pass == MP_PASS_STACK_SIZE) {
        emit->ct_cur_obj_base = emit->ct_cur_obj;
        emit->ct_cur_raw_code_base = emit->ct_cur_raw_code;
    } else if (pass > MP_PASS_STACK_SIZE) {
        emit->ct_cur_obj = emit->ct_cur_obj_base;
        emit->ct_cur_raw_code = emit->ct_cur_raw_code_base;
    }
    //printf("ct_cur_obj = %d\n", (int)emit->ct_cur_obj);
    //printf("ct_cur_raw_code = %d\n", (int)emit->ct_cur_raw_code);
    #endif
}

mp_uint_t mp_emit_common_qstr_map_to_index(mp_emit_common_t *emit, qstr qst) {
    #if MICROPY_PERSISTENT_CODE
    #if 0
    size_t i;
    for (i = 0; i < emit->qstr_table_used; ++i) {
        if (emit->qstr_table[i] == qst) {
            break;
        }
    }
    if (i == emit->qstr_table_used) {
        if (emit->qstr_table_used >= emit->qstr_table_alloc) {
            size_t new_alloc = emit->qstr_table_alloc + 4;
            emit->qstr_table = m_renew(qstr_short_t, emit->qstr_table, emit->qstr_table_alloc, new_alloc);
            emit->qstr_table_alloc = new_alloc;
        }
        emit->qstr_table[i] = qst;
        //printf("pass=%d add %d=%s as %d\n", (int)emit->pass, (int)qst, qstr_str(qst), (int)i);
        emit->qstr_table_used += 1;
        if (emit->pass > MP_PASS_SCOPE && emit->qstr_table_used > emit->qstr_table_max) {
            emit->qstr_table_max = emit->qstr_table_used;
        }
    }
    return i;
    #else
    //printf("pass=%d lookup %d=%s\n", (int)emit->pass, (int)qst, qstr_str(qst));
    #define QSTR_LAST_STATIC MP_QSTR_zip
    if (qst <= QSTR_LAST_STATIC) {
        return qst << 1;
    }
    mp_map_elem_t *elem = mp_map_lookup(&emit->qstr_map, MP_OBJ_NEW_QSTR(qst), MP_MAP_LOOKUP_ADD_IF_NOT_FOUND);
    if (elem->value == MP_OBJ_NULL) {
        assert(emit->pass == MP_PASS_SCOPE);
        elem->value = MP_OBJ_NEW_SMALL_INT(emit->qstr_map.used - 1);
    }
    return MP_OBJ_SMALL_INT_VALUE(elem->value) << 1 | 1;
    #endif
    #else
    return qst;
    #endif
}

void mp_emit_common_use_qstr(mp_emit_common_t *emit, qstr qst) {
    mp_emit_common_qstr_map_to_index(emit, qst);
}

size_t mp_emit_common_alloc_const_obj(mp_emit_common_t *emit, mp_obj_t obj) {
    if (emit->pass == MP_PASS_EMIT) {
        emit->const_table[emit->ct_cur_obj] = (mp_uint_t)obj;
    }
    return emit->ct_cur_obj++;
}

size_t mp_emit_common_alloc_const_raw_code(mp_emit_common_t *emit, mp_raw_code_t *rc) {
    // TODO this pass should be the pass of the caller emit, not emit_bc
    if (emit->pass == MP_PASS_EMIT) {
        emit->const_table[emit->ct_num_obj + emit->ct_cur_raw_code] = (mp_uint_t)(uintptr_t)rc;
    }
    return emit->ct_num_obj + emit->ct_cur_raw_code++;
}

void mp_emit_common_finalise(mp_emit_common_t *emit, bool has_native_code) {
    #if MICROPY_PERSISTENT_CODE
    emit->ct_cur_obj += has_native_code;
    emit->ct_num_obj = emit->ct_cur_obj;
    emit->const_table = m_new0(mp_uint_t, emit->ct_cur_obj + emit->ct_cur_raw_code);
    emit->ct_cur_obj = has_native_code; // reserve slot 0 for &mp_fun_table
    emit->ct_cur_raw_code = 0;
    #endif
    if (has_native_code) {
        // Store mp_fun_table pointer just after qstrs
        emit->const_table[0] = (mp_uint_t)(uintptr_t)&mp_fun_table;
    }
}

mp_compiled_module_t *mp_emit_common_create_compiled_module(mp_emit_common_t *emit, bool has_native_code) {
    assert(emit->ct_cur_obj == emit->ct_num_obj);

    size_t qstr_map_used = emit->qstr_map.used;
    size_t nq = (qstr_map_used * sizeof(qstr_short_t) + sizeof(mp_uint_t) - 1) / sizeof(mp_uint_t);
    size_t nc = emit->ct_cur_obj + emit->ct_cur_raw_code;
    mp_compiled_module_t *cm = m_new_obj_var(mp_compiled_module_t, mp_uint_t, nq + nc);
    cm->const_table = (mp_obj_t *)&((mp_uint_t *)&cm->qstr_table[0])[nq];
    for (size_t i = 0; i < emit->qstr_map.alloc; ++i) {
        if (mp_map_slot_is_filled(&emit->qstr_map, i)) {
            size_t idx = MP_OBJ_SMALL_INT_VALUE(emit->qstr_map.table[i].value);
            qstr qst = MP_OBJ_QSTR_VALUE(emit->qstr_map.table[i].key);
            cm->qstr_table[idx] = qst;
        }
    }
    memcpy(cm->const_table, emit->const_table, nc * sizeof(mp_uint_t));
    #if 0
    printf("table %d\n", (int)nc);
    printf("  %p\n", cm->const_table[0]);
    printf("  %p\n", cm->const_table[1]);
    printf("  %p\n", cm->const_table[2]);
    #endif

    #if MICROPY_PERSISTENT_CODE_SAVE
    cm->n_qstr = emit->qstr_map.used;
    cm->n_obj = emit->ct_cur_obj;
    cm->n_raw_code = emit->ct_cur_raw_code;
    #endif

    return cm;
}

void mp_emit_common_get_id_for_modification(scope_t *scope, qstr qst) {
    // name adding/lookup
    id_info_t *id = scope_find_or_add_id(scope, qst, ID_INFO_KIND_GLOBAL_IMPLICIT);
    if (SCOPE_IS_FUNC_LIKE(scope->kind) && id->kind == ID_INFO_KIND_GLOBAL_IMPLICIT) {
        // rebind as a local variable
        id->kind = ID_INFO_KIND_LOCAL;
    }
}

void mp_emit_common_id_op(emit_t *emit, const mp_emit_method_table_id_ops_t *emit_method_table, scope_t *scope, qstr qst) {
    // assumes pass is greater than 1, ie that all identifiers are defined in the scope

    id_info_t *id = scope_find(scope, qst);
    assert(id != NULL);

    // call the emit backend with the correct code
    if (id->kind == ID_INFO_KIND_GLOBAL_IMPLICIT) {
        emit_method_table->global(emit, qst, MP_EMIT_IDOP_GLOBAL_NAME);
    } else if (id->kind == ID_INFO_KIND_GLOBAL_EXPLICIT) {
        emit_method_table->global(emit, qst, MP_EMIT_IDOP_GLOBAL_GLOBAL);
    } else if (id->kind == ID_INFO_KIND_LOCAL) {
        emit_method_table->local(emit, qst, id->local_num, MP_EMIT_IDOP_LOCAL_FAST);
    } else {
        assert(id->kind == ID_INFO_KIND_CELL || id->kind == ID_INFO_KIND_FREE);
        emit_method_table->local(emit, qst, id->local_num, MP_EMIT_IDOP_LOCAL_DEREF);
    }
}

#endif // MICROPY_ENABLE_COMPILER
