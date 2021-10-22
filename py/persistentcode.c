/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013-2020 Damien P. George
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "py/reader.h"
#include "py/nativeglue.h"
#include "py/persistentcode.h"
#include "py/bc0.h"
#include "py/objstr.h"
#include "py/mpthread.h"

#if MICROPY_PERSISTENT_CODE_LOAD || MICROPY_PERSISTENT_CODE_SAVE

#include "py/smallint.h"

#define QSTR_LAST_STATIC MP_QSTR_zip

#if MICROPY_DYNAMIC_COMPILER
#define MPY_FEATURE_ARCH_DYNAMIC mp_dynamic_compiler.native_arch
#else
#define MPY_FEATURE_ARCH_DYNAMIC MPY_FEATURE_ARCH
#endif

#if MICROPY_PERSISTENT_CODE_LOAD || (MICROPY_PERSISTENT_CODE_SAVE && !MICROPY_DYNAMIC_COMPILER)
// The bytecode will depend on the number of bits in a small-int, and
// this function computes that (could make it a fixed constant, but it
// would need to be defined in mpconfigport.h).
STATIC int mp_small_int_bits(void) {
    mp_int_t i = MP_SMALL_INT_MAX;
    int n = 1;
    while (i != 0) {
        i >>= 1;
        ++n;
    }
    return n;
}
#endif

typedef struct _bytecode_prelude_t {
    uint n_state;
    uint n_exc_stack;
    uint scope_flags;
    uint n_pos_args;
    uint n_kwonly_args;
    uint n_def_pos_args;
    uint code_info_size;
} bytecode_prelude_t;

#endif // MICROPY_PERSISTENT_CODE_LOAD || MICROPY_PERSISTENT_CODE_SAVE

#if MICROPY_PERSISTENT_CODE_LOAD

#include "py/parsenum.h"

STATIC int read_byte(mp_reader_t *reader);
STATIC size_t read_uint(mp_reader_t *reader, byte **out);

#if MICROPY_EMIT_MACHINE_CODE

typedef struct _reloc_info_t {
    mp_reader_t *reader;
    mp_uint_t *const_table;
} reloc_info_t;

#if 0
#if MICROPY_EMIT_THUMB
STATIC void asm_thumb_rewrite_mov(uint8_t *pc, uint16_t val) {
    // high part
    *(uint16_t *)pc = (*(uint16_t *)pc & 0xfbf0) | (val >> 1 & 0x0400) | (val >> 12);
    // low part
    *(uint16_t *)(pc + 2) = (*(uint16_t *)(pc + 2) & 0x0f00) | (val << 4 & 0x7000) | (val & 0x00ff);

}
#endif

STATIC void arch_link_qstr(uint8_t *pc, bool is_obj, qstr qst) {
    mp_uint_t val = qst;
    if (is_obj) {
        val = (mp_uint_t)MP_OBJ_NEW_QSTR(qst);
    }
    #if MICROPY_EMIT_X86 || MICROPY_EMIT_X64 || MICROPY_EMIT_ARM || MICROPY_EMIT_XTENSA || MICROPY_EMIT_XTENSAWIN
    pc[0] = val & 0xff;
    pc[1] = (val >> 8) & 0xff;
    pc[2] = (val >> 16) & 0xff;
    pc[3] = (val >> 24) & 0xff;
    #elif MICROPY_EMIT_THUMB
    if (is_obj) {
        // qstr object, movw and movt
        asm_thumb_rewrite_mov(pc, val); // movw
        asm_thumb_rewrite_mov(pc + 4, val >> 16); // movt
    } else {
        // qstr number, movw instruction
        asm_thumb_rewrite_mov(pc, val); // movw
    }
    #endif
}
#endif

void mp_native_relocate(void *ri_in, uint8_t *text, uintptr_t reloc_text) {
    // Relocate native code
    reloc_info_t *ri = ri_in;
    uint8_t op;
    uintptr_t *addr_to_adjust = NULL;
    while ((op = read_byte(ri->reader)) != 0xff) {
        if (op & 1) {
            // Point to new location to make adjustments
            size_t addr = read_uint(ri->reader, NULL);
            if ((addr & 1) == 0) {
                // Point to somewhere in text
                addr_to_adjust = &((uintptr_t *)text)[addr >> 1];
            } else {
                // Point to somewhere in rodata
                addr_to_adjust = &((uintptr_t *)ri->const_table[1])[addr >> 1];
            }
        }
        op >>= 1;
        uintptr_t dest;
        size_t n = 1;
        if (op <= 5) {
            if (op & 1) {
                // Read in number of adjustments to make
                n = read_uint(ri->reader, NULL);
            }
            op >>= 1;
            if (op == 0) {
                // Destination is text
                dest = reloc_text;
            } else {
                // Destination is rodata (op=1) or bss (op=1 if no rodata, else op=2)
                dest = ri->const_table[op];
            }
        } else if (op == 6) {
            // Destination is mp_fun_table itself
            dest = (uintptr_t)&mp_fun_table;
        } else {
            // Destination is an entry in mp_fun_table
            dest = ((uintptr_t *)&mp_fun_table)[op - 7];
        }
        while (n--) {
            *addr_to_adjust++ += dest;
        }
    }
}

#endif

STATIC int read_byte(mp_reader_t *reader) {
    return reader->readbyte(reader->data);
}

STATIC void read_bytes(mp_reader_t *reader, byte *buf, size_t len) {
    while (len-- > 0) {
        *buf++ = reader->readbyte(reader->data);
    }
}

STATIC size_t read_uint(mp_reader_t *reader, byte **out) {
    size_t unum = 0;
    for (;;) {
        byte b = reader->readbyte(reader->data);
        if (out != NULL) {
            **out = b;
            ++*out;
        }
        unum = (unum << 7) | (b & 0x7f);
        if ((b & 0x80) == 0) {
            break;
        }
    }
    return unum;
}

STATIC qstr load_qstr(mp_reader_t *reader) {
    size_t len = read_uint(reader, NULL);
    if (len & 1) {
        // static qstr
        return len >> 1;
    }
    len >>= 1;
    char *str = m_new(char, len);
    read_bytes(reader, (byte *)str, len);
    qstr qst = qstr_from_strn(str, len);
    m_del(char, str, len);
    return qst;
}

STATIC mp_obj_t load_obj(mp_reader_t *reader) {
    byte obj_type = read_byte(reader);
    if (obj_type == 't') {
        return MP_OBJ_FROM_PTR(&mp_fun_table);
    } else if (obj_type == 'e') {
        return MP_OBJ_FROM_PTR(&mp_const_ellipsis_obj);
    } else {
        size_t len = read_uint(reader, NULL);
        vstr_t vstr;
        vstr_init_len(&vstr, len);
        read_bytes(reader, (byte *)vstr.buf, len);
        if (obj_type == 's' || obj_type == 'b') {
            return mp_obj_new_str_from_vstr(obj_type == 's' ? &mp_type_str : &mp_type_bytes, &vstr);
        } else if (obj_type == 'i') {
            return mp_parse_num_integer(vstr.buf, vstr.len, 10, NULL);
        } else {
            assert(obj_type == 'f' || obj_type == 'c');
            return mp_parse_num_decimal(vstr.buf, vstr.len, obj_type == 'c', false, NULL);
        }
    }
}

STATIC mp_raw_code_t *load_raw_code(mp_reader_t *reader) {
    // Load function kind and data length
    size_t kind_len = read_uint(reader, NULL);
    int kind = (kind_len & 3) + MP_CODE_BYTECODE;
    size_t fun_data_len = kind_len >> 2;

    #if !MICROPY_EMIT_MACHINE_CODE
    if (kind != MP_CODE_BYTECODE) {
        mp_raise_ValueError(MP_ERROR_TEXT("incompatible .mpy file"));
    }
    #endif

    uint8_t *fun_data = NULL;
    #if MICROPY_EMIT_MACHINE_CODE
    size_t prelude_offset = 0;
    //mp_uint_t type_sig = 0;
    //size_t n_qstr_link = 0;
    #endif

    if (kind == MP_CODE_BYTECODE) {
        // Allocate memory for the bytecode
        fun_data = m_new(uint8_t, fun_data_len);
        // Load bytecode
        read_bytes(reader, fun_data, fun_data_len);

    #if MICROPY_EMIT_MACHINE_CODE
    } else {
        // Allocate memory for native data and load it
        size_t fun_alloc;
        MP_PLAT_ALLOC_EXEC(fun_data_len, (void **)&fun_data, &fun_alloc);
        read_bytes(reader, fun_data, fun_data_len);

        #if 0
        if (kind == MP_CODE_NATIVE_PY || kind == MP_CODE_NATIVE_VIPER) {
            // Parse qstr link table and link native code
            n_qstr_link = read_uint(reader, NULL);
            for (size_t i = 0; i < n_qstr_link; ++i) {
                size_t off = read_uint(reader, NULL);
                qstr qst = load_qstr(reader, qw);
                uint8_t *dest = fun_data + (off >> 2);
                if ((off & 3) == 0) {
                    // Generic 16-bit link
                    dest[0] = qst & 0xff;
                    dest[1] = (qst >> 8) & 0xff;
                } else if ((off & 3) == 3) {
                    // Generic, aligned qstr-object link
                    *(mp_obj_t *)dest = MP_OBJ_NEW_QSTR(qst);
                } else {
                    // Architecture-specific link
                    arch_link_qstr(dest, (off & 3) == 2, qst);
                }
            }
        }
        #endif

        if (kind == MP_CODE_NATIVE_PY) {
            // Extract prelude for later use
            prelude_offset = read_uint(reader, NULL);
            (void)prelude_offset;
            #if 0
            const byte *ip = fun_data + prelude_offset;
            byte *ip_info = extract_prelude(&ip, &prelude);
            // Load qstrs in prelude
            load_prelude_qstrs(reader, qw, ip_info);
            #endif
        } else {
            #if 0
            // Load basic scope info for viper and asm
            prelude.scope_flags = read_uint(reader, NULL);
            prelude.n_pos_args = 0;
            prelude.n_kwonly_args = 0;
            if (kind == MP_CODE_NATIVE_ASM) {
                prelude.n_pos_args = read_uint(reader, NULL);
                type_sig = read_uint(reader, NULL);
            }
            #endif
        }
    }
    #endif

    #if 0 && MICROPY_EMIT_MACHINE_CODE
    if (kind != MP_CODE_NATIVE_ASM) {
        // Load constant table for bytecode, native and viper

        if (kind != MP_CODE_BYTECODE) {
            ++n_alloc; // additional entry for mp_fun_table
            if (prelude.scope_flags & MP_SCOPE_FLAG_VIPERRODATA) {
                ++n_alloc; // additional entry for rodata
            }
            if (prelude.scope_flags & MP_SCOPE_FLAG_VIPERBSS) {
                ++n_alloc; // additional entry for BSS
            }
        }

        if (kind != MP_CODE_BYTECODE) {
            // Populate mp_fun_table entry
            *ct++ = (mp_uint_t)(uintptr_t)&mp_fun_table;

            // Allocate and load rodata if needed
            if (prelude.scope_flags & MP_SCOPE_FLAG_VIPERRODATA) {
                size_t size = read_uint(reader, NULL);
                uint8_t *rodata = m_new(uint8_t, size);
                read_bytes(reader, rodata, size);
                *ct++ = (uintptr_t)rodata;
            }

            // Allocate BSS if needed
            if (prelude.scope_flags & MP_SCOPE_FLAG_VIPERBSS) {
                size_t size = read_uint(reader, NULL);
                uint8_t *bss = m_new0(uint8_t, size);
                *ct++ = (uintptr_t)bss;
            }
        }
    }
    #endif

    // Create raw_code and return it
    mp_raw_code_t *rc = mp_emit_glue_new_raw_code();
    if (kind == MP_CODE_BYTECODE) {
        const byte * ip = fun_data;
        MP_BC_PRELUDE_SIG_DECODE(ip);
        // Assign bytecode to raw code object
        mp_emit_glue_assign_bytecode(rc, fun_data,
            #if MICROPY_PERSISTENT_CODE_SAVE || MICROPY_DEBUG_PRINTERS
            fun_data_len,
            #endif
            #if MICROPY_PERSISTENT_CODE_SAVE
            n_obj, n_raw_code,
            #endif
            scope_flags);

    #if MICROPY_EMIT_MACHINE_CODE
    } else {
        // Relocate and commit code to executable address space
        #if 0
        reloc_info_t ri = {reader, const_table};
        #if defined(MP_PLAT_COMMIT_EXEC)
        void *opt_ri = (prelude.scope_flags & MP_SCOPE_FLAG_VIPERRELOC) ? &ri : NULL;
        fun_data = MP_PLAT_COMMIT_EXEC(fun_data, fun_data_len, opt_ri);
        #else
        if (prelude.scope_flags & MP_SCOPE_FLAG_VIPERRELOC) {
            #if MICROPY_PERSISTENT_CODE_TRACK_RELOC_CODE
            // If native code needs relocations then it's not guaranteed that a pointer to
            // the head of `buf` (containing the machine code) will be retained for the GC
            // to trace.  This is because native functions can start inside `buf` and so
            // it's possible that the only GC-reachable pointers are pointers inside `buf`.
            // So put this `buf` on a list of reachable root pointers.
            if (MP_STATE_PORT(track_reloc_code_list) == MP_OBJ_NULL) {
                MP_STATE_PORT(track_reloc_code_list) = mp_obj_new_list(0, NULL);
            }
            mp_obj_list_append(MP_STATE_PORT(track_reloc_code_list), MP_OBJ_FROM_PTR(fun_data));
            #endif
            // Do the relocations.
            mp_native_relocate(&ri, fun_data, (uintptr_t)fun_data);
        }
        #endif
        #endif

        // Assign native code to raw code object
        mp_emit_glue_assign_native(rc, kind,
            fun_data, fun_data_len,
            #if MICROPY_PERSISTENT_CODE_SAVE
            prelude_offset,
            0, NULL,
            #endif
            0, 0, 0
            //prelude.n_pos_args, prelude.scope_flags, type_sig
            );
    #endif
    }
    return rc;
}

mp_compiled_module_t *mp_raw_code_load(mp_reader_t *reader) {
    byte header[4];
    read_bytes(reader, header, sizeof(header));
    if (header[0] != 'M'
        || header[1] != MPY_VERSION
        || MPY_FEATURE_DECODE_FLAGS(header[2]) != MPY_FEATURE_FLAGS
        || header[3] > mp_small_int_bits()) {
        mp_raise_ValueError(MP_ERROR_TEXT("incompatible .mpy file"));
    }
    if (MPY_FEATURE_DECODE_ARCH(header[2]) != MP_NATIVE_ARCH_NONE) {
        byte arch = MPY_FEATURE_DECODE_ARCH(header[2]);
        if (!MPY_FEATURE_ARCH_TEST(arch)) {
            mp_raise_ValueError(MP_ERROR_TEXT("incompatible .mpy arch"));
        }
    }

    size_t n_qstr = read_uint(reader, NULL);
    size_t n_obj = read_uint(reader, NULL);
    size_t n_raw_code = read_uint(reader, NULL);
    mp_compiled_module_t *cm = m_new_obj_var(mp_compiled_module_t, mp_uint_t, n_qstr + n_obj + n_raw_code); // TODO properly
    cm->const_table = (void *)&cm->qstr_table[n_qstr];

    #if MICROPY_PERSISTENT_CODE_SAVE
    cm->has_native = false;
    cm->n_qstr = n_qstr;
    cm->n_obj = n_obj;
    cm->n_raw_code = n_raw_code;
    #endif

    // Load qstrs.
    for (size_t i = 0; i < n_qstr; ++i) {
        cm->qstr_table[i] = load_qstr(reader);
    }

    // Load constant objects.
    for (size_t i = 0; i < n_obj; ++i) {
        cm->const_table[i] = load_obj(reader);
    }

    // Load top-level module.
    cm->rc = load_raw_code(reader);

    // Load raw code children.
    for (size_t i = 0; i < n_raw_code; ++i) {
        cm->const_table[n_obj + i] = (mp_obj_t)(uintptr_t)load_raw_code(reader);
    }

    reader->close(reader->data);

    return cm;
}

mp_compiled_module_t *mp_raw_code_load_mem(const byte *buf, size_t len) {
    mp_reader_t reader;
    mp_reader_new_mem(&reader, buf, len, 0);
    return mp_raw_code_load(&reader);
}

#if MICROPY_HAS_FILE_READER

mp_compiled_module_t *mp_raw_code_load_file(const char *filename) {
    mp_reader_t reader;
    mp_reader_new_file(&reader, filename);
    return mp_raw_code_load(&reader);
}

#endif // MICROPY_HAS_FILE_READER

#endif // MICROPY_PERSISTENT_CODE_LOAD

#if MICROPY_PERSISTENT_CODE_SAVE

#include "py/objstr.h"

STATIC void mp_print_bytes(mp_print_t *print, const byte *data, size_t len) {
    print->print_strn(print->data, (const char *)data, len);
}

#define BYTES_FOR_INT ((MP_BYTES_PER_OBJ_WORD * 8 + 6) / 7)
STATIC void mp_print_uint(mp_print_t *print, size_t n) {
    byte buf[BYTES_FOR_INT];
    byte *p = buf + sizeof(buf);
    *--p = n & 0x7f;
    n >>= 7;
    for (; n != 0; n >>= 7) {
        *--p = 0x80 | (n & 0x7f);
    }
    print->print_strn(print->data, (char *)p, buf + sizeof(buf) - p);
}

STATIC void save_qstr(mp_print_t *print, qstr qst) {
    if (qst <= QSTR_LAST_STATIC) {
        // encode static qstr
        mp_print_uint(print, qst << 1 | 1);
        return;
    }
    size_t len;
    const byte *str = qstr_data(qst, &len);
    mp_print_uint(print, len << 1);
    mp_print_bytes(print, str, len);
}

STATIC void save_obj(mp_print_t *print, mp_obj_t o) {
    if (o == MP_OBJ_FROM_PTR(&mp_fun_table)) {
        byte obj_type = 't';
        mp_print_bytes(print, &obj_type, 1);
    } else if (mp_obj_is_str_or_bytes(o)) {
        byte obj_type;
        if (mp_obj_is_str(o)) {
            obj_type = 's';
        } else {
            obj_type = 'b';
        }
        size_t len;
        const char *str = mp_obj_str_get_data(o, &len);
        mp_print_bytes(print, &obj_type, 1);
        mp_print_uint(print, len);
        mp_print_bytes(print, (const byte *)str, len);
    } else if (MP_OBJ_TO_PTR(o) == &mp_const_ellipsis_obj) {
        byte obj_type = 'e';
        mp_print_bytes(print, &obj_type, 1);
    } else {
        // we save numbers using a simplistic text representation
        // TODO could be improved
        byte obj_type;
        if (mp_obj_is_type(o, &mp_type_int)) {
            obj_type = 'i';
        #if MICROPY_PY_BUILTINS_COMPLEX
        } else if (mp_obj_is_type(o, &mp_type_complex)) {
            obj_type = 'c';
        #endif
        } else {
            assert(mp_obj_is_float(o));
            obj_type = 'f';
        }
        vstr_t vstr;
        mp_print_t pr;
        vstr_init_print(&vstr, 10, &pr);
        mp_obj_print_helper(&pr, o, PRINT_REPR);
        mp_print_bytes(print, &obj_type, 1);
        mp_print_uint(print, vstr.len);
        mp_print_bytes(print, (const byte *)vstr.buf, vstr.len);
        vstr_clear(&vstr);
    }
}

STATIC void save_raw_code(mp_print_t *print, const mp_raw_code_t *rc) {
    // Save function kind and data length
    mp_print_uint(print, (rc->fun_data_len << 2) | (rc->kind - MP_CODE_BYTECODE));

    // Save function code.
    mp_print_bytes(print, rc->fun_data, rc->fun_data_len);

    #if MICROPY_EMIT_MACHINE_CODE
    if (rc->kind == MP_CODE_NATIVE_PY) {
        // Save prelude size
        mp_print_uint(print, rc->prelude_offset);
    }

    assert(rc->n_qstr == 0);
    #if 0
    if (rc->kind == MP_CODE_NATIVE_PY || rc->kind == MP_CODE_NATIVE_VIPER) {
        // Save qstr link table for native code
        mp_print_uint(print, rc->n_qstr);
        for (size_t i = 0; i < rc->n_qstr; ++i) {
            mp_print_uint(print, rc->qstr_link[i].off);
            save_qstr(print, rc->qstr_link[i].qst);
        }
    }

    if (rc->kind == MP_CODE_NATIVE_VIPER || rc->kind == MP_CODE_NATIVE_ASM) {
        // Save basic scope info for viper and asm
        mp_print_uint(print, rc->scope_flags & MP_SCOPE_FLAG_ALL_SIG);
        if (rc->kind == MP_CODE_NATIVE_ASM) {
            mp_print_uint(print, rc->n_pos_args);
            mp_print_uint(print, rc->type_sig);
        }
    }
    #endif
    #endif
}

void mp_raw_code_save(mp_compiled_module_t *cm, mp_print_t *print) {
    // header contains:
    //  byte  'M'
    //  byte  version
    //  byte  feature flags
    //  byte  number of bits in a small int
    //  uint  size of qstr window
    byte header[4] = {
        'M',
        MPY_VERSION,
        MPY_FEATURE_ENCODE_FLAGS(MPY_FEATURE_FLAGS_DYNAMIC),
        #if MICROPY_DYNAMIC_COMPILER
        mp_dynamic_compiler.small_int_bits,
        #else
        mp_small_int_bits(),
        #endif
    };
    if (cm->has_native) {
        header[2] |= MPY_FEATURE_ENCODE_ARCH(MPY_FEATURE_ARCH_DYNAMIC);
    }
    mp_print_bytes(print, header, sizeof(header));

    // Number of entries in constant table.
    mp_print_uint(print, cm->n_qstr);
    mp_print_uint(print, cm->n_obj);
    mp_print_uint(print, cm->n_raw_code);

    // Save function argument names (initial entries in const_table)
    // (viper has n_pos_args=n_kwonly_args=0 so doesn't save any qstrs here)
    for (size_t i = 0; i < cm->n_qstr; ++i) {
        save_qstr(print, cm->qstr_table[i]);
    }

    // Save constant objects.
    for (size_t i = 0; i < cm->n_obj; ++i) {
        save_obj(print, (mp_obj_t)cm->const_table[i]);
    }

    save_raw_code(print, cm->rc);

    // Save raw code children.
    for (size_t i = 0; i < cm->n_raw_code; ++i) {
        save_raw_code(print, (void *)cm->const_table[cm->n_obj + i]);
    }
}

#if MICROPY_PERSISTENT_CODE_SAVE_FILE

#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>

STATIC void fd_print_strn(void *env, const char *str, size_t len) {
    int fd = (intptr_t)env;
    MP_THREAD_GIL_EXIT();
    ssize_t ret = write(fd, str, len);
    MP_THREAD_GIL_ENTER();
    (void)ret;
}

void mp_raw_code_save_file(mp_compiled_module_t *cm, const char *filename) {
    MP_THREAD_GIL_EXIT();
    int fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    MP_THREAD_GIL_ENTER();
    mp_print_t fd_print = {(void *)(intptr_t)fd, fd_print_strn};
    mp_raw_code_save(cm, &fd_print);
    MP_THREAD_GIL_EXIT();
    close(fd);
    MP_THREAD_GIL_ENTER();
}

#endif // MICROPY_PERSISTENT_CODE_SAVE_FILE

#endif // MICROPY_PERSISTENT_CODE_SAVE
