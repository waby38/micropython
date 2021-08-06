/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Damien P. George
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

#include "softtimer.h"
#include "usb.h"
#include "irq.h"

#define USBH_MAX_PIPES_NBR 15

enum {
    USBH_EVENT_CONNECT,
    USBH_EVENT_DISCONNECT,
    USBH_EVENT_PORT_ENABLED,
    USBH_EVENT_PORT_DISABLED,
    USBH_EVENT_URB_CHANGE,
};

void usb_host_signal_event(void *pyb_usb_host, int event);

static uint32_t Pipes[USBH_MAX_PIPES_NBR];
HCD_HandleTypeDef hhcd;

void HAL_HCD_MspInit(HCD_HandleTypeDef *hhcd) {
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    NVIC_SetPriority(OTG_FS_IRQn, IRQ_PRI_OTG_FS);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

void HAL_HCD_MspDeInit(HCD_HandleTypeDef *hhcd) {
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
}

void HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd) {
}

void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd) {
    usb_host_signal_event(hhcd->pData, USBH_EVENT_CONNECT);
}

void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd) {
    usb_host_signal_event(hhcd->pData, USBH_EVENT_DISCONNECT);
}

#if 1
// These are used only by recent HAL versions.
void HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd) {
    usb_host_signal_event(hhcd->pData, USBH_EVENT_PORT_ENABLED);
}

void HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd) {
    usb_host_signal_event(hhcd->pData, USBH_EVENT_PORT_DISABLED);
}
#endif

void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum, HCD_URBStateTypeDef urb_state) {
    usb_host_signal_event(hhcd->pData, USBH_EVENT_URB_CHANGE);
}

void PYB_USBH_Init(HCD_HandleTypeDef *hhcd) {
    for (size_t i = 0; i < USBH_MAX_PIPES_NBR; ++i) {
        Pipes[i] = 0;
    }

    hhcd->Instance = USB_OTG_FS;
    hhcd->Init.Host_channels = 11;
    hhcd->Init.dma_enable = 0;
    hhcd->Init.low_power_enable = 0;
    hhcd->Init.phy_itface = HCD_PHY_EMBEDDED;
    hhcd->Init.Sof_enable = 0;
    hhcd->Init.speed = HCD_SPEED_FULL;
    hhcd->Init.vbus_sensing_enable = 0;
    hhcd->Init.lpm_enable = 0;
    HAL_HCD_Init(hhcd);

    //USBH_LL_SetTimer(phost, HAL_HCD_GetCurrentFrame(&hhcd));
}

uint8_t USBH_AllocPipe2(uint8_t ep_addr) {
    for (uint8_t idx = 0; idx < 11; ++idx) {
        if ((Pipes[idx] & 0x8000U) == 0U) {
            Pipes[idx & 0xFU] = 0x8000U | ep_addr;
            return idx;
        }
    }
    return 0xFF;
}

/*************************************************************************/

#include "py/runtime.h"
#include "py/stream.h"

#define MICROPY_HW_USB_HOST_NUM (1)

typedef struct _pyb_usb_host_obj_t {
    mp_obj_base_t base;
    uint32_t event;
    HCD_HandleTypeDef *hhcd;
} pyb_usb_host_obj_t;

STATIC pyb_usb_host_obj_t pyb_usb_host_obj[MICROPY_HW_USB_HOST_NUM];

void usb_host_signal_event(void *pyb_usb_host, int event) {
    pyb_usb_host_obj_t *self = pyb_usb_host;
    //printf("EVENT(%d)\n", 1<<event);
    self->event |= 1 << event;
}

STATIC void pyb_usb_host_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    int id = 0;
    mp_printf(print, "USBHost(%u)", id);
}

STATIC mp_obj_t pyb_usb_host_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 0, 1, false);

    int id = (n_args == 0) ? 0 : mp_obj_get_int(args[0]);
    if (!(0 <= id && id < MICROPY_HW_USB_HOST_NUM)) {
        mp_raise_ValueError(NULL);
    }

    pyb_usb_host_obj_t *self = &pyb_usb_host_obj[id];
    self->base.type = &pyb_usb_host_type;
    self->hhcd = &hhcd;

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t pyb_usb_host_init_(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    pyb_usb_host_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    self->event = 0;
    PYB_USBH_Init(self->hhcd);
    self->hhcd->pData = self;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_usb_host_init_obj, 1, pyb_usb_host_init_);

STATIC mp_obj_t pyb_usb_host_start(mp_obj_t self_in) {
    pyb_usb_host_obj_t *self = MP_OBJ_TO_PTR(self_in);
    HAL_HCD_Start(self->hhcd);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_usb_host_start_obj, pyb_usb_host_start);

STATIC mp_obj_t pyb_usb_host_reset(mp_obj_t self_in) {
    pyb_usb_host_obj_t *self = MP_OBJ_TO_PTR(self_in);
    HAL_HCD_ResetPort(self->hhcd);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_usb_host_reset_obj, pyb_usb_host_reset);

STATIC mp_obj_t pyb_usb_host_event(mp_obj_t self_in) {
    pyb_usb_host_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint32_t irq = disable_irq();
    uint32_t event = self->event;
    self->event = 0;
    enable_irq(irq);
    return MP_OBJ_NEW_SMALL_INT(event);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_usb_host_event_obj, pyb_usb_host_event);

STATIC mp_obj_t pyb_usb_host_get_speed(mp_obj_t self_in) {
    pyb_usb_host_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int speed = HAL_HCD_GetCurrentSpeed(self->hhcd);
    #if 0
    case 0: speed = USBH_SPEED_HIGH;
    case 1: speed = USBH_SPEED_FULL;
    case 2: speed = USBH_SPEED_LOW;
    #endif
    return MP_OBJ_NEW_SMALL_INT(speed);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_usb_host_get_speed_obj, pyb_usb_host_get_speed);

// alloc_pipe(ep)
STATIC mp_obj_t pyb_usb_host_alloc_pipe(mp_obj_t self_in, mp_obj_t ep) {
    //pyb_usb_host_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t ret = USBH_AllocPipe2(mp_obj_get_int(ep));
    return MP_OBJ_NEW_SMALL_INT(ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_usb_host_alloc_pipe_obj, pyb_usb_host_alloc_pipe);

// open_pipe(pipe, ep, dev_addr, dev_speed, ep_type, mps)
// pipe is returned by alloc_pipe()
STATIC mp_obj_t pyb_usb_host_open_pipe(size_t num_args, const mp_obj_t *args) {
    pyb_usb_host_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint8_t pipe = mp_obj_get_int(args[1]);
    uint8_t ep = mp_obj_get_int(args[2]);
    uint32_t dev_addr = mp_obj_get_int(args[3]);
    uint32_t dev_speed = mp_obj_get_int(args[4]);
    uint32_t ep_type = mp_obj_get_int(args[5]);
    uint16_t mps = mp_obj_get_int(args[6]);
    HAL_HCD_HC_Init(self->hhcd, pipe, ep, dev_addr, dev_speed, ep_type, mps);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_usb_host_open_pipe_obj, 7, 7, pyb_usb_host_open_pipe);

// submit_urb(pipe, direction, ep_type, token, buf, do_ping)
STATIC mp_obj_t pyb_usb_host_submit_urb(size_t num_args, const mp_obj_t *args) {
    pyb_usb_host_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    uint8_t pipe = mp_obj_get_int(args[1]);
    uint8_t direction = mp_obj_get_int(args[2]);
    uint8_t ep_type = mp_obj_get_int(args[3]);
    uint8_t token = mp_obj_get_int(args[4]);
    mp_buffer_info_t bufinfo;
    if (args[5] == mp_const_none) {
        bufinfo.len = 0;
        bufinfo.buf = NULL;
    } else {
        mp_get_buffer_raise(args[5], &bufinfo, direction == 0 ? MP_BUFFER_READ : MP_BUFFER_WRITE);
    }
    uint8_t do_ping = mp_obj_get_int(args[6]); // high speed only
    HAL_HCD_HC_SubmitRequest(self->hhcd, pipe, direction, ep_type, token, bufinfo.buf, bufinfo.len, do_ping);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(pyb_usb_host_submit_urb_obj, 7, 7, pyb_usb_host_submit_urb);

STATIC mp_obj_t pyb_usb_host_get_urb_state(mp_obj_t self_in, mp_obj_t pipe_in) {
    pyb_usb_host_obj_t *self = MP_OBJ_TO_PTR(self_in);
    uint8_t pipe = mp_obj_get_int(pipe_in);
    int ret = HAL_HCD_HC_GetURBState(self->hhcd, pipe);
    return MP_OBJ_NEW_SMALL_INT(ret);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_usb_host_get_urb_state_obj, pyb_usb_host_get_urb_state);

STATIC const mp_rom_map_elem_t pyb_usb_host_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&pyb_usb_host_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_start), MP_ROM_PTR(&pyb_usb_host_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&pyb_usb_host_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_event), MP_ROM_PTR(&pyb_usb_host_event_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_speed), MP_ROM_PTR(&pyb_usb_host_get_speed_obj) },
    { MP_ROM_QSTR(MP_QSTR_alloc_pipe), MP_ROM_PTR(&pyb_usb_host_alloc_pipe_obj) },
    { MP_ROM_QSTR(MP_QSTR_open_pipe), MP_ROM_PTR(&pyb_usb_host_open_pipe_obj) },
    { MP_ROM_QSTR(MP_QSTR_submit_urb), MP_ROM_PTR(&pyb_usb_host_submit_urb_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_urb_state), MP_ROM_PTR(&pyb_usb_host_get_urb_state_obj) },
};
STATIC MP_DEFINE_CONST_DICT(pyb_usb_host_locals_dict, pyb_usb_host_locals_dict_table);

STATIC mp_uint_t pyb_usb_host_ioctl(mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    mp_uint_t ret;
    pyb_usb_host_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (request == MP_STREAM_POLL) {
        uintptr_t flags = arg;
        ret = 0;
        if ((flags & MP_STREAM_POLL_RD) && self->event) {
            ret |= MP_STREAM_POLL_RD;
        }
    } else {
        *errcode = MP_EINVAL;
        ret = MP_STREAM_ERROR;
    }
    return ret;
}

STATIC const mp_stream_p_t pyb_usb_host_stream_p = {
    .ioctl = pyb_usb_host_ioctl,
};

const mp_obj_type_t pyb_usb_host_type = {
    { &mp_type_type },
    .name = MP_QSTR_USB_VCP,
    .print = pyb_usb_host_print,
    .make_new = pyb_usb_host_make_new,
    //.getiter = mp_identity_getiter,
    //.iternext = mp_stream_unbuffered_iter,
    .protocol = &pyb_usb_host_stream_p,
    .locals_dict = (mp_obj_dict_t *)&pyb_usb_host_locals_dict,
};
