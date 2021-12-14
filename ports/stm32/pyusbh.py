import os
import machine, pyb, time, select, uasyncio as asyncio, struct
import uctypes, binascii
from collections import namedtuple

EVENT_CONNECTED = 1
EVENT_PORT_ENABLED = 4

USBH_EP_CONTROL = 0
USBH_EP_INTERRUPT = 3
USB_REQ_RECIPIENT_DEVICE = 0
USB_REQ_TYPE_STANDARD = 0
USB_REQ_TYPE_CLASS = 0x20
USB_DESC_DEVICE = 0x0100
USB_DESC_CONFIGURATION = 0x0200
USB_DESC_STRING = 0x0300
USB_H2D = 0x00
USB_D2H = 0x80
USB_REQ_GET_STATUS = 0x00
USB_REQ_CLEAR_FEATURE = 0x01
USB_REQ_SET_FEATURE = 0x03
USB_REQ_SET_ADDRESS = 0x05
USB_REQ_GET_DESCRIPTOR = 0x06
USB_REQ_SET_CONFIGURATION = 0x09
USBH_PID_SETUP = 0
USBH_PID_DATA = 1

USB_SPEED_HIGH = 0
USB_SPEED_FULL = 1
USB_SPEED_LOW = 2

# return values for get_urb_state
USBH_URB_IDLE = 0
USBH_URB_DONE = 1
USBH_URB_NOTREADY = 2
USBH_URB_NYET = 3
USBH_URB_ERROR = 4
USBH_URB_STALL = 5

# type of descriptor
USB_DESC_TYPE_DEVICE = 1
USB_DESC_TYPE_CONFIGURATION = 2
USB_DESC_TYPE_STRING = 3
USB_DESC_TYPE_INTERFACE = 4
USB_DESC_TYPE_ENDPOINT = 5

_DFU_DNLOAD = 1
_DFU_UPLOAD = 2
_DFU_GETSTATUS = 3
_DFU_CLRSTATUS = 4
_DFU_ABORT = 6

_DFU_STATE_DFU_IDLE = 0x02
_DFU_STATE_DFU_DOWNLOAD_SYNC = 0x03
_DFU_STATE_DFU_DOWNLOAD_BUSY = 0x04
_DFU_STATE_DFU_DOWNLOAD_IDLE = 0x05
_DFU_STATE_DFU_MANIFEST_SYNC = 0x06
_DFU_STATE_DFU_UPLOAD_IDLE = 0x09

HUB_FEATURE_PORT_RESET = 4
HUB_FEATURE_PORT_POWER = 8
HUB_FEATURE_PORT_CONNECTION_CHANGE = 16
HUB_FEATURE_PORT_RESET_CHANGE = 20


class USBHost:
    def __init__(self, usbh):
        self.usbh = usbh
        self._cur_addr = 0

    def alloc_addr(self):
        self._cur_addr += 1
        return self._cur_addr

    async def wait_event(self, event):
        while True:
            yield asyncio.core._io_queue.queue_read(self.usbh)
            ev = self.usbh.event()
            # print('event', ev)
            if ev & event:
                return ev

    def ctl_send_setup(self, buf, pipe):
        self.usbh.submit_urb(pipe, 0, USBH_EP_CONTROL, USBH_PID_SETUP, buf, 0)

    def ctl_send_data(self, buf, pipe, do_ping):
        # TODO need to know the speed to disable pings
        # if self.dev_speed != 0:  # USBH_SPEED_HIGH
        #    do_ping = False
        do_ping = False
        self.usbh.submit_urb(pipe, 0, USBH_EP_CONTROL, USBH_PID_DATA, buf, do_ping)

    def ctl_receive_data(self, buf, pipe):
        self.usbh.submit_urb(pipe, 1, USBH_EP_CONTROL, USBH_PID_DATA, buf, 0)

    def int_in_data(self, pipe, buf):
        self.usbh.submit_urb(pipe, 1, USBH_EP_INTERRUPT, USBH_PID_DATA, buf, 0)

    async def wait_urb(self, pipe, allowed_s=-1):
        while True:
            yield asyncio.core._io_queue.queue_read(self.usbh)
            ev = self.usbh.event()  # TODO: store event to self.event
            s = self.usbh.get_urb_state(pipe)
            # print('wait_urb', pipe, ev, s)
            if (
                s == USBH_URB_DONE
                or s == USBH_URB_ERROR
                or s == allowed_s
                or s == USBH_URB_NOTREADY
            ):  # or s == USBH_URB_STALL:
                return s

    async def ctl_req(self, pipe_out, pipe_in, cmd, payload):
        self.ctl_send_setup(cmd, pipe_out)

        urb_status = await self.wait_urb(pipe_out)
        if urb_status == USBH_URB_NOTREADY:
            print("setup retry")
            urb_status = await self.wait_urb(pipe_out)

        if urb_status in (USBH_URB_ERROR, USBH_URB_NOTREADY):
            raise Exception(urb_status)

        assert urb_status == USBH_URB_DONE

        direction = cmd[0] & 0x80

        # check if there is a data stage
        if payload:
            if direction == USB_D2H:
                # Data Direction is IN, perform data-in
                self.ctl_receive_data(payload, pipe_in)
                urb_status = await self.wait_urb(pipe_in)
                if urb_status == USBH_URB_DONE:
                    pass
                else:
                    assert 0, urb_status

            else:
                # Data Direction is OUT, perform data-out
                while True:
                    self.ctl_send_data(payload, pipe_out, 1)
                    urb_status = await self.wait_urb(pipe_out, USBH_URB_STALL)
                    if urb_status == USBH_URB_DONE:
                        break
                    elif urb_status == USBH_URB_NOTREADY:
                        # NACK received from device; retry
                        pass
                    else:
                        assert 0, urb_status

        if direction == USB_D2H:
            # Data Direction is IN, perform ctrl-status-out
            while True:
                self.ctl_send_data(None, pipe_out, True)
                urb_status = await self.wait_urb(pipe_out)
                if urb_status == USBH_URB_DONE:
                    # complete
                    return
                elif urb_status == USBH_URB_NOTREADY:
                    # NACK received from device; retry
                    pass
                elif urb_status == USBH_URB_ERROR:
                    assert 0
                else:
                    assert 0, urb_status

        else:
            # Data Direction is OUT, perform ctrl-status-in
            self.ctl_receive_data(None, pipe_in)
            urb_status = await self.wait_urb(pipe_in, USBH_URB_STALL)
            if urb_status == USBH_URB_DONE:
                # complete
                return
            else:
                assert 0, urb_status

    # TODO handle case of error


class USBConnection:
    def __init__(self, usbh, dev_speed):
        self.usbh = usbh
        self.pipe_out = usbh.usbh.alloc_pipe(0x00)
        self.pipe_in = usbh.usbh.alloc_pipe(0x80)
        self.pipe_size = 64  # default control pipe size
        self.dev_addr = 0  # default address
        self.dev_speed = dev_speed

    def init_pipes(self):
        self.usbh.usbh.open_pipe(
            self.pipe_in, 0x80, self.dev_addr, self.dev_speed, USBH_EP_CONTROL, self.pipe_size
        )
        self.usbh.usbh.open_pipe(
            self.pipe_out, 0x00, self.dev_addr, self.dev_speed, USBH_EP_CONTROL, self.pipe_size
        )

    async def ctrl_transfer(self, bmRequestType, bRequest, wValue, wIndex, payload):
        wLength = len(payload) if payload else 0
        cmd = struct.pack("<BBHHH", bmRequestType, bRequest, wValue, wIndex, wLength)
        await asyncio.wait_for(self.usbh.ctl_req(self.pipe_out, self.pipe_in, cmd, payload), 5)
        return payload

    async def get_descr(self, req_type, val_idx, payload):
        if val_idx & 0xFF00 == USB_DESC_STRING:
            wIndex = 0x0409
        else:
            wIndex = 0
        return await self.ctrl_transfer(
            USB_D2H | req_type, USB_REQ_GET_DESCRIPTOR, val_idx, wIndex, payload
        )

    async def get_dev_descr(self, buf):
        return await self.get_descr(
            USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD, USB_DESC_DEVICE, buf
        )

    async def get_cfg_descr(self, buf):
        return await self.get_descr(
            USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD, USB_DESC_CONFIGURATION, buf
        )

    async def set_addr(self, addr):
        self.dev_addr = addr
        bmRequestType = USB_H2D | USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD
        bRequest = USB_REQ_SET_ADDRESS
        await self.ctrl_transfer(bmRequestType, bRequest, addr, 0, None)

    async def get_string_descr(self, idx):
        buf = bytearray(255)
        await self.get_descr(
            USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD, USB_DESC_STRING | idx, buf
        )
        assert buf[1] == 3  # string type
        s = ""
        for i in range(2, buf[0], 2):
            s += chr(buf[i])
        return s

    async def set_cfg(self, cfg=1):
        await self.ctrl_transfer(
            USB_H2D | USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD,
            USB_REQ_SET_CONFIGURATION,
            cfg,
            0,
            None,
        )


async def do_enum(usbh, dev_speed, verbose=False):
    conn = USBConnection(usbh, dev_speed)
    conn.init_pipes()

    # get truncated device descriptor
    buf = await conn.get_dev_descr(bytearray(8))
    conn.pipe_size = buf[7]  # bMaxPacketSize

    # TODO when on a hub, potentially do hub.port_reset(port)?

    # modify control channels configuration for MaxPacket size
    conn.init_pipes()

    # get full device descriptor
    buf = await conn.get_dev_descr(bytearray(0x12))
    dev_descr = struct.unpack("<BBHBBBBHHHBBBB", buf)

    # set address of device
    await conn.set_addr(usbh.alloc_addr())
    await asyncio.sleep_ms(2)

    # modify control channels to update device address
    conn.init_pipes()

    # get truncated config descriptor
    buf = await conn.get_cfg_descr(bytearray(9))
    cfg_descr = struct.unpack("<BBHBBBBB", buf)

    # get full config descriptor
    size = cfg_descr[2]
    cfg_descr = await conn.get_cfg_descr(bytearray(size))

    # store device and config descriptors in USBConnection object
    conn.dev_descr = dev_descr
    conn.cfg_descr = cfg_descr

    if verbose:
        print("device:{} speed:{}".format(conn.dev_addr, ("high", "full", "low")[conn.dev_speed]))
        print("dev descr:", conn.dev_descr)
        print("VID:PID={:04x}:{:04x}".format(conn.dev_descr[7], conn.dev_descr[8]))

        if dev_descr[10] != 0:
            print("iManufacturer:", await conn.get_string_descr(conn.dev_descr[10]))
        if dev_descr[11] != 0:
            print("iProduct:", await conn.get_string_descr(conn.dev_descr[11]))
        if dev_descr[12] != 0:
            print("iSerial:", await conn.get_string_descr(conn.dev_descr[12]))

        parse_cfg_descr(conn.cfg_descr, print_cfg_descr_callback)

    return conn


def print_cfg_descr_callback(descr):
    if descr[1] == USB_DESC_TYPE_CONFIGURATION:
        # length, type, total_len, num_itf, cfg_value, i_cfg, attr, max_power
        cfg = struct.unpack("<BBHBBBBB", descr)
        print("Configuration", cfg)
    elif descr[1] == USB_DESC_TYPE_INTERFACE:
        # length, type, itfnum, alt, num_ep, itf_class, itf_subclass, itf_prot, i_itf
        itf = struct.unpack("<BBBBBBBBB", descr)
        print("  Interface", itf)
    elif descr[1] == USB_DESC_TYPE_ENDPOINT:
        # length, type, ep_addr, attr, mps, interval
        ep = struct.unpack("<BBBBHB", descr)
        print("    Endpoint", ep)
    else:
        print("    Unknown", descr[0], descr[1], descr)

def parse_cfg_descr(cfg_descr, callback):
    # length, type, total_len (uint16_t), num_itf, cfg_value, i_cfg, attr, max_power
    l = cfg_descr[0]
    t = cfg_descr[1]
    assert t == USB_DESC_TYPE_CONFIGURATION
    callback(cfg_descr[:l])
    offset = l
    while offset < len(cfg_descr):
        # length, type, itfnum, alt, num_ep, itf_class, itf_subclass, itf_prot, i_itf
        l = cfg_descr[offset]
        t = cfg_descr[offset + 1]
        total_ep = cfg_descr[offset + 4]
        assert t == USB_DESC_TYPE_INTERFACE
        callback(cfg_descr[offset:offset + l])
        offset += l
        found_ep = 0
        while offset < len(cfg_descr) and cfg_descr[offset + 1] != USB_DESC_TYPE_INTERFACE:
            l = cfg_descr[offset]
            t = cfg_descr[offset + 1]
            if t == USB_DESC_TYPE_ENDPOINT:
                found_ep += 1
            callback(cfg_descr[offset:offset + l])
            offset += l
        assert found_ep == total_ep
    assert offset == len(cfg_descr), (offset, len(cfg_descr))


class USBHub:
    def __init__(self, conn):
        self.conn = conn

    async def poll_interrupt_endpoint(self):
        buf = bytearray(1)
        self.conn.usbh.int_in_data(self.pipe_int, buf)
        await asyncio.wait_for(self.conn.usbh.wait_urb(self.pipe_int, USBH_URB_IDLE), 2)
        return buf[0]

    async def get_descr(self):
        hub_descr = await self.conn.get_descr(
            USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_CLASS, 0, bytearray(9)
        )
        hub_descr = struct.unpack("<BBBHBBBB", hub_descr)
        self.num_ports = hub_descr[2]
        self.top_port = self.num_ports + 1
        return hub_descr

    async def port_get_status(self, port):
        buf = await self.conn.ctrl_transfer(
            USB_D2H | 3 | USB_REQ_TYPE_CLASS, USB_REQ_GET_STATUS, 0, port, bytearray(4)
        )
        return struct.unpack("<HH", buf)

    async def print_port_status(self):
        for port in range(1, self.top_port):
            print(port, await self.port_get_status(port))

    async def port_set_feature(self, port, feature):
        await self.conn.ctrl_transfer(
            USB_H2D | 3 | USB_REQ_TYPE_CLASS, USB_REQ_SET_FEATURE, feature, port, None
        )

    async def port_clear_feature(self, port, feature):
        await self.conn.ctrl_transfer(
            USB_H2D | 3 | USB_REQ_TYPE_CLASS, USB_REQ_CLEAR_FEATURE, feature, port, None
        )

    def port_set_power(self, port):
        return self.port_set_feature(port, HUB_FEATURE_PORT_POWER)

    def port_reset(self, port):
        return self.port_set_feature(port, HUB_FEATURE_PORT_RESET)


class USBDFU:
    @staticmethod
    def parse_memory_layout(s):
        layout = []
        s = s.split("/")
        s.pop(0)
        while s:
            addr = int(s.pop(0), 16)
            sectors = s.pop(0).split(",")
            for sec in sectors:
                num, size_type = sec.split("*")
                num = int(num)
                assert size_type[-1] == "g"
                size = int(size_type[:-2])
                if size_type[-2] == "K":
                    size *= 1024
                elif size_type[-2] == "M":
                    size *= 1024 * 1024
                else:
                    assert 0
                for _ in range(num):
                    layout.append((addr, size))
                    addr += size
        return layout

    def __init__(self, conn):
        self.conn = conn

    async def _abort(self):
        await self.conn.ctrl_transfer(0x21, _DFU_ABORT, 0, 0, None)

    async def _clrstatus(self):
        await self.conn.ctrl_transfer(0x21, _DFU_CLRSTATUS, 0, 0, None)

    async def _get_status(self):
        stat = await self.conn.ctrl_transfer(0xA1, _DFU_GETSTATUS, 0, self._dfu_itf, bytearray(6))
        # TODO stat[5] is optional string index for any error; print it out
        return stat[4]

    async def _ensure_idle(self):
        # Get device into idle state.
        for attempt in range(4):
            status = await self._get_status()
            if status == _DFU_STATE_DFU_IDLE:
                break
            elif status in (_DFU_STATE_DFU_DOWNLOAD_IDLE, _DFU_STATE_DFU_UPLOAD_IDLE):
                await self._abort()
            else:
                await self._clrstatus()

    async def _upload(self, buf):
        await self.conn.ctrl_transfer(0x80 | 0x21, _DFU_UPLOAD, 2, self._dfu_itf, buf)
        status = await self._get_status()
        assert status == _DFU_STATE_DFU_UPLOAD_IDLE, status

    async def _dnload(self, a, b):
        await self.conn.ctrl_transfer(0x21, _DFU_DNLOAD, a, self._dfu_itf, b)
        status = await self._get_status()
        assert status == _DFU_STATE_DFU_DOWNLOAD_BUSY, status
        status = await self._get_status()
        assert status == _DFU_STATE_DFU_DOWNLOAD_IDLE, status

    async def _set_addr(self, addr):
        await self._dnload(0, struct.pack("<BI", 0x21, addr))

    async def _page_erase(self, addr):
        await self._dnload(0, struct.pack("<BI", 0x41, addr))

    async def _write(self, addr, data):
        mv = memoryview(data)
        offset = 0
        remain = len(data)
        progress = 0
        while remain:
            await self._set_addr(addr)
            l = min(self._transfer_size, remain)
            await self._dnload(2, mv[offset : offset + l])
            addr += l
            offset += l
            remain -= l
            progress += 1
            if not progress & 31:
                print(".", end="")

    async def start(self):
        self._dfu_itf = 1  # TODO need to retrieve the correct itf number
        self._transfer_size = 128  # maximum supported by the F767
        self._memory_layout = None
        def cfg_callback(descr):
            nonlocal self
            if descr[1] == USB_DESC_TYPE_INTERFACE:
                if descr[5] == 0xFE and descr[6] == 0x01:
                    # DFU interface; class=0xFE, subclass=0x01
                    if self._memory_layout is None:
                        self._memory_layout = descr[-1]
            elif descr[0] == 9 and descr[1] == 0x21:
                # DFU configuration descriptor
                cfg = struct.unpack("<BBBHHH", descr)
                print("DFU CFG", cfg)
                self._transfer_size = min(self._transfer_size, cfg[4])
        parse_cfg_descr(self.conn.cfg_descr, cfg_callback)
        if self._memory_layout is None:
            raise Exception("could not find DFU device")
        await self.conn.set_cfg()
        self._memory_layout = self.parse_memory_layout(await self.conn.get_string_descr(self._memory_layout))
        await self._ensure_idle()
        await asyncio.sleep_ms(500)

    async def exit_dfu(self):
        await self._ensure_idle()
        await self._set_addr(0x0800_0000)
        await self.conn.ctrl_transfer(0x21, _DFU_DNLOAD, 0, self._dfu_itf, None)
        try:
            # Execute last command
            if await self._get_status() != _DFU_STATE_DFU_MANIFEST:
                print("Failed to reset device")
        except:
            pass

    async def upload(self, addr, buf):
        await self._set_addr(addr)
        await self._abort()
        status = await self._get_status()
        assert status == _DFU_STATE_DFU_IDLE
        await self._upload(buf)
        await self._abort()

    async def download(self, addr, data, *, skip=0):
        for mem_i in range(len(self._memory_layout)):
            if self._memory_layout[mem_i][0] == addr:
                break
        else:
            assert 0, "Write not aligned to a page"
        mv = memoryview(data)
        offset = 0
        remain = len(data)
        while remain:
            l = min(self._memory_layout[mem_i][1], remain)
            print(f"  Erase 0x{addr:08x}")
            await self._page_erase(addr)
            print(f"  Write 0x{addr + skip:08x} {l - skip} bytes", end="")
            await self._write(addr + skip, mv[offset + skip : offset + l])
            print()
            skip = 0
            mem_i += 1
            addr += l
            offset += l
            remain -= l

    async def verify(self, addr, data):
        print(f"  Verify 0x{addr:08x} {len(data)} bytes", end="")
        buf = bytearray(128)
        mv = memoryview(data)
        offset = 0
        remain = len(data)
        progress = 0
        while remain:
            l = min(len(buf), remain)
            await self.upload(addr, buf)
            if l < len(buf):
                buf = buf[:l]
            if buf != mv[offset : offset + l]:
                print("x")
                return False, addr
            addr += l
            offset += l
            remain -= l
            progress += 1
            if not progress & 31:
                print(".", end="")
        print()
        return True, addr


async def handle_hub(conn):
    assert conn.dev_descr[3] == 9 and conn.dev_descr[4] == 0

    hub = USBHub(conn)
    await hub.conn.set_cfg()
    hub_descr = await hub.get_descr()
    # print('hub descr:', hub_descr)

    hub.pipe_int = conn.usbh.usbh.alloc_pipe(129)
    conn.usbh.usbh.open_pipe(
        hub.pipe_int, 129, conn.dev_addr, conn.dev_speed, USBH_EP_INTERRUPT, 1
    )

    for port in range(1, hub.top_port):
        await hub.port_set_power(port)

    # get status via interrupt endpoint
    # for _ in range(2):
    #     int_status = await hub.poll_interrupt_endpoint()
    #     print('int', int_status)
    #     await asyncio.sleep(0.5)
    int_status = await hub.poll_interrupt_endpoint()

    # check if any port has a connection change, and enumerate new devices
    conns = []
    for port in range(1, hub.num_ports + 1):
        if int_status & (1 << port):
            # print(await hub.port_get_status(port))
            # ack port connection change
            await hub.port_clear_feature(port, HUB_FEATURE_PORT_CONNECTION_CHANGE)
            await hub.port_reset(port)
            await asyncio.sleep(0.5)

            # enum device on hub port
            status, change = await hub.port_get_status(port)
            assert status & 1, "not connected"
            assert status & 2, "not enabled"

            # reset changed, clear it
            assert change & 16, "not reset changed"
            await hub.port_clear_feature(port, HUB_FEATURE_PORT_RESET_CHANGE)

            # begin standard enumeration
            if status & (1 << 9):
                dev_speed = USB_SPEED_LOW
            elif status & (1 << 10):
                dev_speed = USB_SPEED_HIGH
            else:
                dev_speed = USB_SPEED_FULL
            conn = await do_enum(conn.usbh, dev_speed)
            conns.append(conn)

    # await hub.print_port_status()

    # #for conn in conns:
    # #    await handle_dfu(conn)

    # await asyncio.sleep(0.5)
    # await hub.print_port_status()

    # for _ in range(2):
    #     print('int', await hub.poll_interrupt_endpoint())
    #     await asyncio.sleep(0.5)

    return conns


usb_list_entry = namedtuple("usb_list_entry", ["addr", "vid", "pid", "product"])


class USBHostInterface:
    # usb_id: 0=FS, 1=HS port
    # vbus: VBUS pin (optional)
    def __init__(self, usb_id, vbus=None):
        # Create low-level USBHost object
        self.usbh_ll = pyb.USBHost(usb_id)

        # Create high-level USBHost object
        self.usbh = USBHost(self.usbh_ll)

        # Turn off VBUS
        if vbus is None:
            self.vbus = lambda x:None
        else:
            self.vbus = vbus
        self.vbus(0)

    async def start(self):
        # Reset and initialise USBH bus
        self.usbh_ll.init()
        self.usbh_ll.start(0)
        self.usbh_ll.start(1)
        self.vbus(1)
        time.sleep_ms(200)
        await self.usbh.wait_event(EVENT_CONNECTED)
        time.sleep_ms(200)
        self.usbh_ll.reset()
        # await self.usbh.wait_event(EVENT_PORT_ENABLED)
        await self.usbh.wait_event(EVENT_CONNECTED)
        # device is now attached

        # now go into enumeration
        time.sleep_ms(100)
        dev_speed = self.usbh_ll.get_speed()
        conn_root = await do_enum(self.usbh, dev_speed)

        if conn_root.dev_descr[3] == 9 and conn_root.dev_descr[4] == 0:
            # A hub is connected to the root port
            conns = await handle_hub(conn_root)
            conns.insert(0, conn_root)
            self.conns = conns
        else:
            # A device is connected to the root port
            self.conns = [conn_root]

    def stop(self):
        # Shut down
        self.vbus(0)
        time.sleep_ms(100)
        # print(await self.usbh.wait_event(7))

    async def list(self):
        ls = []
        for conn in self.conns:
            vid = conn.dev_descr[7]
            pid = conn.dev_descr[8]
            if conn.dev_descr[11] == 0:
                prod = ""
            else:
                prod = await conn.get_string_descr(conn.dev_descr[11])
            ls.append(usb_list_entry(conn.dev_addr, vid, pid, prod))
        return ls

    def get_device(self, addr):
        for conn in self.conns:
            if conn.dev_addr == addr:
                return conn
        return None


class MemoryFile:
    def __init__(self, addr, size):
        self.data = memoryview(uctypes.bytearray_at(addr, size))
        self.offset = 0

    def unpack(self, fmt):
        elems = struct.unpack_from(fmt, self.data, self.offset)
        self.offset += struct.calcsize(fmt)
        return elems

    def read(self, size):
        data = self.data[self.offset:self.offset + size]
        self.offset += size
        return data


def dfu_read(file):
    elems = []

    sig, ver, size, num_targ = file.unpack("<5sBIB")
    if not (sig == b"DfuSe" and ver == 1):
        raise Exception(f"invalid firmware: {sig=} {ver=}")

    for i in range(num_targ):
        sig, alt, has_name, name, t_size, num_elem = file.unpack("<6sBi255sII")

        file_offset_t = file.offset
        for j in range(num_elem):
            addr, e_size = file.unpack("<II")
            data = file.read(e_size)
            elems.append((addr, data))

        if t_size != file.offset - file_offset_t:
            raise Exception(f"corrupt DFU: {t_size} {file.offset - file_offset_t}")

    if size != file.offset:
        raise Exception("corrupt DFU: {size} {file.offset}")

    _, _, _, _, _, _, file_crc = file.unpack("<HHHH3sBI")

    crc = ~binascii.crc32(file.data[:file.offset - 4]) & 0xFFFFFFFF
    if crc != file_crc:
        print("CRC failed", crc, file_crc)
        return None

    return elems


async def main():
    # Hardware config and initialisation.
    if "NUCLEO-F767" in os.uname().machine:
        vbus = machine.Pin("OTG_FS_POWER", machine.Pin.OUT)
        vbus(1)
        vbus = None
        usb_id = 0
    else:
        vbus = None
        usb_id = 1

    # Bytes to skip programming in the firmware until the rest of the firmware is verified to have
    # been programmed correctly.
    FIRMWARE_VERIFY_SKIP = 8

    # Load the DFU file to program.
    DFU_ADDR = 0x0800_0000 + 1024 * 1024
    DFU_MAX_SIZE = 1024 * 1024
    dfu_file = dfu_read(MemoryFile(DFU_ADDR, DFU_MAX_SIZE))
    if dfu_file is None:
        # Loading failed.
        return

    # Print out info about the DFU file.
    print(f"DFU loaded from memory at 0x{DFU_ADDR:08x}")
    for addr, data in dfu_file:
        print(f"DFU element addr=0x{addr:08x} len={len(data)}")
    print()

    # Start the USB host.
    usbh = USBHostInterface(usb_id, vbus)
    await usbh.start()

    # List connected USB devices.
    print(f"Connected USB devices:")
    for item in await usbh.list():
        print(f"{item.addr:02} {item.vid:04x}:{item.pid:04x} {item.product}")
    print()

    for dev_addr in (2,):
        print(f"Connecting to DFU device at address {dev_addr}")
        dfu = USBDFU(usbh.get_device(dev_addr))
        try:
            await dfu.start()
        except Exception as er:
            print(f"error: {er}")
            break

        if 0:
            # Debug: dump initial flash.
            buf = bytearray(64)
            await dfu.upload(0x0800_8000, buf)
            for i in range(0, len(buf), 4):
                print(f"{struct.unpack_from('<I', buf, i)[0]:08x} ", end="")
                if i % 16 == 12:
                    print()

        if 1:
            # Download firmware.
            skip = FIRMWARE_VERIFY_SKIP
            for addr, data in dfu_file:
                print(f"Download element 0x{addr:08x} {len(data)} bytes")
                await dfu.download(addr, data, skip=skip)
                skip = 0
                print(f"  Download of element succeeded")

        verify_element_count = 0
        if 1:
            # Verify firmware.
            skip = FIRMWARE_VERIFY_SKIP
            for addr, data in dfu_file:
                print(f"Verify element 0x{addr:08x} {len(data)} bytes")
                verify = await dfu.verify(addr + skip, data[skip:])
                if verify[0]:
                    verify_element_count += 1
                    print(f"  Verify of element succeeded")
                else:
                    print(f"  Verify of element failed at address 0x{verify[1]:08x}")

        if verify_element_count == len(dfu_file):
            # Write initial bytes.
            addr, data = dfu_file[0]
            print(f"Write to 0x{addr:08x} initial {FIRMWARE_VERIFY_SKIP} bytes")
            await dfu._write(addr, data[:8])
            print(f"  Write finished")
        else:
            print(f"Not writing initial {FIRMWARE_VERIFY_SKIP} bytes")

        print("Trigger device to exit DFU")
        await dfu.exit_dfu()

    await asyncio.sleep(0.5)
    usbh.stop()


asyncio.run(main())
