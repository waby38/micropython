import machine,pyb, time, select, uasyncio as asyncio, struct
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

# return values for get_urb_state
USBH_URB_IDLE = 0
USBH_URB_DONE = 1
USBH_URB_NOTREADY = 2
USBH_URB_NYET = 3
USBH_URB_ERROR = 4
USBH_URB_STALL = 5

# type of descriptor
USB_DESC_TYPE_DEVICE         = 1
USB_DESC_TYPE_CONFIGURATION  = 2
USB_DESC_TYPE_STRING         = 3
USB_DESC_TYPE_INTERFACE      = 4
USB_DESC_TYPE_ENDPOINT       = 5

_DFU_DNLOAD = 1
_DFU_GETSTATUS = 3
_DFU_CLRSTATUS = 4

_DFU_STATE_DFU_DOWNLOAD_SYNC = 0x03
_DFU_STATE_DFU_DOWNLOAD_BUSY = 0x04
_DFU_STATE_DFU_DOWNLOAD_IDLE = 0x05
_DFU_STATE_DFU_MANIFEST_SYNC = 0x06

HUB_FEATURE_PORT_RESET = 4
HUB_FEATURE_PORT_POWER = 8
HUB_FEATURE_PORT_CONNECTION_CHANGE = 16
HUB_FEATURE_PORT_RESET_CHANGE = 20

Pipes = namedtuple("Pipes", ("out", "in_"))

class USBHost:
    def __init__(self, usbh, vbus):
        self.usbh = usbh
        self.vbus = vbus
        self.vbus(0)
        self._cur_addr = 0

    def alloc_pipes(self, out, in_):
        return Pipes(self.usbh.alloc_pipe(out), self.usbh.alloc_pipe(in_))

    def alloc_addr(self):
        self._cur_addr += 1
        return self._cur_addr

    async def wait_event(self, event):
        while True:
            yield asyncio.core._io_queue.queue_read(self.usbh)
            ev = self.usbh.event()
            #print('event', ev)
            if ev & event:
                return ev

    def ctl_send_setup(self, buf, pipe):
        self.usbh.submit_urb(pipe, 0, USBH_EP_CONTROL, USBH_PID_SETUP, buf, 0)

    def ctl_send_data(self, buf, pipe, do_ping):
        # TODO need to know the speed to disable pings
        #if self.dev_speed != 0:  # USBH_SPEED_HIGH
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
            #print('wait_urb', pipe, ev, s)
            if s == USBH_URB_DONE or s == USBH_URB_ERROR or s == allowed_s or s == USBH_URB_NOTREADY:# or s == USBH_URB_STALL:
                return s

    async def ctl_req(self, pipes, cmd, payload):
        self.ctl_send_setup(cmd, pipes.out)

        urb_status = await self.wait_urb(pipes.out)
        if urb_status in (USBH_URB_ERROR, USBH_URB_NOTREADY):
            raise Exception
            #handle CTRL_ERROR

        assert urb_status == USBH_URB_DONE

        direction = cmd[0] & 0x80

        # check if there is a data stage
        if payload:
            if direction == USB_D2H:
                # Data Direction is IN
                # Issue an IN token
                #phost->Control.timer = (uint16_t)phost->Timer
                self.ctl_receive_data(payload, pipes.in_)
                urb_status = await self.wait_urb(pipes.in_)

                # check is DATA packet transferred successfully
                if urb_status == USBH_URB_DONE:
                    #phost->Control.state = CTRL_STATUS_OUT
                    #print("CTRL_STATUS_OUT")
                    # fall through to handle this
                    pass

                # manage error cases*/
                elif urb_status == USBH_URB_STALL:
                    # In stall case, return to previous machine state
                    status = USBH_NOT_SUPPORTED
                    assert 0

                elif urb_status == USBH_URB_ERROR:
                    # Device error
                    print("CTRL_ERROR")
                    assert 0
                    #phost->Control.state = CTRL_ERROR

            else:
                # Data Direction is OUT
                # case CTRL_DATA_OUT:

                while True:
                    self.ctl_send_data(payload, pipes.out, 1)
                    #phost->Control.timer = (uint16_t)phost->Timer;
                    urb_status = await self.wait_urb(pipes.out, USBH_URB_STALL)

                    if urb_status == USBH_URB_DONE:
                        # If the Setup Pkt is sent successful, then change the state
                        #phost->Control.state = CTRL_STATUS_IN;
                        break
                    elif urb_status == USBH_URB_NOTREADY:
                        # NACK received from device; retry
                        pass
                    else:
                        assert 0
                        """
                        # handle error cases */
                        elif urb_status == USBH_URB_STALL:
                        {
                          /* In stall case, return to previous machine state*/
                          phost->Control.state = CTRL_STALLED;
                          status = USBH_NOT_SUPPORTED;

                        }
                        else
                        {
                          if (URB_Status == USBH_URB_ERROR)
                          {
                            /* device error */
                            phost->Control.state = CTRL_ERROR;
                            status = USBH_FAIL;

                          }
                        }
                        """


        if direction == USB_D2H:
            # Data Direction is IN
            #phost->Control.state = CTRL_STATUS_OUT
            #print("CTRL_STATUS_OUT")
            self.ctl_send_data(None, pipes.out, True)
            #phost->Control.timer = (uint16_t)phost->Timer;
            urb_status = await self.wait_urb(pipes.out)
            if urb_status == USBH_URB_DONE:
                # complete
                return
            elif urb_status == USBH_URB_ERROR:
                #phost->Control.state = CTRL_ERROR;
                assert 0
            else:
                assert 0

        else:
            # Data Direction is OUT
            #phost->Control.state = CTRL_STATUS_IN
            #print("CTRL_STATUS_IN")
            # Send 0 bytes out packet
            self.ctl_receive_data(None, pipes.in_)

            #phost->Control.timer = (uint16_t)phost->Timer;

            urb_status = await self.wait_urb(pipes.in_, USBH_URB_STALL)

            if urb_status == USBH_URB_DONE:
                # complete
                return
            elif urb_status == USBH_URB_ERROR:
                #phost->Control.state = CTRL_ERROR;
                assert 0
            elif urb_status == USBH_URB_STALL:
                # Control transfers completed, Exit the State Machine
                raise Exception("USBH_NOT_SUPPORTED")

    """


        case CTRL_ERROR:
          /*
          After a halt condition is encountered or an error is detected by the
          host, a control endpoint is allowed to recover by accepting the next Setup
          PID; i.e., recovery actions via some other pipe are not required for control
          endpoints. For the Default Control Pipe, a device reset will ultimately be
          required to clear the halt or error condition if the next Setup PID is not
          accepted.
          */
          if (++phost->Control.errorcount <= USBH_MAX_ERROR_COUNT)
          {
            /* Do the transmission again, starting from SETUP Packet */
            phost->Control.state = CTRL_SETUP;
            phost->RequestState = CMD_SEND;
          }
          else
          {
            phost->pUser(phost, HOST_USER_UNRECOVERED_ERROR);
            phost->Control.errorcount = 0U;
            USBH_ErrLog("Control error: Device not responding");

            /* Free control pipes */
            USBH_FreePipe(phost, phost->Control.pipe_out);
            USBH_FreePipe(phost, phost->Control.pipe_in);

            phost->gState = HOST_IDLE;
            status = USBH_FAIL;
          }
          break;

        default:
          break;
      }
    """

    # TODO consider putting these methods in a USBDevice class, which knows the pipes

    async def get_descr(self, pipes, req_type, val_idx, payload):
        bmRequestType = USB_D2H | req_type
        bRequest = USB_REQ_GET_DESCRIPTOR
        wValue = val_idx
        if (val_idx & 0xff00) == USB_DESC_STRING:
            wIndex = 0x0409
        else:
            wIndex = 0
        wLength = len(payload)
        cmd = struct.pack("<BBHHH", bmRequestType, bRequest, wValue, wIndex, wLength)
        await self.ctl_req(pipes, cmd, payload)
        return payload

    async def get_dev_descr(self, pipes, buf):
        return await self.get_descr(pipes, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD, USB_DESC_DEVICE, buf)

    async def get_cfg_descr(self, pipes, buf):
        return await self.get_descr(pipes, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD, USB_DESC_CONFIGURATION, buf)

    async def set_addr(self, pipes, addr):
        bmRequestType = USB_H2D | USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD
        bRequest = USB_REQ_SET_ADDRESS
        cmd = struct.pack("<BBHHH", bmRequestType, bRequest, addr, 0, 0)
        await self.ctl_req(pipes, cmd, None)

    async def get_string_descr(self, pipes, idx):
        buf = bytearray(255)
        await self.get_descr(pipes, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD, USB_DESC_STRING | idx, buf)
        assert buf[1] == 3  # string type
        s = ""
        for i in range(2, buf[0], 2):
            s += chr(buf[i])
        return s

    async def ctrl_transfer(self, pipes, bmRequestType, bRequest, wValue, wIndex, payload):
        wLength = len(payload) if payload else 0
        cmd = struct.pack("<BBHHH", bmRequestType, bRequest, wValue, wIndex, wLength)
        await self.ctl_req(pipes, cmd, payload)
        return payload

    async def set_cfg(self, pipes, cfg=1):
        await self.ctrl_transfer(pipes, USB_H2D | USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD, USB_REQ_SET_CONFIGURATION, cfg, 0, None)


async def do_enum(usbh, dev_speed):
    dev_addr = 0  # default address
    pipe_size = 64  # control pipe size
    pipes = usbh.alloc_pipes(0x00, 0x80)
    usbh.usbh.open_pipe(pipes.in_, 0x80, dev_addr, dev_speed, USBH_EP_CONTROL, pipe_size)
    usbh.usbh.open_pipe(pipes.out, 0x00, dev_addr, dev_speed, USBH_EP_CONTROL, pipe_size)

    # get truncated device descriptor
    buf = await usbh.get_dev_descr(pipes, bytearray(8))
    print('dev descr:', buf)
    pipe_size = buf[7]  # bMaxPacketSize
    print('pipe_size:', pipe_size)

    # TODO when on a hub, potentially do hub.port_reset(port)?

    # modify control channels configuration for MaxPacket size
    usbh.usbh.open_pipe(pipes.in_, 0x80, dev_addr, dev_speed, USBH_EP_CONTROL, pipe_size)
    usbh.usbh.open_pipe(pipes.out, 0x00, dev_addr, dev_speed, USBH_EP_CONTROL, pipe_size)

    # get full device descriptor
    buf = await usbh.get_dev_descr(pipes, bytearray(0x12))
    print('dev descr:', buf)
    dev_descr = struct.unpack("<BBHBBBBHHHBBBB", buf)
    print(dev_descr)
    print("VID:PID={:04x}:{:04x}".format(dev_descr[7], dev_descr[8]))

    # set address of device
    dev_addr = usbh.alloc_addr()
    await usbh.set_addr(pipes, dev_addr)

    print("device:", dev_addr, "speed:", ("high", "full", "low")[dev_speed])

    await asyncio.sleep_ms(2)

    # modify control channels to update device address
    usbh.usbh.open_pipe(pipes.in_, 0x80, dev_addr, dev_speed, USBH_EP_CONTROL, pipe_size)
    usbh.usbh.open_pipe(pipes.out, 0x00, dev_addr, dev_speed, USBH_EP_CONTROL, pipe_size)

    # get truncated config descriptor
    buf = await usbh.get_cfg_descr(pipes, bytearray(9))
    cfg_descr = struct.unpack("<BBHBBBBB", buf)

    # get full config descriptor
    size = cfg_descr[2]
    cfg_descr = await usbh.get_cfg_descr(pipes, bytearray(size))
    print('cfg:', cfg_descr)
    parse_cfg_descr(cfg_descr)

    # get imanuf string
    if dev_descr[10] != 0:
        print('iManufacturer:', await usbh.get_string_descr(pipes, dev_descr[10]))
    if dev_descr[11] != 0:
        print('iProduct:', await usbh.get_string_descr(pipes, dev_descr[11]))
    if dev_descr[12] != 0:
        print('iSerial:', await usbh.get_string_descr(pipes, dev_descr[12]))

    return pipes, dev_addr


def parse_cfg_descr(cfg_descr):
    # length, type, total_len, num_itf, cfg_value, i_cfg, attr, max_power
    cfg = struct.unpack_from("<BBHBBBBB", cfg_descr, 0)
    offset = cfg[0]
    assert cfg[1] == USB_DESC_TYPE_CONFIGURATION
    print("Configuration", cfg)
    while offset < len(cfg_descr):
        # length, type, itfnum, alt, num_ep, itf_class, itf_subclass, itf_prot, i_itf
        itf = struct.unpack_from("<BBBBBBBBB", cfg_descr, offset)
        offset += itf[0]
        assert itf[1] == USB_DESC_TYPE_INTERFACE
        print("  Interface", itf)
        found_ep = 0
        while offset < len(cfg_descr) and cfg_descr[offset + 1] != USB_DESC_TYPE_INTERFACE:
            l = cfg_descr[offset]
            t = cfg_descr[offset + 1]
            if t == USB_DESC_TYPE_ENDPOINT:
                found_ep += 1
                # length, type, ep_addr, attr, mps, interval
                ep = struct.unpack_from("<BBBBHB", cfg_descr, offset)
                print("    Endpoint", ep)
            else:
                print("    Unknown", l, t, cfg_descr[offset:offset + l])
            offset += l
        assert found_ep == itf[4]
    assert offset == len(cfg_descr), (offset, len(cfg_descr))


class USBHub:
    def __init__(self, usbh, pipes):
        self.usbh = usbh
        self.pipes = pipes

    async def get_descr(self):
        hub_descr = await self.usbh.get_descr(self.pipes, USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_CLASS, 0, bytearray(9))
        hub_descr = struct.unpack("<BBBHBBBB", hub_descr)
        self.num_ports = hub_descr[2]
        self.top_port = self.num_ports + 1
        return hub_descr

    async def port_get_status(self, port):
        buf = await self.usbh.ctrl_transfer(self.pipes, USB_D2H | 3 | USB_REQ_TYPE_CLASS, USB_REQ_GET_STATUS, 0, port, bytearray(4))
        return struct.unpack("<HH", buf)

    async def print_port_status(self):
        for port in range(1, self.top_port):
            print(port, await self.port_get_status(port))

    async def port_set_feature(self, port, feature):
        await self.usbh.ctrl_transfer(self.pipes, USB_H2D | 3 | USB_REQ_TYPE_CLASS, USB_REQ_SET_FEATURE, feature, port, None)

    async def port_clear_feature(self, port, feature):
        await self.usbh.ctrl_transfer(self.pipes, USB_H2D | 3 | USB_REQ_TYPE_CLASS, USB_REQ_CLEAR_FEATURE, feature, port, None)

    def port_set_power(self, port):
        return self.port_set_feature(port, HUB_FEATURE_PORT_POWER)

    def port_reset(self, port):
        return self.port_set_feature(port, HUB_FEATURE_PORT_RESET)


class USBDFU:
    def __init__(self, usbh, pipes, dev_addr):
        self.usbh = usbh
        self.pipes = pipes
        self.dev_addr = dev_addr

    async def dfu_clrstatus(self):
        await self.usbh.ctrl_transfer(self.pipes, 0x21, _DFU_CLRSTATUS, 0, 0, None)

    async def dfu_get_status(self):
        stat = await self.usbh.ctrl_transfer(self.pipes, 0xA1, _DFU_GETSTATUS, 0, self.dfu_itf, bytearray(6))
        # TODO stat[5] is optional string index for any error; print it out
        return stat[4]

    async def dfu_dnload(self, a, b):
        await self.usbh.ctrl_transfer(self.pipes, 0x21, _DFU_DNLOAD, a, self.dfu_itf, b)
        status = await self.dfu_get_status()
        assert status == _DFU_STATE_DFU_DOWNLOAD_BUSY, status
        status = await self.dfu_get_status()
        assert status == _DFU_STATE_DFU_DOWNLOAD_IDLE, status

    async def dfu_set_addr(self, addr):
        buf = struct.pack("<BI", 0x21, addr)
        await self.dfu_dnload(0, buf)

    async def dfu_exit(self):
        await self.dfu_set_addr(0x08000000)
        await self.usbh.ctrl_transfer(self.pipes, 0x21, _DFU_DNLOAD, 0, self.dfu_itf, None)
        try:
            # Execute last command
            if await self.dfu_get_status() != _DFU_STATE_DFU_MANIFEST:
                print("Failed to reset device")
        except:
            pass


async def handle_hub(usbh, pipes, dev_addr, dev_speed):
    hub = USBHub(usbh, pipes)
    hub_descr = await hub.get_descr()
    print('hub descr:', hub_descr)

    for port in range(1, hub.top_port):
        await hub.port_set_power(port)

    # get status via interrupt endpoint
    pipe_int = usbh.usbh.alloc_pipe(129)
    usbh.usbh.open_pipe(pipe_int, 129, dev_addr, dev_speed, USBH_EP_INTERRUPT, 1)
    buf = bytearray(1)
    for _ in range(2):
        print("do int")
        usbh.int_in_data(pipe_int, buf)
        await usbh.wait_urb(pipe_int)
        print('int', buf)
        for port in range(1, hub.num_ports + 1):
            if buf[0] & (1 << port):
                print(await hub.port_get_status(port))
                # ack port connection change
                await hub.port_clear_feature(port, HUB_FEATURE_PORT_CONNECTION_CHANGE)
                await hub.port_reset(port)
        await asyncio.sleep(0.5)

    await hub.print_port_status()

    # enum device on hub port
    port = 1
    status, change = await hub.port_get_status(port)
    assert status & 1, "not connected"
    assert status & 2, "not enabled"

    # reset changed, clear it
    assert change & 16, "not reset changed"
    await hub.port_clear_feature(port, HUB_FEATURE_PORT_RESET_CHANGE)

    # begin standard enumeration
    dev_speed2 = 1  # TODO check speed, assume full
    pipes2, dev_addr2 = await do_enum(usbh, dev_speed2)

    await handle_dfu(usbh, pipes2, dev_addr2)


async def handle_dfu(usbh, pipes, dev_addr):
    dfu = USBDFU(usbh, pipes, dev_addr)
    print(await usbh.get_string_descr(pipes, 4))
    dfu.dfu_itf = 1  # TODO need to retrieve the correct itf number
    print("dfu:", await dfu.dfu_get_status())
    await dfu.dfu_clrstatus()
    print("dfu:", await dfu.dfu_get_status())

    await dfu.dfu_exit()


async def main():
    machine.Pin("USB_DM", machine.Pin.ALT, alt=10)
    machine.Pin("USB_DP", machine.Pin.ALT, alt=10)
    usbh = USBHost(pyb.USBHost(), machine.Pin("OTG_FS_POWER", machine.Pin.OUT))
    usbh.usbh.init()
    usbh.usbh.start()
    usbh.vbus(1)
    time.sleep_ms(200)
    await usbh.wait_event(EVENT_CONNECTED)
    time.sleep_ms(200)
    print('reset')
    usbh.usbh.reset()
    #await usbh.wait_event(EVENT_PORT_ENABLED)
    await usbh.wait_event(EVENT_CONNECTED)
    # device is now attached

    # now go into enumeration
    time.sleep_ms(100)
    dev_speed = usbh.usbh.get_speed()
    pipes, dev_addr = await do_enum(usbh, dev_speed)

    # set configuration
    await usbh.set_cfg(pipes)

    await handle_hub(usbh, pipes, dev_addr, dev_speed)
    #await handle_dfu(usbh, pipes, dev_addr)

    # Shut down
    time.sleep_ms(1000)
    usbh.vbus(0)
    time.sleep_ms(100)
    print(await usbh.wait_event(7))

asyncio.run(main())
