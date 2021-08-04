import machine,pyb, time, select, uasyncio as asyncio, struct

EVENT_CONNECTED = 1
EVENT_PORT_ENABLED = 4

USBH_EP_CONTROL = 0
USB_REQ_RECIPIENT_DEVICE = 0
USB_REQ_TYPE_STANDARD = 0
USB_DESC_DEVICE = 0x0100
USB_DESC_CONFIGURATION = 0x0200
USB_DESC_STRING = 0x0300
USB_H2D = 0x00
USB_D2H = 0x80
USB_REQ_SET_ADDRESS = 0x05
USB_REQ_GET_DESCRIPTOR = 0x06
USBH_PID_SETUP = 0
USBH_PID_DATA = 1

# return values for get_urb_state
USBH_URB_IDLE = 0
USBH_URB_DONE = 1
USBH_URB_NOTREADY = 2
USBH_URB_NYET = 3
USBH_URB_ERROR = 4
USBH_URB_STALL = 5

class USBHost:
    def __init__(self, usbh, vbus):
        self.usbh = usbh
        self.vbus = vbus
        self.vbus(0)

    async def wait_event(self, event):
        while True:
            yield asyncio.core._io_queue.queue_read(self.usbh)
            ev = self.usbh.event()
            if ev & event:
                return ev

    def ctl_send_setup(self, buf, pipe):
        self.usbh.submit_urb(pipe, 0, USBH_EP_CONTROL, USBH_PID_SETUP, buf, 0)

    def ctl_send_data(self, buf, pipe, do_ping):
        if self.dev_speed != 0:  # USBH_SPEED_HIGH
            do_ping = False
        self.usbh.submit_urb(pipe, 0, USBH_EP_CONTROL, USBH_PID_DATA, buf, do_ping)

    def ctl_receive_data(self, buf, pipe):
        self.usbh.submit_urb(pipe, 1, USBH_EP_CONTROL, USBH_PID_DATA, buf, 0)

    async def wait_urb(self, pipe, allowed_s=-1):
        while True:
            yield asyncio.core._io_queue.queue_read(self.usbh)
            self.usbh.event()  # TODO: store event to self.event
            s = self.usbh.get_urb_state(pipe)
            #print('wait_urb', pipe, s)
            if s == USBH_URB_DONE or s == USBH_URB_ERROR or s == allowed_s or s == USBH_URB_NOTREADY:
                return s

    async def ctl_req(self, cmd, payload):
        self.ctl_send_setup(cmd, self.pipe_out)

        urb_status = await self.wait_urb(self.pipe_out)
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
                self.ctl_receive_data(payload, self.pipe_in)
                urb_status = await self.wait_urb(self.pipe_in)

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
                    self.ctl_send_data(payload, self.pipe_out, 1)
                    #phost->Control.timer = (uint16_t)phost->Timer;
                    urb_status = await self.wait_urb(self.pipe_out, USBH_URB_STALL)

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
            self.ctl_send_data(None, self.pipe_out, True)
            #phost->Control.timer = (uint16_t)phost->Timer;
            urb_status = await self.wait_urb(self.pipe_out)
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
            self.ctl_receive_data(None, self.pipe_in)

            #phost->Control.timer = (uint16_t)phost->Timer;

            urb_status = await self.wait_urb(self.pipe_in, USBH_URB_STALL)

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

    async def get_descr(self, req_type, val_idx, payload):
        bmRequestType = USB_D2H | req_type
        bRequest = USB_REQ_GET_DESCRIPTOR
        wValue = val_idx
        if (val_idx & 0xff00) == USB_DESC_STRING:
            wIndex = 0x0409
        else:
            wIndex = 0
        wLength = len(payload)
        cmd = struct.pack("<BBHHH", bmRequestType, bRequest, wValue, wIndex, wLength)
        await self.ctl_req(cmd, payload)

    async def get_dev_descr(self, buf):
        await self.get_descr(USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD, USB_DESC_DEVICE, buf)

    async def get_cfg_descr(self, buf):
        await self.get_descr(USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD, USB_DESC_CONFIGURATION, buf)

    async def set_addr(self, addr):
        bmRequestType = USB_H2D | USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD
        bRequest = USB_REQ_SET_ADDRESS
        cmd = struct.pack("<BBHHH", bmRequestType, bRequest, addr, 0, 0)
        await self.ctl_req(cmd, None)

    async def get_string_descr(self, idx):
        buf = bytearray(255)
        await self.get_descr(USB_REQ_RECIPIENT_DEVICE | USB_REQ_TYPE_STANDARD, USB_DESC_STRING | idx, buf)
        assert buf[1] == 3  # string type
        s = ""
        for i in range(2, buf[0], 2):
            s += chr(buf[i])
        return s

    async def ctrl_transfer(self, bmRequestType, bRequest, wValue, wIndex, payload):
        wLength = len(payload) if payload else 0
        cmd = struct.pack("<BBHHH", bmRequestType, bRequest, wValue, wIndex, wLength)
        await self.ctl_req(cmd, payload)
        return payload

    async def dfu_clrstatus(self):
        await self.ctrl_transfer(0x21, _DFU_CLRSTATUS, 0, 0, None)

    async def dfu_get_status(self):
        stat = await self.ctrl_transfer(0xA1, _DFU_GETSTATUS, 0, self.dfu_itf, bytearray(6))
        # TODO stat[5] is optional string index for any error; print it out
        return stat[4]

    async def dfu_dnload(self, a, b):
        await self.ctrl_transfer(0x21, _DFU_DNLOAD, a, self.dfu_itf, b)
        status = await self.dfu_get_status()
        assert status == _DFU_STATE_DFU_DOWNLOAD_BUSY, status
        status = await self.dfu_get_status()
        assert status == _DFU_STATE_DFU_DOWNLOAD_IDLE, status

    async def dfu_set_addr(self, addr):
        buf = struct.pack("<BI", 0x21, addr)
        await self.dfu_dnload(0, buf)

    async def dfu_exit(self):
        await self.dfu_set_addr(0x08000000)
        await self.ctrl_transfer(0x21, _DFU_DNLOAD, 0, self.dfu_itf, None)
        try:
            # Execute last command
            if await self.dfu_get_status() != _DFU_STATE_DFU_MANIFEST:
                print("Failed to reset device")
        except:
            pass


_DFU_DNLOAD = 1
_DFU_GETSTATUS = 3
_DFU_CLRSTATUS = 4

_DFU_STATE_DFU_DOWNLOAD_SYNC = 0x03
_DFU_STATE_DFU_DOWNLOAD_BUSY = 0x04
_DFU_STATE_DFU_DOWNLOAD_IDLE = 0x05
_DFU_STATE_DFU_MANIFEST_SYNC = 0x06


async def main():
    machine.Pin("USB_DM", machine.Pin.ALT, alt=10)
    machine.Pin("USB_DP", machine.Pin.ALT, alt=10)
    usbh = USBHost(pyb.USBHost(), machine.Pin("OTG_FS_POWER", machine.Pin.OUT))
    usbh.usbh.init()
    usbh.usbh.start()
    usbh.vbus(1)
    await usbh.wait_event(EVENT_CONNECTED)
    time.sleep_ms(200)
    print('reset')
    usbh.usbh.reset()
    #await usbh.wait_event(EVENT_PORT_ENABLED)
    await usbh.wait_event(EVENT_CONNECTED)
    # device is now attached

    time.sleep_ms(100)
    dev_addr = 0  # default address
    pipe_size = 64  # control pipe size
    usbh.dev_speed = usbh.usbh.get_speed()
    usbh.pipe_out = usbh.usbh.alloc_pipe(0x00)
    usbh.pipe_in = usbh.usbh.alloc_pipe(0x80)
    usbh.usbh.open_pipe(usbh.pipe_in, 0x80, dev_addr, usbh.dev_speed, USBH_EP_CONTROL, pipe_size)
    usbh.usbh.open_pipe(usbh.pipe_out, 0x00, dev_addr, usbh.dev_speed, USBH_EP_CONTROL, pipe_size)
    print(usbh.dev_speed, usbh.pipe_out, usbh.pipe_in)

    # now go into enumeration

    # get truncated device descriptor
    buf = bytearray(8)
    await usbh.get_dev_descr(buf)
    print('dev descr:', buf)
    pipe_size = buf[7]  # bMaxPacketSize
    print('pipe_size:', pipe_size)

    # modify control channels configuration for MaxPacket size
    usbh.usbh.open_pipe(usbh.pipe_in, 0x80, dev_addr, usbh.dev_speed, USBH_EP_CONTROL, pipe_size)
    usbh.usbh.open_pipe(usbh.pipe_out, 0x00, dev_addr, usbh.dev_speed, USBH_EP_CONTROL, pipe_size)

    # get full device descriptor
    buf = bytearray(0x12)
    await usbh.get_dev_descr(buf)
    print('dev descr:', buf)
    dev_descr = struct.unpack("<BBHBBBBHHHBBBB", buf)
    print(dev_descr)
    print("VID:PID={:04x}:{:04x}".format(dev_descr[7], dev_descr[8]))

    # set address of device
    await usbh.set_addr(1)

    await asyncio.sleep_ms(2)
    dev_addr = 1

    # modify control channels to update device address
    usbh.usbh.open_pipe(usbh.pipe_in, 0x80, dev_addr, usbh.dev_speed, USBH_EP_CONTROL, pipe_size)
    usbh.usbh.open_pipe(usbh.pipe_out, 0x00, dev_addr, usbh.dev_speed, USBH_EP_CONTROL, pipe_size)

    # get truncated config descriptor
    buf = bytearray(9)
    await usbh.get_cfg_descr(buf)
    cfg_descr = struct.unpack("<BBHBBBBB", buf)

    # get full config descriptor
    size = cfg_descr[2]
    cfg_descr = bytearray(size)
    await usbh.get_cfg_descr(cfg_descr)
    print('cfg:', cfg_descr)

    # get imanuf string
    if dev_descr[10] != 0:
        print('iManufacturer:', await usbh.get_string_descr(dev_descr[10]))
    if dev_descr[11] != 0:
        print('iProduct:', await usbh.get_string_descr(dev_descr[11]))
    if dev_descr[12] != 0:
        print('iSerial:', await usbh.get_string_descr(dev_descr[12]))

    print(await usbh.get_string_descr(4))
    usbh.dfu_itf = 1  # TODO need to retrieve the correct itf number
    print("dfu:", await usbh.dfu_get_status())
    await usbh.dfu_clrstatus()
    print("dfu:", await usbh.dfu_get_status())

    await usbh.dfu_exit()

    time.sleep_ms(1000)
    usbh.vbus(0)
    time.sleep_ms(100)
    print(await usbh.wait_event(7))

asyncio.run(main())
