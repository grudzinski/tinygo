//go:build stm32f103

package machine

import (
	"device/arm"
	"device/stm32"
	"machine/usb"
	"runtime/interrupt"
	"runtime/volatile"
	"unsafe"
)

const NumberOfUSBEndpoints = 8

const (
	// Each tBTDEntry is unsafe.Sizeof(tBTDEntry{}) CPU bytes; PMA uses 2-byte stride
	// so it occupies half as many PMA bytes.
	pmaBTDSize = uint32(NumberOfUSBEndpoints) * uint32(unsafe.Sizeof(tBTDEntry{}))/2
	pmaBufSize = usb.EndpointPacketSize

	ep0TXOffset = pmaBTDSize + 0*pmaBufSize
	ep0RXOffset = pmaBTDSize + 1*pmaBufSize
	ep1TXOffset = pmaBTDSize + 2*pmaBufSize
	ep2RXOffset = pmaBTDSize + 3*pmaBufSize
	ep3TXOffset = pmaBTDSize + 4*pmaBufSize
)

var epBufTXOffset = [NumberOfUSBEndpoints]uint32{
	0: ep0TXOffset,
	1: ep1TXOffset,
	3: ep3TXOffset,
}

var epBufRXOffset = [NumberOfUSBEndpoints]uint32{
	0: ep0RXOffset,
	2: ep2RXOffset,
}

var endPoints = []uint32{
	usb.CONTROL_ENDPOINT: usb.ENDPOINT_TYPE_CONTROL,
	usb.CDC_ENDPOINT_ACM: usb.ENDPOINT_TYPE_INTERRUPT | usb.EndpointIn,
	usb.CDC_ENDPOINT_OUT: usb.ENDPOINT_TYPE_BULK | usb.EndpointOut,
	usb.CDC_ENDPOINT_IN:  usb.ENDPOINT_TYPE_BULK | usb.EndpointIn,
	4:                    usb.ENDPOINT_TYPE_DISABLE,
	5:                    usb.ENDPOINT_TYPE_DISABLE,
	6:                    usb.ENDPOINT_TYPE_DISABLE,
	7:                    usb.ENDPOINT_TYPE_DISABLE,
}

// tBTDEntry is one entry in the USB Buffer Table Descriptor.
// Each 16-bit field occupies a 32-bit CPU word (PMA stride).
type tBTDEntry struct {
	addrTX  volatile.Register32
	countTX volatile.Register32
	addrRX  volatile.Register32
	countRX volatile.Register32
}

// usbPMABase is the base address of the USB Packet Memory Area (RM0008 Table 3).
const usbPMABase = uintptr(0x40006000)

// pmaMem maps the 512-byte PMA as [256]uint32 at 32-bit stride
// (each uint32 holds 2 PMA bytes). Index = PMA byte offset / 2.
var pmaMem = (*[256]uint32)(unsafe.Pointer(usbPMABase))

var btable = (*[8]tBTDEntry)(unsafe.Pointer(usbPMABase))

func pmaCopy(pmaOffset uint32, src []byte) {
	for i := 0; i < len(src); i += 2 {
		v := uint32(src[i])
		if i+1 < len(src) {
			v |= uint32(src[i+1]) << 8
		}
		pmaMem[(pmaOffset+uint32(i))/2] = v
	}
}

func pmaFetch(pmaOffset uint32, dst []byte) {
	for i := 0; i < len(dst); i += 2 {
		v := pmaMem[(pmaOffset+uint32(i))/2]
		dst[i] = byte(v)
		if i+1 < len(dst) {
			dst[i+1] = byte(v >> 8)
		}
	}
}

const (
	epStatDisabled = iota
	epStatStall
	epStatNAK
	epStatValid

	eprInvariant  = stm32.USB_EPR_EP_TYPE_Msk | stm32.USB_EPR_EP_KIND_Msk | stm32.USB_EPR_EA_Msk
	eprPreserve = eprInvariant | stm32.USB_EPR_CTR_TX_Msk | stm32.USB_EPR_CTR_RX_Msk
	eprToggleBits = stm32.USB_EPR_STAT_TX_Msk | stm32.USB_EPR_DTOG_TX_Msk |
		stm32.USB_EPR_STAT_RX_Msk | stm32.USB_EPR_DTOG_RX_Msk

	// eprTypeControl/Interrupt/Bulk: EP_TYPE field value shifted to position.
	eprTypeControl   = stm32.USB_EPR_EP_TYPE_Control << stm32.USB_EPR_EP_TYPE_Pos
	eprTypeInterrupt = stm32.USB_EPR_EP_TYPE_Interrupt << stm32.USB_EPR_EP_TYPE_Pos
	eprTypeBulk      = stm32.USB_EPR_EP_TYPE_Bulk << stm32.USB_EPR_EP_TYPE_Pos

	// countRXBuf64: countRX value for a 64-byte RX buffer.
	// BL_SIZE=1 (32-byte blocks), NUM_BLOCK=1 → (1+1)×32 = 64 bytes.
	countRXBuf64 = (1 << 15) | (1 << 10)

	// countRXMask: mask for the received byte count field in countRX.
	countRXMask = 0x3FF

	// usbDisconnectMs: how long to hold D+ low to signal disconnect to host.
	usbDisconnectMs = 50
	// usbStartupUs: tSTARTUP delay after releasing FRES (USB analog section).
	usbStartupUs = 10
)

var eprRegs = (*[8]volatile.Register32)(unsafe.Pointer(&stm32.USB.EP0R))

func eprGet(ep uint32) uint32 {
	return eprRegs[ep].Get()
}

func eprSet(ep uint32, val uint32) {
	eprRegs[ep].Set(val)
}

// STAT_TX/RX are toggle bits: writing 1 flips, writing 0 keeps. XOR gives the needed toggle.
func setStat(ep, status, pos, msk uint32) {
	v := eprGet(ep)
	cur := (v >> pos) & (msk >> pos)
	eprSet(ep, (v&eprPreserve)|((cur^status)<<pos))
}

func setStatTX(ep, status uint32) {
	setStat(ep, status, stm32.USB_EPR_STAT_TX_Pos, stm32.USB_EPR_STAT_TX_Msk)
}

func setStatRX(ep, status uint32) {
	setStat(ep, status, stm32.USB_EPR_STAT_RX_Pos, stm32.USB_EPR_STAT_RX_Msk)
}

// CTR_TX[7] and CTR_RX[15] are write-0-to-clear.
func clearCTR(ep, keepCTR uint32) {
	v := eprGet(ep)
	eprSet(ep, ((v&eprInvariant)|keepCTR)&^eprToggleBits)
}

func clearCTRTX(ep uint32) {
	clearCTR(ep, stm32.USB_EPR_CTR_RX_Msk)
}

func clearCTRRX(ep uint32) {
	clearCTR(ep, stm32.USB_EPR_CTR_TX_Msk)
}

var (
	pendingAddress uint8

	sendOnEP0DATADONE struct {
		data   []byte
		offset int
	}
)


func (dev *USBDevice) Configure(_ UARTConfig) error {
	// Drive PA12 (D+) low briefly so the host sees a disconnect/reconnect edge.
	// Without this, the host may have given up on enumeration before our firmware
	// finished booting (the external 1.5kΩ pull-up is always live).
	PA12.Configure(PinConfig{Mode: PinOutput})
	PA12.Low()
	for range CPUFrequency() / 1000 * usbDisconnectMs {
		arm.Asm("nop")
	}
	PA12.Configure(PinConfig{Mode: PinInput})

	stm32.RCC.SetAPB1ENR_USBEN(1)
	stm32.RCC.SetCFGR_USBPRE(stm32.RCC_CFGR_USBPRE_DIV1_5) // 72 MHz / 1.5 = 48 MHz

	stm32.USB.CNTR.Set(stm32.USB_CNTR_FRES) // power up analog section, hold in reset
	// tSTARTUP ≥ 1μs required before releasing FRES
	for range CPUFrequency() / 1000000 * usbStartupUs {
		arm.Asm("nop")
	}
	stm32.USB.CNTR.Set(0)
	stm32.USB.ISTR.Set(0)
	stm32.USB.BTABLE.Set(0)
	stm32.USB.CNTR.Set(stm32.USB_CNTR_CTRM | stm32.USB_CNTR_RESETM)
	stm32.USB.DADDR.Set(stm32.USB_DADDR_EF) // EF=1, address=0

	intr := interrupt.New(stm32.IRQ_USB_LP_CAN_RX0, handleUSBIRQ)
	intr.SetPriority(0xC0)
	intr.Enable()

	return nil
}

func handleUSBIRQ(_ interrupt.Interrupt) {
	istr := stm32.USB.ISTR.Get()

	switch {
	case istr&stm32.USB_ISTR_RESET_Msk != 0:
		stm32.USB.SetISTR_RESET(0)
		onUSBReset()

	case istr&stm32.USB_ISTR_CTR_Msk != 0:
		ep := istr & stm32.USB_ISTR_EP_ID_Msk
		switch {
		case istr&stm32.USB_ISTR_DIR == 0:
			handleTXDone(ep)
		case ep == 0 && eprGet(ep)&stm32.USB_EPR_SETUP_Msk != 0:
			handleSetup()
		default:
			handleRXDone(ep)
		}
	}
}

func handleTXDone(ep uint32) {
	clearCTRTX(ep)
	switch {
	case ep == 0:
		if pendingAddress != 0 {
			stm32.USB.DADDR.Set(stm32.USB_DADDR_EF | (uint32(pendingAddress) & stm32.USB_DADDR_ADD_Msk))
			pendingAddress = 0
		}
		if sendOnEP0DATADONE.offset > 0 {
			data := sendOnEP0DATADONE.data[sendOnEP0DATADONE.offset:]
			count := len(data)
			if count > usb.EndpointPacketSize {
				count = usb.EndpointPacketSize
				sendOnEP0DATADONE.offset += count
			} else {
				sendOnEP0DATADONE.offset = 0
			}
			sendViaEPIn(0, data, count)
		}
	case usbTxHandler[ep] != nil:
		usbTxHandler[ep]()
	}
}

func handleSetup() {
	clearCTRRX(0)
	pmaFetch(ep0RXOffset, udd_ep_control_cache_buffer[:8])
	setup := usb.NewSetup(udd_ep_control_cache_buffer[:8])
	setStatRX(0, epStatValid)

	ok := false
	for _, h := range usbSetupHandler {
		if h != nil {
			ok = h(setup) || ok
		}
	}
	if !ok {
		ok = handleStandardSetup(setup)
	}
	if !ok {
		USBDev.SetStallEPIn(0)
	}
}

func handleRXDone(ep uint32) {
	clearCTRRX(ep)
	data := handleEndpointRx(ep)
	if h := usbRxHandler[ep]; h != nil {
		h(data)
	}
	AckUsbOutTransfer(ep)
}

func onUSBReset() {
	initControlEndpoint(0)

	stm32.USB.DADDR.Set(stm32.USB_DADDR_EF)
	pendingAddress = 0
	sendOnEP0DATADONE.offset = 0
	USBDev.InitEndpointComplete = false
}

func handleUSBSetAddress(setup usb.Setup) bool {
	SendZlp()
	// Address must be applied after the status-stage ZLP is sent, not before.
	pendingAddress = setup.WValueL & stm32.USB_DADDR_ADD_Msk
	return true
}

func initControlEndpoint(ep uint32) {
	e := &btable[ep]
	e.addrTX.Set(ep0TXOffset)
	e.countTX.Set(0)
	e.addrRX.Set(ep0RXOffset)
	e.countRX.Set(countRXBuf64)
	eprSet(ep, eprTypeControl|ep)
	setStatTX(ep, epStatNAK)
	setStatRX(ep, epStatValid)
}

func initTXEndpoint(ep, eprType uint32) {
	e := &btable[ep]
	e.addrTX.Set(epBufTXOffset[ep])
	e.countTX.Set(0)
	eprSet(ep, eprType|ep)
	setStatTX(ep, epStatNAK)
}

func initRXEndpoint(ep, eprType uint32) {
	e := &btable[ep]
	e.addrRX.Set(epBufRXOffset[ep])
	e.countRX.Set(countRXBuf64)
	eprSet(ep, eprType|ep)
	setStatRX(ep, epStatValid)
}

func initEndpoint(ep, config uint32) {
	switch config {
	case usb.ENDPOINT_TYPE_CONTROL:
		initControlEndpoint(ep)

	case usb.ENDPOINT_TYPE_INTERRUPT | usb.EndpointIn:
		initTXEndpoint(ep, eprTypeInterrupt)

	case usb.ENDPOINT_TYPE_BULK | usb.EndpointOut:
		initRXEndpoint(ep, eprTypeBulk)

	case usb.ENDPOINT_TYPE_BULK | usb.EndpointIn:
		initTXEndpoint(ep, eprTypeBulk)
	}
}

func sendViaEPIn(ep uint32, data []byte, count int) {
	if count > 0 {
		pmaCopy(epBufTXOffset[ep], data[:count])
	}
	btable[ep].countTX.Set(uint32(count))
	setStatTX(ep, epStatValid)
}

//go:noinline
func sendUSBPacket(ep uint32, data []byte) {
	count := len(data)
	if ep == 0 {
		if count > usb.EndpointPacketSize {
			count = usb.EndpointPacketSize
			sendOnEP0DATADONE.data = data
			sendOnEP0DATADONE.offset = count
		} else {
			sendOnEP0DATADONE.offset = 0
		}
	}
	sendViaEPIn(ep, data, count)
}

func SendUSBInPacket(ep uint32, data []byte) bool {
	sendUSBPacket(ep, data)
	return true
}

func SendZlp() {
	sendViaEPIn(0, nil, 0)
}

func rxCount(ep uint32) uint32 {
	return btable[ep].countRX.Get() & countRXMask
}

func handleEndpointRx(ep uint32) []byte {
	count := rxCount(ep)
	buf := udd_ep_out_cache_buffer[ep][:count]
	pmaFetch(epBufRXOffset[ep], buf)
	return buf
}

func AckUsbOutTransfer(ep uint32) {
	setStatRX(ep&^uint32(usb.EndpointIn), epStatValid)
}

func ReceiveUSBControlPacket() ([cdcLineInfoSize]byte, error) {
	setStatRX(0, epStatValid)
	for !eprRegs[0].HasBits(stm32.USB_EPR_CTR_RX_Msk) {
	}
	count := rxCount(0)
	if count > cdcLineInfoSize {
		count = cdcLineInfoSize
	}
	var b [cdcLineInfoSize]byte
	pmaFetch(ep0RXOffset, b[:count])
	clearCTRRX(0)
	return b, nil
}

// aircr VECTKEY write key — required by Cortex-M to unlock AIRCR writes.
const aircrVECTKEY = 0x05FA << stm32.SCB_AIRCR_VECTKEYSTAT_Pos

func EnterBootloader() {
	stm32.SCB.AIRCR.Set(aircrVECTKEY | stm32.SCB_AIRCR_SYSRESETREQ)
	for {
	}
}

func (dev *USBDevice) SetStallEPIn(ep uint32) {
	setStatTX(ep, epStatStall)
}

func (dev *USBDevice) SetStallEPOut(ep uint32) {
	setStatRX(ep, epStatStall)
}

func (dev *USBDevice) ClearStallEPIn(ep uint32) {
	setStatTX(ep, epStatNAK)
}

func (dev *USBDevice) ClearStallEPOut(ep uint32) {
	setStatRX(ep, epStatNAK)
}
