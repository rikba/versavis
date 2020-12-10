// Modified from https://github.com/manitou48/ZERO/blob/master/SPIdma.ino

// DMA memory to memory   ZERO
// ch 18   beat burst block
//  xdk sam0/drivers/dma/dma.c
// packages/arduino/tools/CMSIS/4.0.0-atmel/Device/ATMEL/samd21/include/component/dmac.h
// https://asf.microchip.com/docs/latest/samd21/html/asfdoc_sam0_sercom_spi_dma_use_case.html
//  assume normal SPI setup, then we take over with DMA


#ifndef Drivers_Dmac_h
#define Drivers_Dmac_h

#define BYTES 28
uint8_t txbuf[BYTES], rxbuf[BYTES];

// DMA   12 channels
typedef struct {
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
} dmacdescriptor;
volatile dmacdescriptor wrb[12] __attribute__((aligned(16)));
dmacdescriptor descriptor_section[12] __attribute__((aligned(16)));
dmacdescriptor descriptor __attribute__((aligned(16)));

static uint32_t chnltx = 0, chnlrx = 1; // DMA channels
enum XfrType { DoTX, DoRX, DoTXRX };
static XfrType xtype;
static uint8_t rxsink[1], txsrc[1] = {0xff};
volatile uint32_t dmadone;

void DMAC_Handler() {
  // interrupts DMAC_CHINTENCLR_TERR DMAC_CHINTENCLR_TCMPL DMAC_CHINTENCLR_SUSP
  uint8_t active_channel;

  // disable irqs ?
  __disable_irq();
  active_channel =
      DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk; // get channel number
  DMAC->CHID.reg = DMAC_CHID_ID(active_channel);
  dmadone = DMAC->CHINTFLAG.reg;
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR;
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_SUSP;
  __enable_irq();
}

void dma_init() {
  // probably on by default
  PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC;
  NVIC_EnableIRQ(DMAC_IRQn);

  DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
  DMAC->WRBADDR.reg = (uint32_t)wrb;
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
}

Sercom *sercom = (Sercom *)SERCOM4; // SPI SERCOM

void spi_xfr(void *txdata, void *rxdata, size_t n) {
  uint32_t temp_CHCTRLB_reg;

  // set up transmit channel
  DMAC->CHID.reg = DMAC_CHID_ID(chnltx);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << chnltx));
  temp_CHCTRLB_reg = DMAC_CHCTRLB_LVL(0) |
                     DMAC_CHCTRLB_TRIGSRC(SERCOM4_DMAC_ID_TX) |
                     DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHCTRLB.reg = temp_CHCTRLB_reg;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK; // enable all 3 interrupts
  descriptor.descaddr = 0;
  descriptor.dstaddr = (uint32_t)&sercom->SPI.DATA.reg;
  descriptor.btcnt = n;
  descriptor.srcaddr = (uint32_t)txdata;
  descriptor.btctrl = DMAC_BTCTRL_VALID;
  if (xtype != DoRX) {
    descriptor.srcaddr += n;
    descriptor.btctrl |= DMAC_BTCTRL_SRCINC;
  }
  memcpy(&descriptor_section[chnltx], &descriptor, sizeof(dmacdescriptor));

  // rx channel    enable interrupts
  DMAC->CHID.reg = DMAC_CHID_ID(chnlrx);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << chnlrx));
  temp_CHCTRLB_reg = DMAC_CHCTRLB_LVL(0) |
                     DMAC_CHCTRLB_TRIGSRC(SERCOM4_DMAC_ID_RX) |
                     DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHCTRLB.reg = temp_CHCTRLB_reg;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK; // enable all 3 interrupts
  dmadone = 0;
  descriptor.descaddr = 0;
  descriptor.srcaddr = (uint32_t)&sercom->SPI.DATA.reg;
  descriptor.btcnt = n;
  descriptor.dstaddr = (uint32_t)rxdata;
  descriptor.btctrl = DMAC_BTCTRL_VALID;
  if (xtype != DoTX) {
    descriptor.dstaddr += n;
    descriptor.btctrl |= DMAC_BTCTRL_DSTINC;
  }
  memcpy(&descriptor_section[chnlrx], &descriptor, sizeof(dmacdescriptor));

  // start both channels  ? order matter ?
  DMAC->CHID.reg = DMAC_CHID_ID(chnltx);
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
  DMAC->CHID.reg = DMAC_CHID_ID(chnlrx);
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  while (!dmadone)
    ; // await DMA done isr

  DMAC->CHID.reg = DMAC_CHID_ID(chnltx); // disable DMA to allow lib SPI
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHID.reg = DMAC_CHID_ID(chnlrx);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
}

void spi_transfer(void *txdata, void *rxdata, size_t n) {
  xtype = DoTXRX;
  spi_xfr(txdata, rxdata, n);
}

// void setup() {
//   SerialUSB.begin(9600);
//   delay(1000);
//   pinMode(SPISS, OUTPUT);
//   digitalWrite(SPISS, HIGH);
//   SPI.begin();
//   SPI.beginTransaction(SPISettings(SPISPEED, MSBFIRST, SPI_MODE3));
//   dma_init();
// }
//
// void loop() {
//   int i, errs = 0;
//   unsigned long t1;
//
//   SerialUSB.print("SPI clk ");
//   SerialUSB.println(SPISPEED);
//   digitalWrite(SPISS, LOW);
//   for (i = 0; i < BYTES; i++)
//     txbuf[i] = i;
//   digitalWrite(SPISS, HIGH);
//   delayMicroseconds(15);
//
//   t1 = micros();
//   digitalWrite(SPISS, LOW);
//   spi_write(txbuf, BYTES);
//   digitalWrite(SPISS, HIGH);
//   t1 = micros() - t1;
//   prmbs("DMA write", t1, BYTES * 8);
//
//   t1 = micros();
//   digitalWrite(SPISS, LOW);
//   spi_read(rxbuf, BYTES);
//   digitalWrite(SPISS, HIGH);
//   t1 = micros() - t1;
//   prmbs("DMA read", t1, BYTES * 8);
//
//   t1 = micros();
//   digitalWrite(SPISS, LOW);
//   spi_transfer(txbuf, rxbuf, BYTES);
//   digitalWrite(SPISS, HIGH);
//   t1 = micros() - t1;
//   prmbs("DMA read/write", t1, BYTES * 8);
//
//   t1 = micros();
//   digitalWrite(SPISS, LOW);
//   for (i = 0; i < BYTES; i++)
//     SPI.transfer(txbuf[i]);
//   digitalWrite(SPISS, HIGH);
//   t1 = micros() - t1;
//   prmbs("SPI transfer", t1, BYTES * 8);
//
//   delay(3000);
// }

#endif // Drivers_Dmac_h
