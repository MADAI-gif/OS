#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "fsl_pint.h"
#include "fsl_inputmux.h"
#include "fsl_usart.h"
#include "vfs.h"
#include "target.h"
#include "fsl_i2c.h"


//#define FIND_SD_LIB


/***************************************************************************
 * external symbols
 ***************************************************************************/
extern FileObject * opened_fds[];	/* device/vfs.c  */

/***************************************************************************
 * test device driver
 ***************************************************************************/
static int dev_init_test(Device *dev);
static int dev_open_test(FileObject *f);
static int dev_close_test(FileObject *f);
static int dev_read_test(FileObject *f, void *buf, size_t len);

Device dev_test={
    .name="test",
    .refcnt=0,
    .sem_read=NULL,
    .sem_write=NULL,
    .init=dev_init_test,
    .open=dev_open_test,
    .close=dev_close_test,
    .read=dev_read_test,
    .write=NULL,
    .ioctl=NULL
};

static int dev_init_test(Device *dev)
{
    dev->mutex=sem_new(1);
    if (dev->mutex) return 1;
    return 0;
}

static int dev_open_test(FileObject *f)
{
	sem_p(f->dev->mutex);
    if (f->flags & (O_READ)) {
        f->dev->refcnt++;
        sem_v(f->dev->mutex);
        return 1;
    }
    sem_v(f->dev->mutex);
    return 0;
}

static int dev_close_test(FileObject *f)
{
	sem_p(f->dev->mutex);
    f->dev->refcnt--;
    sem_v(f->dev->mutex);
    return 1;
}

static int dev_read_test(FileObject *f, void *buf, size_t len)
{
	int n;
	char *file="ceci est un test\r\n";
	sem_p(f->dev->mutex);
	// calculate max available readable data length
	n=f->offset<strlen(file) ? strlen(file)-f->offset : 0;
	// actually, we have to read the min from available length and expected length
	n=n<(int)len ? n : (int)len;
	memcpy(buf, file+f->offset, n);
	f->offset+=n;
	sem_v(f->dev->mutex);
	return n;
}

/***************************************************************************
 * leds device driver
 ***************************************************************************/
static void leds(uint32_t val)
{
	// bit 0 of val controls LED RED
	GPIO_PinWrite(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PORT,BOARD_LED_RED_GPIO_PIN, (~(val>>0))&1);
	//  bit 1 of val controls LED GREEN
	GPIO_PinWrite(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PORT,BOARD_LED_GREEN_GPIO_PIN, (~(val>>1))&1);
	//  bit 2 of val controls LED BLUE
	GPIO_PinWrite(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PORT,BOARD_LED_BLUE_GPIO_PIN, (~(val>>2))&1);
}

static int dev_init_leds(Device *dev);
static int dev_open_leds(FileObject *f);
static int dev_close_leds(FileObject *f);
static int dev_write_leds(FileObject *f, void *buf, size_t len);

Device dev_leds={
    .name="leds",
    .refcnt=0,
    .init=dev_init_leds,
    
	/* A COMPLETER */

	.sem_read=NULL,
	.sem_write=NULL,
	.open=dev_open_leds,
	.close=dev_close_leds,
	.read=NULL,
	.write=dev_write_leds,
	.ioctl=NULL
};

static int dev_init_leds(Device *dev)
{
	// leds init
	gpio_pin_config_t ledcfg = { kGPIO_DigitalOutput, 1};
	GPIO_PortInit(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PORT);
	GPIO_PinInit(BOARD_LED_RED_GPIO,BOARD_LED_RED_GPIO_PORT,BOARD_LED_RED_GPIO_PIN,&ledcfg);
	GPIO_PinInit(BOARD_LED_GREEN_GPIO,BOARD_LED_GREEN_GPIO_PORT,BOARD_LED_GREEN_GPIO_PIN,&ledcfg);
	GPIO_PinInit(BOARD_LED_BLUE_GPIO,BOARD_LED_BLUE_GPIO_PORT,BOARD_LED_BLUE_GPIO_PIN,&ledcfg);

	leds(0);

	/* A COMPLETER */
	dev->mutex = sem_new(1);//create r/w mutex
	return 1;
}

static int dev_open_leds(FileObject *f)
{
	sem_p(f->dev->mutex);
    if (f->dev->refcnt || (f->flags & (O_READ|O_NONBLOCK|O_APPEND|O_SHLOCK|O_EXLOCK|O_ASYNC|O_SYNC|O_CREAT|O_TRUNC|O_EXCL)))
        goto err;
    if (f->flags & O_WRITE) {
        f->dev->refcnt++;
        sem_v(f->dev->mutex);
        return 1;
    }
err:
	sem_v(f->dev->mutex);
    return 0;
}

static int dev_close_leds(FileObject *f)
{
	sem_p(f->dev->mutex);
    f->dev->refcnt--;
    sem_v(f->dev->mutex);
    return 1;
}

static int dev_write_leds(FileObject *f, void *buf, size_t len)
{
	/* A COMPLETER */
	sem_p(f->dev->mutex);
	leds(*(uint32_t*)buf);
	sem_v(f->dev->mutex);
    return 1;
}

/***************************************************************************
 * SWCenter External Interrupt Button device driver
 ***************************************************************************/
static int dev_init_btn(Device *dev);
static int dev_open_btn(FileObject *f);
static int dev_close_btn(FileObject *f);
static int dev_read_btn(FileObject *f, void *buf, size_t len);

Device dev_swuser={
    .name="swuser",
    .refcnt=0,
    .init=dev_init_btn,
    
	/* A COMPLETER */
	.open = dev_open_btn,
	.close = dev_close_btn,
	.read = dev_read_btn,
	.write = NULL,
	.sem_read = NULL,
	.sem_write = NULL,
	.ioctl = NULL
};

/*
 *  ISR callback (IRQ mode !!)
 */
static void on_swuser_cb(pint_pin_int_t pintr, uint32_t pmatch_status)
{
	/* A COMPLETER */
	sem_v(dev_swuser.sem_read);
}

static int dev_init_btn(Device *dev)
{
    /* Connect trigger sources to PINT */
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kPINT_PinInt0, kINPUTMUX_GpioPort1Pin9ToPintsel);
    /* Turnoff clock to inputmux to save power. Clock is only needed to make changes */
    INPUTMUX_Deinit(INPUTMUX);
    /* Initialize PINT */
    PINT_Init(PINT);
    /* Setup Pin Interrupt 0 for rising edge */
    PINT_PinInterruptConfig(PINT, kPINT_PinInt0, kPINT_PinIntEnableRiseEdge, on_swuser_cb);
    NVIC_SetPriority(PIN_INT0_IRQn,10);
	/* Enable callbacks for PINT0 by Index */
    PINT_EnableCallbackByIndex(PINT, kPINT_PinInt0);

	/* A COMPLETER */
    dev->sem_read = sem_new(1);// init pour lu sem
    dev->mutex = sem_new(1); //init pour deblock

    return 1;
}

static int dev_open_btn(FileObject *f)
{
	sem_p(f->dev->mutex);
    if (f->dev->refcnt || (f->flags & (O_WRITE|O_NONBLOCK|O_APPEND|O_SHLOCK|O_EXLOCK|O_ASYNC|O_SYNC|O_CREAT|O_TRUNC|O_EXCL)))
        goto err;
    if (f->flags & O_READ) {
        f->dev->refcnt++;
		sem_v(f->dev->mutex);
        return 1;
    }
err:
	sem_v(f->dev->mutex);
    return 0;
}

static int dev_close_btn(FileObject *f)
{
	sem_p(f->dev->mutex);
    f->dev->refcnt--;
	sem_v(f->dev->mutex);
    return 1;
}

static int dev_read_btn(FileObject *f, void *buf, size_t len)
{
	/* A COMPLETER */
	sem_p(f->dev->sem_read);
    return 4;
}

/***************************************************************************
 * Serial Port Device Driver
 ***************************************************************************/

static int dev_init_ser(Device *dev);
static int dev_open_ser(FileObject *f);
static int dev_close_ser(FileObject *f);
static int dev_read_ser(FileObject *fileObj, void *buffer, size_t length);
static int dev_write_ser(FileObject *fileObj, const void *buffer, size_t length);

Device dev_serial = {
    .name = "serial",
    .refcnt = 0,
    .init = dev_init_ser,

	/* A COMPLETER */
    .open = dev_open_ser,
    .close = dev_close_ser,
    .read = dev_read_ser,
    .write = dev_write_ser,
    .sem_read = NULL,
    .sem_write = NULL,
    .ioctl = NULL
};

#define RING_BUF_SIZE	32

typedef volatile struct RingBuffer {
	char data[RING_BUF_SIZE];
	int	i_w;
	int i_r;
} RingBuffer;

RingBuffer rxbuf = {
	.i_w=0,
	.i_r=0
};

RingBuffer txbuf = {
	.i_w=0,
	.i_r=0
};

volatile int user_break=0;

// USART Interrupt Service Routine
void FLEXCOMM0_IRQHandler()
{
    uint32_t status = USART_GetStatusFlags(USART0);

    // Handle RX errors
    if ((USART0->FIFOSTAT & USART_FIFOSTAT_RXERR_MASK) != 0U) {
        USART0->FIFOSTAT |= USART_FIFOSTAT_RXERR_MASK;
        USART0->FIFOCFG |= USART_FIFOCFG_EMPTYRX_MASK;  // Clear RX FIFO
    } else if (((rxbuf.i_w + 1) % RING_BUF_SIZE) != rxbuf.i_r) {
        // Receive data into RX buffer
        while ((status & kUSART_RxFifoNotEmptyFlag) && (((rxbuf.i_w + 1) % RING_BUF_SIZE) != rxbuf.i_r)) {
            rxbuf.data[rxbuf.i_w] = USART0->FIFORD;
            rxbuf.i_w = (rxbuf.i_w + 1) % RING_BUF_SIZE;
            status = USART_GetStatusFlags(USART0);
            user_break = 1;
        }
    } else {
        USART_ReadByte(USART0);  // Discard the byte if buffer is full
    }

    // Transmit data from TX buffer
    while ((status & kUSART_TxFifoNotFullFlag) && (txbuf.i_r != txbuf.i_w)) {
        USART0->FIFOWR = txbuf.data[txbuf.i_r];
        txbuf.i_r = (txbuf.i_r + 1) % RING_BUF_SIZE;
        status = USART_GetStatusFlags(USART0);

        if (txbuf.i_r == txbuf.i_w) {
            USART_DisableInterrupts(USART0, kUSART_TxLevelInterruptEnable);
        }
    }
}

// USART Initialization Function
void uart_init(USART_Type *base, uint32_t baudrate)
{
    usart_config_t config;
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);

    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = baudrate;
    config.enableTx = true;
    config.enableRx = true;

    USART_Init(base, &config, CLOCK_GetFlexCommClkFreq(0U));

    // Enable RX interrupt
    USART_EnableInterrupts(base, kUSART_RxLevelInterruptEnable);
    NVIC_SetPriority(FLEXCOMM0_IRQn, 3);
    NVIC_EnableIRQ(FLEXCOMM0_IRQn);
}

// Initialize serial device
static int dev_init_ser(Device *dev)
{
	uart_init(USART0,115200U);
    dev->mutex=sem_new(1);
    if (dev->mutex)
       	return 1;
    return 0;
}

// Open the serial device
static int dev_open_ser(FileObject *f)
{
	sem_p(f->dev->mutex);
	    if (f->dev->refcnt || (f->flags & (O_NONBLOCK|O_APPEND|O_SHLOCK|O_EXLOCK|O_ASYNC|O_SYNC|O_CREAT|O_TRUNC|O_EXCL)))
	        goto err;
	    if (f->flags & (O_READ | O_WRITE)) {
	        f->dev->refcnt++;
			sem_v(f->dev->mutex);
	        return 1;
	    }
	err:
		sem_v(f->dev->mutex);
	    return 0;
}

// Close the serial device
static int dev_close_ser(FileObject *f)
{
    sem_p(f->dev->mutex);
    f->dev->refcnt--;
    sem_v(f->dev->mutex);
    return 1;
}

// Read from serial device using ring buffer
static int dev_read_ser(FileObject *fileObj, void *buffer, size_t length)
{
    sem_p(fileObj->dev->mutex);
    size_t bytesRead = 0;
    char *dataBuffer = (char *)buffer;

    while (bytesRead < length && rxbuf.i_r != rxbuf.i_w) {
        dataBuffer[bytesRead] = rxbuf.data[rxbuf.i_r];
        rxbuf.i_r = (rxbuf.i_r + 1) % RING_BUF_SIZE;
        bytesRead++;
    }
    fileObj->offset += bytesRead;

    sem_v(fileObj->dev->mutex);
    return bytesRead;
}

// Write to serial device using ring buffer
static int dev_write_ser(FileObject *fileObj, const void *buffer, size_t length)
{
    sem_p(fileObj->dev->mutex);
    const char *dataBuffer = (const char *)buffer;

    for (size_t i = 0; i < length; i++) {
        while ((txbuf.i_w + 1) % RING_BUF_SIZE == txbuf.i_r);  // Wait if buffer is full
        txbuf.data[txbuf.i_w] = dataBuffer[i];
        txbuf.i_w = (txbuf.i_w + 1) % RING_BUF_SIZE;
    }
    USART_EnableInterrupts(USART0, kUSART_TxLevelInterruptEnable);  // Enable TX interrupt
    sem_v(fileObj->dev->mutex);
    return length;
}


/***************************************************************************
 * accelerometer Device Driver
 ***************************************************************************/

#define MMA8652_RES_12				(0)
#define MMA8652_RES_8				(2)

#define MMA8652_SCALE_2G			(0<<16)
#define MMA8652_SCALE_4G			(1<<16)
#define MMA8652_SCALE_8G			(2<<16)

#define MMA8652_RATE_800			(0<<3)
#define MMA8652_RATE_400			(1<<3)
#define MMA8652_RATE_200			(2<<3)
#define MMA8652_RATE_100			(3<<3)
#define MMA8652_RATE_50				(4<<3)
#define MMA8652_RATE_12_5			(5<<3)
#define MMA8652_RATE_6_25			(6<<3)
#define MMA8652_RATE_1_56			(7<<3)

#define MMA8652_INT					(1<<8)

#define MMA8652_DATA_READY			(8)

static I2C_Type* i2c;
static i2c_master_handle_t i2c_handle;
static uint32_t mma8652_cfg;

static volatile status_t i2c_res;

#define MMA8652_ADDR				0x1D

#define MMA8652_STATUS      		0x00
#define MMA8652_OUT_X_MSB			0x01
#define MMA8652_OUT_X_LSB			0x02
#define MMA8652_OUT_Y_MSB			0x03
#define MMA8652_OUT_Y_LSB			0x04
#define MMA8652_OUT_Z_MSB			0x05
#define MMA8652_OUT_Z_LSB			0x06
#define MMA8652_F_SETUP				0x09
#define MMA8652_TRIG_CFG			0x0A
#define MMA8652_SYSMOD				0x0B
#define MMA8652_INT_SOURCE			0x0C
#define MMA8652_WHOAMI				0x0D
#define MMA8652_XYZ_DATA_CFG		0x0E
#define MMA8652_CTRL_REG1			0x2A
#define MMA8652_CTRL_REG2			0x2B
#define MMA8652_CTRL_REG3			0x2C
#define MMA8652_CTRL_REG4			0x2D
#define MMA8652_CTRL_REG5			0x2E

#define I2C4_MASTER_CLK		12000000

status_t mma8652_read_xyz(int32_t* data);
status_t mma8652_init(I2C_Type *base, uint32_t cfg);
static int dev_init_acc(Device *dev);
static int dev_open_acc(FileObject *f);
static int dev_close_acc(FileObject *f);
static int dev_read_acc(FileObject *fileObj, void *buffer, size_t length);


Device dev_accelerometer = {
	.name = "accel",
    .refcnt = 0,
    .init = dev_init_acc,


    .open = dev_open_acc,
    .close = dev_close_acc,
    .read = dev_read_acc,
    .write = NULL,
    .sem_read = NULL,
    .sem_write = NULL,
    .ioctl = NULL
};



static void i2c_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
   i2c_res = status;
}

static int dev_init_acc(Device *dev){

    i2c_master_config_t fc4_config = {
      .enableMaster = true,
      .baudRate_Bps = 400000,		/* mode Fast */
      .enableTimeout = false
    };

    RESET_PeripheralReset(kFC4_RST_SHIFT_RSTn);
    I2C_MasterInit(I2C4, &fc4_config, I2C4_MASTER_CLK);
    NVIC_SetPriority(FLEXCOMM4_IRQn,3);

    status_t initialized = !mma8652_init(I2C4, MMA8652_RATE_6_25|MMA8652_SCALE_2G|MMA8652_RES_12|MMA8652_INT);

    dev->mutex=sem_new(1);
    if (dev->mutex)
       	return 1;
    return 0;
}

static int dev_open_acc(FileObject *f)
{
	sem_p(f->dev->mutex);
    if (f->dev->refcnt || (f->flags & (O_WRITE|O_NONBLOCK|O_APPEND|O_SHLOCK|O_EXLOCK|O_ASYNC|O_SYNC|O_CREAT|O_TRUNC|O_EXCL)))
        goto err;
    if (f->flags & O_READ) {
        f->dev->refcnt++;
		sem_v(f->dev->mutex);
        return 1;
    }
err:
	sem_v(f->dev->mutex);
    return 0;
}

static int dev_close_acc(FileObject *f)
{
    sem_p(f->dev->mutex);
    f->dev->refcnt--;
    sem_v(f->dev->mutex);
    return 1;
}
static int dev_read_acc(FileObject *fileObj, void *buffer, size_t length) {
    int32_t data[3];
    status_t res = mma8652_read_xyz(data); // Read XYZ data from the accelerometer

    if (res != kStatus_Success) {
        return -1;
    }
    if (length < sizeof(data)) {
        return -1;
    }

    memcpy(buffer, data, sizeof(data));

    return sizeof(data); // Return the size of the data written to the buffer
}


status_t mma8652_init(I2C_Type *base, uint32_t cfg)
{
	uint8_t buf[5];
    i2c_master_transfer_t xfer = {
    	.slaveAddress 	= MMA8652_ADDR,		// sensor address
        .direction 		= kI2C_Write,
        .subaddress 	= MMA8652_CTRL_REG1,
        .subaddressSize = 1,
        .data 			= buf,
        .dataSize 		= 5,
        .flags 			= kI2C_TransferNoStopFlag
    };

    /* get a handle to deal with all the transfers on the I2C bus */
	I2C_MasterTransferCreateHandle(base, &i2c_handle, i2c_callback, NULL);
	i2c=base;
	mma8652_cfg=cfg;

	/* initialize the sensor according to cfg */
	/* CTRL_REGx */
	buf[0] = cfg;				// CTRL_REG1: rate, resolution, STANDBY MODE
	buf[1] = 0;					// CTRL_REG2: normal mode, no sleep
	buf[2] = 0;					// CTRL_REG3: no FIFO, no sleep, so no need for wake, low level on INT
	buf[3] = (cfg>>8) & 1;		// CTRL_REG4: allow Data Ready Interrupt if in cfg
	buf[4] = 1;					// CTRL_REG5: Data Ready Interrupt routed to pin INT1
	i2c_res=kStatus_I2C_Busy;
    I2C_MasterTransferNonBlocking(i2c, &i2c_handle, &xfer);

    while (i2c_res==kStatus_I2C_Busy) {
    }
    if (i2c_res!=kStatus_Success) return kStatus_Fail;

    /* XYZ_DATA_CFG */
    buf[0]=cfg>>16;
	xfer.slaveAddress 	= MMA8652_ADDR;		// sensor address
    xfer.direction 		= kI2C_Write;
    xfer.subaddress 	= MMA8652_XYZ_DATA_CFG;
    xfer.subaddressSize = 1;
    xfer.data 			= buf;
    xfer.dataSize 		= 1;
    xfer.flags 			= kI2C_TransferRepeatedStartFlag|kI2C_TransferNoStopFlag;

	i2c_res=kStatus_I2C_Busy;
    I2C_MasterTransferNonBlocking(i2c, &i2c_handle, &xfer);

    while (i2c_res==kStatus_I2C_Busy) {
    }
    if (i2c_res!=kStatus_Success) return kStatus_Fail;

    /* CTRL_REG1: ACTIVE MODE */
    buf[0]=cfg | 1;
	xfer.slaveAddress 	= MMA8652_ADDR;		// sensor address
    xfer.direction 		= kI2C_Write;
    xfer.subaddress 	= MMA8652_CTRL_REG1;
    xfer.subaddressSize = 1;
    xfer.data 			= buf;
    xfer.dataSize 		= 1;
    xfer.flags 			= kI2C_TransferRepeatedStartFlag;

	i2c_res=kStatus_I2C_Busy;
    I2C_MasterTransferNonBlocking(i2c, &i2c_handle, &xfer);

    while (i2c_res==kStatus_I2C_Busy) {
    }
    if (i2c_res!=kStatus_Success) return kStatus_Fail;

    /* Everything done, nice! */
    return kStatus_Success;
}

status_t mma8652_read_xyz(int32_t* data) {
    uint32_t scale = (mma8652_cfg >> 16) & 3; // full scale 2G, 4G, 8G
    uint8_t buf[6];
    i2c_master_transfer_t xfer = {
        .slaveAddress = MMA8652_ADDR,
        .direction = kI2C_Read,
        .subaddress = MMA8652_OUT_X_MSB, // Le registre de départ qui contient la partie MSB de l'axe X
        .subaddressSize = 1,
        .data = buf,
        .dataSize = 6,
        .flags = kI2C_TransferDefaultFlag
    };

    //Ajustement du dataSize en fonction de la résolution
    //Si la résolution configurée est de 8 bits (MMA8652_RES_8), la taille des données à lire est réduite à 3 octets (un seul octet par axe)

    if (mma8652_cfg & MMA8652_RES_8) xfer.dataSize = 3;

    //La transaction I2C est initiée de manière non bloquante. Cela signifie que la fonction commence le transfert et retourne immédiatement, tandis que le transfert se poursuit en arrière-plan. Le statut i2c_res est d'abord défini à kStatus_I2C_Busy pour indiquer que la transaction est en cours.

    i2c_res = kStatus_I2C_Busy;
    I2C_MasterTransferNonBlocking(i2c, &i2c_handle, &xfer);

    while (i2c_res == kStatus_I2C_Busy) {
    }

    if (i2c_res != kStatus_Success) return kStatus_Fail;

    /* Combine MSB and LSB for X, Y, Z */
    int16_t x = (int16_t)((buf[0] << 8) | buf[1]) >> 4;
    int16_t y = (int16_t)((buf[2] << 8) | buf[3]) >> 4;
    int16_t z = (int16_t)((buf[4] << 8) | buf[5]) >> 4;

    /* Convert according to the table 15 */

    switch(scale) {
        case 0: // ±2g
            data[0] = ((1000 * x + 512) >> 10);
            data[1] = ((1000 * y + 512) >> 10);
            data[2] = ((1000 * z + 512) >> 10);
            break;
        case 1: // ±4g
            data[0] = ((1000 * x + 256) >> 9);
            data[1] = ((1000 * y + 256) >> 9);
            data[2] = ((1000 * z + 256) >> 9);
            break;
        case 2: // ±8g
            data[0] = ((1000 * x + 128) >> 8);
            data[1] = ((1000 * y + 128) >> 8);
            data[2] = ((1000 * z + 128) >> 8);
            break;
        default:
            return kStatus_Fail;
    }

    return i2c_res;
}


#ifdef FIND_SD_LIB
/***************************************************************************

 * SD Card Device Driver

 ***************************************************************************/

static int dev_init_sd(Device *dev);

static int dev_open_sd(FileObject *f);

static int dev_close_sd(FileObject *f);

static int dev_read_sd(FileObject *fileObj, void *buffer, size_t length);

static int dev_write_sd(FileObject *fileObj, const void *buffer, size_t length);

bool is_sd_card_present;

static bool card_ready = false;

Device dev_sd = {

 .name = "sd",

 .refcnt = 0,

 .init=dev_init_sd,

 .open=dev_open_sd,

 .close=dev_close_sd,

 .write = dev_write_sd,

 .read=dev_read_sd

};


#define RING_BUF_SIZE 32

sd_card_t card;


status_t sd_init(sd_card_t *card, sd_cd_t detectCb, void *userData)

{

 SDK_ALIGN(static uint32_t s_sdmmcHostDmaBuffer[64], SDMMCHOST_DMA_DESCRIPTOR_BUFFER_ALIGN_SIZE);

 static sd_detect_card_t s_cd;

 static sdmmchost_t s_host;


 /* attach main clock to SDIF */

 CLOCK_AttachClk(kMAIN_CLK_to_SDIO_CLK);

 /* need call this function to clear the halt bit in clock divider register */

 CLOCK_SetClkDiv(kCLOCK_DivSdioClk, (uint32_t)(SystemCoreClock / FSL_FEATURE_SDIF_MAX_SOURCE_CLOCK + 1U), true);


 memset(card, 0U, sizeof(sd_card_t));

 card->host = &s_host;

 card->host->dmaDesBuffer = s_sdmmcHostDmaBuffer;

 card->host->dmaDesBufferWordsNum = 64;

 card->host->hostController.base = SDIF;

 card->host->hostController.sourceClock_Hz = CLOCK_GetSdioClkFreq();


 /* install card detect callback */

 s_cd.cdDebounce_ms = 100u;

 s_cd.type = kSD_DetectCardByHostCD;

 s_cd.callback = detectCb;

 s_cd.userData = userData;

 card->usrParam.cd = &s_cd;


// NVIC_SetPriority(SDIO_IRQn, 5);


 return SD_HostInit(card);

}



static void sd_info(sd_card_t *card)

{

 assert(card);


 printf("\nCard size %d * %d bytes\n", card->blockCount, card->blockSize);

 printf("Working condition:\n");

 if (card->operationVoltage==kSDMMC_OperationVoltage330V) {

 printf(" Voltage : 3.3V\n");

 } else if (card->operationVoltage==kSDMMC_OperationVoltage180V) {

 printf(" Voltage : 1.8V\n");

 }


 if (card->currentTiming == kSD_TimingSDR12DefaultMode) {

 if (card->operationVoltage==kSDMMC_OperationVoltage330V) {

 printf(" Timing mode: Default mode\n");

 } else if (card->operationVoltage==kSDMMC_OperationVoltage180V) {

 printf(" Timing mode: SDR12 mode\n");

 }

 } else if (card->currentTiming==kSD_TimingSDR25HighSpeedMode) {

 if (card->operationVoltage==kSDMMC_OperationVoltage180V) {

 printf(" Timing mode: SDR25\n");

 } else {

 printf(" Timing mode: High Speed\n");

 }

 } else if (card->currentTiming==kSD_TimingSDR50Mode) {

 printf(" Timing mode: SDR50\n");

 } else if (card->currentTiming==kSD_TimingSDR104Mode) {

 printf(" Timing mode: SDR104\n");

 } else if (card->currentTiming==kSD_TimingDDR50Mode) {

 printf(" Timing mode: DDR50\n");

 }


 printf("\r\n Freq : %d HZ\r\n",card->busClock_Hz);

}


static void card_detect_cb(bool isInserted, void *userData)

{

 if (isInserted) {

 printf("SD card inserted\n");


 is_sd_card_present = true;



 } else {

 printf("SD card removed\n");

 is_sd_card_present = false;


 }

}

// Initialize serial device

static int dev_init_sd(Device *dev)

{

 BOARD_InitPins();

 BOARD_InitPins2();

 BOARD_InitBootClocks();

 BOARD_SDIF0ClockConfiguration();

 BOARD_SD_Config(&card, card_detect_cb, 5, NULL);

 SD_HostInit(&card);

 sd_init(&card,card_detect_cb,NULL);



 dev->mutex = sem_new(1); // Initialize semaphore for thread synchronization

 return 1; // Initialization succeeded

}

#define SD_BLOCK_SIZE 512

// Open the serial device

static int dev_open_sd(FileObject *f)

{


 sem_p(f->dev->mutex);


 if (!is_sd_card_present) {

 printf("Error: SD card is not present.\n");

 sem_v(f->dev->mutex);

 return -1;

 }

 for (int k=0;k<50000;k++);

 SD_CardInit(&card);

 sd_info(&card);

 if (f->dev->refcnt > 0) {

 sem_v(f->dev->mutex);

 return 0;

 }



 f->dev->refcnt++;

 sem_v(f->dev->mutex);

 return 1;

}
// Close the serial device
static int dev_close_sd(FileObject *f)

{

 sem_p(f->dev->mutex);

 f->dev->refcnt--;

 sem_v(f->dev->mutex);

 return 1; // Successfully closed

}

// Read from serial device using ring buffer

static int dev_write_sd(FileObject *fileObj, const void *buffer, size_t length)

{

 sem_p(fileObj->dev->mutex);



 uint32_t blockAddr = fileObj->offset / SD_BLOCK_SIZE;

 uint32_t blockCount = (length + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;


 if (blockAddr >= card.blockCount) {

 printf("Error: Block address exceeds available blocks on the SD card.\n");

 sem_v(fileObj->dev->mutex);

 return -1;

 }


 if (blockAddr + blockCount > card.blockCount) {

 printf("Error: Write operation exceeds available blocks on the SD card.\n");

 sem_v(fileObj->dev->mutex);

 return -1;

 }


 status_t status = SD_WriteBlocks(&card, (const uint8_t *)buffer, blockAddr, blockCount);

 if (status != kStatus_Success) {

 printf("Error: SD_WriteBlocks failed with status %d\n", status);

 sem_v(fileObj->dev->mutex);

 return -1;

 }



 fileObj->offset += length;


 sem_v(fileObj->dev->mutex);

 return length;

}

static int dev_read_sd(FileObject *fileObj, void *buffer, size_t length) {



 sem_p(fileObj->dev->mutex);


 uint32_t blockAddr = fileObj->offset / SD_BLOCK_SIZE;

 uint32_t blockCount = (length + SD_BLOCK_SIZE - 1) / SD_BLOCK_SIZE;




 FileObject backup_fileObj = *fileObj;


 status_t status = SD_ReadBlocks(&card, buffer, blockAddr, blockCount);


 *fileObj = backup_fileObj;

 fileObj->offset += length;

 sem_v(fileObj->dev->mutex);


 printf("FileObject after read: %p\n", (void*)fileObj);

 return length;

}

#endif


/***************************************************************************
 * Device table
 ***************************************************************************/
Device * device_table[]={
	&dev_test,
	&dev_leds,
    &dev_swuser,
	&dev_serial,
	&dev_accelerometer,
#ifdef FIND_SD_LIB
	&dev_sd
#endif
	NULL
};

/*****************************************************************************
 * Target Init
 *****************************************************************************/
extern Semaphore* vfs_mutex;

void dev_init()
{
    int i=0;
    Device *dev=device_table[0];
    while (dev) {
        if (dev->init) dev->init(dev);
        dev=device_table[++i];
    }
    
    for (int i=0;i<MAX_OPENED_FDS;i++)
        opened_fds[i]=NULL;

    vfs_mutex=sem_new(1);
}

