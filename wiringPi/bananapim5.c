/*----------------------------------------------------------------------------*/
//
//
//	WiringPi BANANAPI-M5 Board Control file (AMLogic 64Bits Platform)
//
//
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <sys/mman.h>

/*----------------------------------------------------------------------------*/
#include "softPwm.h"
#include "softTone.h"

/*----------------------------------------------------------------------------*/
#include "wiringPi.h"
#include "bananapim5.h"

/*----------------------------------------------------------------------------*/
// wiringPi gpio map define
/*----------------------------------------------------------------------------*/
static const int pinToGpio[64] = {
	// wiringPi number to native gpio number
	479, 504,	//  0 |  1 : GPIOX.3, GPIOAO.8
	480, 483,	//  2 |  3 : GPIOX.4, GPIOX.7
	476, 477,	//  4 |  5 : GPIOX.0, GPIOX.1
	478, 481,	//  6 |  7 : GPIOX.2, GPIOX.5
	493, 494,	//  8 |  9 : GPIOX.17(I2C-2_SDA), GPIOX.18(I2C-2_SCL)
	486, 492,	// 10 | 11 : GPIOX.10(SPI_SS), GPIOX.16
	484, 485,	// 12 | 13 : GPIOX.8(SPI_MOSI), GPIOX.9(SPI_MISO)
	487, 488,	// 14 | 15 : GPIOX.11(SPI_CLK), GPIOX.12(UART_TX_B)
	489,  -1,	// 16 | 17 : GPIOX.13(UART_RX_B),
	 -1,  -1,	// 18 | 19 :
	 -1, 490,	// 20 | 21 : , GPIOX.14
	491, 482,	// 22 | 23 : GPIOX.15, GPIOX.6
	503, 505,	// 24 | 25 : GPIOAO.7, GPIOAO.9
	495, 432,	// 26 | 27 : GPIOX.19, GPIOH.5
	506, 500,	// 28 | 29 : GPIOAO.10, GPIOAO.4
	474, 475,	// 30 | 31 : GPIOA.14(I2C-3_SDA), GPIOA.15(I2C-3_SCL)
	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 32...47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// 48...63
};

static const int phyToGpio[64] = {
	// physical header pin number to native gpio number
	 -1,		//  0
	 -1,  -1,	//  1 |  2 : 3.3V, 5.0V
	493,  -1,	//  3 |  4 : GPIOX.17(I2C-2_SDA), 5.0V
	494,  -1,	//  5 |  6 : GPIOX.18(I2C-2_SCL), GND
	481, 488,	//  7 |  8 : GPIOX.5, GPIOX.12(UART_TX_B)
	 -1, 489,	//  9 | 10 : GND, GPIOX.13(UART_RX_B)
	479, 504,	// 11 | 12 : GPIOX.3, GPIOAO.8
	480,  -1,	// 13 | 14 : GPIOX.4, GND
	483, 476,	// 15 | 16 : GPIOX.7, GPIOX.0
	 -1, 477,	// 17 | 18 : 3.3V, GPIOX.1
	484,  -1,	// 19 | 20 : GPIOX.8(SPI_MOSI), GND
	485, 478,	// 21 | 22 : GPIOX.9(SPI_MISO), GPIOX.2
	487, 486,	// 23 | 24 : GPIOX.11(SPI_CLK), GPIOX.10(SPI_SS)
	 -1, 492,	// 25 | 26 : GND, GPIOX.16
	474, 475,	// 27 | 28 : GPIOA.14(I2C-3_SDA), GPIOA.15(I2C-3_SCL)
	490,  -1,	// 29 | 30 : GPIOX.14, GND
	491, 495,	// 31 | 32 : GPIOX.15, GPIOX.19
	482,  -1,	// 33 | 34 : GPIOX.6, GND
	503, 432,	// 35 | 36 : GPIOAO.7, GPIOH.5
	505, 506,	// 37 | 38 : GPIOAO.9, GPIOAO.10
	 -1, 500,	// 39 | 40 : GND, GPIOAO.4
	// Not used
	-1, -1, -1, -1, -1, -1, -1, -1,	// 41...48
	-1, -1, -1, -1, -1, -1, -1, -1,	// 49...56
	-1, -1, -1, -1, -1, -1, -1	// 57...63
};

/*----------------------------------------------------------------------------*/
//
// Global variable define
//
/*----------------------------------------------------------------------------*/
// wiringPi Pinmap control arrary
/*----------------------------------------------------------------------------*/
/* GPIO mmap control */
static volatile uint32_t *gpio;
static volatile uint32_t *gpioao;

/* wiringPi Global library */
static struct libodroid	*lib = NULL;

/*----------------------------------------------------------------------------*/
// Function prototype define
/*----------------------------------------------------------------------------*/
static int	isGpioAOPin	(int pin);
static int	isBananapiM5Pin (int pin);
static int	gpioToGPSETReg	(int pin);
static int	gpioToGPLEVReg	(int pin);
static int	gpioToPUENReg	(int pin);
static int	gpioToPUPDReg	(int pin);
static int	gpioToShiftReg	(int pin);
static int	gpioToGPFSELReg	(int pin);
static int	gpioToDSReg	(int pin);
static int	gpioToMuxReg	(int pin);

/*----------------------------------------------------------------------------*/
// wiringPi core function
/*----------------------------------------------------------------------------*/
static int		_getModeToGpio		(int mode, int pin);
static int		_setDrive		(int pin, int value);
static int		_getDrive		(int pin);
static int		_pinMode		(int pin, int mode);
static int		_getAlt			(int pin);
static int		_getPUPD		(int pin);
static int		_pullUpDnControl	(int pin, int pud);
static int		_digitalRead		(int pin);
static int		_digitalWrite		(int pin, int value);
static int		_digitalWriteByte	(const unsigned int value);
static unsigned int	_digitalReadByte	(void);

/*----------------------------------------------------------------------------*/
// board init function
/*----------------------------------------------------------------------------*/
static 	void init_gpio_mmap	(void);

void init_bananapim5 	(struct libodroid *libwiring);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Set regsiter
//
/*----------------------------------------------------------------------------*/
static int isGpioAOPin(int pin)
{
	if (pin >= M5_GPIOAO_PIN_START && pin <= M5_GPIOAO_PIN_END)
		return 1;
	else
		return 0;
}

static int isBananapiM5Pin(int pin)
{
	if (pin >= M5_GPIO_PIN_BASE && pin <= M5_GPIOAO_PIN_END)
		return 1;
	else
		return 0;
}

static int gpioToGPSETReg (int pin)
{
	if (pin >= M5_GPIOH_PIN_START && pin <= M5_GPIOH_PIN_END)
		return  M5_GPIOH_OUTP_REG_OFFSET;
	if (pin >= M5_GPIOA_PIN_START && pin <= M5_GPIOA_PIN_END)
		return  M5_GPIOA_OUTP_REG_OFFSET;
	if (pin >= M5_GPIOX_PIN_START && pin <= M5_GPIOX_PIN_END)
		return  M5_GPIOX_OUTP_REG_OFFSET;
	if (pin >= M5_GPIOAO_PIN_START && pin <= M5_GPIOAO_PIN_END)
		return  M5_GPIOAO_OUTP_REG_OFFSET;
	return	-1;
}

/*---------------------------------------------------------------------------r-*/
//
// offset to the GPIO Input regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToGPLEVReg (int pin)
{
	if (pin >= M5_GPIOH_PIN_START && pin <= M5_GPIOH_PIN_END)
		return  M5_GPIOH_INP_REG_OFFSET;
	if (pin >= M5_GPIOA_PIN_START && pin <= M5_GPIOA_PIN_END)
		return  M5_GPIOA_INP_REG_OFFSET;
	if (pin >= M5_GPIOX_PIN_START && pin <= M5_GPIOX_PIN_END)
		return  M5_GPIOX_INP_REG_OFFSET;
	if (pin >= M5_GPIOAO_PIN_START && pin <= M5_GPIOAO_PIN_END)
		return  M5_GPIOAO_INP_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down enable regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUENReg (int pin)
{
	if (pin >= M5_GPIOH_PIN_START && pin <= M5_GPIOH_PIN_END)
		return  M5_GPIOH_PUEN_REG_OFFSET;
	if (pin >= M5_GPIOA_PIN_START && pin <= M5_GPIOA_PIN_END)
		return  M5_GPIOA_PUEN_REG_OFFSET;
	if (pin >= M5_GPIOX_PIN_START && pin <= M5_GPIOX_PIN_END)
		return  M5_GPIOX_PUEN_REG_OFFSET;
	if (pin >= M5_GPIOAO_PIN_START && pin <= M5_GPIOAO_PIN_END)
		return  M5_GPIOAO_PUEN_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pull up/down regsiter
//
/*----------------------------------------------------------------------------*/
static int gpioToPUPDReg (int pin)
{
	if (pin >= M5_GPIOH_PIN_START && pin <= M5_GPIOH_PIN_END)
		return  M5_GPIOH_PUPD_REG_OFFSET;
	if (pin >= M5_GPIOA_PIN_START && pin <= M5_GPIOA_PIN_END)
		return  M5_GPIOA_PUPD_REG_OFFSET;
	if (pin >= M5_GPIOX_PIN_START && pin <= M5_GPIOX_PIN_END)
		return	M5_GPIOX_PUPD_REG_OFFSET;
	if (pin >= M5_GPIOAO_PIN_START && pin <= M5_GPIOAO_PIN_END)
		return	M5_GPIOAO_PUPD_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO bit
//
/*----------------------------------------------------------------------------*/
static int gpioToShiftReg (int pin)
{
	if (pin >= M5_GPIOH_PIN_START && pin <= M5_GPIOH_PIN_END)
		return  pin - M5_GPIOH_PIN_START;
	if (pin >= M5_GPIOA_PIN_START && pin <= M5_GPIOA_PIN_END)
		return  pin - M5_GPIOA_PIN_START;
	if (pin >= M5_GPIOX_PIN_START && pin <= M5_GPIOX_PIN_END)
		return  pin - M5_GPIOX_PIN_START;
	if (pin >= M5_GPIOAO_PIN_START && pin <= M5_GPIOAO_PIN_END)
		return  pin - M5_GPIOAO_PIN_START;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Function register
//
/*----------------------------------------------------------------------------*/
static int gpioToGPFSELReg (int pin)
{
	if (pin >= M5_GPIOH_PIN_START && pin <= M5_GPIOH_PIN_END)
		return  M5_GPIOH_FSEL_REG_OFFSET;
	if (pin >= M5_GPIOA_PIN_START && pin <= M5_GPIOA_PIN_END)
		return  M5_GPIOA_FSEL_REG_OFFSET;
	if (pin >= M5_GPIOX_PIN_START && pin <= M5_GPIOX_PIN_END)
		return  M5_GPIOX_FSEL_REG_OFFSET;
	if (pin >= M5_GPIOAO_PIN_START && pin <= M5_GPIOAO_PIN_END)
		return  M5_GPIOAO_FSEL_REG_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Drive Strength register
//
/*----------------------------------------------------------------------------*/
static int gpioToDSReg (int pin)
{
	if (pin >= M5_GPIOH_PIN_START && pin <= M5_GPIOH_PIN_END)
		return  M5_GPIOH_DS_REG_3A_OFFSET;
	if (pin >= M5_GPIOA_PIN_START && pin <= M5_GPIOA_PIN_END)
		return  M5_GPIOA_DS_REG_5A_OFFSET;
	if (pin >= M5_GPIOX_PIN_START && pin <= M5_GPIOX_PIN_MID)
		return  M5_GPIOX_DS_REG_2A_OFFSET;
	if (pin > M5_GPIOX_PIN_MID && pin <= M5_GPIOX_PIN_END)
		return  M5_GPIOX_DS_REG_2B_OFFSET;
	if (pin >= M5_GPIOAO_PIN_START && pin <= M5_GPIOAO_PIN_END)
		return  M5_GPIOAO_DS_REG_A_OFFSET;
	return	-1;
}

/*----------------------------------------------------------------------------*/
//
// offset to the GPIO Pin Mux register
//
/*----------------------------------------------------------------------------*/
static int gpioToMuxReg (int pin)
{
	switch (pin) {
	case	M5_GPIOH_PIN_START	...M5_GPIOH_PIN_END:
		return  M5_GPIOH_MUX_B_REG_OFFSET;
	case	M5_GPIOA_PIN_START	...M5_GPIOA_PIN_START + 7:
		return  M5_GPIOA_MUX_D_REG_OFFSET;
	case	M5_GPIOA_PIN_START + 8	...M5_GPIOA_PIN_END:
		return  M5_GPIOA_MUX_E_REG_OFFSET;
	case	M5_GPIOX_PIN_START	...M5_GPIOX_PIN_START + 7:
		return  M5_GPIOX_MUX_3_REG_OFFSET;
	case	M5_GPIOX_PIN_START + 8	...M5_GPIOX_PIN_START + 15:
		return  M5_GPIOX_MUX_4_REG_OFFSET;
	case	M5_GPIOX_PIN_START + 16	...M5_GPIOX_PIN_END:
		return  M5_GPIOX_MUX_5_REG_OFFSET;
	case	M5_GPIOAO_PIN_START	...M5_GPIOAO_PIN_START + 7:
		return  M5_GPIOAO_MUX_REG0_OFFSET;
	case	M5_GPIOAO_PIN_START + 8	...M5_GPIOAO_PIN_START + 11:
		return  M5_GPIOAO_MUX_REG1_OFFSET;
	default:
		return -1;
	}
}

/*----------------------------------------------------------------------------*/
static int _getModeToGpio (int mode, int pin)
{
	int retPin = -1;

	switch (mode) {
	/* Native gpio number */
	case	MODE_GPIO:
		retPin = isBananapiM5Pin(pin) ? pin : -1;
		break;
	/* Native gpio number for sysfs */
	case	MODE_GPIO_SYS:
		retPin = lib->sysFds[pin] != -1 ? pin : -1;
		break;
	/* wiringPi number */
	case	MODE_PINS:
		retPin = pin < 64 ? pinToGpio[pin] : -1;
		break;
	/* header pin number */
	case	MODE_PHYS:
		retPin = pin < 64 ? phyToGpio[pin] : -1;
		break;
	default	:
		msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
		return -1;
	}

	return retPin;
}

/*----------------------------------------------------------------------------*/
static int _setDrive (int pin, int value)
{
	int ds, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	if (value < 0 || value > 3) {
		msg(MSG_WARN, "%s : Invalid value %d (Must be 0 ~ 3)\n", __func__, value);
		return -1;
	}

	ds    = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin);
	
	if ( pin > M5_GPIOX_PIN_MID && pin <= M5_GPIOX_PIN_END)
		shift = (shift - 16) * 2;
	else
		shift = shift * 2;

	*((isGpioAOPin(pin) ? gpioao : gpio) + ds) &= ~(0b11 << shift);
	*((isGpioAOPin(pin) ? gpioao : gpio) + ds) |= (value << shift);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getDrive (int pin)
{
	int ds, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	ds    = gpioToDSReg(pin);
	shift = gpioToShiftReg(pin);
	
	if ( pin > M5_GPIOX_PIN_MID && pin <= M5_GPIOX_PIN_END)
		shift = (shift - 16) * 2;
	else
		shift = shift * 2;

	return (*((isGpioAOPin(pin) ? gpioao : gpio) + ds)	>> shift) & 0b11;
}

/*----------------------------------------------------------------------------*/
static int _pinMode (int pin, int mode)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int fsel, shift, origPin = pin;

	//printf("%s\n", __func__);

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) >= 0) {
		softPwmStop  (origPin);
		softToneStop (origPin);

		fsel  = gpioToGPFSELReg(pin);
		shift = gpioToShiftReg (pin);

		switch (mode) {
		case	INPUT:
			*((isGpioAOPin(pin) ? gpioao : gpio) + fsel) = (*((isGpioAOPin(pin) ? gpioao : gpio) + fsel) | (1 << shift));
			break;
		case	OUTPUT:
			*((isGpioAOPin(pin) ? gpioao : gpio) + fsel) = (*((isGpioAOPin(pin) ? gpioao : gpio) + fsel) & ~(1 << shift));
			break;
		case	SOFT_PWM_OUTPUT:
			softPwmCreate (pin, 0, 100);
			break;
		case	SOFT_TONE_OUTPUT:
			softToneCreate (pin);
			break;
		default:
			msg(MSG_WARN, "%s : Unknown Mode %d\n", __func__, mode);
			return -1;
		}
	} else {
		//printf("%s, not bananapi m5 pins\n", __func__);
		if ((node = wiringPiFindNode (origPin)) != NULL)
			node->pinMode (node, origPin, mode) ;	
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _getAlt (int pin)
{
	int fsel, mux, shift, target, mode;

	if (lib->mode == MODE_GPIO_SYS)
		return	-1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return	-1;

	fsel   = gpioToGPFSELReg(pin);
	mux    = gpioToMuxReg(pin);
	target = shift = gpioToShiftReg(pin);

	while (target >= 8) {
		target -= 8;
	}

	mode = (*((isGpioAOPin(pin) ? gpioao : gpio) + mux) >> (target * 4)) & 0xF;
	return	mode ? mode + 1 : (*((isGpioAOPin(pin) ? gpioao : gpio) + fsel) & (1 << shift)) ? 0 : 1;
}

/*----------------------------------------------------------------------------*/
static int _getPUPD (int pin)
{
	int puen, pupd, shift;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) < 0)
		return -1;

	puen  = gpioToPUENReg(pin);
	pupd  = gpioToPUPDReg(pin);
	shift = gpioToShiftReg(pin);

	if (*((isGpioAOPin(pin) ? gpioao : gpio) + puen) & (1 << shift))
		return *((isGpioAOPin(pin) ? gpioao : gpio) + pupd) & (1 << shift) ? 1 : 2;
	else
		return 0;
}

/*----------------------------------------------------------------------------*/
static int _pullUpDnControl (int pin, int pud)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int shift = 0, origPin = pin;

	//printf("%s, pin = %d, pud = %d\n", __func__, pin, pud);

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	if ((pin = _getModeToGpio(lib->mode, pin)) >= 0)
	{

		shift = gpioToShiftReg(pin);

		if (pud) {
			// Enable Pull/Pull-down resister
			*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUENReg(pin)) =
				(*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUENReg(pin)) | (1 << shift));

			if (pud == PUD_UP)
				*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUPDReg(pin)) =
					(*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUPDReg(pin)) |  (1 << shift));
			else
				*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUPDReg(pin)) =
					(*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUPDReg(pin)) & ~(1 << shift));
		} else	// Disable Pull/Pull-down resister
			*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUENReg(pin)) =
				(*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToPUENReg(pin)) & ~(1 << shift));
	} else {
		//printf("%s, not bananapi m5 pins, pin = %d, pud = %d\n", __func__, origPin, pud);
		if ((node = wiringPiFindNode (origPin)) != NULL)
			node->pullUpDnControl (node, origPin, pud) ;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _digitalRead (int pin)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	char c;
	int origPin = pin;

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] == -1)
			return -1;

		lseek	(lib->sysFds[pin], 0L, SEEK_SET);
		if (read(lib->sysFds[pin], &c, 1) < 0) {
			msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			return -1;
		}

		return	(c == '0') ? LOW : HIGH;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) >= 0)
	{
		if ((*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToGPLEVReg(pin)) & (1 << gpioToShiftReg(pin))) != 0)
			return HIGH ;
		else
			return LOW ;
	} else {
		if ((node = wiringPiFindNode (origPin)) != NULL)
                        return node->digitalRead (node, origPin) ;
	}

	return -1;
}

/*----------------------------------------------------------------------------*/
static int _digitalWrite (int pin, int value)
{
	struct wiringPiNodeStruct *node = wiringPiNodes;
	int origPin = pin;

	//printf("%s\n", __func__);

	if (lib->mode == MODE_GPIO_SYS) {
		if (lib->sysFds[pin] != -1) {
			if (value == LOW) {
				if (write(lib->sysFds[pin], "0\n", 2) < 0)
					msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			} else {
				if (write(lib->sysFds[pin], "1\n", 2) < 0)
					msg(MSG_WARN, "%s: Failed with reading from sysfs GPIO node. \n", __func__);
			}
		}
		return -1;
	}

	if ((pin = _getModeToGpio(lib->mode, pin)) >= 0) {

		if (value == LOW)
			*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToGPSETReg(pin)) &= ~(1 << gpioToShiftReg(pin));
		else
			*((isGpioAOPin(pin) ? gpioao : gpio) + gpioToGPSETReg(pin)) |=  (1 << gpioToShiftReg(pin));

	} else {
		if ((node = wiringPiFindNode (origPin)) != NULL)
			node->digitalWrite (node, origPin, value) ;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int _digitalWriteByte (const unsigned int value)
{
	union	reg_bitfield	gpiox;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	gpiox.wvalue = *(gpio + M5_GPIOX_INP_REG_OFFSET);

	/* Wiring PI GPIO0 = M5 GPIOX.3 */
	gpiox.bits.bit3 = (value & 0x01);
	/* Wiring PI GPIO1 = M5 GPIOX.16 */
	gpiox.bits.bit16 = (value & 0x02);
	/* Wiring PI GPIO2 = M5 GPIOX.4 */
	gpiox.bits.bit4 = (value & 0x04);
	/* Wiring PI GPIO3 = M5 GPIOX.7 */
	gpiox.bits.bit7 = (value & 0x08);
	/* Wiring PI GPIO4 = M5 GPIOX.0 */
	gpiox.bits.bit0 = (value & 0x10);
	/* Wiring PI GPIO5 = M5 GPIOX.1 */
	gpiox.bits.bit1 = (value & 0x20);
	/* Wiring PI GPIO6 = M5 GPIOX.2 */
	gpiox.bits.bit2 = (value & 0x40);
	/* Wiring PI GPIO7 = M5 GPIOX.5 */
	gpiox.bits.bit5 = (value & 0x80);

	*(gpio + M5_GPIOX_OUTP_REG_OFFSET) = gpiox.wvalue;

	return 0;
}

/*----------------------------------------------------------------------------*/
static unsigned int _digitalReadByte (void)
{
	union	reg_bitfield	gpiox;
	unsigned int		value = 0;

	if (lib->mode == MODE_GPIO_SYS)
		return -1;

	gpiox.wvalue = *(gpio + M5_GPIOX_INP_REG_OFFSET);

	/* Wiring PI GPIO0 = M5 GPIOX.3 */
	if (gpiox.bits.bit3)
		value |= 0x01;
	/* Wiring PI GPIO1 = M5 GPIOX.16 */
	if (gpiox.bits.bit16)
		value |= 0x02;
	/* Wiring PI GPIO2 = M5 GPIOX.4 */
	if (gpiox.bits.bit4)
		value |= 0x04;
	/* Wiring PI GPIO3 = M5 GPIOX.7 */
	if (gpiox.bits.bit7)
		value |= 0x08;
	/* Wiring PI GPIO4 = M5 GPIOX.0 */
	if (gpiox.bits.bit0)
		value |= 0x10;
	/* Wiring PI GPIO5 = M5 GPIOX.1 */
	if (gpiox.bits.bit1)
		value |= 0x20;
	/* Wiring PI GPIO6 = M5 GPIOX.2 */
	if (gpiox.bits.bit2)
		value |= 0x40;
	/* Wiring PI GPIO7 = M5 GPIOX.5 */
	if (gpiox.bits.bit5)
		value |= 0x80;

	return	value;
}

/*----------------------------------------------------------------------------*/
static void init_gpio_mmap (void)
{
	int fd = -1;
	void *mapped_gpio;
	void *mapped_gpioao;

	/* GPIO mmap setup */
	if (!getuid()) {
		if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
			msg (MSG_ERR,
				"wiringPiSetup: Unable to open /dev/mem: %s\n",
				strerror (errno));
	} else {
		if (access("/dev/gpiomem",0) == 0) {
			if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
				msg (MSG_ERR,
					"wiringPiSetup: Unable to open /dev/gpiomem: %s\n",
					strerror (errno));
			setUsingGpiomem(TRUE);
		} else
			msg (MSG_ERR,
				"wiringPiSetup: /dev/gpiomem doesn't exist. Please try again with sudo.\n");
	}

	if (fd < 0) {
		msg(MSG_ERR, "wiringPiSetup: Cannot open memory area for GPIO use. \n");
	} else {
		// #define M5_GPIO_BASE		0xff634000
#ifdef ANDROID
#if defined(__aarch64__)
		mapped_gpio = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M5_GPIO_BASE);
		mapped_gpioao = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M5_GPIO_AO_BASE);
#else
		mapped_gpio = mmap64(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, (off64_t)M5_GPIO_BASE);
		mapped_gpioao = mmap64(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, (off64_t)M5_GPIO_AO_BASE);
#endif
#else
		mapped_gpio = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M5_GPIO_BASE);
		mapped_gpioao = mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, M5_GPIO_AO_BASE);
#endif

		if (mapped_gpio == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		else
			gpio = (uint32_t *) mapped_gpio;

		if (mapped_gpioao == MAP_FAILED)
			msg(MSG_ERR, "wiringPiSetup: mmap (GPIO) failed: %s \n", strerror (errno));
		else
			gpioao = (uint32_t *) mapped_gpioao;
	}
}

/*----------------------------------------------------------------------------*/
void init_bananapim5 (struct libodroid *libwiring)
{
	init_gpio_mmap();

	/* wiringPi Core function initialize */
	libwiring->getModeToGpio	= _getModeToGpio;
	libwiring->setDrive		= _setDrive;
	libwiring->getDrive		= _getDrive;
	libwiring->pinMode		= _pinMode;
	libwiring->getAlt		= _getAlt;
	libwiring->getPUPD		= _getPUPD;
	libwiring->pullUpDnControl	= _pullUpDnControl;
	libwiring->digitalRead		= _digitalRead;
	libwiring->digitalWrite		= _digitalWrite;
	libwiring->digitalWriteByte	= _digitalWriteByte;
	libwiring->digitalReadByte	= _digitalReadByte;

	/* specify pin base number */
	libwiring->pinBase		= M5_GPIO_PIN_BASE;

	/* global variable setup */
	lib = libwiring;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
