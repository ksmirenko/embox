/**
 * @file
 *
 * @data 01.04.2016
 * @author: Anton Bondarev
 */

#include <drivers/common/memory.h>
#include <drivers/diag.h>
#include <drivers/serial/uart_device.h>
#include <drivers/serial/diag_serial.h>
#include <embox/unit.h>
#include <framework/mod/options.h>
#include <hal/reg.h>

EMBOX_UNIT_INIT(uart_init);

#define IOMUXC_BASE	OPTION_GET(NUMBER,iomuxc_base)
#define UART_NUM	OPTION_GET(NUMBER,num)
#define IRQ_NUM		(58 + UART_NUM)

#if UART_NUM == 0
#define UART_BASE 0x02020000
#elif UART_NUM == 1
#define UART_BASE 0x021E8000
#elif UART_NUM == 2
#define UART_BASE 0x021EC000
#elif UART_NUM == 3
#define UART_BASE 0x021F0000
#elif UART_NUM == 4
#define UART_BASE 0x021F4000
#else
#error Wrong UART number!
#endif

#define UART(x)		(*(volatile uint32_t *)(UART_BASE + (x)))

#define RXR		0x00
#define TXR		0x40
#define UCR1		0x80
# define UCR1_UARTEN	(1 << 0)
# define UCR1_RRDYEN	(1 << 9)
#define UCR2		0x84
# define UCR2_NORESET	(1 << 0)
# define UCR2_RXEN	(1 << 1)
# define UCR2_TXEN	(1 << 2)
# define UCR2_WORDSIZE	(1 << 5)
# define UCR2_IRTS	(1 << 14)
#define UCR3		0x88
# define UCR3_RXDMUXSEL (1 << 2)
# define UCR3_RI	(1 << 8)
# define UCR3_DSR	(1 << 9)
# define UCR3_DCD	(1 << 10)
#define UCR4		0x8C
#define UFCR		0x90
# define UFCR_RFDIF	7
#define USR1		0x94
# define USR1_RRDY	(1 << 9)
# define USR1_RTSD	(1 << 12)
#define USR2		0x98
# define USR2_RDR	1
# define USR2_TXFE	(1 << 14)
#define UESC		0x9C
#define UTIM		0xA0
#define UBIR		0xA4
#define UBMR		0xA8
#define ONEMS		0xB0
#define UTS		0xB4
# define UTS_RST	(1 << 0)
# define UTS_TXEMPTY	(1 << 6)
#define UMCR		0xB8

#include <kernel/printk.h>

#define UART_SELECT_PIN(n)	(IOMUXC_BASE + 0x920 + n * 8)
#define MUX_PIN_CTL_TX(n)	(IOMUXC_BASE + 0x2a8 + n * 8)
#define MUX_PIN_CTL_RX(n)	(IOMUXC_BASE + 0x2ac + n * 8)a

static void imxuart_configure_pins(void) {
	/* We need to configure UART pins to use them as
	 * RXD/TXD. To do it, we need first to config MUX mode register
	 * and then set source for the UART like follows; look manual for
	 * more details */

	switch(UART_NUM) {
	case 0:
		/* TX */
		REG32_STORE(IOMUXC_BASE + 0x2A8, 1);
		REG32_STORE(IOMUXC_BASE + 0x920, 2);
		/* RX */
		REG32_STORE(IOMUXC_BASE + 0x2AC, 1);
		REG32_STORE(IOMUXC_BASE + 0x920, 3);
		break;
	case 1:
		/* Nothing to be done */
		break;
	case 2:
		/* TX */
		REG32_STORE(IOMUXC_BASE + 0x0B4, 2);
		REG32_STORE(IOMUXC_BASE + 0x930, 0);
		/* RX */
		REG32_STORE(IOMUXC_BASE + 0x0B4, 2);
		REG32_STORE(IOMUXC_BASE + 0x930, 1);
		break;
	case 3:
		/* TX */
		REG32_STORE(IOMUXC_BASE + 0x1F8, 4);
		REG32_STORE(IOMUXC_BASE + 0x938, 0);
		/* RX */
		REG32_STORE(IOMUXC_BASE + 0x1FC, 4);
		REG32_STORE(IOMUXC_BASE + 0x938, 1);
		break;
	}

	return;
}

static int imxuart_setup(struct uart *dev, const struct uart_params *params) {
	imxuart_configure_pins();

	/* Reset */
	UART(UCR2) = 0;
	while(UART(UTS) & UTS_RST);

	UART(UCR1) = UCR1_UARTEN;
	UART(UCR2) = UCR2_NORESET | UCR2_RXEN | UCR2_TXEN | UCR2_WORDSIZE | UCR2_IRTS;
	UART(UCR3) = UCR3_RXDMUXSEL | UCR3_RI | UCR3_DCD | UCR3_DSR;
	UART(UCR4) = 0;
	UART(UFCR) = 4 << UFCR_RFDIF;
	UART(UBIR) = 0x0F;
	UART(UBMR) = 0x015b;
	UART(UMCR) = 0;

	if (params && params->irq) {
		uint32_t reg;

		reg = UART(UFCR);
		reg &= ~0x1F;
		reg |= 0x1; /* one character in RxFIFO */
		UART(UFCR) = reg;

		reg = UART(UCR1);
		reg |= UCR1_RRDYEN;
		UART(UCR1) = reg;
	}
	return 0;
}

static int imxuart_has_symbol(struct uart *dev) {
	return UART(USR2) & USR2_RDR;
}

static int imxuart_getc(struct uart *dev) {
	return (char)UART(RXR);
}

static int imxuart_putc(struct uart *dev, int ch) {
	UART(TXR) = ch;

	while (!(UART(UTS) & UTS_TXEMPTY)) {
	}

	return 0;
}

static const struct uart_ops imxuart_uart_ops = {
	.uart_getc = imxuart_getc,
	.uart_putc = imxuart_putc,
	.uart_hasrx = imxuart_has_symbol,
	.uart_setup = imxuart_setup,
};

static struct uart uart0 = {
	.uart_ops = &imxuart_uart_ops,
	.irq_num = IRQ_NUM,
	.base_addr = UART_BASE,
};

static const struct uart_params uart_defparams = {
	.baud_rate = OPTION_GET(NUMBER,baud_rate),
	.parity = 0,
	.n_stop = 1,
	.n_bits = 8,
	.irq = true,
};

static const struct uart_params uart_diag_params = {
	.baud_rate = OPTION_GET(NUMBER,baud_rate),
	.parity = 0,
	.n_stop = 1,
	.n_bits = 8,
	.irq = false,
};

const struct uart_diag DIAG_IMPL_NAME(__EMBUILD_MOD__) = {
	.diag = {
		.ops = &uart_diag_ops,
	},
	.uart = &uart0,
	.params = &uart_diag_params,
};

static int uart_init(void) {
	return uart_register(&uart0, &uart_defparams);
}

static struct periph_memory_desc imx_uart_mem = {
	.start = UART_BASE,
	.len   = 0xAC,
};

PERIPH_MEMORY_DEFINE(imx_uart_mem);

static struct periph_memory_desc iomuxc_mem = {
	.start = IOMUXC_BASE,
	.len   = 0x1000,
};

PERIPH_MEMORY_DEFINE(iomuxc_mem);
