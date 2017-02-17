/**
 * @file
 * @brief
 *
 * @author  Anton Kozlov
 * @date    02.07.2012
 */

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <kernel/critical.h>
#include <hal/reg.h>
#include <hal/ipl.h>
#include <drivers/irqctrl.h>

#include <kernel/irq.h>
#include <embox/unit.h>

#define NVIC_BASE 0xe000e100
#define NVIC_ENABLE_BASE NVIC_BASE
#define NVIC_CLEAR_BASE (NVIC_BASE + 0x80)
#define NVIC_SET_PEND_BASE (NVIC_BASE + 0x100)
#define NVIC_CLR_PEND_BASE (NVIC_BASE + 0x180)
#define NVIC_ACTIVE_BASE (NVIC_BASE + 0x200)
#define NVIC_PRIOR_BASE (NVIC_BASE + 0x300)

#define SCB_BASE 0xe000ed00
#define SCB_ICSR  (SCB_BASE + 0x04)
#define SCB_VTOR  (SCB_BASE + 0x08)
#define SCB_SHPR1 (SCB_BASE + 0x18)
#define SCB_SHPR3 (SCB_BASE + 0x20)

#define EXCEPTION_TABLE_SZ OPTION_GET(NUMBER,irq_table_size)

/**
 * ENABLE, CLEAR, SET_PEND, CLR_PEND, ACTIVE is a base of bit arrays
 * to calculate bit offset in array: calculate 32-bit word offset
 *     nr / 32 * sizeof(int) == nr / 8
 * and calculate bit offset in word
 *     nr / 32
 */

#ifndef STATIC_IRQ_EXTENTION

EMBOX_UNIT_INIT(nvic_init);

static uint32_t exception_table[EXCEPTION_TABLE_SZ] __attribute__ ((aligned (128 * sizeof(int))));

extern void *trap_table_start;
extern void *trap_table_end;

extern void __irq_trampoline(void);
extern void __pending_handle(void);
extern void __pendsv_handle(void);

struct cpu_saved_ctx {
	uint32_t r[5];
	uint32_t lr;
	uint32_t pc;
	uint32_t psr;
};

struct cpu_saved_ctx saved_ctx;
uint32_t saved_lr;
uint32_t saved_sp;
uint32_t saved_r7;

void interrupt_handle(void) {
	uint32_t source;
	struct cpu_saved_ctx *ctx;

	__asm__ __volatile__ (
		"mov %0, lr;\n\t"
		"mov %1, sp;\n\t"
	    : "=r"(saved_lr), "=r" (saved_sp)
	);
	/* 8 is size of the stuff saving on the stack when entering a function */
	saved_r7 = *(uint32_t*)(saved_sp + 8);
	/* 16 = 8 + sizeof(source) + sizeof (ctx) */
	ctx = (struct cpu_saved_ctx*) (saved_sp + 16);

	source = REG_LOAD(SCB_ICSR) & 0x1ff;

	assert(!critical_inside(CRITICAL_IRQ_LOCK));

	critical_enter(CRITICAL_IRQ_HANDLER);

	irq_dispatch(source);

	critical_leave(CRITICAL_IRQ_HANDLER);

	memcpy(&saved_ctx, ctx, sizeof saved_ctx);
	saved_ctx.pc |= 1;

	ctx->lr = (uint32_t) __pending_handle;
	ctx->pc = ctx->lr;
	/* It does not matter what value of psr is, just set up sime correct value.
	 * This value only used to go further, after return from interrupt_handle,
     * it will be changed to the correct one - saved_ctx.psr */
	ctx->psr = 0x01000000;

	/* Now return from interrupt context into __pending_handle */
	__irq_trampoline();
}

void nvic_set_pendsv(void) {
	REG_STORE(SCB_ICSR, 1 << 28);
}

static int nvic_init(void) {
	ipl_t ipl;
	int i;
	void *ptr;

	for (i = 0; i < EXCEPTION_TABLE_SZ; i++) {
		exception_table[i] = ((int) interrupt_handle) | 1;
	}
	exception_table[14] = ((int) __pendsv_handle) | 1;

	/* load head from bootstrap table */
	for (ptr = &trap_table_start, i = 0; ptr != &trap_table_end; ptr += 4, i++) {
		exception_table[i] = * (int32_t *) ptr;
	}

	ipl = ipl_save();

	REG_STORE(SCB_VTOR, 1 << 29 /* indicate, table in SRAM */ |
			(int) exception_table);

	ipl_restore(ipl);

	return 0;
}

#endif

void irqctrl_enable(unsigned int interrupt_nr) {
	int nr = (int) interrupt_nr - 16;
	if (nr >= 0) {
		REG_STORE(NVIC_ENABLE_BASE + 4 * (nr / 32), 1 << (nr % 32));
	}
}

void irqctrl_disable(unsigned int interrupt_nr) {
	int nr = (int) interrupt_nr - 16;
	if (nr >= 0) {
		REG_STORE(NVIC_CLEAR_BASE + 4 * (nr / 32), 1 << (nr % 32));
	}
}

void irqctrl_clear(unsigned int interrupt_nr) {
	int nr = (int) interrupt_nr - 16;
	if (nr >= 0) {
		REG_STORE(NVIC_CLR_PEND_BASE + 4 * (nr / 32), 1 << (nr % 32));
	}
}

void irqctrl_force(unsigned int interrupt_nr) {
	int nr = (int) interrupt_nr - 16;
	if (nr >= 0) {
		REG_STORE(NVIC_SET_PEND_BASE + 4 * (nr / 32), 1 << (nr % 32));
	}
}

static void hnd_stub(void) {
	/* It's just a stub. DO NOTHING */
}

void nvic_table_fill_stubs(void) {
	int i;

	for (i = 0; i < EXCEPTION_TABLE_SZ; i++) {
		exception_table[i] = ((int) hnd_stub) | 1;
	}

	REG_STORE(SCB_VTOR, 1 << 29 /* indicate, table in SRAM */ |
			(int) exception_table);
}
