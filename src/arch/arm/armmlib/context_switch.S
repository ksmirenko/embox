/**
 * @file
 * @brief
 *
 * @author  Anton Kozlov
 * @date    25.10.2012
 */

.text
.thumb
.syntax unified
.global context_switch
context_switch:

    stmia   r0, {r0 - r12, r14}
    str	    sp, [r0, #56]
    mrs     r2, CONTROL
    str     r2, [r0, #60]

    ldr     sp, [r1, #56]
    ldr     r2, [r1, #60]
    msr     CONTROL, r2

    add     r0, r1, #56
    ldmdb   r0, {r2 - r12, r14}
    ldr     r0, [r1]
    ldr     r1, [r1, #4]

    mov	    pc, lr

.text
.thumb
.syntax unified
.global __irq_trampoline
__irq_trampoline:

    cpsid  i
    ldr    sp, =saved_sp
    ldr    sp, [sp]
    add    sp, #16

    ldr    r14, =saved_lr
    ldr    r14, [r14]
    bx     r14


.text
.thumb
.syntax unified
.global __pendsv_handle
__pendsv_handle:

    add    sp, #32
    bx     r14

.text
.thumb
.syntax unified
.global __pending_handle
__pending_handle:

    ldr    r0, =saved_ctx

    ldr    r7, =saved_r7
    ldr    r7, [r7]

    # push saved_ctx on top of the stack
    add    r0, #16
    ldmia  r0, {r1 - r3, r12}
    push   {r1 - r3, r12}
    sub    r0, #16
    ldmia  r0, {r1 - r3, r12}
    push   {r1 - r3, r12}

    cpsie  i
    bl     critical_dispatch_pending
    cpsid  i

    bl     nvic_set_pendsv
    cpsie  i
fl: b      fl
