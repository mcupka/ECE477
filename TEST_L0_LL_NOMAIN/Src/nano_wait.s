.cpu cortex-m0
.thumb
.syntax unified

.global nano_wait
nano_wait:
	subs r0, #83
	bge nano_wait
	bx lr

.global wait_for_interrupt
wait_for_interrupt:
	wfi
