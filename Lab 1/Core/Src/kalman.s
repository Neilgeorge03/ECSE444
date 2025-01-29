.syntax unified
.align 16

.global KalmanFilter

KalmanFilter:
	push {lr}
    vpush {s16-s21}
    // s0 contains measurement
    vldr s16, [r0, #0]		// q
    vldr s17, [r0, #4]		// r
    vldr s18, [r0, #8]		// x
    vldr s19, [r0, #12]		// p
    vldr s20, [r0, #16]		// k
    vsub.f32 s21, s21, s21	// -> 0.0

    vadd.f32 s19, s19, s16	// self.p = self.p + self.q
	vmrs r1, FPSCR
	tst r1, #0b1111
	bne overflow_handler

    vadd.f32 s20, s19, s17	// self.k = self.p + self.r
    vmrs r1, FPSCR
	tst r1, #0b1111
	bne overflow_handler

    // handle zero div error (see epsilon method)
    vcmp.f32 s20, #0
    vmrs APSR_nzcv, FPSCR
    beq div_by_zero_handler


    vdiv.f32 s20, s19, s20 // self.k = selp.p / self.k
    vmrs r1, FPSCR
	tst r1, #0b1111
	bne overflow_handler

    vsub.f32 s21, s0, s18  // y = measurement - self.x
    vmrs r1, FPSCR
	tst r1, #0b1111
	bne overflow_handler

    vmla.f32 s18, s20, s21 // self.x = self.x + self.k * y
    vmrs r1, FPSCR
	tst r1, #0b1111
	bne overflow_handler

    vmov.f32 s21, #1.0 // y = 1 check if load 1 works
    vsub.f32 s21, s21, s20 // y = 1 - self.k
    vmrs r1, FPSCR
	tst r1, #0b1111
	bne overflow_handler

    vmul.f32 s19, s21, s19 // self.p = y * self.p
    vmrs r1, FPSCR
	tst r1, #0b1111
	bne overflow_handler

    vstr s16, [r0]      // q
    vstr s17, [r0, #4]  // r
    vstr s18, [r0, #8]  // x
    vstr s19, [r0, #12] // p
    vstr s20, [r0, #16] // k
    mov r0, #0 			// If success, return code 0
    B return

div_by_zero_handler:
	mov r0, #1
	B return

overflow_handler:
	mov r0, #2
	B return

return:
	vpop {s16-s21}
    bx lr // return to C
