    .global kalmanFilter
.syntax unified
.thumb

kalmanFilter:
	# floats are each 4 bytes, the struct is located at R0 and we want each value q, r, x p which are found at each 4 bytes

    VLDR S1, [R0, #0]		// q
    VLDR S2, [R0, #4]		// r
    VLDR S3, [R0, #8]		// x
    VLDR S4, [R0, #12]		// p
    VLDR S5, [R0, #16]		// k

    // p = p + q
	VADD.F32 S4, S4, S1

    // k = p / (p + r)
	VADD.F32 S6, S4, S2
	VDIV.F32 S5, S4, S6

    // x = x + k * (measurement - x)
	VSUB.F32 S6, S0, S3
	VMUL.F32 S6, S5, S6
	VADD.F32 S3, S3, S6

    // p = (1 - k) * p
    VMOV.F32 S7, #1.0
    VSUB.F32 S6, S7, S5
    VMUL.F32 S4, S6, S4

    // Loading it back into memor
    VSTR S1, [R0, #0]		// q
    VSTR S2, [R0, #4]		// r
    VSTR S3, [R0, #8]		// x
    VSTR S4, [R0, #12]		// p
    VSTR S5, [R0, #16]		// k

    BX LR                   // Return from subroutine
