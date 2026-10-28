# Formula sheet

## Reading G(s) = K / (s + a)

1. **Pole:** set the bottom to zero, s + a = 0 → s = −a. It tells how fast the system
   responds and settles (or doesn't).
2. **Stable?** Pole negative → stable. Positive (even slightly) → unstable, so stop here:
   there's no final value.
3. **Time constant:** τ = 1/a
4. **Settling time:** ≈ 4τ (2 % rule)
5. **Final value:** set s = 0 → G(0) × step size. Only if stable.
6. **Digital version:** p = e^(−a·T), T in seconds. Stable if |p| < 1; wobbles if p is
   negative.
