#Power Factor
Power Factor is the ratio between Real (W) and Apparent (VA) power.

The process for measuring those power parameters is described below:

- Record V and I measurements multiple times a single cycle (20ms)
- Calculate Vrms and Irms from the V and I readings above

Interrogate each **positive half wave** like this:

- For each recording instant calculate VI
- If VI is positive -> add VI to VA_ACC
- If VI is negative -> add VI to W_ACC

Procedure to calculate final VA and W:

- IF VA_ACC > 0 THEN VA = VA_ACC / reading_count
- IF W_ACC > 0 THEN W = W_ACC / reading_count
- W=W-VA
- IF W < 0 THEN W=0
- VA=VA+W
- IF W > VI THEN W = VI

