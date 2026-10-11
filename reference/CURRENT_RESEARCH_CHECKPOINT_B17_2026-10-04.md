# Multi-UAV Suspended-Load Real-Time Obstacle Avoidance
## Current Research Checkpoint — B17
**Date:** 2026-10-04  
**Status:** Success-first architecture + C6 safe stop + timing forensics

---

## 1. Current objective

Develop a real-time replanning method for **4 UAVs cooperatively transporting one cable-suspended payload** in an unknown environment, under the following requirements:

- One onboard simulated sensor.
- Obstacles are unknown until detected within a fixed sensing sphere.
- Replanning is event-driven: detection alone does not trigger replanning; future collision prediction does.
- Position reference must be globally **C6**.
- Nominal yaw is preserved; full `[position, yaw]` C6 requires nominal yaw itself to be C6.
- Final execution authority is a **continuous-time exact certificate**.
- Payload, UAVs, and cables are handled separately in the system separation bound.
- Obstacles are 3D ellipsoids.
- Recursive replanning from the currently active avoidance trajectory is supported.
- Old-plan retention is allowed only after exact re-certification against the updated known obstacle set.
- Safety-stop availability is tracked separately from avoidance success.
- The real-time target remains approximately **2–3 ms from sensing to corrected reference**, with ≤4 ms as an important hard target.

---

## 2. Frozen safety principles

The following design principles must not be weakened during optimization:

1. Sampled checks, broad-phase tests, predictors, and candidate-ranking logic are never execution authority.
2. Every executed trajectory must pass the exact continuous-time safety certificate.
3. Every active phase is certified against all currently known obstacles.
4. Candidate-generation heuristics may rank or prune only when theoretically safe to do so.
5. Do not count an early MISS as a real-time success.
6. Safety-stop success must never be counted as avoidance success.
7. Offline/preflight/warm-up costs must remain separate from online timing.
8. No arbitrary empirical weighted score should decide trajectory selection.
9. MATLAB optimization should be exhausted before deciding which kernels deserve MEX conversion.

---

## 3. Core trajectory representation

The avoidance offset uses the C6 smooth ramp

\[
S(\tau)
=
1716\tau^7
-9009\tau^8
+20020\tau^9
-24024\tau^{10}
+16380\tau^{11}
-6006\tau^{12}
+924\tau^{13}.
\]

Its derivative satisfies

\[
S'(\tau)=12012\tau^6(1-\tau)^6\ge0.
\]

The phase structure remains:

- **Departure:** \(p=p_{\rm base}+\rho n S(\tau)\)
- **Bypass:** \(p=p_{\rm base}+\rho n\)
- **Rejoin:** \(p=p_{\rm nom}+\rho n(1-S(\tau))\)

Representation-aware duration bounds are based on

\[
\|e_{\rm spline}^{(r)}\|
\le
\frac{\rho K_r(N)}{T^r}.
\]

Finite controller-derived numerical limits currently exist only for derivative orders 1–3. Orders 4–6 remain structurally represented but are not assigned guessed finite limits.

---

## 4. Ellipsoid and system safety model

Obstacle model:

\[
\mathcal O
=
\left\{
x:
(x-c)^\top S^{-1}(x-c)\le1
\right\},
\]

with support

\[
h_{\mathcal O}(n)
=
n^\top c+\sqrt{n^\top S n}.
\]

System separation uses the separate payload/UAV/cable envelope

\[
\Delta_{\rm sys}(n,n_T)
=
\max
\left[
R_{\rm payload},
R_{\rm uav}-L_{\rm cable}(n^\top n_T),
R_{\rm cable}
+
\max(0,-L_{\rm cable}(n^\top n_T))
\right].
\]

No single large enclosing sphere is used.

---

## 5. Controller-aware constraints

The uploaded HLC analysis established:

\[
r_x=r_y=r_z=6,
\qquad
r_\psi=2.
\]

The controller explicitly uses position derivatives through the 6th derivative.

Important regularity conditions include:

\[
|p_{T3}|>0,
\qquad
\sigma_{\min}(\beta_2)>0,
\]

together with actuator saturation margins.

The current exact authority includes the proven axis-based lower bound for

\[
W=a+ge_3
\]

and derives continuous-time lower bounds for cable tension and \(p_{T3}\) regularity.

---

## 6. Optimization progression

### B12
- Fast SAFE: **8/12**
- Main failures: 3 exact-geometry failures + 1 candidate miss.
- FAST POST median: approximately 5.9 ms.
- Exact certificate remained the dominant cost.

### B13
Hypothesis-validation bakeoff established:

- All-known-obstacle \(\rho\) alone did **not** fix the failures.
- Failed exact phases were caused by the **primary triggered obstacle itself**.
- Failures occurred mainly in Departure.
- Vectorized duration, relevant-slice \(t_2\), and combined exact gate were numerically equivalent to the prior implementation.

### B14
Departure-aware fixed-point candidate generation was tested.

Result:
- It was too conservative when used as a hard candidate condition.
- SAFE collapsed to **1/12**.
- However, one previously failing S-curve case became safe.

Conclusion:
- Departure-aware logic is useful as an additional candidate family or ranking aid.
- It must not replace the baseline bank.

### B15
Lazy certified multi-candidate search:

\[
L0:\text{B12-equivalent best}
\rightarrow
L1:\text{remaining baseline directions}
\rightarrow
L2:\text{additional candidate family}.
\]

Result:
- SAFE improved to **10/12**.
- Eight events were solved at `BASE_L0`.
- Two additional events were solved at `BASE_L1`.
- Lazy expansion was therefore validated as a useful success-rate mechanism.

### B16/B17
Multi-separator rejoin candidate logic was added.

Current ladder:

\[
L0:
\text{baseline best}
\rightarrow
L1:
\text{remaining fixed directions}
\rightarrow
L2:
\text{multi-separator rejoin}
\rightarrow
L3:
\text{adaptive obstacle-geometry directions}.
\]

The key change in L2 is that Phase-C candidate generation is no longer forced to use the same separating direction as the avoidance offset. The final exact authority was already free to use another separator, so candidate generation was relaxed to better match the authority.

---

## 7. Current B17 success result

### Fast avoidance

\[
\boxed{
\text{AVOID\_FAST\_SUCCESS}=11/12
}
\]

This corresponds to **91.7% fast avoidance success** in the current 12-event validation set.

### Slow shadow rescue

The one remaining fast failure is successfully solved by the slow shadow planner:

\[
\boxed{
\text{AVOID\_SHADOW\_SUCCESS}=1/1
}
\]

Therefore, a safe avoidance trajectory exists for all 12 tested events, but only 11 are currently found by the fast architecture.

### Remaining fast failure

The only remaining fast failure is:

- Mission: HELIX
- Event time: 79.15 s
- Obstacle: O3
- Result: `NO_CANDIDATE_ALL_BANKS`
- Fast planner tested 0 candidates
- Shadow planner succeeds
- Shadow runtime ≈ 487 ms

This is now the **single primary success-rate problem**.

The next success-rate task is therefore:

> Inspect the shadow solution at HELIX 79.15 s and identify the minimum missing degree of freedom in the fast candidate bank.

Do not broaden the entire planner unless the shadow analysis shows it is necessary.

---

## 8. C6 safe-stop status

Safety stop is tracked separately from avoidance.

Current result:

\[
\boxed{
\text{C6\_STOP\_KNOWN\_SAFE}=12/12
}
\]

and

\[
\boxed{
\text{C6\_STOP\_LIMITED\_SAFE}=12/12.
}
\]

The stop trajectory is now generated directly as a degree-12 Bezier curve rather than being fit through another spline representation.

The start jet satisfies

\[
p_{\rm stop}^{(r)}(t_0)
=
p_{\rm current}^{(r)}(t_0),
\qquad
r=0,\ldots,6,
\]

and the terminal derivatives satisfy

\[
p_{\rm stop}^{(r)}(t_0+T)=0,
\qquad
r=1,\ldots,6.
\]

The terminal position is then held, yielding a globally C6 translational stop.

The limited-sensing certificate additionally requires the complete payload/UAV/cable stop envelope to remain inside already sensed space.

Important:

> Safe-stop success does **not** increase avoidance-success statistics.

---

## 9. Current real-time performance

Current online medians:

\[
T_{\rm sensor}
\approx0.028\ {\rm ms},
\]

\[
T_{\rm trigger}
\approx1.59\ {\rm ms},
\]

\[
T_{\rm planner}
\approx6.75\ {\rm ms},
\]

and

\[
\boxed{
T_{\rm sensor+trigger+planner}
\approx8.75\ {\rm ms}\quad\text{median}.
}
\]

Therefore the target of approximately 2–3 ms end-to-end has **not yet been reached**.

The current architecture is functionally much stronger than before, but real-time performance is still the major remaining problem.

---

## 10. Key timing-forensics result

For already selected and certified successful trajectories, 20 excluded replay runs give approximately:

\[
T_{\rm trajectory\ kernel,median}
\approx2.12\ {\rm ms}.
\]

Typical components:

\[
T_{\rm build}
\approx0.17\ {\rm ms},
\]

\[
T_{\rm offset}
\approx0.009\ {\rm ms},
\]

\[
T_{\rm exact}
\approx1.93\ {\rm ms}.
\]

This is a critical result.

The online planner median is about 6.75 ms, while the chosen-trajectory kernel is only about 2.12 ms.

Therefore approximately 4–5 ms of the online planner cost comes from:

- candidate-bank generation,
- candidate selection,
- failed-candidate work,
- lazy L1/L2/L3 expansion,
- multi-separator trace generation,
- allocations,
- MATLAB cold/JIT/runtime jitter,
- other search overhead.

The trajectory construction itself is no longer the main bottleneck.

---

## 11. Cold vs steady execution

Measured cold/steady trajectory-kernel ratio:

- median ≈ 1.14
- maximum ≈ 2.22

Therefore MATLAB runtime effects are non-negligible.

Future profiling must separate:

1. algorithmic complexity,
2. first-use/JIT effects,
3. temporary-array allocation,
4. memory/cache behavior,
5. operating-system scheduling jitter.

---

## 12. Current confidence in timing sub-breakdown

The overall repeated values

- build,
- offset,
- exact,
- total trajectory kernel

are considered useful.

However, the current single-run deep exact decomposition

- dynamic,
- control polygon,
- support,
- projection,
- separator,
- reduction

is **not yet reliable enough for absolute millisecond conclusions**.

Some logged events show deep-kernel sub-times whose sum is much larger than the independently measured total exact time.

Therefore:

> Before MEX decisions, each exact sub-kernel must itself be benchmarked repeatedly (e.g. 20–50 repetitions) without mixing diagnostic timing overhead into the production path.

---

## 13. Current MEX position

Do **not** move the whole planner to MEX yet.

### Low priority for MEX

- Offset control-point addition (~0.01 ms)
- Basic trajectory build (~0.17 ms)

These are already too small to matter significantly.

### Potential MEX candidates after MATLAB optimization

- future-collision trigger kernel,
- exact projection,
- separator lower-bound computation,
- obstacle × direction × span reduction.

These scale approximately with:

\[
N_{\rm gate}
\times
N_{\rm span}
\times
N_{\rm obstacle}.
\]

Current successful trajectories commonly use:

- gate directions: 162
- total spans: 69
- known obstacles: 1–3

so the exact authority processes a substantial number of direction/span/obstacle combinations.

MEX should be considered only after:
1. repeated stable profiling,
2. allocation reduction,
3. precomputation/caching,
4. batch MATLAB operations.

---

## 14. Current priority order

### Priority A — reach 12/12 fast avoidance

Analyze HELIX 79.15 s shadow solution in detail:

- successful direction,
- \(\rho\),
- \(T_A\),
- \(t_2\),
- \(T_C\),
- candidate rank,
- separator family,
- whether the solution uses a direction outside the fixed bank,
- whether timing rather than geometry is the missing freedom.

Then add **only the required missing mechanism** to the fast candidate bank.

Target:

\[
\boxed{11/12\rightarrow12/12}
\]

without damaging the 11 already successful cases.

### Priority B — freeze the successful planner architecture

Once 12/12 is achieved:

- do not keep adding candidate families,
- freeze trajectory-generation logic,
- move to pure runtime optimization.

### Priority C — reduce online latency

Recommended order:

1. candidate/search overhead,
2. trigger,
3. exact authority,
4. remaining minor kernels.

### Priority D — repeat exact-kernel profiling

Benchmark each exact sub-kernel 20–50 times in isolation.

Classify each cost as:

- fixed,
- obstacle-count dependent,
- span-count dependent,
- candidate-count dependent,
- cold-only,
- cacheable,
- MATLAB-optimizable,
- probable MEX candidate.

### Priority E — optimize C6 safe stop

The safe-stop theorem works, but the current audit searches many duration candidates.

Replace brute-force duration stepping with a theoretically derived lower bound / monotone duration search.

Safe-stop optimization remains secondary to achieving 12/12 fast avoidance.

---

## 15. Current research position

The current system can be summarized as:

### Safety
Strong.

- continuous-time ellipsoidal safety authority,
- payload/UAV/cable separation,
- controller regularity,
- C6 trajectory construction,
- recursive replanning,
- exact re-certification,
- C6 limited-sensing safe stop.

### Fast avoidance success
Very close, but not complete.

\[
\boxed{11/12}
\]

### Existence of safe avoidance in current validation set

\[
\boxed{12/12}
\]

because the slow shadow method rescues the one remaining case.

### Fail-safe availability

\[
\boxed{12/12}
\]

for both known-obstacle-safe and current limited-sensing-safe stop certificates.

### Runtime

Still not acceptable for the final onboard objective.

\[
\boxed{
T_{\rm end-to-end,median}\approx8.75\ {\rm ms}
}
\]

but the certified chosen-trajectory kernel itself is already approximately

\[
\boxed{
2.12\ {\rm ms}.
}
\]

This strongly suggests that the next major gains should come from **search architecture, trigger optimization, caching, batching, and allocation reduction**, not from redesigning the C6 trajectory representation.

---

## 16. Next concrete experiment

The next code version should **not** broadly redesign the planner.

It should perform a targeted forensic comparison of:

\[
\text{HELIX 79.15 s fast failure}
\]

versus

\[
\text{HELIX 79.15 s shadow-safe solution}.
\]

The experiment should log:

- fast-bank directions and infeasibility reason,
- shadow-selected direction,
- shadow candidate rank,
- \(\rho\),
- \(T_A,T_B,T_C\),
- \(t_1,t_2,t_3\),
- exact phase margins,
- worst obstacle/span/direction,
- dynamic bounds,
- cable/regularity bounds,
- and incremental cost of adding the missing candidate freedom.

Only after this comparison should the fast bank be modified.

---

## 17. Files representing the current stage

Current main audit file:

`CURRENT_B17_SUCCESS_FIRST_C6_STOP_TIMING_FORENSICS_FIXED3.m`

Current B17 result log:

`貼り付けたMarkdown(20261003-160658).md`

This checkpoint should be treated as the restart point for subsequent development.
