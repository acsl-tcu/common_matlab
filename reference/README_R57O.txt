AF7R57O GLOBAL THEORY / SOURCE / SCOPE CONSISTENCY RE-AUDIT
===========================================================

Purpose
-------
Read-only re-audit of the AF7 R57 chain before any resource stress envelope.
It separates:
  1) continuous-time safety mathematics,
  2) finite algorithmic resources,
  3) fail-closed transport,
  4) platform timing qualification,
  5) final production commit authority.

This audit does NOT modify production source, build MEX, generate a trajectory,
or select production resource caps.

Run
---
Place AF7R57O_GLOBAL_THEORY_CONSISTENCY_AUDIT.m in the reference folder and run:

    O57 = AF7R57O_GLOBAL_THEORY_CONSISTENCY_AUDIT();

No replanner_instance is required for this audit.

Entrypoint SHA-256
------------------
a112e3389afc3848e0ae6f752a3597024c0c7ebfce6ecf3dc4254321a8de878d

Interpretation
--------------
The audit may intentionally report OPEN items.  OPEN means the earlier result
was scope-limited or a resource theorem is still missing; it does NOT turn a
sampled diagnostic into a proof and does NOT weaken the fail-closed policy.

Expected key distinction
------------------------
- R57K safety mathematics: preserved if exact source bindings pass.
- V10/V11 fail-closed resource transport: preserved if exact source bindings pass.
- V12 split/depth dominance: retained as a valid source-bound lemma.
- V12 'independent basis/global finite work': re-evaluated against active-reference
  parse/eval cost, corridor construction, obstacle-motion scope, and timing scope.
