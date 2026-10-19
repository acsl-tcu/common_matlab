AF7R57N V8 — branch-local episode return normalization
========================================================

Why V7 stopped
--------------
V7 assumed the phrase "fixed episode unavailable" occurred exactly once in
the corrected serializer source.  The live source contains that same message
in multiple branches, so the source-binding assertion failed before any MEX
rebuild or runtime test.  This did not invalidate V6.

V8 correction
-------------
V8 makes no global occurrence-count assumption.  It examines every exact
throw-shaped line independently and patches a branch only when a UNIQUE nearest
existing `plhs[0]=result_struct(...)` serializer call is available in the local
source window.  The branch's existing result schema/context is reused.

Only resource reasons 202/203 are converted to ordinary rejection.  Other
`fixed episode unavailable` conditions remain exceptions.

The runtime fault-injection gate remains decisive:
  202 -> RETURN, success=false, exact reason 202
  203 -> RETURN, success=false, exact reason 203
All other exercised resource codes must also remain RETURN + closed=1.

V8 retains the QPC gates from V7:
- admitted current packet: 2000 trials
- reason 202 reject: 1000 trials
- reason 203 reject: 1000 trials
Every measured trial must be <4 ms; no percentile relaxation.

Production caps are still not selected and production-general 4 ms is not
claimed.
