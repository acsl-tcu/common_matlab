AF7R57N V9 — exact episode-wrapper source forensic
===================================================
Purpose: capture the exact R57K corrected MEX wrapper around every
"fixed episode unavailable" throw and the op-dispatch/serializer code.

This is READ ONLY:
- no source modification
- no MEX compilation
- no planner run
- no cap changes
- no production changes

Run from MATLAB after adding the folder containing this file to the path:

    V9 = AF7R57N_V9_EPISODE_WRAPPER_FORENSIC_AUDIT();

Paste the complete console output. The next patch will be built against the
exact op=4 branch rather than distance-based source assumptions.
