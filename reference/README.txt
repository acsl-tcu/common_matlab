AF7R57N V6 — sticky resource abort for Stage-A / Stage-B
===========================================================

V5 found three real guard defects:
- Stage-A leaf cap set resource code 209 but planner could later succeed.
- Stage-B leaf cap was misreported as 212 because the split path used one
  combined reason for leaf and split exhaustion.
- Stage-B split cap set 212 but planner could continue to another candidate
  and later succeed.

V6 fixes those defects in the generated guarded shadow core:

1. Immediately after `tighten_stageA(...)`, if any resource code is set:
       R.success = false;
       R.reason  = resource_reason();
       return R;
   This is inserted in both native planner paths.

2. Stage-B split accounting is separated:
       leaf cap  -> 211
       split cap -> 212

3. Immediately after `stageB_speed(...)`, any resource code causes the same
   top-level sticky reject. No next-candidate fallback is allowed.

The episode 202/203 behavior is intentionally not hidden:
the existing op=4 fixed-episode wrapper may still transport those two resource
rejections as `fixed episode unavailable` exceptions. V6 continues to audit
them as fail-closed with exact op=9 reason codes, but production transport is
not closed until they are normalized to ordinary reject results.

Expected focused V6 result:
    209 -> got 209, transport RETURN, closed 1
    211 -> got 211, transport RETURN, closed 1
    212 -> got 212, transport RETURN, closed 1

If those close, only episode exception transport should remain open.
