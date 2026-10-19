AF7R57T V4 DEFAULT-WIRING CORRECTION AUDIT
==========================================

Run:
    T57V4 = AF7R57T_V4_DEFAULT_WIRING_CORRECTION_AUDIT();

Purpose:
- correct the V3 false-negative classification of backend 1
- bind exact current REPLANNING_SYSTEM and AF7R36 helper SHAs
- prove source order code==1 -> runtime_prime -> proposal -> locked callback
- keep backends 2-4 callback-only and fail-closed when unregistered
- make no production modification and no timing claim upgrade

Entrypoint SHA-256:
    64a4bb2cd15988d6f0c28e77756d9d7e842f9010eada061f26ff28fa91a85435
