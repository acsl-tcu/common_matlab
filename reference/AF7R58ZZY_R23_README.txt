AF7R58ZZY-R23-R3
=================

R23-R3 fixes the pre-motion synthetic postcallback-prime bridge only.
The inherited prime temporarily creates a certified REPAIRED authority before
R23 staged compilation exists. R23-R3 compiles the exact same canonical
correction model only for that temporary prime and attaches runtime metadata;
the inherited prime then restores the complete pre-prime authority.

The bridge is fail-closed and permitted only while all are false:
  online_prime_done
  generator_backend_kernel_prime_done
  postcallback_prime_done
A real Flight missing compiled metadata therefore fails; it cannot fall back.

Trajectory mathematics, continuous geometry/C7 certificates, speed/sensor
certificates, strict support, derivative recursion, correction order,
activation trigger+2dt and deadline rollback are unchanged.

Install:
  AF7R58ZZY_R23_R3_INSTALL
Run:
  run_flight_headless
  AF7R58ZZY_R23_REPORT
