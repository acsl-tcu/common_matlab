# Runtime authority after final integration

The runtime replanner class is now intended to be only:

- `reference/REPLANNING_SYSTEM.m`
- SHA-256 `643d35abe47f7c79f5ff796ea476988469a02909ebd053bb71c668527790ea9e`

`run_flight_headless.m` instantiates `REPLANNING_SYSTEM` directly. Historical
`REPLANNING_SYSTEM_R19...R33` classes are no longer runtime dependencies after
the final integrated run is validated.

Exact support source authorities packaged here:

| file | SHA-256 |
|---|---|
| `af7r75_posaccel_batch_R19.m` | `69b0996e0a5dd2fa55a580214f9f8edbb517e0ebca7ed614a49814e612560f80` |
| `af7r76_compile_correction_packet_R23.m` | `e0f3ab777a2f2ccb9fdd02c3b10bae775aa8a37a4aeafac8ed402d7216a75177` |
| `af7r77_compile_correction_packet_R25.m` | `8be00803a58c2563e12ab4ef0f2e666a7161304e596724f39e616e6cc90efb44` |
| `af7r77_merge_result_reference_fast_R25.m` | `f7b5f4c46710c0bc4ee64c3a7e839717a14bc9821291bb8e15bddefeef976a36` |
| `af7r77_compile_dcp_mex.cpp` | `c6005d87c2033375cc17a76c8aad3a13b50f4a772eb4a600a49eb6571ed23e42` |
| `af7r79_packet_retarget_mex.cpp` | `794194666d68dead1a884467c82bb3dee484528a0f8a6f62f3cdab0570d07a60` |
| `af7r36_single_core_mex.cpp` | `bd4474a41ae93eba3e42f152ae8ba582ddf0dd59a89853fa07a90f79fbe5520a` |
| `af7r36_core.hpp` | `ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be` |

Existing MEX binaries that remain prerequisites and are not rebuilt by the
final installer:

- `af7r70_observer_mex`
- `af7r71_decasteljau_batch_mex`
- `af7r75_correction_posaccel_mex`
- `af7r76_compiled_eval_mex`
- `af7r77_compile_dcp_mex`
- `af7r36_single_core_mex` — revision must be `AF7R58ZZY_R28_NATIVE_ONCE_V1`
- `af7r79_packet_retarget_mex` — revision must be `AF7R58ZZY_R29_PACKET_RETARGET_V1`

The installer verifies these before changing the production source.
