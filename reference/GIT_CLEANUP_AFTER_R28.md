# Git cleanup after the R28 checkpoint

Yes: old **generated test outputs** may be deleted after you make the R28
checkpoint commit/tag. Do not bulk-delete all `AF7R*` files because several
lowercase helpers and R21/R23/R25 files are still runtime dependencies.

## Keep
- `reference/REPLANNING_SYSTEM.m`
- `REPLANNING_SYSTEM_R21_BALANCED.m`
- current `REPLANNING_SYSTEM_R28_NATIVE_ONCE.m` (until R29 is accepted)
- `reference/af7r36_single_core_mex.cpp`
- `reference/af7r36_core.hpp`
- `af7r33_*`, `af7r34_*`, `af7r58i_*`
- `af7r75_*`, `af7r76_*`, `af7r77_*`
- `run_flight_headless.m`
- any source file actually returned by `which(...)` for the active run

## Normally safe to remove from Git/worktree after archiving the checkpoint
- `tp*/` and nested `**/tp*/` audit/snapshot directories
- `AF7R58ZZY_*_BACKUP_*/`
- `AF7R58ZZY_*_BUILD_*/`
- old `*_OUT.mat`
- old `*_RAW_CAPTURE.mat`
- old generated `*_REPORT.txt`
- old ZIP packages already preserved elsewhere
- superseded one-off install/repair scripts once their resulting source state
  has been committed/tagged

## Recommended `.gitignore`
```gitignore
**/tp*/
AF7R58ZZY_*_BACKUP_*/
AF7R58ZZY_*_BUILD_*/
*_OUT.mat
*_RAW_CAPTURE.mat
*_REPORT.txt
*.zip
*.mexw64
```

If an output/report is part of a paper result you want to preserve, archive it
outside the source tree or commit only that selected report before ignoring the
general pattern.
