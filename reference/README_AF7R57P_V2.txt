AF7R57P V2 - ACTIVE-REFERENCE PRE-PARSE FORENSIC

Run only:
    P57 = AF7R57P_V2_ACTIVE_REFERENCE_PREPARSE_FORENSIC_AUDIT();

V2 changes only the read-only C++ source parser utility:
- uses char(92) for backslash (no MATLAB multi-character literal ambiguity),
- ignores C++ // and /* */ comments while matching braces,
- handles C++ char/string literals and escaped characters,
- runs a synthetic lexical self-test before reading authority functions.

No safety theorem, resource theorem, source authority, acceptance condition,
or timing claim is changed from R57P V1.

Entrypoint SHA-256:
    810b9105f9167fc77293e6e8c1bda0549cd6e0cc51b28ac7d2889cff40234674
