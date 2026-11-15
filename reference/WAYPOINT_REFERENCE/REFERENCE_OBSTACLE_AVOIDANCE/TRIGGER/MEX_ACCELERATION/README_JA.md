# C方式の任意MEX高速化

このフォルダは **TRIGGER内の実行依存** です。旧`MEX_OPTIONAL`は不要です。

- `SOURCE/wp_six_trigger_C_chain_numeric_mex_v8.c`：C方式の球列AABB足切りカーネル（元V8と同じ）。
- `BIN/`：このPC用のMEXバイナリ自動生成先。配布物にバイナリは含みません。
- `reference_avoidance_mex_ensure.m`：**ファイルが無い場合／ソース更新時に自動ビルド**し、MATLABカーネルと80ケースの数値一致を確認。失敗時はMATLABに戻ります。
- `reference_avoidance_mex_verify.m`：独立した80ケースの入出力一致試験。
- `BUILD_REFERENCE_AVOIDANCE_C_MEX.m`：任意の強制リビルド関数。通常は呼ぶ必要なし。

## 自動動作

1. `reference_avoidance_prepare_observed(agent,'C',...)` でCを選んだ準備段階でビルドします。**B/Dはビルドしません。**
2. 80ケースの一致試験にPASSした場合のみ、`cfg.useMexCChainPreclear=true`にします。
3. 毎25 msの`reference_avoidance_tick`でファイルが消えた場合も自動で再ビルドします。ただし**この1回は秒単位でブロックし得ます**。Flightを起動する前に必ず準備関数を完了しておくことが必要です。ビルドに失敗すればMATLABへフォールバックし、失敗を記録します。
4. 繰り返しコンパイラ失敗時に毎25 msビルドし直さないよう、失敗をキャッシュします。`mex -setup C` を変更後の手動再試行は`BUILD_REFERENCE_AVOIDANCE_C_MEX(true)`です。

**本番硬リアルタイム性の保証はありません。** コンパイルを制御周期内で避けたい場合は `cfg.autoRebuildMexDuringTick=false` にして、次の準備タイミングで自動生成してください。`MEX`はC球列の粗判定だけで、B/DやHLC予測をMEX化したものではありません。幾何学的判定の安全証明を意味しません。
