# トリガ実装（選択1方式・4秒名目予測）

**外部入口は原則3つだけ**：`reference_avoidance_prepare_forecast`、`reference_avoidance_prepare_observed`、`reference_avoidance_tick`（上位フォルダ）。内部の歴史的関数名は動作互換のため改名しません。

| 機能フォルダ | 含むもの | 更新タイミング |
|---|---|---|
| `ENTRY_AND_OBSERVATION` | 最新EKF `mL`・状態、形状設定、観測済みキャッシュ、25ms履歴 | 観測更新時／毎25ms |
| `FOUR_SECOND_PREDICTION` | 元HLC 6次・2次誤差モード、6段Governor、元参照経路PCHIP | キャッシュ周期外、予測は必要時のみ |
| `GEOMETRY_SCREENING` | 全体足切り、各区間包絡、C/B/Dの判定。`fast_v3/v4`も**現行依存** | 選択された1方式のみ |
| `MEX_ACCELERATION` | C方式の球列AABB用Cソース、生成物、MEXの自動生成・80条件一致試験 | Cの観測準備で自動生成、実行中の消失時にも再生成可能 |

V8はC方式で `wp_six_trigger_C_chain_aabb_v8` を追加採用し、B/DはV4/V3へ委譲します。MEXはC方式のAABB数値カーネルだけを高速化します。MEX無しでもC方式のMATLAB経路で動作します。

公式の`reference_avoidance_tick`は予測→幾何に加え、選択MEXの存在確認と実行時間計測を行います。実行中のMEX再コンパイルには秒単位の遅延があり得るため、**必ずFlight開始前に`prepare_observed`を呼ぶ**こと。タイムクリティカル用途では`cfg.autoRebuildMexDuringTick=false`にして、次回観測更新の準備時に自動再構築してください。

`NOMINAL_POLYLINE_SEPARATED__NOT_CERTIFIED`は4秒の名目折れ線に対する判定であって、物理的な安全保証ではありません。元のモデル、HLC、CBF、参照生成器は変更しません。模擬センサは別フォルダで今後実装します。
