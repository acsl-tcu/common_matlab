# 再編第2版：移設元との関係

- `SAFETY_PREVIEW_GEOMETRY.zip`：`SYSTEM_APPROXIMATION`のC/B/D定義と`PREVIEW_TOOLS`へ移設。元ZIP・元フォルダは削除していません。
- `TRIGGER_LIGHT.zip`の本番候補V8：`TRIGGER/ENTRY_AND_OBSERVATION`, `FOUR_SECOND_PREDICTION`, `GEOMETRY_SCREENING`に配置。元コードの形状・未来予測・幾何数式は維持。
- 以前ルートにあった`MEX_OPTIONAL`：**削除して`TRIGGER/MEX_ACCELERATION`へ移設**。Cソース本体は同じ。MEX自動ビルド・数値一致の管理を新たに追加。
- 以前本番フォルダに同居した`VALIDATION`：`common_matlab/REFERENCE_OBSTACLE_AVOIDANCE_TESTS`へ移設し機能別のテストを追加。
- `SIMULATED_SENSOR`：依然未実装（フォルダ・形式説明のみ）。
- MATLAB既存旧`TRIGGER_LIGHT`や`SAFETY_PREVIEW_GEOMETRY`がパスに残る場合は `SETUP_REFERENCE_OBSTACLE_AVOIDANCE` が新配置を優先。起動後 `which wp_six_trigger_tick_v8 -all` 等で確認。
- まだ実Flightへ組み込んでいません。本配布に旧フォルダの削除処理はありません。
