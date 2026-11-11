今回の修正（2026-11-11）
==========================

■ 対象
TEST_DRONE_ELLIPSOID_DIRECT_TEMP.m の検証ロジックだけを更新。
本番の楕円体フィット関数、障害物シーン、DRAW、runには変更なし。

■ 修正理由
元形状が球・楕円体の場合は、直接フィットが元形状を解析的に一致させる。
従来テストは、元形状のさらに外側にある外接支持多面体の頂点まで含めるという
誤った条件を要求していたため、sphere_demoで偽の失敗を起こしていた。

■ 今回の検証手順
1. 同梱 TEST_DRONE_ELLIPSOID_DIRECT_TEMP.m を commonmatlab/ に置いて上書き。
2. MATLABで commonmatlab/ をカレントフォルダにする。
3. 既存の直接フィット実装がMATLABパスにあることを確認する。
   which obs_approx_single_ellipsoid -all
   which obs_approx_multiple_ellipsoids -all
4. TEST_DRONE_ELLIPSOID_DIRECT_TEMP を実行する。
5. すべてPASSしたら、この一時TESTファイルは削除してよい。

■ 新しい判定の内容
・球／楕円体：中心、回転行列、軸長の解析的包含条件＋境界点テスト。
・それ以外：外接支持多面体の全頂点包含テスト＋実体内サンプル。
・要素数予算、姿勢変換、旧箱ベースとの体積比較は既存のまま。

■ 制限
MATLABはこの環境にはないため、修正版テストの実行結果はまだ未確認。
新たなエラーが出た場合はエラーログを確認して切り分ける。

■ 完全ガイド
同梱の DRONE_OBSTACLES_最新版_完全使用ガイド_20261111.txt は
元形状13種類、近似6方式、シーン、動的軌道、地面、描画、run、検証の全手順を含む。
