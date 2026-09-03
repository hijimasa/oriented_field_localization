# リリースレビュー履歴

[English](en/release_review_history.md) | 日本語

この文書は公開前レビューの現在の判断を要約する。詳細な根拠と対応は
[`reviews/`](reviews/)を参照する。

| ID | 日付 | 対象 | 指摘 | 対応 | 判定 |
|---|---|---|---|---|---|
| R01 | 2026-09-03 | `698f134` + R01対応commit | [Critical 0 / High 3 / Medium 9 / Low 5](reviews/r01-2026-09-03-findings.md) | [全件へ実装・公開方針で対応](reviews/r01-2026-09-03-response.md) | **Conditional Go** |

## 現在の状態

High 3件、Medium 9件、Low 5件へ対応し、ROS 2 Humbleのbuild、5 test、install後の
downstream link、最小評価、公開snapshotのpositive/negative検査、10秒のGazebo smokeが成功した。旧記述が残る
既存Git履歴は公開せず、cleanな最終commitからdeny-pattern検査済みの1 commit snapshotを作り、
その別repositoryだけを公開することがGo条件である。長時間Nav2 simulationと非0 LiDAR外部
パラメータを含む実ROS結合試験は未実施であり、実機投入前の検証項目として残る。
