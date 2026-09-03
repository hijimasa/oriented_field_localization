# リリースレビュー記録規則

[English](README.en.md) | 日本語

このディレクトリは、公開判断に至るまでの指摘と対応を監査可能な形で保存する。
利用者向けの現行仕様は上位ディレクトリの文書を正とし、過去レビュー中の記述を
現在の仕様として引用しない。

## ファイル名

```text
rNN-YYYY-MM-DD-findings.md
rNN-YYYY-MM-DD-response.md
```

- `rNN`: 公開後に変更しない2桁のレビューID
- `YYYY-MM-DD`: findingsを確定した日付（ISO 8601）
- `findings`: レビュー時点の指摘、根拠、判定
- `response`: 指摘に対する変更、検証、残項目

findingsとresponseは同じID・日付を使い、相互にリンクする。後続レビューで結論が
変わっても過去記録を書き換えず、後続記録と
[リリースレビュー履歴](../release_review_history.md)で訂正・撤回を明示する。

## 必須情報

- レビューIDと日付
- 対象package commit（未コミットなら基点commitとworktreeである旨）
- 対応するfindingsまたはresponseへのリンク
- 判定（`Hold`、`Conditional Go`、`Go`）
- 実施した検証と、未実施・失敗した検証
