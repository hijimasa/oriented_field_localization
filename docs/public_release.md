# Public 公開用 snapshot の作成

[English](en/public_release.md) | 日本語

`scripts/create_public_snapshot.sh` は、現在のリポジトリの履歴やrefを書き換えずに、
レビュー済みの`HEAD`から**1 commitだけの別リポジトリ**を作る。出力先が
既に存在する場合は上書きせずに停止し、source側に未commitまたは未追跡のfileが
ある場合も停止する。remoteの設定やpushは行わない。

## 手順

1. 公開対象をレビューし、それらをcommitする。`git status --short`が空であることを
   確認する。
2. 公開物に含めない固定文字列を1行に1つ書いたfileを、**この
   repositoryの外側**に作る。空行と`#`から始まる行は無視される。
3. 未使用の出力先pathを指定して実行する。

```bash
./scripts/create_public_snapshot.sh ../oriented-field-public \
  --deny-pattern-file /secure/path/public-release-deny-patterns.txt
```

scriptは追跡済みの`HEAD`だけをexportし、file内容とpathを固定文字列で検査する。
1件でも一致すれば非0で終了し、出力先は作らない。成功時は`main`に
`Public release snapshot`の1 commitだけを持つrepositoryができる。

## 公開前の確認

```bash
git -C ../oriented-field-public status --short
git -C ../oriented-field-public rev-list --count --all  # 1
git -C ../oriented-field-public log --oneline --all
```

出力先の全fileとlicenseを再確認した後、公開用remoteの設定とpushは
明示的に行う。元repositoryの履歴を維持したい場合と、過去commitを公開しない
要件を両立するための手順である。
