# Release review record policy

English | [日本語](README.md)

This directory is an audit archive of findings and responses leading to a public-release
decision. Current user-facing documents in the parent directory are authoritative; historical
review statements are not the current specification.

## File names

```text
rNN-YYYY-MM-DD-findings.md
rNN-YYYY-MM-DD-response.md
```

- `rNN`: immutable two-digit review ID
- `YYYY-MM-DD`: date on which the findings were finalized (ISO 8601)
- `findings`: findings, evidence, and decision at review time
- `response`: changes, verification, and remaining work

Findings and responses use the same ID and date and link to each other. A later decision does
not rewrite an older record; it explicitly corrects or withdraws it in a later record and in
the [release review history](../en/release_review_history.md).

## Required information

- review ID and date
- package commit under review (or the base commit plus an explicit worktree note)
- link to the corresponding findings or response
- decision: `Hold`, `Conditional Go`, or `Go`
- verification performed, skipped, or failed
