# Creating a public-release snapshot

English | [日本語](../public_release.md)

`scripts/create_public_snapshot.sh` creates a **separate, one-commit repository** from the
reviewed `HEAD` without rewriting the source repository's history or refs. It refuses to
overwrite an existing destination and stops if the source has staged, unstaged, or untracked
work. It never configures a remote or pushes anything.

## Procedure

1. Review and commit the intended release files. Confirm that `git status --short` is empty.
2. Outside this repository, create a file containing one fixed string per line for every string
   that the public artifact must not contain. Blank lines and lines beginning with `#` are
   ignored.
3. Run the script with a destination path that does not exist:

```bash
./scripts/create_public_snapshot.sh ../oriented-field-public \
  --deny-pattern-file /secure/path/public-release-deny-patterns.txt
```

The script exports only files tracked by `HEAD`, scans both file contents and paths for every
fixed string, and exits nonzero without creating the destination if it finds a match. On
success, the output is a repository whose `main` branch has one `Public release snapshot`
commit.

## Checks before publication

```bash
git -C ../oriented-field-public status --short
git -C ../oriented-field-public rev-list --count --all  # 1
git -C ../oriented-field-public log --oneline --all
```

Review every exported file and its licence once more, then configure the public remote and push
explicitly. This procedure preserves the source repository's history while keeping its earlier
commits out of the published repository.
