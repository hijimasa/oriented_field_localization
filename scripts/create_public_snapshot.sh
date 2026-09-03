#!/usr/bin/env bash
# Create a one-commit public repository from the reviewed, tracked HEAD.
# The source repository is read-only: this script never changes its refs or index.
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  create_public_snapshot.sh DESTINATION --deny-pattern-file FILE

FILE contains one fixed string per line. Blank lines and lines beginning with '#'
are ignored. Keep FILE outside this repository so the strings are not published.
EOF
}

if [ "$#" -ne 3 ] || [ "$2" != "--deny-pattern-file" ]; then
    usage >&2
    exit 2
fi

DESTINATION="$1"
DENY_FILE="$3"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
SOURCE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd -P)"

if [ ! -f "${DENY_FILE}" ]; then
    echo "error: deny-pattern file does not exist: ${DENY_FILE}" >&2
    exit 2
fi
if [ -z "${DESTINATION}" ] || [ "${DESTINATION}" = "." ] || [ "${DESTINATION}" = ".." ]; then
    echo "error: DESTINATION must name a new directory" >&2
    exit 2
fi

DEST_PARENT="$(cd "$(dirname "${DESTINATION}")" && pwd -P)"
DEST_NAME="$(basename "${DESTINATION}")"
DEST_FINAL="${DEST_PARENT}/${DEST_NAME}"
if [ -e "${DEST_FINAL}" ]; then
    echo "error: destination already exists; refusing to overwrite: ${DEST_FINAL}" >&2
    exit 2
fi
case "${DEST_FINAL}" in
    "${SOURCE_DIR}"|"${SOURCE_DIR}"/*)
        echo "error: destination must be outside the source repository" >&2
        exit 2
        ;;
esac

if [ -n "$(git -C "${SOURCE_DIR}" status --porcelain --untracked-files=all)" ]; then
    echo "error: source repository is not clean" >&2
    echo "Commit the reviewed release files and resolve every untracked file first." >&2
    exit 1
fi

PATTERN_COUNT=0
while IFS= read -r pattern || [ -n "${pattern}" ]; do
    case "${pattern}" in
        ""|'#'*) continue ;;
    esac
    PATTERN_COUNT=$((PATTERN_COUNT + 1))
done < "${DENY_FILE}"
if [ "${PATTERN_COUNT}" -eq 0 ]; then
    echo "error: deny-pattern file contains no patterns" >&2
    exit 2
fi

TEMP_ROOT="$(mktemp -d)"
STAGE_DIR="${TEMP_ROOT}/repository"
cleanup() {
    rm -rf -- "${TEMP_ROOT}"
}
trap cleanup EXIT
mkdir "${STAGE_DIR}"

# git archive exports tracked content only and never copies the source .git directory.
git -C "${SOURCE_DIR}" archive --format=tar HEAD | tar -xf - -C "${STAGE_DIR}"
git -C "${STAGE_DIR}" init --quiet --initial-branch=main
git -C "${STAGE_DIR}" add --all

FOUND=0
while IFS= read -r pattern || [ -n "${pattern}" ]; do
    case "${pattern}" in
        ""|'#'*) continue ;;
    esac
    if git -C "${STAGE_DIR}" grep --cached -n -F -e "${pattern}" -- .; then
        FOUND=1
    fi
    while IFS= read -r path; do
        case "${path}" in
            *"${pattern}"*)
                echo "forbidden string in path: ${path}" >&2
                FOUND=1
                ;;
        esac
    done < <(git -C "${STAGE_DIR}" ls-files)
done < "${DENY_FILE}"
if [ "${FOUND}" -ne 0 ]; then
    echo "error: snapshot contains a denied fixed string; no destination was created" >&2
    exit 1
fi

git -C "${STAGE_DIR}" \
    -c user.name='Release Snapshot' \
    -c user.email='release-snapshot@invalid' \
    commit --quiet -m 'Public release snapshot'

if [ "$(git -C "${STAGE_DIR}" rev-list --count --all)" -ne 1 ]; then
    echo "error: internal check failed: snapshot history is not exactly one commit" >&2
    exit 1
fi

mv -- "${STAGE_DIR}" "${DEST_FINAL}"
echo "Created one-commit public snapshot: ${DEST_FINAL}"
echo "Review it, configure the intended remote, and push it explicitly."
