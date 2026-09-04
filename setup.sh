#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

supports_project() {
  "$1" -c 'import sys; raise SystemExit(sys.version_info < (3, 10))' \
    >/dev/null 2>&1
}

case "${1:-}" in
  "") ;;
  -h|--help)
    echo "Usage: ./setup.sh"
    exit 0
    ;;
  *)
    echo "Unknown option: $1" >&2
    echo "Usage: ./setup.sh" >&2
    exit 2
    ;;
esac

python_executable="${CRAWLER_PYTHON:-}"
if [[ -n "${python_executable}" ]]; then
  if ! supports_project "${python_executable}"; then
    echo "CRAWLER_PYTHON must point to Python 3.10 or newer." >&2
    exit 1
  fi
else
  for candidate in python3 /usr/bin/python3 python3.12 python3.11 python3.10; do
    if command -v "${candidate}" >/dev/null 2>&1 && supports_project "${candidate}"; then
      python_executable="${candidate}"
      break
    fi
  done
  if [[ -z "${python_executable}" ]]; then
    echo "Python 3.10 or newer was not found." >&2
    exit 1
  fi
fi

if [[ ! -x "${SCRIPT_DIR}/.venv/bin/python" ]]; then
  if ! "${python_executable}" -m venv "${SCRIPT_DIR}/.venv"; then
    echo "Failed to create .venv. Install python3-venv and retry." >&2
    exit 1
  fi
elif ! supports_project "${SCRIPT_DIR}/.venv/bin/python"; then
  echo "Existing .venv uses Python older than 3.10; remove it and retry." >&2
  exit 1
fi
"${SCRIPT_DIR}/.venv/bin/python" -m pip install --upgrade pip
"${SCRIPT_DIR}/.venv/bin/python" -m pip install -r "${SCRIPT_DIR}/requirements.txt"
echo "MuJoCo environment: ${SCRIPT_DIR}/.venv"
echo "Run: ${SCRIPT_DIR}/.venv/bin/python ${SCRIPT_DIR}/scripts/crawler_mujoco --help"
