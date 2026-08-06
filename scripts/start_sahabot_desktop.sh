#!/usr/bin/env bash
set -uo pipefail

readonly APP_DIRECTORY="/home/sahabat/sahabot"
readonly APP_URL="http://127.0.0.1:8000/?screen=explore&runtime=electron&build=20260806-battery-tour-v1"
readonly SERVER="${APP_DIRECTORY}/.venv/bin/uvicorn"
readonly ELECTRON="${APP_DIRECTORY}/node_modules/electron/dist/electron"
readonly ELECTRON_APP="${APP_DIRECTORY}/electron/main.cjs"
readonly LOG_FILE="/tmp/sahabot-app.log"

if [[ ! -x "${SERVER}" || ! -x "${ELECTRON}" || ! -f "${ELECTRON_APP}" ]]; then
  exit 1
fi

if ! curl --fail --silent --max-time 1 "${APP_URL}" >/dev/null 2>&1; then
  cd "${APP_DIRECTORY}" || exit 1
  nohup "${SERVER}" main:app \
    --app-dir backend \
    --host 127.0.0.1 \
    --port 8000 \
    >>"${LOG_FILE}" 2>&1 </dev/null &
fi

for _attempt in $(seq 1 30); do
  if curl --fail --silent --max-time 1 "${APP_URL}" >/dev/null 2>&1; then
    cd "${APP_DIRECTORY}" || exit 1
    exec "${ELECTRON}" --no-sandbox "${ELECTRON_APP}" "${APP_URL}" \
      >>"${LOG_FILE}" 2>&1
  fi
  sleep 1
done

exit 1
