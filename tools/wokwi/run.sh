#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/../.."

if [[ "${WOKWI_CLI_TOKEN:-}" != wok_* || ${#WOKWI_CLI_TOKEN} -ne 44 ]]; then
    printf '%s\n' 'Exporta un token activo de https://wokwi.com/dashboard/ci como WOKWI_CLI_TOKEN.' >&2
    exit 1
fi

wokwi_cli="${WOKWI_CLI:-wokwi-cli}"
if ! command -v "$wokwi_cli" >/dev/null; then
    printf '%s\n' 'Instala wokwi-cli o indica su ruta en WOKWI_CLI.' >&2
    exit 1
fi

profile="${WOKWI_PROFILE:-pio}"
case "$profile" in
    pio) feature=wokwi-pio-test ;;
    dma) feature=wokwi-test ;;
    *) printf '%s\n' 'WOKWI_PROFILE debe ser pio o dma.' >&2; exit 1 ;;
esac

capture="target/wokwi/step-dir-$profile.vcd"
mkdir -p "$(dirname "$capture")"
(
    cd firmware/pico2
    cargo build --release --locked --features "$feature" --target-dir target/wokwi
)
rm -f -- "$capture"
"$wokwi_cli" . --timeout "${WOKWI_TIMEOUT_MS:-1500}" --timeout-exit-code 0 --vcd-file "$capture"
cargo run --quiet --locked -p scara-wokwi -- compare "$capture" --profile "$profile" "$@"
