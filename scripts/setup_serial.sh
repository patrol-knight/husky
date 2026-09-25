#!/usr/bin/env bash
# Create a persistent /dev/clearpath/prolific symlink for the PL2303
# USB-to-RS232 adapter used to talk to the Husky A200 MCU, and make sure the
# current user can access it (dialout group). One-time host setup, needed
# before launch/husky.launch.py can open the serial port as a normal user.
#
# This script is READ + PRINT before it WRITES: every change it intends to
# make is echoed first. It only writes to:
#   /etc/udev/rules.d/99-clearpath-prolific.rules   (new file)
#   dialout group membership of the invoking user    (via usermod -aG)
# It does not touch any other system configuration.
#
# Usage:
#   ./scripts/setup_serial.sh              # detect + apply
#   ./scripts/setup_serial.sh --dry-run    # show what would change, do nothing
#   ./scripts/setup_serial.sh --vendor-id 067b --product-id 2303   # force IDs
set -euo pipefail

RULE_FILE="/etc/udev/rules.d/99-clearpath-prolific.rules"
DRY_RUN=false
FORCE_VENDOR=""
FORCE_PRODUCT=""
# Most common Prolific PL2303 vendor:product ID. Used only as a fallback
# suggestion when no adapter is currently plugged in -- other PL2303 chip
# revisions (e.g. 067b:23a3, 067b:23b3) exist and, if yours differs, pass
# --vendor-id/--product-id explicitly (check with `lsusb`).
DEFAULT_VENDOR="067b"
DEFAULT_PRODUCT="2303"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=true; shift ;;
    --vendor-id) FORCE_VENDOR="$2"; shift 2 ;;
    --product-id) FORCE_PRODUCT="$2"; shift 2 ;;
    -h|--help)
      awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

fail() { echo "[setup_serial] ERROR: $*" >&2; exit 1; }
info() { echo "[setup_serial] $*"; }

command -v udevadm >/dev/null 2>&1 || fail "udevadm not found (package 'udev')."

VENDOR_ID=""
PRODUCT_ID=""
MATCHED_DEV=""

if [[ -n "${FORCE_VENDOR}" && -n "${FORCE_PRODUCT}" ]]; then
  VENDOR_ID="${FORCE_VENDOR}"
  PRODUCT_ID="${FORCE_PRODUCT}"
  info "Using explicitly-provided IDs: ${VENDOR_ID}:${PRODUCT_ID}"
else
  # Detect by actually inspecting connected /dev/ttyUSB* devices rather than
  # assuming ttyUSB0 or a single hardcoded VID:PID -- USB enumeration order
  # is not stable across reboots/reconnects.
  shopt -s nullglob
  candidates=(/dev/ttyUSB*)
  shopt -u nullglob

  if [[ ${#candidates[@]} -eq 0 ]]; then
    info "No /dev/ttyUSB* devices currently present."
    info "Falling back to the default Prolific PL2303 ID: ${DEFAULT_VENDOR}:${DEFAULT_PRODUCT}"
    info "(unverified -- plug in the adapter and re-run, or pass --vendor-id/--product-id, to confirm)"
    VENDOR_ID="${DEFAULT_VENDOR}"
    PRODUCT_ID="${DEFAULT_PRODUCT}"
  else
    for dev in "${candidates[@]}"; do
      props="$(udevadm info -q property -n "${dev}" 2>/dev/null || true)"
      vid="$(grep -oP '(?<=^ID_VENDOR_ID=).*' <<<"${props}" || true)"
      pid="$(grep -oP '(?<=^ID_MODEL_ID=).*' <<<"${props}" || true)"
      vendor_str="$(grep -oP '(?<=^ID_VENDOR=).*' <<<"${props}" || true)"
      if [[ "${vid}" == "${DEFAULT_VENDOR}" ]] || [[ "${vendor_str}" =~ [Pp]rolific ]]; then
        MATCHED_DEV="${dev}"
        VENDOR_ID="${vid}"
        PRODUCT_ID="${pid}"
        info "Detected Prolific-family adapter at ${dev} (${VENDOR_ID}:${PRODUCT_ID}, ${vendor_str:-unknown vendor})"
        break
      fi
    done

    if [[ -z "${VENDOR_ID}" ]]; then
      info "Found /dev/ttyUSB* device(s) but none matched a known Prolific vendor ID:"
      for dev in "${candidates[@]}"; do
        udevadm info -q property -n "${dev}" 2>/dev/null | grep -E '^(ID_VENDOR|ID_MODEL|ID_VENDOR_ID|ID_MODEL_ID)=' | sed "s/^/  ${dev}: /"
      done
      fail "Re-run with --vendor-id/--product-id matching your adapter (see IDs above), or plug in the Husky's PL2303 adapter."
    fi
  fi
fi

RULE_CONTENT="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"${VENDOR_ID}\", ATTRS{idProduct}==\"${PRODUCT_ID}\", SYMLINK+=\"clearpath/prolific\", GROUP=\"dialout\", MODE=\"0660\""

info "Planned udev rule (${RULE_FILE}):"
echo "  ${RULE_CONTENT}"

CURRENT_USER="${SUDO_USER:-${USER}}"
NEEDS_GROUP=false
if ! id -nG "${CURRENT_USER}" 2>/dev/null | grep -qw dialout; then
  NEEDS_GROUP=true
  info "User '${CURRENT_USER}' is not in the 'dialout' group (needed to open the adapter without root)."
fi

if ${DRY_RUN}; then
  info "--dry-run: no changes made."
  ${NEEDS_GROUP} && info "Would run: sudo usermod -aG dialout ${CURRENT_USER}"
  info "Would write ${RULE_FILE} and run: sudo udevadm control --reload-rules && sudo udevadm trigger"
  exit 0
fi

[[ ${EUID} -eq 0 ]] || fail "This step requires root. Re-run with: sudo $0 $*"

info "Writing ${RULE_FILE} ..."
printf '%s\n' "${RULE_CONTENT}" > "${RULE_FILE}"

info "Reloading udev rules..."
udevadm control --reload-rules
udevadm trigger

if ${NEEDS_GROUP}; then
  info "Adding '${CURRENT_USER}' to the 'dialout' group..."
  usermod -aG dialout "${CURRENT_USER}"
  info "You must log out/in (or reboot) for the new group membership to take effect."
fi

sleep 1
if [[ -e /dev/clearpath/prolific ]]; then
  info "OK: /dev/clearpath/prolific -> $(readlink -f /dev/clearpath/prolific)"
else
  info "Rule installed, but /dev/clearpath/prolific does not exist yet."
  info "This is expected if the adapter is not currently plugged in; it will appear on next connect."
fi
