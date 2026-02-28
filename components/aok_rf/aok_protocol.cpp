#include "aok_protocol.h"
#include "esphome/core/log.h"

namespace esphome {
namespace aok_rf {

// ─── Encoder ──────────────────────────────────────────────────────────────────
void AOKProtocol::encode(remote_base::RemoteTransmitData *dst, const AOKData &data) {
  uint64_t bits = data.to_uint64();

  // 2 sync items + 64 data bits×2 + 1 trailing bit×2 = 132 items
  dst->reserve(132);
  dst->set_carrier_frequency(0);  // OOK — no carrier

  // Sync pulse
  dst->item(AOK_SYNC_HIGH_US, AOK_SYNC_LOW_US);

  // 64 data bits, MSB first
  for (int i = 63; i >= 0; i--) {
    if ((bits >> i) & 1u) {
      dst->item(AOK_ONE_HIGH_US, AOK_ONE_LOW_US);
    } else {
      dst->item(AOK_ZERO_HIGH_US, AOK_ZERO_LOW_US);
    }
  }

  // Trailing '1' bit
  dst->item(AOK_ONE_HIGH_US, AOK_ONE_LOW_US);
}

// ─── Decoder ──────────────────────────────────────────────────────────────────
// Some A-OK remotes emit up to 7 '0' bits before the sync pulse.
// We therefore scan forward through the received items looking for
// the sync mark (≈5100 µs HIGH + ≈600 µs LOW) instead of requiring
// it to be the very first item.
//
// expect_item(mark, space) uses the tolerance configured on the
// remote_receiver component (tolerance: 40% in the YAML) — no third
// argument, that overload does not exist in ESPHome's remote_base.
optional<AOKData> AOKProtocol::decode(remote_base::RemoteReceiveData src) {
  // ── Scan for sync pulse ───────────────────────────────────────────────────
  // Skip any leading '0' bits (or other noise) until we find the long
  // sync HIGH. We allow up to 10 items to be skipped so the decoder is
  // not fooled by a buffer that starts mid-frame.
  bool sync_found = false;
  for (int skip = 0; skip <= 10; skip++) {
    if (src.expect_item(AOK_SYNC_HIGH_US, AOK_SYNC_LOW_US)) {
      sync_found = true;
      break;
    }
    // Not a sync item — try to consume one '0' bit and advance
    if (!src.expect_item(AOK_ZERO_HIGH_US, AOK_ZERO_LOW_US)) {
      // Not a valid '0' bit either; give up scanning
      break;
    }
    ESP_LOGV(TAG, "AOK decode: skipped leading '0' bit (skip=%d)", skip);
  }

  if (!sync_found) {
    return {};
  }

  // ── 64 data bits, MSB first ───────────────────────────────────────────────
  uint64_t bits = 0;
  for (int i = 63; i >= 0; i--) {
    if (src.expect_item(AOK_ONE_HIGH_US, AOK_ONE_LOW_US)) {
      bits |= (1ULL << i);
    } else if (src.expect_item(AOK_ZERO_HIGH_US, AOK_ZERO_LOW_US)) {
      // bit stays 0
    } else {
      ESP_LOGV(TAG, "AOK decode: invalid timing at bit %d", i);
      return {};
    }
  }

  // ── Trailing '1' — consume if present ────────────────────────────────────
  // The receiver may have already cut off the last LOW period.
  src.expect_item(AOK_ONE_HIGH_US, AOK_ONE_LOW_US);

  // ── Validate start byte ───────────────────────────────────────────────────
  uint8_t start = (bits >> 56) & 0xFF;
  if (start != 0xA3) {
    ESP_LOGV(TAG, "AOK decode: bad start byte 0x%02X (expected 0xA3)", start);
    return {};
  }

  // ── Unpack fields ─────────────────────────────────────────────────────────
  AOKData data;
  data.remote_id   = (bits >> 32) & 0x00FFFFFF;
  data.address     = (bits >> 16) & 0xFFFF;
  data.command     = (bits >>  8) & 0xFF;
  uint8_t rx_crc   =  bits        & 0xFF;
  uint8_t calc_crc = data.checksum();

  if (rx_crc != calc_crc) {
    ESP_LOGW(TAG, "AOK decode: checksum mismatch rx=0x%02X calc=0x%02X", rx_crc, calc_crc);
    return {};
  }

  return data;
}

// ─── Dump (logger) ────────────────────────────────────────────────────────────
void AOKProtocol::dump(const AOKData &data) {
  const char *cmd_str = "UNKNOWN";
  switch (data.command) {
    case AOK_CMD_UP:      cmd_str = "UP";      break;
    case AOK_CMD_STOP:    cmd_str = "STOP";    break;
    case AOK_CMD_DOWN:    cmd_str = "DOWN";    break;
    case AOK_CMD_PROGRAM: cmd_str = "PROGRAM"; break;
  }
  ESP_LOGI(TAG, "Received A-OK: remote_id=0x%06X address=0x%04X command=%s (0x%02X)",
           data.remote_id, data.address, cmd_str, data.command);
}

}  // namespace aok_rf
}  // namespace esphome
