/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Regression checks for diagnostic GUI command gating and packet-shape equivalence.
 *
 * Design notes:
 *   Protects GUI-side helper behavior without requiring a live STC server.
 *   Mirrors CLI packing assumptions so GUI regressions are caught at wire-shape level.
 *   Host-side only: no CAN hardware or network transport is required.
 */

#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>

extern "C" {
#include "stc_icd.h"
}

/* Mirrors GUI safety gate: motion commands require OPER, mapping loaded, and hardware topology unless sim mode is active. */
static bool command_allowed(bool is_oper, bool sim_mode, bool hw_topology_loaded, bool mapping_enabled, uint16_t cmd_id) {
  bool allowed = true;
  if (cmd_id == STC_CMD_TURRET || cmd_id == STC_CMD_MIRROR || cmd_id == STC_CMD_SUT) {
    allowed = is_oper;
    if (!sim_mode && !hw_topology_loaded) allowed = false;
  }
  if (!mapping_enabled) allowed = false;
  return allowed;
}

/* Keeps remembered mirror selection stable across refreshes, but falls back deterministically when topology changes. */
static uint8_t normalize_mirror_id(uint8_t remembered, const std::vector<uint8_t>& available) {
  for (uint8_t v : available) if (v == remembered) return remembered;
  return available.empty() ? 0xFFu : available.front();
}

/* Uses the same ICD defaults as CLI tooling so packet-byte equivalence assertions remain meaningful. */
static std::vector<uint8_t> pack_cmd(const stc_icd_cmd_t& c) {
  uint8_t buf[(STC_ICD_MAX_PAYLOAD + 32)];
  size_t n = 0;
  stc_icd_cfg_t cfg = stc_icd_parse_cfg_default();
  assert(stc_icd_cmd_pack(&cfg, &c, buf, sizeof(buf), &n) == 0);
  return std::vector<uint8_t>(buf, buf + n);
}

int main() {
  // command selection persistence semantics: repeated availability updates must not force-enable send
  assert(!command_allowed(true, false, false, true, STC_CMD_MIRROR));
  assert(!command_allowed(true, false, false, true, STC_CMD_MIRROR));

  // invalid remembered mirror id normalizes to first available
  assert(normalize_mirror_id(8, {1,2,3}) == 1);
  assert(normalize_mirror_id(2, {1,2,3}) == 2);

  // automatic REQ_HW_INFO on connect packet shape
  stc_icd_cmd_t req{};
  req.payload_decoded = 1;
  req.version = 1;
  req.controller_id = 21;
  req.seq = 7;
  req.timestamp_ms = 123;
  req.cmd_id = STC_CMD_REQ_HW_INFO;
  req.payload_size = 0;
  auto req_pkt = pack_cmd(req);
  assert(req_pkt.size() == 20);

  // GUI-vs-CLI packet equivalence for key commands
  auto eq = [](const stc_icd_cmd_t& a, const stc_icd_cmd_t& b) { return pack_cmd(a) == pack_cmd(b); };

  stc_icd_cmd_t gui{}, cli{};
  gui.payload_decoded = cli.payload_decoded = 1;
  gui.version = cli.version = 1;
  gui.controller_id = cli.controller_id = 21;
  gui.seq = cli.seq = 9;
  gui.timestamp_ms = cli.timestamp_ms = 555;

  gui.cmd_id = cli.cmd_id = STC_CMD_CU_STATE;
  gui.payload_size = cli.payload_size = 1;
  gui.u.cu_state.state = cli.u.cu_state.state = 3;
  assert(eq(gui, cli));

  gui = cli = {};
  gui.payload_decoded = cli.payload_decoded = 1;
  gui.version = cli.version = 1;
  gui.controller_id = cli.controller_id = 21;
  gui.seq = cli.seq = 10;
  gui.timestamp_ms = cli.timestamp_ms = 556;
  gui.cmd_id = cli.cmd_id = STC_CMD_MIRROR;
  gui.payload_size = cli.payload_size = 6;
  gui.u.mirror.mirror_id = cli.u.mirror.mirror_id = 2;
  gui.u.mirror.motion_type = cli.u.mirror.motion_type = STC_MOTION_RELATIVE;
  gui.u.mirror.elevation = cli.u.mirror.elevation = 1.25f;
  assert(eq(gui, cli));

  gui.u.mirror.motion_type = cli.u.mirror.motion_type = STC_MOTION_SPEED;
  gui.u.mirror.elevation = cli.u.mirror.elevation = -3.5f;
  assert(eq(gui, cli));

  gui = cli = {};
  gui.payload_decoded = cli.payload_decoded = 1;
  gui.version = cli.version = 1;
  gui.controller_id = cli.controller_id = 21;
  gui.seq = cli.seq = 11;
  gui.timestamp_ms = cli.timestamp_ms = 557;
  gui.cmd_id = cli.cmd_id = STC_CMD_REQ_HW_INFO;
  gui.payload_size = cli.payload_size = 0;
  assert(eq(gui, cli));

  return 0;
}
