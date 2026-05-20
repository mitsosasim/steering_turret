/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Internal module implementation for STC runtime support services.
 *
 * Design notes:
 *   Comment-only documentation refactor; runtime behavior is unchanged.
 */

#include "stc_config.h"
#include "stc_log.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <strings.h>
#include <sched.h>

static void trim_inplace(char* s) {
  if (!s) return;
  size_t n = strlen(s);
  while (n > 0 && isspace((unsigned char)s[n - 1])) {
    s[--n] = '\0';
  }

  size_t start = 0;
  while (s[start] && isspace((unsigned char)s[start])) start++;
  if (start > 0) memmove(s, s + start, strlen(s + start) + 1);
}

/** Parse sections like "Axis3"/"Mirror2" into bounded zero-based indices. */
static int parse_indexed_section(const char* section, const char* prefix, int max_count, int base_one) {
  const size_t prefix_len = strlen(prefix);
  if (strncmp(section, prefix, prefix_len) != 0) return -1;

  char* endp = NULL;
  long idx = strtol(section + prefix_len, &endp, 10);
  if (endp == section + prefix_len || *endp != '\0') return -1;

  if (base_one) idx -= 1;
  if (idx < 0 || idx >= max_count) return -1;
  return (int)idx;
}

static int parse_int(const char* value, int* out) {
  char* endp = NULL;
  long v = strtol(value, &endp, 0);
  if (endp == value || *endp != '\0') return -1;
  *out = (int)v;
  return 0;
}

static int parse_u16(const char* value, uint16_t* out) {
  char* endp = NULL;
  unsigned long v = strtoul(value, &endp, 0);
  if (endp == value || *endp != '\0' || v > 0xFFFFu) return -1;
  *out = (uint16_t)v;
  return 0;
}

static int parse_u8(const char* value, uint8_t* out) {
  char* endp = NULL;
  unsigned long v = strtoul(value, &endp, 0);
  if (endp == value || *endp != '\0' || v > 0xFFu) return -1;
  *out = (uint8_t)v;
  return 0;
}

static int parse_double(const char* value, double* out) {
  char* endp = NULL;
  double v = strtod(value, &endp);
  if (endp == value || *endp != '\0') return -1;
  *out = v;
  return 0;
}

static int parse_hz_to_interval_ms(const char* value, int* out_ms) {
  double hz = 0.0;
  if (parse_double(value, &hz) != 0 || hz <= 0.0) return -1;
  const double interval_ms = 1000.0 / hz;
  if (interval_ms < 1.0) return -1;
  *out_ms = (int)(interval_ms + 0.5);
  return 0;
}

static int valid_axis_ref(int axis_ref) {
  return axis_ref == -1 || (axis_ref >= 0 && axis_ref < (int)STC_AXIS_COUNT);
}


static int parse_bool(const char* value, int* out) {
  if (strcmp(value, "1") == 0 || strcmp(value, "true") == 0 || strcmp(value, "TRUE") == 0 || strcmp(value, "yes") == 0 || strcmp(value, "on") == 0) {
    *out = 1;
    return 0;
  }
  if (strcmp(value, "0") == 0 || strcmp(value, "false") == 0 || strcmp(value, "FALSE") == 0 || strcmp(value, "no") == 0 || strcmp(value, "off") == 0) {
    *out = 0;
    return 0;
  }
  return -1;
}

static int parse_runtime_backend(const char* s, stc_runtime_backend_t* out) {
  if (!s || !out) {
    return -1;
  }

  if (strcasecmp(s, "can") == 0 ||
      strcasecmp(s, "hardware") == 0 ||
      strcasecmp(s, "hardware_can") == 0) {
    *out = STC_RUNTIME_BACKEND_HARDWARE_CAN;
    return 0;
  }

  if (strcasecmp(s, "sim") == 0 ||
      strcasecmp(s, "simulator") == 0) {
    *out = STC_RUNTIME_BACKEND_SIMULATOR;
    return 0;
  }

  return -1;
}

static int parse_axis_driver(const char* s, stc_axis_driver_t* out) {
  if (!s || !out) return -1;

  if (strcasecmp(s, "disabled") == 0 || strcasecmp(s, "none") == 0) {
    *out = STC_AXIS_DRIVER_DISABLED;
    return 0;
  }

  if (strcasecmp(s, "pd4c") == 0 || strcasecmp(s, "nanotec") == 0 || strcasecmp(s, "nanotec_pd4c") == 0) {
    *out = STC_AXIS_DRIVER_PD4C;
    return 0;
  }

  if (strcasecmp(s, "myact") == 0 || strcasecmp(s, "myactuator") == 0 || strcasecmp(s, "myact_x8_25") == 0 || strcasecmp(s, "x8_25") == 0) {
    *out = STC_AXIS_DRIVER_MYACT_X8_25;
    return 0;
  }

  return -1;
}
/** Initialize deterministic baseline defaults before user INI overrides. */
void stc_config_set_defaults(stc_config_t* cfg) {
  if (!cfg) return;
  memset(cfg, 0, sizeof(*cfg));

  cfg->tcp_port = 7000;
  snprintf(cfg->backend, sizeof(cfg->backend), "%s", "simulator");
  cfg->runtime_backend = STC_RUNTIME_BACKEND_SIMULATOR;
  snprintf(cfg->can_if, sizeof(cfg->can_if), "%s", "can0");
  cfg->primary_node_id = 1;
  cfg->primary_tx_id = 0x141; cfg->primary_rx_id = 0x241;
  cfg->primary_motion_tx_id = 0x401; cfg->primary_motion_rx_id = 0x501;
  cfg->travel_min_deg = -140.0; cfg->travel_max_deg = 140.0;
  cfg->profile_max_speed_deg_s = 45.0; cfg->profile_max_accel_deg_s2 = 180.0;
  cfg->watchdog_timeout_ms = 250;
  cfg->telemetry_interval_ms = 10;
  cfg->experimental_native_speed_mode = 0;
  snprintf(cfg->tlm_dest_ip, sizeof(cfg->tlm_dest_ip), "%s", "127.0.0.1");
  snprintf(cfg->tlmh_ip, sizeof(cfg->tlmh_ip), "%s", cfg->tlm_dest_ip);
  snprintf(cfg->tlml_ip, sizeof(cfg->tlml_ip), "%s", cfg->tlm_dest_ip);
  cfg->tlmh_port = 6010;
  cfg->tlml_port = 6020;
  cfg->tlmh_interval_ms = 10;
  cfg->tlml_interval_ms = 100;
  cfg->can_bitrate = 500000;
  cfg->can_restart_ms = 100;
  cfg->can_scan_max_node = 32;
  cfg->can_timeout_ms = 1000;
  cfg->startup_feedback_grace_ms = 3000;
  cfg->strict_stale_policy = 0;
  cfg->speed_watchdog_ms = 250;
  cfg->speed_stream_period_ms = 50;
  cfg->position_tolerance_deg = 1.0;
  cfg->position_timeout_ms = 5000;
  cfg->position_poll_ms = 50;
  cfg->rt_enabled = 1;
  cfg->rt_strict = 0;
  cfg->rt_sched_policy = SCHED_FIFO;
  cfg->can_worker_priority = 80;
  cfg->tlmh_tx_priority = 70;
  cfg->tlml_tx_priority = 60;
  cfg->controller_id_strict_legacy = 0;

  cfg->num_axes = (int)STC_AXIS_COUNT;
  cfg->turret_az_axis = -1;
  cfg->turret_el_axis = -1;
  cfg->sut_count = 0;
  cfg->mirror_count = 0;

  for (size_t i = 0; i < STC_AXIS_COUNT; i++) {
    cfg->axis_enabled[i] = 1;
    cfg->axis_driver[i] = STC_AXIS_DRIVER_PD4C;
    cfg->axis_node_id[i] = (int)(i + 1);
    cfg->axis_command_sign[i] = 1.0;
    cfg->axis_telemetry_sign[i] = 1.0;
    cfg->axis_zero_physical_deg[i] = 0.0;
    cfg->axis_zero_raw_deg[i] = 0.0;
    cfg->axis_zero_on_start[i] = 0;
    cfg->axis_counts_per_rev[i] = 4096.0;
    cfg->axis_vel_scale[i] = 1.0;
    cfg->axis_vel_raw_mode[i] = 0;
    cfg->axis_storage_deg[i] = 0.0;
    cfg->axis_pp_profile_vel[i] = 500;
    cfg->axis_brake_release_timeout_ms[i] = 3000;
    cfg->axis_brake_poll_ms[i] = 50;
    cfg->axis_active_status2_enable[i] = 0;
    cfg->axis_active_status2_interval_ms[i] = 50;
    cfg->axis_feedback_timeout_ms[i] = 500;
    cfg->axis_diag_cat[i] = 0x06;
    cfg->axis_diag_subcat[i] = 0x3F;

    if (i == 0) {
      cfg->axis_gear_ratio[i] = 100.0;
      cfg->axis_min_deg[i] = -140.0;
      cfg->axis_max_deg[i] = 140.0;
      cfg->axis_max_speed_deg_s[i] = 45.0;
    } else if (i == 1) {
      cfg->axis_gear_ratio[i] = 20.0;
      cfg->axis_min_deg[i] = -5.0;
      cfg->axis_max_deg[i] = 180.0;
      cfg->axis_max_speed_deg_s[i] = 45.0;
    } else if (i < 10) {
      cfg->axis_gear_ratio[i] = 20.0;
      cfg->axis_min_deg[i] = -35.0;
      cfg->axis_max_deg[i] = 35.0;
      cfg->axis_max_speed_deg_s[i] = 45.0;
    } else {
      cfg->axis_gear_ratio[i] = 20.0;
      cfg->axis_min_deg[i] = -12.5;
      cfg->axis_max_deg[i] = 18.5;
      cfg->axis_max_speed_deg_s[i] = 45.0;
    }
  }

  for (size_t i = 0; i < STC_MAX_SUT; i++) {
    cfg->sut_az_axis[i] = -1;
    cfg->sut_el_axis[i] = -1;
    cfg->sut_laser_on[i] = 0u;
  }
  for (size_t i = 0; i < STC_MAX_MIRROR; i++) {
    cfg->mirror_axis[i] = -1;
  }
}

bool load_config(const char* path, stc_config_t* cfg) {
  if (!path || !cfg) return false;

  stc_config_set_defaults(cfg);

  FILE* f = fopen(path, "r");
  if (!f) {
    STC_LOG_ERROR("%s: %s", path, strerror(errno));
    return false;
  }

  char section[64] = "";
  char parse_key[128] = "";
  const char* parse_error_reason = "invalid value";
  int max_axis_seen = -1;
  int seen_diag_cat[STC_AXIS_COUNT] = {0};
  int seen_diag_subcat[STC_AXIS_COUNT] = {0};

  char line[512];
  int line_no = 0;
  while (fgets(line, sizeof(line), f)) {
    line_no++;

    trim_inplace(line);
    if (line[0] == '\0' || line[0] == '#' || line[0] == ';') continue;

    char* inline_comment = strpbrk(line, "#;");
    if (inline_comment) {
      *inline_comment = '\0';
      trim_inplace(line);
      if (line[0] == '\0') continue;
    }

    if (line[0] == '[') {
      char* close = strchr(line, ']');
      if (!close) {
        STC_LOG_ERROR("%s:%d: malformed section header", path, line_no);
        fclose(f);
        return false;
      }
      *close = '\0';
      {
        const size_t sec_src_len = strlen(line + 1);
        const size_t sec_len = (sec_src_len < (sizeof(section) - 1)) ? sec_src_len : (sizeof(section) - 1);
        memcpy(section, line + 1, sec_len);
        section[sec_len] = '\0';
      }
      trim_inplace(section);

      const int mir_idx = parse_indexed_section(section, "Mirror", STC_MAX_MIRROR, 1);
      if (mir_idx >= 0 && mir_idx + 1 > cfg->mirror_count) {
        cfg->mirror_count = mir_idx + 1;
      }
      continue;
    }

    char* eq = strchr(line, '=');
    if (!eq) {
      STC_LOG_ERROR("%s:%d: expected key=value", path, line_no);
      fclose(f);
      return false;
    }
    *eq = '\0';

    char* key = line;
    char* value = eq + 1;
    trim_inplace(key);
    trim_inplace(value);
    {
      const size_t key_src_len = strlen(key);
      const size_t key_len = (key_src_len < (sizeof(parse_key) - 1)) ? key_src_len : (sizeof(parse_key) - 1);
      memcpy(parse_key, key, key_len);
      parse_key[key_len] = '\0';
    }
    parse_error_reason = "invalid value";

    if (strcmp(section, "Backend") == 0) {
      bool recognized = true;
      if (strcmp(key, "backend") == 0) {
        if (parse_runtime_backend(value, &cfg->runtime_backend) != 0) {
          STC_LOG_ERROR("%s:%d: backend selects simulator or CAN hardware only; motor driver belongs in [AxisN].driver", path, line_no);
          fclose(f);
          return false;
        }
        snprintf(cfg->backend, sizeof(cfg->backend), "%s", runtime_backend_to_string(cfg->runtime_backend));
      }
      else if (strcmp(key, "can_if") == 0) { if (strlen(value) >= sizeof(cfg->can_if)) goto parse_error; strcpy(cfg->can_if, value); }
      else if (strcmp(key, "node_id") == 0) { if (parse_int(value, &cfg->primary_node_id) != 0) goto parse_error; }
      else if (strcmp(key, "tx_id") == 0) { if (parse_int(value, &cfg->primary_tx_id) != 0) goto parse_error; }
      else if (strcmp(key, "rx_id") == 0) { if (parse_int(value, &cfg->primary_rx_id) != 0) goto parse_error; }
      else if (strcmp(key, "motion_tx_id") == 0) { if (parse_int(value, &cfg->primary_motion_tx_id) != 0) goto parse_error; }
      else if (strcmp(key, "motion_rx_id") == 0) { if (parse_int(value, &cfg->primary_motion_rx_id) != 0) goto parse_error; }
      else if (strcmp(key, "travel_min_deg") == 0) { if (parse_double(value, &cfg->travel_min_deg) != 0) goto parse_error; }
      else if (strcmp(key, "travel_max_deg") == 0) { if (parse_double(value, &cfg->travel_max_deg) != 0) goto parse_error; }
      else if (strcmp(key, "profile_max_speed_deg_s") == 0) { if (parse_double(value, &cfg->profile_max_speed_deg_s) != 0) goto parse_error; }
      else if (strcmp(key, "profile_max_accel_deg_s2") == 0) { if (parse_double(value, &cfg->profile_max_accel_deg_s2) != 0) goto parse_error; }
      else if (strcmp(key, "watchdog_timeout_ms") == 0) { if (parse_int(value, &cfg->watchdog_timeout_ms) != 0) goto parse_error; }
      else if (strcmp(key, "telemetry_interval_ms") == 0) { if (parse_int(value, &cfg->telemetry_interval_ms) != 0) goto parse_error; }
      else if (strcmp(key, "experimental_native_speed_mode") == 0) { if (parse_int(value, &cfg->experimental_native_speed_mode) != 0) goto parse_error; }
      else recognized = false;
      if (!recognized) { parse_error_reason = "unknown key"; goto parse_error; }
      continue;
    }

    if (strcmp(section, "Network") == 0) {
      bool recognized = true;
      if (strcmp(key, "tcp_port") == 0) {
        if (parse_int(value, &cfg->tcp_port) != 0) goto parse_error;
      } else if (strcmp(key, "tlm_dest_ip") == 0) {
        if (strlen(value) >= sizeof(cfg->tlm_dest_ip)) goto parse_error;
        strcpy(cfg->tlm_dest_ip, value);
      } else if (strcmp(key, "tlmh_ip") == 0) {
        if (strlen(value) >= sizeof(cfg->tlmh_ip)) goto parse_error;
        strcpy(cfg->tlmh_ip, value);
      } else if (strcmp(key, "tlml_ip") == 0) {
        if (strlen(value) >= sizeof(cfg->tlml_ip)) goto parse_error;
        strcpy(cfg->tlml_ip, value);
      } else if (strcmp(key, "tlmh_port") == 0) {
        if (parse_int(value, &cfg->tlmh_port) != 0) goto parse_error;
      } else if (strcmp(key, "tlml_port") == 0) {
        if (parse_int(value, &cfg->tlml_port) != 0) goto parse_error;
      } else if (strcmp(key, "mcast_ip") == 0) {
        /* Backward-compatibility alias: retained for legacy deployments. */
        if (strlen(value) >= sizeof(cfg->tlm_dest_ip)) goto parse_error;
        strcpy(cfg->tlm_dest_ip, value);
        STC_LOG_WARN("%s:%d: [Network].mcast_ip is deprecated; use tlm_dest_ip", path, line_no);
      } else if (strcmp(key, "mcast_tlmh_port") == 0) {
        if (parse_int(value, &cfg->tlmh_port) != 0) goto parse_error;
        STC_LOG_WARN("%s:%d: [Network].mcast_tlmh_port is deprecated; use tlmh_port", path, line_no);
      } else if (strcmp(key, "mcast_tlml_port") == 0) {
        if (parse_int(value, &cfg->tlml_port) != 0) goto parse_error;
        STC_LOG_WARN("%s:%d: [Network].mcast_tlml_port is deprecated; use tlml_port", path, line_no);
      } else if (strcmp(key, "tlmh_interval_ms") == 0) {
        if (parse_int(value, &cfg->tlmh_interval_ms) != 0) goto parse_error;
      } else if (strcmp(key, "tlmh_hz") == 0) {
        if (parse_hz_to_interval_ms(value, &cfg->tlmh_interval_ms) != 0) goto parse_error;
      } else if (strcmp(key, "tlml_interval_ms") == 0) {
        if (parse_int(value, &cfg->tlml_interval_ms) != 0) goto parse_error;
      } else if (strcmp(key, "tlml_hz") == 0) {
        if (parse_hz_to_interval_ms(value, &cfg->tlml_interval_ms) != 0) goto parse_error;
      } else if (strcmp(key, "can_scan_max_node") == 0) {
        if (parse_int(value, &cfg->can_scan_max_node) != 0) goto parse_error;
      } else if (strcmp(key, "can_timeout_ms") == 0) {
        if (parse_int(value, &cfg->can_timeout_ms) != 0) goto parse_error;
      } else if (strcmp(key, "strict_stale_policy") == 0) {
        if (parse_int(value, &cfg->strict_stale_policy) != 0) goto parse_error;
      } else if (strcmp(key, "speed_watchdog_ms") == 0) {
        if (parse_int(value, &cfg->speed_watchdog_ms) != 0) goto parse_error;
      } else if (strcmp(key, "speed_stream_period_ms") == 0) {
        if (parse_int(value, &cfg->speed_stream_period_ms) != 0) goto parse_error;
      } else if (strcmp(key, "position_tolerance_deg") == 0) {
        if (parse_double(value, &cfg->position_tolerance_deg) != 0) goto parse_error;
      } else if (strcmp(key, "position_timeout_ms") == 0) {
        if (parse_int(value, &cfg->position_timeout_ms) != 0) goto parse_error;
      } else if (strcmp(key, "position_poll_ms") == 0) {
        if (parse_int(value, &cfg->position_poll_ms) != 0) goto parse_error;
      } else {
        recognized = false;
      }
      if (!recognized) {
        if (strcmp(key, "startup_feedback_grace_ms") == 0) {
          STC_LOG_ERROR("%s:%d: startup_feedback_grace_ms belongs in [Runtime]", path, line_no);
          fclose(f);
          return false;
        }
        parse_error_reason = "unknown key";
        goto parse_error;
      }
      continue;
    }
    if (strcmp(section, "CAN") == 0) {
      bool recognized = true;
      if (strcmp(key, "ifname") == 0) {
        if (strlen(value) >= sizeof(cfg->can_if)) goto parse_error;
        strcpy(cfg->can_if, value);
      } else if (strcmp(key, "bitrate") == 0) {
        if (parse_int(value, &cfg->can_bitrate) != 0) goto parse_error;
      } else if (strcmp(key, "restart_ms") == 0) {
        if (parse_int(value, &cfg->can_restart_ms) != 0) goto parse_error;
      } else {
        recognized = false;
      }
      if (!recognized) {
        parse_error_reason = "unknown key";
        goto parse_error;
      }
      continue;
    }

    if (strcmp(section, "Runtime") == 0) {
      bool recognized = true;
      if (strcmp(key, "backend") == 0) {
        if (parse_runtime_backend(value, &cfg->runtime_backend) != 0) {
          STC_LOG_ERROR("%s:%d: backend selects simulator or CAN hardware only; motor driver belongs in [AxisN].driver", path, line_no);
          fclose(f);
          return false;
        }
        snprintf(cfg->backend, sizeof(cfg->backend), "%s", runtime_backend_to_string(cfg->runtime_backend));
      } else if (strcmp(key, "rt_enabled") == 0) {
        if (parse_int(value, &cfg->rt_enabled) != 0) goto parse_error;
      } else if (strcmp(key, "rt_strict") == 0) {
        if (parse_int(value, &cfg->rt_strict) != 0) goto parse_error;
      } else if (strcmp(key, "rt_sched_policy") == 0) {
        if (strcmp(value, "fifo") == 0 || strcmp(value, "FIFO") == 0) {
          cfg->rt_sched_policy = SCHED_FIFO;
        } else if (strcmp(value, "rr") == 0 || strcmp(value, "RR") == 0) {
          cfg->rt_sched_policy = SCHED_RR;
        } else if (strcmp(value, "other") == 0 || strcmp(value, "OTHER") == 0) {
          cfg->rt_sched_policy = SCHED_OTHER;
        } else {
          goto parse_error;
        }
      } else if (strcmp(key, "can_worker_priority") == 0) {
        if (parse_int(value, &cfg->can_worker_priority) != 0) goto parse_error;
      } else if (strcmp(key, "tlmh_tx_priority") == 0) {
        if (parse_int(value, &cfg->tlmh_tx_priority) != 0) goto parse_error;
      } else if (strcmp(key, "tlml_tx_priority") == 0) {
        if (parse_int(value, &cfg->tlml_tx_priority) != 0) goto parse_error;
      } else if (strcmp(key, "controller_id_strict_legacy") == 0) {
        if (parse_int(value, &cfg->controller_id_strict_legacy) != 0) goto parse_error;
      } else if (strcmp(key, "startup_feedback_grace_ms") == 0) {
        if (parse_int(value, &cfg->startup_feedback_grace_ms) != 0) goto parse_error;
      } else if (strcmp(key, "strict_stale_policy") == 0) {
        if (parse_int(value, &cfg->strict_stale_policy) != 0) goto parse_error;
      } else if (strcmp(key, "speed_watchdog_ms") == 0) {
        if (parse_int(value, &cfg->speed_watchdog_ms) != 0) goto parse_error;
      } else if (strcmp(key, "speed_stream_period_ms") == 0) {
        if (parse_int(value, &cfg->speed_stream_period_ms) != 0) goto parse_error;
      } else if (strcmp(key, "position_tolerance_deg") == 0) {
        if (parse_double(value, &cfg->position_tolerance_deg) != 0) goto parse_error;
      } else if (strcmp(key, "position_timeout_ms") == 0) {
        if (parse_int(value, &cfg->position_timeout_ms) != 0) goto parse_error;
      } else if (strcmp(key, "position_poll_ms") == 0) {
        if (parse_int(value, &cfg->position_poll_ms) != 0) goto parse_error;
      } else {
        recognized = false;
      }
      if (!recognized) {
        parse_error_reason = "unknown key";
        goto parse_error;
      }
      continue;
    }

    const int axis_idx = parse_indexed_section(section, "Axis", (int)STC_AXIS_COUNT, 0);
    if (axis_idx >= 0) {
      bool recognized = true;
      if (strcmp(key, "enabled") == 0) {
        if (parse_bool(value, &cfg->axis_enabled[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "driver") == 0) {
        if (parse_axis_driver(value, &cfg->axis_driver[axis_idx]) != 0) {
          STC_LOG_ERROR("%s:%d: unknown axis driver '%s'", path, line_no, value);
          fclose(f);
          return false;
        }
      } else if (strcmp(key, "node_id") == 0) {
        if (parse_int(value, &cfg->axis_node_id[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "counts_per_rev") == 0) {
        if (parse_double(value, &cfg->axis_counts_per_rev[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "gear_ratio") == 0) {
        if (parse_double(value, &cfg->axis_gear_ratio[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "min_deg") == 0) {
        if (parse_double(value, &cfg->axis_min_deg[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "max_deg") == 0) {
        if (parse_double(value, &cfg->axis_max_deg[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "max_speed_deg_s") == 0) {
        if (parse_double(value, &cfg->axis_max_speed_deg_s[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "storage_deg") == 0) {
        if (parse_double(value, &cfg->axis_storage_deg[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "command_sign") == 0) {
        if (parse_double(value, &cfg->axis_command_sign[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "telemetry_sign") == 0) {
        if (parse_double(value, &cfg->axis_telemetry_sign[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "zero_physical_deg") == 0) {
        if (parse_double(value, &cfg->axis_zero_physical_deg[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "zero_raw_deg") == 0) {
        if (parse_double(value, &cfg->axis_zero_raw_deg[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "zero_on_start") == 0) {
        if (parse_bool(value, &cfg->axis_zero_on_start[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "vel_scale") == 0) {
        if (parse_double(value, &cfg->axis_vel_scale[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "vel_raw_mode") == 0) {
        if (parse_int(value, &cfg->axis_vel_raw_mode[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "brake_release_timeout_ms") == 0) {
        if (parse_int(value, &cfg->axis_brake_release_timeout_ms[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "brake_poll_ms") == 0) {
        if (parse_int(value, &cfg->axis_brake_poll_ms[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "active_status2_enable") == 0) {
        if (parse_bool(value, &cfg->axis_active_status2_enable[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "active_status2_interval_ms") == 0) {
        if (parse_int(value, &cfg->axis_active_status2_interval_ms[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "feedback_timeout_ms") == 0) {
        if (parse_int(value, &cfg->axis_feedback_timeout_ms[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "name") == 0 || strcmp(key, "role") == 0) {
        /* accepted metadata */
      } else if (strcmp(key, "pp_profile_vel") == 0) {
        if (parse_u16(value, &cfg->axis_pp_profile_vel[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "diag_cat") == 0) {
        if (parse_u8(value, &cfg->axis_diag_cat[axis_idx]) != 0) goto parse_error;
        seen_diag_cat[axis_idx] = 1;
      } else if (strcmp(key, "diag_subcat") == 0) {
        if (parse_u8(value, &cfg->axis_diag_subcat[axis_idx]) != 0) goto parse_error;
        seen_diag_subcat[axis_idx] = 1;
      } else {
        recognized = false;
      }

      if (!recognized) {
        if (strcmp(key, "startup_feedback_grace_ms") == 0) {
          STC_LOG_ERROR("%s:%d: startup_feedback_grace_ms belongs in [Runtime], not [AxisN]", path, line_no);
          fclose(f);
          return false;
        }
        parse_error_reason = "unknown key";
        goto parse_error;
      }

      if (axis_idx > max_axis_seen) max_axis_seen = axis_idx;
      continue;
    }

    if (strcmp(section, "diagnostics") == 0) {
      bool recognized = true;
      if (strcmp(key, "motor_status_map") == 0) {
        STC_LOG_WARN("%s:%d: motor_status_map is deprecated and ignored; TLML uses canonical motor_status[18]", path, line_no);
      } else {
        int axis = -1;
        if (sscanf(key, "axis[%d].category", &axis) == 1) {
          if (axis < 0 || axis >= (int)STC_AXIS_COUNT) goto parse_error;
          if (parse_u8(value, &cfg->axis_diag_cat[axis]) != 0) goto parse_error;
          seen_diag_cat[axis] = 1;
        } else if (sscanf(key, "axis[%d].subcategory", &axis) == 1) {
          if (axis < 0 || axis >= (int)STC_AXIS_COUNT) goto parse_error;
          if (parse_u8(value, &cfg->axis_diag_subcat[axis]) != 0) goto parse_error;
          seen_diag_subcat[axis] = 1;
        } else {
          recognized = false;
        }
      }
      if (!recognized) {
        parse_error_reason = "unknown key";
        goto parse_error;
      }
      continue;
    }

    if (strcmp(section, "Turret") == 0) {
      bool recognized = true;
      if (strcmp(key, "az_axis") == 0) {
        if (parse_int(value, &cfg->turret_az_axis) != 0) goto parse_error;
      } else if (strcmp(key, "el_axis") == 0) {
        if (parse_int(value, &cfg->turret_el_axis) != 0) goto parse_error;
      } else {
        recognized = false;
      }
      if (!recognized) {
        parse_error_reason = "unknown key";
        goto parse_error;
      }
      continue;
    }

    const int sut_idx = parse_indexed_section(section, "SUT", STC_MAX_SUT, 1);
    if (sut_idx >= 0) {
      bool recognized = true;
      if (strcmp(key, "az_axis") == 0) {
        if (parse_int(value, &cfg->sut_az_axis[sut_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "el_axis") == 0) {
        if (parse_int(value, &cfg->sut_el_axis[sut_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "laser_on") == 0) {
        int laser_on = 0;
        if (parse_int(value, &laser_on) != 0 || (laser_on != 0 && laser_on != 1)) goto parse_error;
        cfg->sut_laser_on[sut_idx] = (uint8_t)laser_on;
      } else {
        recognized = false;
      }
      if (!recognized) {
        parse_error_reason = "unknown key";
        goto parse_error;
      }
      if (sut_idx + 1 > cfg->sut_count) cfg->sut_count = sut_idx + 1;
      continue;
    }

    const int mir_idx = parse_indexed_section(section, "Mirror", STC_MAX_MIRROR, 1);
    if (mir_idx >= 0) {
      bool recognized = true;
      if (strcmp(key, "axis") == 0) {
        if (parse_int(value, &cfg->mirror_axis[mir_idx]) != 0) goto parse_error;
      } else {
        recognized = false;
      }
      if (!recognized) {
        parse_error_reason = "unknown key";
        goto parse_error;
      }
      continue;
    }

    parse_error_reason = "unknown section";
    goto parse_error;
  }

  fclose(f);

  cfg->num_axes = max_axis_seen + 1;
  if (cfg->num_axes <= 0) {
    STC_LOG_ERROR("%s: no [AxisN] sections were defined", path);
    return false;
  }

  if (!valid_axis_ref(cfg->turret_az_axis) || cfg->turret_az_axis >= cfg->num_axes) {
    STC_LOG_ERROR("%s: [Turret].az_axis is missing or out of range", path);
    return false;
  }
  if (!valid_axis_ref(cfg->turret_el_axis) || cfg->turret_el_axis >= cfg->num_axes) {
    STC_LOG_ERROR("%s: [Turret].el_axis out of range", path);
    return false;
  }
  for (int i = 0; i < cfg->sut_count; i++) {
    if (!valid_axis_ref(cfg->sut_az_axis[i]) || cfg->sut_az_axis[i] >= cfg->num_axes) {
      STC_LOG_ERROR("%s: [SUT%d].az_axis out of range", path, i + 1);
      return false;
    }
    if (!valid_axis_ref(cfg->sut_el_axis[i]) || cfg->sut_el_axis[i] >= cfg->num_axes) {
      STC_LOG_ERROR("%s: [SUT%d].el_axis out of range", path, i + 1);
      return false;
    }
  }
  for (int i = 0; i < cfg->mirror_count; i++) {
    if (!valid_axis_ref(cfg->mirror_axis[i]) || cfg->mirror_axis[i] >= cfg->num_axes) {
      STC_LOG_ERROR("%s: [Mirror%d].axis out of range", path, i + 1);
      return false;
    }
  }

  for (int i = 0; i < cfg->num_axes; i++) {
    if (!seen_diag_cat[i] || !seen_diag_subcat[i]) {
      STC_LOG_ERROR("%s: missing diagnostic category/subcategory for axis %d", path, i);
      return false;
    }
    if (cfg->axis_enabled[i]) {
      if (cfg->axis_driver[i] != STC_AXIS_DRIVER_PD4C && cfg->axis_driver[i] != STC_AXIS_DRIVER_MYACT_X8_25) {
        STC_LOG_ERROR("%s: axis %d is enabled but has invalid driver", path, i);
        return false;
      }
      if (cfg->axis_node_id[i] <= 0) {
        STC_LOG_ERROR("%s: axis %d has invalid node_id (%d)", path, i, cfg->axis_node_id[i]);
        return false;
      }
      if (cfg->axis_min_deg[i] > cfg->axis_max_deg[i]) {
        STC_LOG_ERROR("%s: axis %d has min_deg > max_deg", path, i);
        return false;
      }
    }
    if (cfg->axis_active_status2_enable[i] && cfg->axis_active_status2_interval_ms[i] <= 0) {
      STC_LOG_ERROR("%s: axis %d has invalid active_status2_interval_ms", path, i);
      return false;
    }
  }

  if (cfg->speed_watchdog_ms < 0) {
    STC_LOG_ERROR("%s: speed_watchdog_ms must be >= 0", path);
    return false;
  }
  if (cfg->speed_stream_period_ms <= 0) {
    STC_LOG_ERROR("%s: speed_stream_period_ms must be > 0", path);
    return false;
  }
  if (cfg->position_tolerance_deg <= 0.0) {
    STC_LOG_ERROR("%s: position_tolerance_deg must be > 0", path);
    return false;
  }
  if (cfg->position_timeout_ms <= 0) {
    STC_LOG_ERROR("%s: position_timeout_ms must be > 0", path);
    return false;
  }
  if (cfg->position_poll_ms <= 0) {
    STC_LOG_ERROR("%s: position_poll_ms must be > 0", path);
    return false;
  }

  if (cfg->startup_feedback_grace_ms < 0) {
    STC_LOG_ERROR("%s: startup_feedback_grace_ms must be >= 0", path);
    return false;
  }

  return true;

parse_error:
  if (strcmp(parse_error_reason, "unknown section") == 0) {
    /* Strict schema: unknown sections are rejected to surface config drift. */
    STC_LOG_ERROR("%s:%d: unknown section [%s]", path, line_no, section);
  } else if (strcmp(parse_error_reason, "unknown key") == 0) {
    /* Strict schema: unknown keys are rejected to prevent typo fallthrough. */
    STC_LOG_ERROR("%s:%d: unknown key '%s' in section [%s]", path, line_no, parse_key, section);
  } else {
    STC_LOG_ERROR("%s:%d: invalid value for %s", path, line_no, parse_key);
  }
  fclose(f);
  return false;
}
