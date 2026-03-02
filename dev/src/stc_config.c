#include "stc_config.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

static int parse_double(const char* value, double* out) {
  char* endp = NULL;
  double v = strtod(value, &endp);
  if (endp == value || *endp != '\0') return -1;
  *out = v;
  return 0;
}

void stc_config_set_defaults(stc_config_t* cfg) {
  if (!cfg) return;
  memset(cfg, 0, sizeof(*cfg));

  cfg->tcp_port = 7000;
  snprintf(cfg->mcast_ip, sizeof(cfg->mcast_ip), "%s", "239.255.10.2");
  cfg->mcast_tlmh_port = 6010;
  cfg->mcast_tlml_port = 6020;
  cfg->tlmh_interval_ms = 10;
  cfg->tlml_interval_ms = 100;

  cfg->turret_az_axis = -1;
  cfg->turret_el_axis = -1;

  for (size_t i = 0; i < STC_AXIS_COUNT; i++) {
    cfg->axis_node_id[i] = 1;
    cfg->axis_counts_per_rev[i] = 4096.0;
    cfg->axis_gear_ratio[i] = 1.0;
    cfg->axis_vel_is_rpm[i] = 0;
    cfg->axis_min_deg[i] = -180.0;
    cfg->axis_max_deg[i] = 180.0;
    cfg->axis_max_speed_deg_s[i] = 90.0;
    cfg->axis_storage_deg[i] = 0.0;
    cfg->axis_pp_profile_vel[i] = 500;
    cfg->axis_diag_cat[i] = 0;
    cfg->axis_diag_subcat[i] = 0;
  }
}

bool load_config(const char* path, stc_config_t* cfg) {
  if (!path || !cfg) return false;

  stc_config_set_defaults(cfg);

  FILE* f = fopen(path, "r");
  if (!f) {
    perror(path);
    return false;
  }

  char section[64] = "";
  char parse_key[128] = "";
  int max_axis_seen = -1;

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
        fprintf(stderr, "%s:%d: malformed section header\n", path, line_no);
        fclose(f);
        return false;
      }
      *close = '\0';
      snprintf(section, sizeof(section), "%s", line + 1);
      trim_inplace(section);
      continue;
    }

    char* eq = strchr(line, '=');
    if (!eq) {
      fprintf(stderr, "%s:%d: expected key=value\n", path, line_no);
      fclose(f);
      return false;
    }
    *eq = '\0';

    char* key = line;
    char* value = eq + 1;
    trim_inplace(key);
    trim_inplace(value);
    strncpy(parse_key, key, sizeof(parse_key) - 1);
    parse_key[sizeof(parse_key) - 1] = '\0';

    if (strcmp(section, "Network") == 0) {
      if (strcmp(key, "tcp_port") == 0) {
        if (parse_int(value, &cfg->tcp_port) != 0) goto parse_error;
      } else if (strcmp(key, "mcast_ip") == 0) {
        if (strlen(value) >= sizeof(cfg->mcast_ip)) goto parse_error;
        strcpy(cfg->mcast_ip, value);
      } else if (strcmp(key, "mcast_tlmh_port") == 0) {
        if (parse_int(value, &cfg->mcast_tlmh_port) != 0) goto parse_error;
      } else if (strcmp(key, "mcast_tlml_port") == 0) {
        if (parse_int(value, &cfg->mcast_tlml_port) != 0) goto parse_error;
      } else if (strcmp(key, "tlmh_interval_ms") == 0) {
        if (parse_int(value, &cfg->tlmh_interval_ms) != 0) goto parse_error;
      } else if (strcmp(key, "tlml_interval_ms") == 0) {
        if (parse_int(value, &cfg->tlml_interval_ms) != 0) goto parse_error;
      }
      continue;
    }

    const int axis_idx = parse_indexed_section(section, "Axis", (int)STC_AXIS_COUNT, 0);
    if (axis_idx >= 0) {
      if (strcmp(key, "node_id") == 0) {
        if (parse_int(value, &cfg->axis_node_id[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "counts_per_rev") == 0) {
        if (parse_double(value, &cfg->axis_counts_per_rev[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "gear_ratio") == 0) {
        if (parse_double(value, &cfg->axis_gear_ratio[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "vel_is_rpm") == 0) {
        if (parse_int(value, &cfg->axis_vel_is_rpm[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "min_deg") == 0) {
        if (parse_double(value, &cfg->axis_min_deg[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "max_deg") == 0) {
        if (parse_double(value, &cfg->axis_max_deg[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "max_speed_deg_s") == 0) {
        if (parse_double(value, &cfg->axis_max_speed_deg_s[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "storage_deg") == 0) {
        if (parse_double(value, &cfg->axis_storage_deg[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "pp_profile_vel") == 0) {
        if (parse_u16(value, &cfg->axis_pp_profile_vel[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "diag_cat") == 0) {
        if (parse_u16(value, &cfg->axis_diag_cat[axis_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "diag_subcat") == 0) {
        if (parse_u16(value, &cfg->axis_diag_subcat[axis_idx]) != 0) goto parse_error;
      }

      if (axis_idx > max_axis_seen) max_axis_seen = axis_idx;
      continue;
    }

    if (strcmp(section, "Turret") == 0) {
      if (strcmp(key, "az_axis") == 0) {
        if (parse_int(value, &cfg->turret_az_axis) != 0) goto parse_error;
      } else if (strcmp(key, "el_axis") == 0) {
        if (parse_int(value, &cfg->turret_el_axis) != 0) goto parse_error;
      }
      continue;
    }

    const int sut_idx = parse_indexed_section(section, "SUT", STC_MAX_SUT, 1);
    if (sut_idx >= 0) {
      if (strcmp(key, "az_axis") == 0) {
        if (parse_int(value, &cfg->sut_az_axis[sut_idx]) != 0) goto parse_error;
      } else if (strcmp(key, "el_axis") == 0) {
        if (parse_int(value, &cfg->sut_el_axis[sut_idx]) != 0) goto parse_error;
      }
      if (sut_idx + 1 > cfg->sut_count) cfg->sut_count = sut_idx + 1;
      continue;
    }

    const int mir_idx = parse_indexed_section(section, "Mirror", STC_MAX_MIRROR, 1);
    if (mir_idx >= 0) {
      if (strcmp(key, "axis") == 0) {
        if (parse_int(value, &cfg->mirror_axis[mir_idx]) != 0) goto parse_error;
      }
      if (mir_idx + 1 > cfg->mirror_count) cfg->mirror_count = mir_idx + 1;
      continue;
    }
  }

  fclose(f);

  cfg->num_axes = max_axis_seen + 1;
  if (cfg->num_axes <= 0) {
    fprintf(stderr, "%s: no [AxisN] sections were defined\n", path);
    return false;
  }

  if (cfg->turret_az_axis < 0 || cfg->turret_az_axis >= cfg->num_axes) {
    fprintf(stderr, "%s: [Turret].az_axis is missing or out of range\n", path);
    return false;
  }
  if (cfg->turret_el_axis >= cfg->num_axes) {
    fprintf(stderr, "%s: [Turret].el_axis out of range\n", path);
    return false;
  }

  return true;

parse_error:
  fprintf(stderr, "%s:%d: invalid value for %s\n", path, line_no, parse_key);
  fclose(f);
  return false;
}
