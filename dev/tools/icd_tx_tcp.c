/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   CLI TCP command injector for deterministic STC ICD CMD/RSP protocol tests.
 *
 * Design notes:
 *   Crafts CU-like command frames, sends them to stc_server, then reads/parses
 *   RSP frames to validate live protocol behavior during integration debugging.
 */

// icd_tx_tcp.c
#define _POSIX_C_SOURCE 200809L
#include "stc_icd.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

static uint64_t now_ms(void) {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
}

static void hexdump(const uint8_t* b, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (i % 16 == 0) printf("%04zu: ", i);
    printf("%02X ", b[i]);
    if (i % 16 == 15 || i + 1 == n) printf("\n");
  }
}

static uint32_t le32(const uint8_t* p) {
  return ((uint32_t)p[0]) |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static int send_all(int fd, const uint8_t* buf, size_t len) {
  size_t off = 0;
  while (off < len) {
    ssize_t w = send(fd, buf + off, len - off, 0);
    if (w < 0) {
      if (errno == EINTR) continue;
      return -1;
    }
    off += (size_t)w;
  }
  return 0;
}

static int recv_exact(int fd, uint8_t* buf, size_t len) {
  size_t off = 0;
  while (off < len) {
    ssize_t r = recv(fd, buf + off, len - off, 0);
    if (r == 0) return 0; // closed
    if (r < 0) {
      if (errno == EINTR) continue;
      return -1;
    }
    off += (size_t)r;
  }
  return 1;
}

static int tcp_connect(const char* host, const char* port) {
  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo* res = NULL;
  int rc = getaddrinfo(host, port, &hints, &res);
  if (rc != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rc));
    return -1;
  }

  int fd = -1;
  for (struct addrinfo* p = res; p; p = p->ai_next) {
    fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
    if (fd < 0) continue;
    if (connect(fd, p->ai_addr, p->ai_addrlen) == 0) break;
    close(fd);
    fd = -1;
  }

  freeaddrinfo(res);
  return fd;
}

static int read_rsp_and_print(int fd, const stc_icd_cfg_t* cfg) {
  /* Read fixed RSP header first, then read payload according to payload_size field. */
  // RSP header is 14 bytes after our ICD-aligned fix
  uint8_t hdr[14];
  int rr = recv_exact(fd, hdr, sizeof(hdr));
  if (rr <= 0) {
    fprintf(stderr, "recv rsp header failed/closed\n");
    return -1;
  }

  // payload_size at offset 12
  uint16_t payload_size = (uint16_t)(hdr[12] | ((uint16_t)hdr[13] << 8));
  if (payload_size > STC_ICD_MAX_PAYLOAD) {
    fprintf(stderr, "rsp payload_size too big: %u\n", payload_size);
    return -1;
  }

  uint8_t buf[14 + STC_ICD_MAX_PAYLOAD];
  memcpy(buf, hdr, 14);

  if (payload_size > 0) {
    rr = recv_exact(fd, buf + 14, payload_size);
    if (rr <= 0) {
      fprintf(stderr, "recv rsp payload failed/closed\n");
      return -1;
    }
  }

  stc_icd_rsp_t rsp;
  stc_icd_rc_t prc = stc_icd_rsp_parse(cfg, buf, 14u + payload_size, &rsp);
  if (prc != STC_ICD_OK) {
    fprintf(stderr, "stc_icd_rsp_parse failed: %d\n", prc);
    printf("Raw RSP:\n"); hexdump(buf, 14u + payload_size);
    return -1;
  }

  const char* result_name = (rsp.result == STC_RSP_ACK ? "ACK" : "NACK");
  const char* err_name = "UNKNOWN";
  switch (rsp.err_code) {
    case STC_ERR_NO_ERROR: err_name = "STC_ERR_NO_ERROR"; break;
    case STC_ERR_CLIENT_UNKNOWN: err_name = "STC_ERR_CLIENT_UNKNOWN"; break;
    case STC_ERR_PROTOCOL_MISMATCH: err_name = "STC_ERR_PROTOCOL_MISMATCH"; break;
    case STC_ERR_SYNTAX_ERROR: err_name = "STC_ERR_SYNTAX_ERROR"; break;
    case STC_ERR_VALUE_OUT_OF_RANGE: err_name = "STC_ERR_VALUE_OUT_OF_RANGE"; break;
    case STC_ERR_NOT_ALLOWED_STATE: err_name = "STC_ERR_NOT_ALLOWED_STATE"; break;
    case STC_ERR_INSUFFICIENT_RES: err_name = "STC_ERR_INSUFFICIENT_RES"; break;
    case STC_ERR_TIMEOUT: err_name = "STC_ERR_TIMEOUT"; break;
    case STC_ERR_COMMAND_NOT_IMPLEMENTED: err_name = "STC_ERR_COMMAND_NOT_IMPLEMENTED"; break;
    case STC_ERR_NOT_CONFIGURED: err_name = "STC_ERR_NOT_CONFIGURED"; break;
    case STC_ERR_AXIS_NOT_CONNECTED: err_name = "STC_ERR_AXIS_NOT_CONNECTED"; break;
    case STC_ERR_GENERIC: err_name = "STC_ERR_GENERIC"; break;
  }

  printf("RSP: cmd_id=0x%04X seq=%u result=%s err_code=%u (%s) payload_size=%u\n",
         rsp.cmd_id, rsp.seq, result_name,
         (unsigned)rsp.err_code, err_name,
         (unsigned)rsp.payload_size);

  printf("%s\n", result_name);
  printf("error_code_name=%s\n", err_name);

  if (rsp.cmd_id == STC_CMD_REQ_WORK_TIME && rsp.payload_size == 4) {
    uint32_t uptime_sec = le32(rsp.payload);
    printf("response_payload.uptime_sec=%u\n", uptime_sec);
  } else if (rsp.payload_size > 0) {
    printf("response_payload=");
    fwrite(rsp.payload, 1, rsp.payload_size, stdout);
    printf("\n");
  }
  return 0;
}

static stc_motion_type_t parse_motion(const char* s) {
  if (!strcmp(s, "rel")) return STC_MOTION_RELATIVE;
  if (!strcmp(s, "abs")) return STC_MOTION_ABSOLUTE;
  if (!strcmp(s, "speed")) return STC_MOTION_SPEED;
  // fallback
  return (stc_motion_type_t)atoi(s);
}

static void usage(const char* prog) {
  fprintf(stderr,
    "Usage:\n"
    "  %s <host> <port> <cmd> [args...]\n"
    "\n"
    "Common cmds:\n"
    "  cu_state <0..255>\n"
    "  selflev <0|1>\n"
    "  ibit <0|1>\n"
    "  storage\n"
    "  laser <id(1-8)> <0|1>\n"
    "  turret <rel|abs|speed> <az> <el>\n"
    "  mirror <id|all(255)> <rel|abs|speed> <elev_or_speed>\n"
    "  sut <id|all(255)> <rel|abs|speed> <az> <el>\n"
    "  reset | factory | shutdown\n"
    "  manualpos <lat> <lon> <alt> <heading>\n"
    "  posmode <gps|manual|0|1>\n"
    "  req_sw | req_sw_version\n"
    "  req_hw_info | req_hw\n"
    "  req_work_time | req_worktime\n"
    "\n"
    "Test suites:\n"
    "  suite          (sends many valid commands)\n"
    "  bad_magic      (wrong CMD magic)\n"
    "  bad_version    (protocol_version=2)\n"
    "  unknown_cmd    (cmd_id=0x9999)\n"
    "  bad_payload    (e.g. turret with wrong payload_size)\n"
    "\n"
    "Options (env vars):\n"
    "  STC_CTRL_ID (default 21)\n"
    "\n", prog);
}

static int send_cmd(int fd, stc_icd_cfg_t* cfg, stc_icd_cmd_t* cmd, int print_hex) {
  uint8_t out[1024];
  size_t out_len = 0;
  stc_icd_rc_t rc = stc_icd_cmd_pack(cfg, cmd, out, sizeof(out), &out_len);
  if (rc != STC_ICD_OK) {
    fprintf(stderr, "stc_icd_cmd_pack failed: %d\n", rc);
    return -1;
  }

  if (print_hex) {
    printf("TX CMD (%zu bytes):\n", out_len);
    hexdump(out, out_len);
  }

  if (send_all(fd, out, out_len) != 0) {
    perror("send");
    return -1;
  }
  return 0;
}

int main(int argc, char** argv) {
  /* Command-line parsing: endpoint + command selector + command-specific args. */
  if (argc < 4) { usage(argv[0]); return 2; }

  const char* host = argv[1];
  const char* port = argv[2];
  const char* cmdn = argv[3];

  if (!strcmp(cmdn, "mirror") && argc != 7) {
    fprintf(stderr, "mirror command takes exactly 3 payload arguments: id, motion, value\n");
    usage(argv[0]);
    return 2;
  }

  int fd = tcp_connect(host, port);
  if (fd < 0) { perror("connect"); return 1; }

  stc_icd_cfg_t cfg = stc_icd_parse_cfg_default();
  cfg.strict_magic = 1;
  cfg.strict_version = 0;

  uint8_t ctrl_id = 21;
  const char* e = getenv("STC_CTRL_ID");
  if (e && *e) ctrl_id = (uint8_t)atoi(e);

  uint16_t seq = 1; /* Monotonic sequence generator for this interactive session. */

  // Helper macro to initialize common CMD header fields before payload filling.
  #define INIT_CMD(CMDID) do { \
    memset(&c, 0, sizeof(c)); \
    c.version = 0x01; \
    c.controller_id = ctrl_id; \
    c.cmd_id = (CMDID); \
    c.seq = seq++; \
    c.timestamp_ms = now_ms(); \
    c.payload_decoded = 1; \
  } while (0)

  stc_icd_cmd_t c; /* Packed by stc_icd_cmd_pack(), then sent as raw TCP bytes. */

  // ---- suite: deterministic positive-path command sequence + selected negatives
  if (!strcmp(cmdn, "suite")) {
    printf("Running suite...\n");

    INIT_CMD(STC_CMD_CU_STATE);
    c.u.cu_state.state = 2;
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    INIT_CMD(STC_CMD_SELFLEV);
    c.u.selflev.v = 1;
    if (send_cmd(fd, &cfg, &c, 0) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    INIT_CMD(STC_CMD_SELFLEV);
    c.u.selflev.v = 0;
    if (send_cmd(fd, &cfg, &c, 0) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    INIT_CMD(STC_CMD_IBIT);
    c.u.ibit.v = 1;
    if (send_cmd(fd, &cfg, &c, 0) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    INIT_CMD(STC_CMD_STORAGE);
    if (send_cmd(fd, &cfg, &c, 0) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    INIT_CMD(STC_CMD_LASER);
    c.u.laser.id = 1;
    c.u.laser.on_off = 1;
    if (send_cmd(fd, &cfg, &c, 0) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    INIT_CMD(STC_CMD_TURRET);
    c.u.turret.motion_type = (uint8_t)STC_MOTION_SPEED;
    c.u.turret.az = 10.0f;
    c.u.turret.el = -3.0f;
    if (send_cmd(fd, &cfg, &c, 0) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    INIT_CMD(STC_CMD_TURRET);
    c.u.turret.motion_type = (uint8_t)STC_MOTION_ABSOLUTE;
    c.u.turret.az = 45.0f;
    c.u.turret.el = 0.0f;
    if (send_cmd(fd, &cfg, &c, 0) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    INIT_CMD(STC_CMD_MIRROR);
    c.u.mirror.mirror_id = 0xFF;
    c.u.mirror.motion_type = (uint8_t)STC_MOTION_ABSOLUTE;
    c.u.mirror.elevation = 12.34f;
    if (send_cmd(fd, &cfg, &c, 0) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    INIT_CMD(STC_CMD_SUT);
    c.u.sut.id = 0xFF;
    c.u.sut.motion_type = (uint8_t)STC_MOTION_RELATIVE;
    c.u.sut.az = 1.0f;
    c.u.sut.el = -1.0f;
    if (send_cmd(fd, &cfg, &c, 0) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    // manualpos double
    INIT_CMD(STC_CMD_MANUALPOS);
    c.u.manualpos.lat = 35.123;
    c.u.manualpos.lon = 24.456;
    c.u.manualpos.alt = 120.0;
    c.u.manualpos.heading = 270.0;
    c.u.manualpos.encoded_as_double = 1;
    c.payload_size = 32;
    if (send_cmd(fd, &cfg, &c, 0) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    INIT_CMD(STC_CMD_POSMODE);
    c.u.posmode.v = 1; // manual
    if (send_cmd(fd, &cfg, &c, 0) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    INIT_CMD(STC_CMD_REQ_SW_VERSION);
    if (send_cmd(fd, &cfg, &c, 0) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    // a couple bad cases
    printf("\nNow some intentional bad packets...\n");

    // bad_version
    INIT_CMD(STC_CMD_CU_STATE);
    c.version = 2;
    c.u.cu_state.state = 2;
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    // bad_payload (turret but wrong payload size by forcing raw)
    INIT_CMD(STC_CMD_TURRET);
    c.payload_decoded = 0;
    c.payload_size = 2;
    c.payload_raw[0] = 0x02;
    c.payload_raw[1] = 0x00;
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;

    printf("Suite done.\n");
    close(fd);
    return 0;
  }

  // ---- negative test: bad magic (patch magic bytes after normal packing)
  if (!strcmp(cmdn, "bad_magic")) {
    INIT_CMD(STC_CMD_CU_STATE);
    c.u.cu_state.state = 2;

    uint8_t out[1024];
    size_t out_len = 0;
    if (stc_icd_cmd_pack(&cfg, &c, out, sizeof(out), &out_len) != STC_ICD_OK) return 1;
    out[0] = 'B'; out[1] = 'A'; out[2] = 'D'; out[3] = '!';
    printf("TX bad_magic:\n"); hexdump(out, out_len);
    if (send_all(fd, out, out_len) != 0) { perror("send"); return 1; }
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
    close(fd);
    return 0;
  }

  // ---- negative test: unknown command ID dispatch behavior
  if (!strcmp(cmdn, "unknown_cmd")) {
    INIT_CMD(0x9999);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
    close(fd);
    return 0;
  }

  // ---- negative test: protocol version mismatch behavior
  if (!strcmp(cmdn, "bad_version")) {
    INIT_CMD(STC_CMD_CU_STATE);
    c.version = 2;
    c.u.cu_state.state = 2;
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
    close(fd);
    return 0;
  }

  // ---- negative test: malformed payload content/length behavior
  if (!strcmp(cmdn, "bad_payload")) {
    INIT_CMD(STC_CMD_TURRET);
    c.payload_decoded = 0;
    c.payload_size = 2;
    c.payload_raw[0] = 0x02;
    c.payload_raw[1] = 0x00;
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
    close(fd);
    return 0;
  }

  // ---- single command modes: argv parsing drives command-family payload generation
  if (!strcmp(cmdn, "cu_state")) {
    if (argc < 5) { usage(argv[0]); return 2; }
    INIT_CMD(STC_CMD_CU_STATE);
    c.u.cu_state.state = (uint8_t)atoi(argv[4]);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "selflev")) {
    if (argc < 5) { usage(argv[0]); return 2; }
    INIT_CMD(STC_CMD_SELFLEV);
    c.u.selflev.v = (uint8_t)atoi(argv[4]);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "ibit")) {
    if (argc < 5) { usage(argv[0]); return 2; }
    INIT_CMD(STC_CMD_IBIT);
    c.u.ibit.v = (uint8_t)atoi(argv[4]);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "storage")) {
    INIT_CMD(STC_CMD_STORAGE);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "laser")) {
    if (argc < 6) { usage(argv[0]); return 2; }
    INIT_CMD(STC_CMD_LASER);
    c.u.laser.id = (uint8_t)atoi(argv[4]);
    c.u.laser.on_off = (uint8_t)atoi(argv[5]);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "turret")) {
    if (argc < 7) { usage(argv[0]); return 2; }
    INIT_CMD(STC_CMD_TURRET);
    c.u.turret.motion_type = (uint8_t)parse_motion(argv[4]);
    c.u.turret.az = (float)atof(argv[5]);
    c.u.turret.el = (float)atof(argv[6]);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "mirror")) {
    if (argc != 7) {
      fprintf(stderr, "mirror command takes exactly 3 payload arguments: id, motion, value\n");
      usage(argv[0]);
      return 2;
    }
    INIT_CMD(STC_CMD_MIRROR);
    c.u.mirror.mirror_id = (uint8_t)atoi(argv[4]); // 255=all
    c.u.mirror.motion_type = (uint8_t)parse_motion(argv[5]);
    c.u.mirror.elevation = (float)atof(argv[6]);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "sut")) {
    if (argc < 8) { usage(argv[0]); return 2; }
    INIT_CMD(STC_CMD_SUT);
    c.u.sut.id = (uint8_t)atoi(argv[4]); // 255=all
    c.u.sut.motion_type = (uint8_t)parse_motion(argv[5]);
    c.u.sut.az = (float)atof(argv[6]);
    c.u.sut.el = (float)atof(argv[7]);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "reset")) {
    INIT_CMD(STC_CMD_RESET);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "factory")) {
    INIT_CMD(STC_CMD_FACTORY);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "shutdown")) {
    INIT_CMD(STC_CMD_SHUTDOWN);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "manualpos")) {
    if (argc < 8) { usage(argv[0]); return 2; }
    INIT_CMD(STC_CMD_MANUALPOS);
    c.u.manualpos.lat = atof(argv[4]);
    c.u.manualpos.lon = atof(argv[5]);
    c.u.manualpos.alt = atof(argv[6]);
    c.u.manualpos.heading = atof(argv[7]);
    c.u.manualpos.encoded_as_double = 1;
    c.payload_size = 32;

    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "posmode")) {
    if (argc < 5) { usage(argv[0]); return 2; }
    INIT_CMD(STC_CMD_POSMODE);
    if (!strcmp(argv[4], "gps")) c.u.posmode.v = 0;
    else if (!strcmp(argv[4], "manual")) c.u.posmode.v = 1;
    else c.u.posmode.v = (uint8_t)atoi(argv[4]);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "req_sw") || !strcmp(cmdn, "req_sw_version")) {
    INIT_CMD(STC_CMD_REQ_SW_VERSION);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "req_hw_info") || !strcmp(cmdn, "req_hw")) {
    INIT_CMD(STC_CMD_REQ_HW_INFO);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else if (!strcmp(cmdn, "req_work_time") || !strcmp(cmdn, "req_worktime")) {
    INIT_CMD(STC_CMD_REQ_WORK_TIME);
    if (send_cmd(fd, &cfg, &c, 1) != 0) return 1;
    if (read_rsp_and_print(fd, &cfg) != 0) return 1;
  } else {
    usage(argv[0]);
    close(fd);
    return 2;
  }

  close(fd);
  return 0;
}
