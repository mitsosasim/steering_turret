/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   UDP telemetry receiver/decoder for STC ICD TLMH/TLML streams.
 *
 * Design notes:
 *   Decodes multicast telemetry by default and also supports direct unicast bind
 *   modes used during bench testing, using the same ICD parse logic as runtime.
 */

// tlm_rx_udp.c
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#include "stc_icd.h"

#include <arpa/inet.h>
#include <errno.h>
#include <net/if.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

static void usage(const char* p) {
  fprintf(stderr,
    "Usage: %s [--ifname eth0]\n"
    "          [--tlmh-ip 239.255.10.2 --tlmh-port 6010]\n"
    "          [--tlml-ip 239.255.10.3 --tlml-port 6020]\n"
    "          [--raw] [--verbose]\n"
    "\n"
    "Examples:\n"
    "  %s --ifname eth0\n"
    "  %s --raw\n"
    "  %s --verbose\n",
    p, p, p, p);
}

static void print_usage(FILE* stream, const char* prog) {
  (void)stream;
  usage(prog);
}

static void enable_tlmh_only(int port, int* tlmh_port, int* tlml_enabled) {
  *tlmh_port = port;
  *tlml_enabled = 0;
}

static void enable_tlml_only(int port, int* tlml_port, int* tlmh_enabled) {
  *tlml_port = port;
  *tlmh_enabled = 0;
}

static void hexdump_line(const uint8_t* b, size_t n) {
  for (size_t i = 0; i < n; i++) {
    printf("%02X", b[i]);
    if (i + 1 < n) printf(" ");
  }
  printf("\n");
}

static int open_mcast_rx(const char* group_or_ip, int port, const char* ifname) {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return -1;

  int yes = 1;
  (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
#ifdef SO_REUSEPORT
  (void)setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
#endif

  struct in_addr parsed_addr;
  if (inet_pton(AF_INET, group_or_ip, &parsed_addr) != 1) {
    close(fd);
    errno = EINVAL;
    return -1;
  }

  const uint32_t addr_be = ntohl(parsed_addr.s_addr);
  const int is_multicast = (addr_be >= 0xE0000000u && addr_be <= 0xEFFFFFFFu);

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  /* Multicast: bind INADDR_ANY + join group; Unicast: bind directly to given IP. */
  addr.sin_addr.s_addr = is_multicast ? htonl(INADDR_ANY) : parsed_addr.s_addr;
  addr.sin_port = htons((uint16_t)port);

  if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    close(fd);
    return -1;
  }

  if (is_multicast) {
    struct ip_mreqn mreq;
    memset(&mreq, 0, sizeof(mreq));
    mreq.imr_multiaddr = parsed_addr;

    // Multicast default: kernel chooses interface unless --ifname overrides.
    mreq.imr_address.s_addr = htonl(INADDR_ANY);

    if (ifname && ifname[0]) {
      unsigned int idx = if_nametoindex(ifname);
      if (idx == 0) {
        close(fd);
        errno = ENODEV;
        return -1;
      }
      mreq.imr_ifindex = (int)idx;
    }

    if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
      close(fd);
      return -1;
    }
  }

  // reduce packet loss risk
  int rcvbuf = 1 << 20;
  (void)setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

  return fd;
}

static double monotonic_sec(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

int main(int argc, char** argv) {
  const char* ifname = NULL;

  const char* tlmh_ip = "239.255.10.2";
  int tlmh_port = 6010;

  const char* tlml_ip = "239.255.10.3";
  int tlml_port = 6020;

  int print_raw = 0;
  int verbose = 0;
  int tlmh_enabled = 1;
  int tlml_enabled = 1;

  if (argc >= 2 && argv[1][0] != '-') {
    /* Compatibility shorthand: a lone port enables only its matching stream. */
    int port = atoi(argv[1]);
    if (port == 6010) {
      enable_tlmh_only(port, &tlmh_port, &tlml_enabled);
      tlmh_ip = "0.0.0.0";
    } else if (port == 6020) {
      enable_tlml_only(port, &tlml_port, &tlmh_enabled);
      tlml_ip = "0.0.0.0";
    } else {
      print_usage(stderr, argv[0]);
      return 2;
    }
  }

  for (int i = (argc >= 2 && argv[1][0] != '-') ? 2 : 1; i < argc; i++) {
    if (!strcmp(argv[i], "--ifname") && i + 1 < argc) ifname = argv[++i];
    else if (!strcmp(argv[i], "--tlmh-ip") && i + 1 < argc) tlmh_ip = argv[++i];
    else if (!strcmp(argv[i], "--tlmh-port") && i + 1 < argc) tlmh_port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--tlml-ip") && i + 1 < argc) tlml_ip = argv[++i];
    else if (!strcmp(argv[i], "--tlml-port") && i + 1 < argc) tlml_port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--raw")) print_raw = 1;
    else if (!strcmp(argv[i], "--verbose")) verbose = 1;
    else { usage(argv[0]); return 2; }
  }

  int fd_h = -1;
  if (tlmh_enabled) {
    fd_h = open_mcast_rx(tlmh_ip, tlmh_port, ifname);
    if (fd_h < 0) { perror("open_mcast_rx(TLMH)"); return 1; }
  }

  int fd_l = -1;
  if (tlml_enabled) {
    fd_l = open_mcast_rx(tlml_ip, tlml_port, ifname);
    if (fd_l < 0) { perror("open_mcast_rx(TLML)"); return 1; }
  }

  if (tlmh_enabled) printf("listening UDP 0.0.0.0:%d stream=TLMH\n", tlmh_port);
  if (tlml_enabled) printf("listening UDP 0.0.0.0:%d stream=TLML\n", tlml_port);

  stc_icd_cfg_t cfg = stc_icd_parse_cfg_default();
  cfg.strict_magic = 1;
  cfg.strict_version = 0;

  double last_h = 0.0, last_l = 0.0;
  unsigned long cnt_h = 0, cnt_l = 0;

  struct pollfd pfds[2];
  int nfds = 0;
  if (tlmh_enabled) {
    pfds[nfds].fd = fd_h;
    pfds[nfds].events = POLLIN;
    nfds++;
  }
  if (tlml_enabled) {
    pfds[nfds].fd = fd_l;
    pfds[nfds].events = POLLIN;
    nfds++;
  }

  for (;;) {
    int pr = poll(pfds, (nfds_t)nfds, 1000);
    if (pr < 0) {
      if (errno == EINTR) continue;
      perror("poll");
      break;
    }
    if (pr == 0) {
      // periodic rate print
      double t = monotonic_sec();
      if (last_h > 0.0 && t - last_h > 1.0) {
        // no packets recently
      }
      continue;
    }

    for (int k = 0; k < nfds; k++) {
      if (!(pfds[k].revents & POLLIN)) continue;

      uint8_t buf[2048];
      ssize_t n = recv(pfds[k].fd, buf, sizeof(buf), 0);
      if (n < 0) {
        if (errno == EINTR) continue;
        perror("recv");
        continue;
      }

      if (print_raw) { /* Optional raw-wire hex dump before decode. */
        printf("RX %s %zd bytes:\n", (k==0 ? "TLMH" : "TLML"), n);
        hexdump_line(buf, (size_t)n);
      }

      if (pfds[k].fd == fd_h) {
        // TLMH
        stc_tlmh_t m;
        stc_icd_rc_t rc = stc_icd_tlmh_parse(&cfg, buf, (size_t)n, &m);
        if (rc != STC_ICD_OK) {
          printf("TLMH parse error rc=%d (len=%zd)\n", rc, n);
          continue;
        }

        cnt_h++;
        double t = monotonic_sec();
        double dt = (last_h > 0.0) ? (t - last_h) : 0.0;
        last_h = t;

        // Print compact line (axis0 is the only one you fill right now)
        printf("[TLMH] seq=%u ts_ms=%llu pos0=% .3f deg vel0=% .3f deg/s acc0=% .3f deg/s^2",
               m.seq, (unsigned long long)m.timestamp_ms,
               m.position[0], m.rates[0], m.acceleration[0]);

        if (verbose) {
          printf("\n  acc[0..17]=");
          for (size_t i = 0; i < STC_ICD_AXES_COUNT; i++) {
            printf(" % .3f", m.acceleration[i]);
          }
        }

        if (dt > 0.0) printf("  (dt=%.3f ms)", dt*1000.0);
        printf("\n");

      } else {
        // TLML
        stc_tlml_t m;
        stc_icd_rc_t rc = stc_icd_tlml_parse(&cfg, buf, (size_t)n, &m);
        if (rc != STC_ICD_OK) {
          printf("TLML parse error rc=%d (len=%zd)\n", rc, n);
          continue;
        }

        cnt_l++;
        double t = monotonic_sec();
        double dt = (last_l > 0.0) ? (t - last_l) : 0.0;
        last_l = t;

        printf("[TLML] seq=%u ts_ms=%llu state=%u uptime=%us lat=%.6f lon=%.6f alt=%.2f hdg=%.2f errors=%u",
               m.seq, (unsigned long long)m.timestamp_ms,
               m.state, (unsigned)m.uptime_sec,
               m.lat, m.lon, m.alt_m, m.heading_deg, (unsigned)m.error_count);

        if (!verbose) {
          printf(" motors=");
          for (size_t i = 0; i < STC_ICD_AXES_COUNT; ++i) {
            printf("%02X%s", m.motor_status[i],
                   (i + 1u < STC_ICD_AXES_COUNT) ? "," : "");
          }

          printf(" lasers=");
          for (size_t i = 0; i < STC_ICD_LASER_COUNT; ++i) {
            printf("%02X%s", m.laser_status[i],
                   (i + 1u < STC_ICD_LASER_COUNT) ? "," : "");
          }
        }

        if (dt > 0.0) printf("  (dt=%.1f ms)", dt*1000.0);
        printf("\n");

        if (verbose) {
          printf("  motor_status[%u]=", (unsigned)STC_ICD_AXES_COUNT);
          for (size_t i = 0; i < STC_ICD_AXES_COUNT; ++i) {
            printf("%02X%s", m.motor_status[i],
                   (i + 1u < STC_ICD_AXES_COUNT) ? " " : "");
          }
          printf("\n");

          printf("  laser_status[%u]=", (unsigned)STC_ICD_LASER_COUNT);
          for (size_t i = 0; i < STC_ICD_LASER_COUNT; ++i) {
            printf("%02X%s", m.laser_status[i],
                   (i + 1u < STC_ICD_LASER_COUNT) ? " " : "");
          }
          printf("\n");
        }

        if (m.error_count > 0) {
          for (unsigned i = 0; i < m.error_count; i++) {
            uint32_t code = m.errors[i];
            uint8_t cat = (uint8_t)((code >> 24) & 0xFFu);
            uint8_t sub = (uint8_t)((code >> 16) & 0xFFu);
            uint16_t err_code = (uint16_t)(code & 0xFFFFu);
            printf("  diag[%u]=0x%08X category=%u subcat=%u code=%u flags=0x%08X\n",
                   i,
                   (unsigned)code,
                   (unsigned)cat,
                   (unsigned)sub,
                   (unsigned)err_code,
                   (unsigned)code);
          }
        }
      }
    }
  }

  if (fd_h >= 0) close(fd_h);
  if (fd_l >= 0) close(fd_l);
  return 0;
}
