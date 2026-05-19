/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   TCP command server + UDP telemetry publisher for the ST communication stub.
 *
 * Design notes:
 *   Command frames are reassembled from a non-blocking TCP stream, responses are
 *   binary-encoded over the same client socket, and telemetry is periodically
 *   published via UDP to the latest connected CU peer address.
 */

// st_com.h - Lightweight communication handler for steering turret stub.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <vector>
#include <array>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <netinet/in.h>
#include "../st_protocol.h"

using namespace st_protocol;
namespace st_com {


struct Config {
    // TCP port for command/response channel.
    uint16_t listenCmdPort{7000};
    // UDP destinations (same peer IP as active TCP client, different ports).
    uint16_t telemetryH_Port{6010};
    uint16_t telemetryL_Port{6020};
    // Periodic send intervals for high/low telemetry streams.
    std::chrono::milliseconds telemetryH_Interval{10};
    std::chrono::milliseconds telemetryL_Interval{100};
};

/** @brief Communication frontend bridging CU TCP commands and stub telemetry streams. */
class StCom {
public:
    using CommandHandler = std::function<void(const CommandMessage&, StCom&)>;
    using TelemetryHProvider = std::function<TelemetryHPayload()>;
    using TelemetryLProvider = std::function<TelemetryLPayload()>;

    explicit StCom(const Config& cfg);
    ~StCom();

    // Inject handlers/providers from the simulator controller.
    void setCommandHandler(CommandHandler handler);
    void setTelemetryHProvider(TelemetryHProvider provider);
    void setTelemetryLProvider(TelemetryLProvider provider);

    // Starts/stops TCP accept loop and both telemetry loops.
    bool start();
    void stop();

    // Sends protocol responses over active TCP client socket.
    bool sendResponse(const ResponseMessage& response);
    bool sendResponse(uint16_t cmdId,
                      uint16_t seq,
                      ResponseResult result,
                      ErrorCode error,
                      const std::vector<uint8_t>& payload = {});
    bool sendTelemetryH(const TelemetryHPayload& payload);
    bool sendTelemetryL(const TelemetryLPayload& payload);

private:
    // Learns destination IP from last connected TCP client.
    void setTelemetryDestinationFromClient(const sockaddr_in& clientAddr);
    bool resolveTelemetryDestination(uint16_t port, sockaddr_in& dest) const;
    void clearTelemetryDestination();
    // Worker loops.
    void acceptLoop();
    void telemetryH_Loop();
    void telemetryL_Loop();
    // Command stream parser and codec helpers.
    bool processClientData();
    bool decodeCommand(const std::vector<uint8_t>& datagram, CommandMessage& out) const;
    std::vector<uint8_t> encodeResponse(const ResponseMessage& msg) const;
    std::vector<uint8_t> encodeTelemetry(const st_protocol::TelemetryHPayload& payload);
    std::vector<uint8_t> encodeTelemetry(const TelemetryLPayload& payload);
    void closeSockets();

private:
    Config cfg_;
    CommandHandler commandHandler_;
    TelemetryHProvider telemetryHProvider_;
    TelemetryLProvider telemetryLProvider_;

    std::atomic<bool> running_{false};
    int listenSocket_{-1};
    int clientSocket_{-1};
    std::vector<uint8_t> recvBuffer_;
    std::atomic<uint16_t> telemetrySeq_{0};
    std::atomic<uint16_t> telemetryLSeq_{0};

    std::thread acceptThread_;
    std::thread telemetryThread_;
    std::thread telemetryLThread_;
    std::mutex sendMutex_;
    mutable std::mutex telemetryDestMutex_;
    in_addr telemetryDestAddr_{};
    bool telemetryDestValid_{false};
};

} // namespace st_com
