/*
 * ALMA SISTEMI SRL - Steering Turret Control (STC)
 *
 * Property of ALMA SISTEMI SRL.
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Implements stub TCP command processing and UDP telemetry transmission.
 *
 * Design notes:
 *   Uses non-blocking sockets, frame reassembly for TCP command streams, and
 *   cooperative thread shutdown by closing sockets and joining worker threads.
 */

// st_com.cpp - Communication handler implementing ICD for turret stub.
#include "st_com.h"

#include "mod_LOG.h"

#include <arpa/inet.h>
#include <cerrno>
#include <array>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <optional>
#include <sstream>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace {
// Builds consistent log messages including current errno text.
std::string errnoMessage(const std::string& prefix) {
    return prefix + ": " + std::string(std::strerror(errno));
}

bool setNonBlocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        return false;
    }
    return true;
}
} // namespace

namespace st_com {

StCom::StCom(const Config& cfg) : cfg_(cfg) {}

StCom::~StCom() {
    stop();
}

void StCom::setCommandHandler(CommandHandler handler) {
    commandHandler_ = std::move(handler);
}

void StCom::setTelemetryHProvider(TelemetryHProvider provider) {
    telemetryHProvider_ = std::move(provider);
}

void StCom::setTelemetryLProvider(TelemetryLProvider provider) {
    telemetryLProvider_ = std::move(provider);
}

bool StCom::start() {
    if (running_.exchange(true)) {
        // Already started.
        return true;
    }

    // TCP server socket for CU command channel.
    listenSocket_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listenSocket_ < 0) {
        mod_LOG::error(errnoMessage("[ST-COM] Unable to create listen socket"));
        running_ = false;
        return false;
    }
    int reuse = 1;
    ::setsockopt(listenSocket_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(cfg_.listenCmdPort);

    if (::bind(listenSocket_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
        mod_LOG::error(errnoMessage("[ST-COM] Unable to bind command socket"));
        stop();
        return false;
    }

    if (::listen(listenSocket_, 1) < 0) {
        mod_LOG::error(errnoMessage("[ST-COM] Unable to listen on command socket"));
        stop();
        return false;
    }

    mod_LOG::info("[ST-COM] Listening for CU on port " + std::to_string(cfg_.listenCmdPort));

    // Background workers: one TCP accept/receive loop and two UDP telemetry publishers.
    acceptThread_ = std::thread(&StCom::acceptLoop, this);
    telemetryThread_ = std::thread(&StCom::telemetryH_Loop, this);
    telemetryLThread_ = std::thread(&StCom::telemetryL_Loop, this);
    return true;
}

void StCom::setTelemetryDestinationFromClient(const sockaddr_in& clientAddr) {
    // Telemetry is unicast back to the peer IP of the active CU TCP session.
    std::lock_guard<std::mutex> lock(telemetryDestMutex_);
    telemetryDestAddr_ = clientAddr.sin_addr;
    telemetryDestValid_ = true;
}

bool StCom::resolveTelemetryDestination(uint16_t port, sockaddr_in& dest) const {
    std::lock_guard<std::mutex> lock(telemetryDestMutex_);
    if (!telemetryDestValid_) {
        return false;
    }
    dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);
    dest.sin_addr = telemetryDestAddr_;
    return true;
}

void StCom::clearTelemetryDestination() {
    std::lock_guard<std::mutex> lock(telemetryDestMutex_);
    telemetryDestValid_ = false;
    telemetryDestAddr_ = {};
}

void StCom::stop() {
    if (!running_.exchange(false)) {
        // Already stopped.
        return;
    }

    closeSockets();

    if (acceptThread_.joinable()) {
        acceptThread_.join();
    }
    if (telemetryThread_.joinable()) {
        telemetryThread_.join();
    }
    if (telemetryLThread_.joinable()) {
        telemetryLThread_.join();
    }
}

void StCom::closeSockets() {
    if (clientSocket_ >= 0) {
        ::close(clientSocket_);
        clientSocket_ = -1;
    }
    if (listenSocket_ >= 0) {
        ::close(listenSocket_);
        listenSocket_ = -1;
    }
}

void StCom::acceptLoop() {
    while (running_) {
        sockaddr_in addr{};
        socklen_t len = sizeof(addr);
        int client = ::accept(listenSocket_, reinterpret_cast<sockaddr*>(&addr), &len);
        if (client < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            mod_LOG::error(errnoMessage("[ST-COM] Accept error"));
            break;
        }

        if (clientSocket_ >= 0) {
            // Replace previous client; this stub supports one active CU connection.
            ::close(clientSocket_);
        }
        clientSocket_ = client;
        setNonBlocking(clientSocket_);

        char ip[INET_ADDRSTRLEN] = {};
        ::inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));
        mod_LOG::info(std::string("[ST-COM] CU connected from ") + ip + ":" +
                      std::to_string(ntohs(addr.sin_port)));
        setTelemetryDestinationFromClient(addr);
        mod_LOG::info(std::string("[ST-COM] Telemetry UDP destination set to ") + ip);

        while (running_ && clientSocket_ >= 0) {
            // Keep draining command stream until disconnect/error.
            if (!processClientData()) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

// Packet boundary handler: drains TCP stream buffer into complete protocol command frames.
bool StCom::processClientData() {
    if (clientSocket_ < 0) {
        return false;
    }
    std::array<uint8_t, 1024> buffer{};
    ssize_t bytes = ::recv(clientSocket_, buffer.data(), buffer.size(), 0);
    if (bytes > 0) {
        recvBuffer_.insert(recvBuffer_.end(), buffer.begin(), buffer.begin() + bytes);
        while (true) {
            if (recvBuffer_.size() < sizeof(CommandHeader)) {
                // Need more bytes to read at least one full header.
                break;
            }
            CommandHeader header{};
            std::memcpy(&header, recvBuffer_.data(), sizeof(CommandHeader));
            const size_t totalSize = sizeof(CommandHeader) + header.payloadSize;
            if (recvBuffer_.size() < totalSize) {
                // Header is present but payload is not complete yet.
                break;
            }

            std::vector<uint8_t> datagram(recvBuffer_.begin(), recvBuffer_.begin() + totalSize);
            recvBuffer_.erase(recvBuffer_.begin(), recvBuffer_.begin() + totalSize);

            CommandMessage cmd;
            if (!decodeCommand(datagram, cmd)) {
                // On syntax errors, mirror request cmdId/seq and send NACK.
                ResponseMessage response;
                response.header.protocolVersion = kProtocolVersion;
                response.header.cmdId = header.cmdId;
                response.header.seq = header.seq;
                response.header.result = ResponseResult::Nack;
                response.header.errCode = ErrorCode::SyntaxError;
                sendResponse(response);
                continue;
            }

            if (commandHandler_) {
                commandHandler_(cmd, *this);
            } else {
                mod_LOG::warning("[ST-COM] Command handler not set");
                sendResponse(cmd.header.cmdId,
                             cmd.header.seq,
                             ResponseResult::Ack,
                             ErrorCode::NoError);
            }
        }
        return true;
    }
    if (bytes == 0) {
        mod_LOG::info("[ST-COM] CU disconnected");
        ::close(clientSocket_);
        clientSocket_ = -1;
        clearTelemetryDestination();
        return false;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return true;
    }
    mod_LOG::error(errnoMessage("[ST-COM] Receive error"));
    ::close(clientSocket_);
    clientSocket_ = -1;
    clearTelemetryDestination();
    return false;
}

bool StCom::decodeCommand(const std::vector<uint8_t>& datagram, CommandMessage& out) const {
    if (datagram.size() < sizeof(CommandHeader)) {
        mod_LOG::warning("[ST-COM] Datagram too short for command header");
        return false;
    }

    std::memcpy(&out.header, datagram.data(), sizeof(CommandHeader));

    if (out.header.magic != kCommandMagic) {
        mod_LOG::warning("[ST-COM] Command magic mismatch");
        return false;
    }
    if (out.header.protocolVersion != kProtocolVersion) {
        mod_LOG::warning("[ST-COM] Protocol version mismatch");
        return false;
    }

    const size_t expectedSize = sizeof(CommandHeader) + out.header.payloadSize;
    if (datagram.size() != expectedSize) {
        mod_LOG::warning("[ST-COM] Payload length mismatch");
        return false;
    }
    out.payload.assign(datagram.begin() + sizeof(CommandHeader), datagram.end());
    return true;
}

bool StCom::sendResponse(const ResponseMessage& response) {
    if (clientSocket_ < 0) {
        mod_LOG::warning("[ST-COM] No active CU connection");
        return false;
    }
    auto buffer = encodeResponse(response);
    size_t totalSent = 0;
    std::lock_guard<std::mutex> lock(sendMutex_);
    while (totalSent < buffer.size()) {
        // send() may write only a partial buffer; loop until message is fully sent.
        ssize_t sent = ::send(clientSocket_, buffer.data() + totalSent, buffer.size() - totalSent, 0);
        if (sent < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            mod_LOG::error(errnoMessage("[ST-COM] Failed to send response"));
            return false;
        }
        if (sent == 0) {
            return false;
        }
        totalSent += static_cast<size_t>(sent);
    }
    return true;
}

bool StCom::sendResponse(uint16_t cmdId,
                         uint16_t seq,
                         ResponseResult result,
                         ErrorCode error,
                         const std::vector<uint8_t>& payload) {
    ResponseMessage resp;
    resp.header.protocolVersion = kProtocolVersion;
    resp.header.cmdId = cmdId;
    resp.header.seq = seq;
    resp.header.result = result;
    resp.header.errCode = error;
    resp.header.payloadSize = static_cast<uint16_t>(payload.size());
    resp.payload = payload;
    return sendResponse(resp);
}

// Serialization boundary: converts internal response struct ownership into wire-format byte payloads.
std::vector<uint8_t> StCom::encodeResponse(const ResponseMessage& msg) const {
    const auto hSize = sizeof(msg.header);
    std::vector<uint8_t> buffer(hSize + msg.payload.size());
    std::memcpy(buffer.data(), &msg.header, hSize);
    if (!msg.payload.empty()) {
        std::memcpy(buffer.data() + hSize, msg.payload.data(), msg.payload.size());
    }
    return buffer;
}

std::vector<uint8_t> StCom::encodeTelemetry(const TelemetryHPayload& payload) {
    // Stamp transport header fields owned by this endpoint.
    TelemetryHPayload packet = payload;
    packet.header.magic = kTelemetryMagicHigh;
    packet.header.protocolVersion = kProtocolVersion;
    packet.header.seq = telemetrySeq_++;

    std::vector<uint8_t> buffer(sizeof(TelemetryHPayload));
    std::memcpy(buffer.data(), &packet, sizeof(TelemetryHPayload));
    return buffer;
}

std::vector<uint8_t> StCom::encodeTelemetry(const TelemetryLPayload& payload) {
    // Build variable-size packet because errorCodes is dynamic.
    TelemetryLPayload packet = payload;
    packet.header.magic = kTelemetryMagicLow;
    packet.header.protocolVersion = kProtocolVersion;
    packet.header.seq = telemetryLSeq_++;

    const size_t fixedSize = sizeof(TelemetryLHeader) + sizeof(packet.timestamp) +
                             sizeof(packet.state) + sizeof(packet.uptime) +
                             sizeof(packet.temperature) + sizeof(packet.humidity) +
                             sizeof(packet.pressure) + packet.geoPos.size()*sizeof(double) +
                             sizeof(packet.orientationAz) + packet.motorStatus.size() +
                             packet.laserStatus.size() + sizeof(packet.levelX) +
                             sizeof(packet.levelY) + sizeof(packet.numErrors);

    std::vector<uint8_t> buffer(fixedSize + packet.errorCodes.size() * sizeof(uint32_t));
    size_t offset = 0;
    auto copy = [&](const void* src, size_t len) {
        std::memcpy(buffer.data() + offset, src, len);
        offset += len;
    };

    copy(&packet.header, sizeof(packet.header));
    copy(&packet.timestamp, sizeof(packet.timestamp));
    copy(&packet.state, sizeof(packet.state));
    copy(&packet.uptime, sizeof(packet.uptime));
    copy(&packet.temperature, sizeof(packet.temperature));
    copy(&packet.humidity, sizeof(packet.humidity));
    copy(&packet.pressure, sizeof(packet.pressure));
    copy(packet.geoPos.data(), sizeof(double)*packet.geoPos.size());
    copy(&packet.orientationAz, sizeof(packet.orientationAz));
    copy(packet.motorStatus.data(), packet.motorStatus.size());
    copy(packet.laserStatus.data(), packet.laserStatus.size());
    copy(&packet.levelX, sizeof(packet.levelX));
    copy(&packet.levelY, sizeof(packet.levelY));
    copy(&packet.numErrors, sizeof(packet.numErrors));

    if (!packet.errorCodes.empty()) {
        copy(packet.errorCodes.data(), packet.errorCodes.size() * sizeof(uint32_t));
    }
    return buffer;
}

bool StCom::sendTelemetryH(const TelemetryHPayload& payload) {
    // UDP socket is created per send to keep implementation simple and stateless.
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        mod_LOG::error(errnoMessage("[ST-COM] Unable to create telemetry socket"));
        return false;
    }

    sockaddr_in dest{};
    if (!resolveTelemetryDestination(cfg_.telemetryH_Port, dest)) {
        ::close(sock);
        return false;
    }

    auto packet = encodeTelemetry(payload);
    ssize_t sent = ::sendto(sock,
                            packet.data(),
                            packet.size(),
                            0,
                            reinterpret_cast<sockaddr*>(&dest),
                            sizeof(dest));
    ::close(sock);
    if (sent < 0) {
        mod_LOG::error(errnoMessage("[ST-COM] Failed to send telemetry"));
        return false;
    }
    return true;
}

bool StCom::sendTelemetryL(const TelemetryLPayload& payload) {
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        mod_LOG::error(errnoMessage("[ST-COM] Unable to create telemetry socket"));
        return false;
    }

    sockaddr_in dest{};
    if (!resolveTelemetryDestination(cfg_.telemetryL_Port, dest)) {
        ::close(sock);
        return false;
    }

    auto packet = encodeTelemetry(payload);
    ssize_t sent = ::sendto(sock,
                            packet.data(),
                            packet.size(),
                            0,
                            reinterpret_cast<sockaddr*>(&dest),
                            sizeof(dest));
    ::close(sock);
    if (sent < 0) {
        mod_LOG::error(errnoMessage("[ST-COM] Failed to send telemetry"));
        return false;
    }
    return true;
}

void StCom::telemetryH_Loop() {
    while (running_) {
        if (telemetryHProvider_) {
            auto payload = telemetryHProvider_();
            sendTelemetryH(payload);
        }
        std::this_thread::sleep_for(cfg_.telemetryH_Interval);
    }
}

void StCom::telemetryL_Loop() {
    while (running_) {
        if (telemetryLProvider_) {
            auto payload = telemetryLProvider_();
            sendTelemetryL(payload);
        }
        std::this_thread::sleep_for(cfg_.telemetryL_Interval);
    }
}

} // namespace st_com
