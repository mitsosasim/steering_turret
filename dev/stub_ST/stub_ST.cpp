/*
 * Steering Turret Control (STC)
 *
 * This source file is part of the STC development codebase.
 *
 * File purpose:
 *   Process entry point for the ST communication simulator executable.
 *
 * Design notes:
 *   Loads axis/config files, acquires single-instance lock file, wires StCom,
 *   TurretSimulator and state machine objects, and dispatches protocol commands.
 */

// stub_ST.cpp - Steering Turret simulator with simple motor dynamics.
#include "VersionInfo.h"
#include "TurretSimulator.h"
#include "StubStateMachine.h"
#include "mod_LOG.h"
#include "st_com.h"
#include "CUStateMachine.h"

#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cerrno>
#include <fstream>
#include <iostream>
#include <limits.h>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace {
// enum class CommandId : uint16_t {
//     CuState       = 0x0001,
//     SelfLev       = 0x0002,
//     Ibit          = 0x0003,
//     Storage       = 0x0004,
//     Laser         = 0x0005,
//     Turret        = 0x0006,
//     Mirror        = 0x0007,
//     Sut           = 0x0008,
//     Camera        = 0x0009,
//     Reset         = 0x000A,
//     DelLog        = 0x000B,
//     Factory       = 0x000C,
//     SelfTest      = 0x000D,
//     Shutdown      = 0x000E,
//     Config        = 0x000F,
//     ManualPos     = 0x0010,
//     PosMode       = 0x0011,
//     ReqSwVersion  = 0x0012,
//     ReqHwInfo     = 0x0013,
//     ReqWorkTime   = 0x0014
// };

bool parseCommandId(uint16_t raw, CommandId& out) {
    switch (raw) {
    case 0x0001: out = CommandId::CuState; return true;
    case 0x0002: out = CommandId::SelfLev; return true;
    case 0x0003: out = CommandId::Ibit; return true;
    case 0x0004: out = CommandId::Storage; return true;
    case 0x0005: out = CommandId::Laser; return true;
    case 0x0006: out = CommandId::Turret; return true;
    case 0x0008: out = CommandId::Sut; return true;
    case 0x000A: out = CommandId::Reset; return true;
    case 0x000B: out = CommandId::DelLog; return true;
    case 0x000C: out = CommandId::Factory; return true;
    case 0x000D: out = CommandId::SelfTest; return true;
    case 0x000E: out = CommandId::Shutdown; return true;
    case 0x000F: out = CommandId::Config; return true;
    case 0x0010: out = CommandId::ManualPos; return true;
    case 0x0011: out = CommandId::PosMode; return true;
    case 0x0012: out = CommandId::ReqSwVersion; return true;
    case 0x0013: out = CommandId::ReqHwInfo; return true;
    case 0x0014: out = CommandId::ReqWorkTime; return true;
    default:
        return false;
    }
}

std::string formatCommandId(uint16_t cmd) {
    std::ostringstream oss;
    oss << "0x" << std::hex << cmd;
    return oss.str();
}

bool readFloat(const std::vector<uint8_t>& payload, size_t offset, float& out) {
    // Defensive payload decoding helper used by binary command parsers.
    if (offset + sizeof(float) > payload.size()) {
        return false;
    }
    std::memcpy(&out, payload.data() + offset, sizeof(float));
    return true;
}

bool readUint8(const std::vector<uint8_t>& payload, size_t offset, uint8_t& out) {
    // Defensive payload decoding helper for uint8 fields.
    if (offset + 1 > payload.size()) {
        return false;
    }
    out = payload[offset];
    return true;
}

bool readUint32(const std::vector<uint8_t>& payload, size_t offset, uint32_t& out) {
    // Defensive payload decoding helper for uint32 fields.
    if (offset + sizeof(uint32_t) > payload.size()) {
        return false;
    }
    std::memcpy(&out, payload.data() + offset, sizeof(uint32_t));
    return true;
}

struct AxisConfig {
    double minAngle{-180.0};
    double maxAngle{180.0};
    double maxVelocity{60.0};
    double maxAcceleration{180.0};
};

using AxisConfigMap = std::unordered_map<std::string, AxisConfig>;

std::string trim(const std::string& input) {
    auto start = std::find_if_not(input.begin(), input.end(), [](unsigned char c) { return std::isspace(c); });
    auto end = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c) { return std::isspace(c); }).base();
    if (start >= end) {
        return {};
    }
    return std::string(start, end);
}

std::string resolveConfigPath(const std::string& fileName) {
    // Prefer a config file co-located with the executable, then fallback to cwd.
    const std::string defaultPath = "./" + fileName;
    char buf[PATH_MAX] = {};
    const ssize_t len = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len <= 0) {
        return defaultPath;
    }
    buf[len] = '\0';
    std::string exePath(buf);
    const auto slashPos = exePath.find_last_of('/');
    if (slashPos == std::string::npos) {
        return defaultPath;
    }
    std::string candidate = exePath.substr(0, slashPos) + "/" + fileName;
    std::ifstream test(candidate);
    if (test.is_open()) {
        return candidate;
    }
    return defaultPath;
}

AxisConfigMap loadMotorConfig(const std::string& path) {
    // INI-like parser: [SECTION] with key=value numeric entries.
    AxisConfigMap configs;
    std::ifstream file(path);
    if (!file.is_open()) {
        mod_LOG::warning("[ST-STUB] Unable to open motor config file " + path + ", using defaults");
        return configs;
    }

    std::string line;
    std::string currentSection;
    while (std::getline(file, line)) {
        auto stripped = trim(line);
        if (stripped.empty() || stripped.front() == '#' || stripped.front() == ';') {
            continue;
        }
        if (stripped.front() == '[' && stripped.back() == ']') {
            currentSection = stripped.substr(1, stripped.size() - 2);
            continue;
        }
        auto equalPos = stripped.find('=');
        if (equalPos == std::string::npos || currentSection.empty()) {
            continue;
        }
        auto key = trim(stripped.substr(0, equalPos));
        auto valueStr = trim(stripped.substr(equalPos + 1));
        if (key.empty() || valueStr.empty()) {
            continue;
        }
        double value = 0.0;
        try {
            value = std::stod(valueStr);
        } catch (...) {
            continue;
        }
        auto& cfg = configs[currentSection];
        if (key == "min") {
            cfg.minAngle = value;
        } else if (key == "max") {
            cfg.maxAngle = value;
        } else if (key == "max_velocity") {
            cfg.maxVelocity = value;
        } else if (key == "max_acceleration") {
            cfg.maxAcceleration = value;
        }
    }
    return configs;
}

bool parseUnsigned(const std::string& value, uint32_t& out) {
    try {
        size_t pos = 0;
        unsigned long parsed = std::stoul(value, &pos);
        if (pos != value.size() || parsed > std::numeric_limits<uint32_t>::max()) {
            return false;
        }
        out = static_cast<uint32_t>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

void loadComConfig(const std::string& path, st_com::Config& cfg) {
    std::ifstream file(path);
    if (!file.is_open()) {
        mod_LOG::warning("[ST-STUB] Unable to open ST com config file " + path + ", using defaults");
        return;
    }

    std::string line;
    std::string currentSection;
    bool inConfigSection = true;
    while (std::getline(file, line)) {
        const auto stripped = trim(line);
        if (stripped.empty() || stripped.front() == '#' || stripped.front() == ';') {
            continue;
        }
        if (stripped.front() == '[' && stripped.back() == ']') {
            currentSection = trim(stripped.substr(1, stripped.size() - 2));
            inConfigSection = (currentSection.empty() || currentSection == "ST_COM" || currentSection == "Config");
            continue;
        }
        if (!inConfigSection) {
            continue;
        }

        const auto equalPos = stripped.find('=');
        if (equalPos == std::string::npos) {
            continue;
        }
        const auto key = trim(stripped.substr(0, equalPos));
        const auto valueStr = trim(stripped.substr(equalPos + 1));
        if (key.empty() || valueStr.empty()) {
            continue;
        }

        uint32_t value = 0;
        if (!parseUnsigned(valueStr, value)) {
            continue;
        }

        if (key == "listen_cmd_port") {
            if (value <= std::numeric_limits<uint16_t>::max()) {
                cfg.listenCmdPort = static_cast<uint16_t>(value);
            }
        } else if (key == "telemetry_h_port") {
            if (value <= std::numeric_limits<uint16_t>::max()) {
                cfg.telemetryH_Port = static_cast<uint16_t>(value);
            }
        } else if (key == "telemetry_l_port") {
            if (value <= std::numeric_limits<uint16_t>::max()) {
                cfg.telemetryL_Port = static_cast<uint16_t>(value);
            }
        } else if (key == "telemetry_h_interval_ms") {
            cfg.telemetryH_Interval = std::chrono::milliseconds(value);
        } else if (key == "telemetry_l_interval_ms") {
            cfg.telemetryL_Interval = std::chrono::milliseconds(value);
        }
    }
}

MotorModel::Limits axisLimits(const AxisConfigMap& map,
                              const std::string& name,
                              const AxisConfig& defaults) {
    // Resolves one axis configuration with fallback defaults.
    const auto it = map.find(name);
    const AxisConfig& cfg = (it != map.end()) ? it->second : defaults;
    MotorModel::Limits limits;
    limits.minPosition = std::min(cfg.minAngle, cfg.maxAngle);
    limits.maxPosition = std::max(cfg.minAngle, cfg.maxAngle);
    limits.maxVelocity = cfg.maxVelocity;
    limits.maxAcceleration = cfg.maxAcceleration;
    return limits;
}

int acquireInstanceLock(const std::string& lockPath) {
    int fd = ::open(lockPath.c_str(), O_CREAT | O_RDWR, 0644);
    if (fd == -1) {
        std::cerr << "[ST-STUB] Unable to open lock file " << lockPath << ": " << std::strerror(errno) << "\n";
        return -1;
    }

    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
        if (errno == EWOULDBLOCK) {
            std::cerr << "[ST-STUB] Another stub_ST instance is already running\n";
        } else {
            std::cerr << "[ST-STUB] Unable to lock file " << lockPath << ": " << std::strerror(errno) << "\n";
        }
        ::close(fd);
        return -1;
    }

    const std::string pidStr = std::to_string(::getpid()) + "\n";
    if (::ftruncate(fd, 0) == 0) {
        (void)::write(fd, pidStr.c_str(), pidStr.size());
    }
    return fd;
}
} // namespace

class StubController {
public:
    StubController(TurretSimulator& simulator, StubStateMachine& fsm)
        : simulator_(simulator), fsm_(fsm) {}

    bool handleCommand(const st_protocol::CommandMessage& message, st_com::StCom& comms) {
        // Decode command id and reject unknown values with protocol syntax error.
        CommandId id{};
        if (!parseCommandId(message.header.cmdId, id)) {
            mod_LOG::warning("[ST-STUB] Ignored unknown command " + formatCommandId(message.header.cmdId));
            comms.sendResponse(message.header.cmdId,
                               message.header.seq,
                               st_protocol::ResponseResult::Nack,
                               st_protocol::ErrorCode::SyntaxError);
            return false;
        }

        const auto& payload = message.payload;

        // Shared response helpers used by all command branches.
        auto sendAck = [&](const std::vector<uint8_t>& resp = {}) {
            comms.sendResponse(message.header.cmdId,
                               message.header.seq,
                               st_protocol::ResponseResult::Ack,
                               st_protocol::ErrorCode::NoError,
                               resp);
            return true;
        };
        auto sendError = [&](st_protocol::ErrorCode err) {
            comms.sendResponse(message.header.cmdId,
                               message.header.seq,
                               st_protocol::ResponseResult::Nack,
                               err);
            return false;
        };
        auto requireOperational = [&]() {
            // Commands that move hardware are accepted only in OPER.
            if (!fsm_.isOperational()) {
                return sendError(st_protocol::ErrorCode::NotAllowedInState);
            }
            return true;
        };
        auto requireSafeOrOper = [&]() {
            // Some commands are allowed in both SAFE and OPER states.
            auto st = fsm_.state();
            if (st == StubStateMachine::State::Safe || st == StubStateMachine::State::Oper) {
                return true;
            }
            return sendError(st_protocol::ErrorCode::NotAllowedInState);
        };

        switch (id) {
        case CommandId::CuState: {
            // Translates CU global state values into the reduced ST stub FSM.
            if (payload.size() != 1) {
                return sendError(st_protocol::ErrorCode::SyntaxError);
            }
            mod_LOG::info("[ST-STUB] Setting CU state to " + std::to_string(payload[0]));
            StubStateMachine::State target{};
            const uint8_t raw = payload[0];
            CUStateMachine::State cuState = (CUStateMachine::State)raw;
            switch (cuState) {
            case CUStateMachine::State::Safe: 
                target = StubStateMachine::State::Safe; break;
            case CUStateMachine::State::OperationalAuto: 
            case CUStateMachine::State::OperationalManual: 
            case CUStateMachine::State::OperationalCalibration: 
            case CUStateMachine::State::MaintenanceIdle: 
                target = StubStateMachine::State::Oper; break;
            // case 4: target = StubStateMachine::State::SelfLevelling; runningTask_.store(st_protocol::RunningTask::Levelling); break;
            // case 5: target = StubStateMachine::State::IbitRunning; runningTask_.store(st_protocol::RunningTask::Ibit); break;
            // case 6: target = StubStateMachine::State::Store; break;
            // case 0xFF: target = StubStateMachine::State::Fail; runningTask_.store(st_protocol::RunningTask::None); break;
            default:
                break;//return sendError(st_protocol::ErrorCode::ValueOutOfRange);
            }
            fsm_.setState(target);
            if (raw < 4 || raw == 6) {
                runningTask_.store(st_protocol::RunningTask::None);
            }
            return sendAck();
        }
        case CommandId::SelfLev: {
            if (payload.size() != 1) {
                return sendError(st_protocol::ErrorCode::SyntaxError);
            }
            const bool start = payload[0] != 0;
            if (start) {
                if (!fsm_.startSelfLevelling()) {
                    return sendError(st_protocol::ErrorCode::NotAllowedInState);
                }
                runningTask_.store(st_protocol::RunningTask::Levelling);
            } else {
                fsm_.stopSelfLevelling();
                runningTask_.store(st_protocol::RunningTask::None);
            }
            return sendAck();
        }
        case CommandId::Ibit: {
            if (payload.size() != 1) {
                return sendError(st_protocol::ErrorCode::SyntaxError);
            }
            const bool start = payload[0] != 0;
            if (start) {
                if (!fsm_.startIbit()) {
                    return sendError(st_protocol::ErrorCode::NotAllowedInState);
                }
                runningTask_.store(st_protocol::RunningTask::Ibit);
            } else {
                fsm_.stopIbit();
                runningTask_.store(st_protocol::RunningTask::None);
            }
            return sendAck();
        }
        case CommandId::Storage: {
            // Forces STORE transition through FSM validation.
            if (!fsm_.goStore()) {
                return sendError(st_protocol::ErrorCode::NotAllowedInState);
            }
            runningTask_.store(st_protocol::RunningTask::None);
            return sendAck();
        }
        case CommandId::Reset:
        case CommandId::DelLog:
        case CommandId::Factory:
        case CommandId::SelfTest:
        case CommandId::Shutdown:
        case CommandId::Config:
            // Administrative commands are acknowledged but not emulated.
            return sendAck();
        case CommandId::Laser: {
            if (payload.size() != 2) {
                return sendError(st_protocol::ErrorCode::SyntaxError);
            }
            if (!requireOperational()) {
                return false;
            }
            const uint8_t id = payload[0];
            if (!(id >= 1 && id <= 8)) {
                return sendError(st_protocol::ErrorCode::ValueOutOfRange);
            }
            return sendAck();
        }
        case CommandId::Turret: {
            if (payload.size() != (1 + 2*sizeof(float))) {
                return sendError(st_protocol::ErrorCode::SyntaxError);
            }
            if (!requireOperational()) {
                return false;
            }
            uint8_t mType = payload[0];
            float az = 0.0f;
            float el = 0.0f;
            if (!readFloat(payload, 1, az) || !readFloat(payload, 1 + sizeof(float), el)) {
                return sendError(st_protocol::ErrorCode::SyntaxError);
            }
            const auto current = simulator_.mainPosition();
            switch (mType) {
            case 0:
                // Relative position command.
                simulator_.setMainAzPosition(current[0] + az);
                simulator_.setMainElPosition(current[1] + el);
                break;
            case 1:
                // Absolute position command.
                simulator_.setMainAzPosition(az);
                simulator_.setMainElPosition(el);
                break;
            case 2:
                // Velocity command.
                //mod_LOG::info("[ST-STUB] Setting turret velocity Az: " + std::to_string(az) + " El: " + std::to_string(el));   
                simulator_.setMainAzVelocity(az);
                simulator_.setMainElVelocity(el);
                break;
            default:
                return sendError(st_protocol::ErrorCode::ValueOutOfRange);
            }
            return sendAck();
        }
        case CommandId::Sut: {
            if (payload.size() != (2 + 2*sizeof(float))) {
                return sendError(st_protocol::ErrorCode::SyntaxError);
            }
            if (!requireOperational()) {
                return false;
            }
            uint8_t id = payload[0];
            uint8_t mType = payload[1];
            float az = 0.0f;
            float el = 0.0f;
            if (!readFloat(payload, 2, az) || !readFloat(payload, 2 + sizeof(float), el)) {
                return sendError(st_protocol::ErrorCode::SyntaxError);
            }
            if (!(id == 0xFF || (id >= 1 && id <= 8))) {
                return sendError(st_protocol::ErrorCode::ValueOutOfRange);
            }
            auto sutPos = simulator_.sutPositions();
            auto applySut = [&](size_t sutId) {
                // Convert 1-based sutId to packed [az, el] pair index.
                const size_t idx = (sutId - 1U);
                switch (mType) {
                case 0:
                    // Relative position command.
                    simulator_.setSutAzPosition(sutId, sutPos[idx*2] + az);
                    simulator_.setSutElPosition(sutId, sutPos[idx*2+1] + el);
                    break;
                case 1:
                    // Absolute position command.
                    mod_LOG::infof("[ST-STUB] Setting SUT %d Az: %.2f El: %.2f", sutId, az, el);   
                    simulator_.setSutAzPosition(sutId, az);
                    simulator_.setSutElPosition(sutId, el);
                break;
                case 2:
                    // Velocity command.
                    simulator_.setSutAzVelocity(sutId, az);
                    simulator_.setSutElVelocity(sutId, el);
                    break;
                default:
                    break;
                }
            };
            if (mType > 2) {
                return sendError(st_protocol::ErrorCode::ValueOutOfRange);
            }
            if (id == 0xFF) {
                for (uint8_t s = 1; s <= 8; ++s) {
                    applySut(s);
                }
            } else {
                applySut(id);
            }
            return sendAck();
        }
        case CommandId::ManualPos: {
            if (payload.size() != sizeof(double) * 4) {
                return sendError(st_protocol::ErrorCode::SyntaxError);
            }
            if (!requireSafeOrOper()) {
                return false;
            }
            // Position not simulated; accept and ack.
            return sendAck();
        }
        case CommandId::PosMode: {
            if (payload.size() != 1) {
                return sendError(st_protocol::ErrorCode::SyntaxError);
            }
            if (!requireSafeOrOper()) {
                return false;
            }
            uint8_t mode = payload[0];
            if (mode > 1) {
                return sendError(st_protocol::ErrorCode::ValueOutOfRange);
            }
            return sendAck();
        }
        case CommandId::ReqSwVersion: {
            const auto version = version_info::detailed();
            return sendAck(std::vector<uint8_t>(version.begin(), version.end()));
        }
        case CommandId::ReqHwInfo: {
            static const std::string hwInfo = "STEERING_TURRET_SIM";
            return sendAck(std::vector<uint8_t>(hwInfo.begin(), hwInfo.end()));
        }
        case CommandId::ReqWorkTime: {
            const auto now = std::chrono::steady_clock::now();
            const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - simulator_.startTime());
            uint32_t seconds = static_cast<uint32_t>(elapsed.count());
            std::vector<uint8_t> resp(sizeof(uint32_t));
            std::memcpy(resp.data(), &seconds, sizeof(uint32_t));
            return sendAck(resp);
        }
        default:
            return sendAck();
        }
    }

    st_protocol::RunningTask runningTask() const { return runningTask_.load(); }

private:
    TurretSimulator& simulator_;
    StubStateMachine& fsm_;
    std::atomic<st_protocol::RunningTask> runningTask_{st_protocol::RunningTask::None};
};

int main() {
    // Enforce single-process execution so the stub does not fight over ports/resources.
    const std::string lockFilePath = "/tmp/stub_ST.lock";
    const int lockFd = acquireInstanceLock(lockFilePath);
    if (lockFd == -1) {
        return 1;
    }

    mod_LOG::initialize("stub_st.log", true, mod_LOG::Level::Info, true);
    const auto swVersion = version_info::detailed();
    mod_LOG::info(std::string("[ST-STUB] Starting turret simulator ") + swVersion);
    std::cout << "[ST-STUB] Starting turret simulator " << swVersion << "\n";

    StubStateMachine fsm;
    fsm.setState(StubStateMachine::State::InitDone);

    const auto motorConfigs = loadMotorConfig(resolveConfigPath("stub_motors_ST.cfg"));
    AxisConfig mainDefault{ -180.0, 180.0, 60.0, 180.0 };
    AxisConfig sutDefault{ -90.0, 90.0, 30.0, 120.0 };

    std::array<MotorModel::Limits, 2> mainLimits = {
        axisLimits(motorConfigs, "ST_AZ", mainDefault),
        axisLimits(motorConfigs, "ST_EL", mainDefault)
    };

    std::array<MotorModel::Limits, 16> sutLimits{};
    for (size_t sut = 0; sut < 8; ++sut) {
        const std::string azKey = "SUT" + std::to_string(sut + 1) + "_AZ";
        const std::string elKey = "SUT" + std::to_string(sut + 1) + "_EL";
        sutLimits[sut * 2 + 0] = axisLimits(motorConfigs, azKey, sutDefault);
        sutLimits[sut * 2 + 1] = axisLimits(motorConfigs, elKey, sutDefault);
    }

    TurretSimulator simulator(mainLimits, sutLimits);
    simulator.start();

    StubController controller(simulator, fsm);

    st_com::Config comCfg;
    loadComConfig(resolveConfigPath("stub_ST.cfg"), comCfg);
    st_com::StCom comms(comCfg);
    // Bind simulator state producers and command dispatcher into communication layer.
    comms.setTelemetryHProvider([&]() { return simulator.buildTelemetryH(fsm); });
    comms.setTelemetryLProvider([&]() { return simulator.buildTelemetryL(fsm); });
    comms.setCommandHandler([&](const st_protocol::CommandMessage& message, st_com::StCom& self) {
        controller.handleCommand(message, self);
    });

    comms.start();

    std::atomic<bool> running{true};
    std::signal(SIGINT, [](int) {});
    std::thread inputThread([&running]() {
        std::string line;
        std::cout << "[ST-STUB] Type 'quit' to stop the simulator\n";
        while (running && std::getline(std::cin, line)) {
            if (line == "quit" || line == "exit" || line == "q") {
                mod_LOG::info("received quit command, stopping simulator");
                break;
            }
        }
        running = false;
    });

    while (running) {
        // Keep main thread alive while workers handle IO and integration.
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    mod_LOG::info("[ST-STUB] Stopping simulator");

    if (inputThread.joinable()) {
        mod_LOG::info("Joining input thread");
        inputThread.join();
    }

    if (lockFd != -1) {
        ::flock(lockFd, LOCK_UN);
        ::close(lockFd);
    }
    // Temporary forced exit: current shutdown path may block in communication threads.
    exit(0);
    comms.stop();
    simulator.stop();
    mod_LOG::info("[ST-STUB] Stopping turret simulator");
    mod_LOG::shutdown();
    return 0;
}
