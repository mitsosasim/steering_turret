#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSizePolicy>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QUdpSocket>
#include <QtCore/QDateTime>
#include <QtCore/QElapsedTimer>
#include <QtCore/QFile>
#include <QtCore/QHash>
#include <QtCore/QRegularExpression>
#include <QtCore/QTime>
#include <QtCore/QTimer>
#include <QtCore/QProcess>

#include <array>
#include <cstdint>
#include <cstring>

#ifdef __linux__
#include <linux/capability.h>
#endif

extern "C" {
#include "stc_icd.h"
}

namespace {
constexpr uint16_t kDefaultControlPort = 7000;
constexpr uint16_t kDefaultTlmHighPort = 6010;
constexpr uint16_t kDefaultTlmLowPort = 6020;
constexpr int kRspHeaderSize = 14;
constexpr int kAxisCount = static_cast<int>(STC_ICD_AXES_COUNT);

QString stateToString(uint8_t state) {
  switch (state) {
    case 0: return "INIT";
    case 1: return "INIT_DONE";
    case 2: return "SAFE";
    case 3: return "OPER";
    case 4: return "SELF_LEVELLING";
    case 5: return "IBIT_RUNNING";
    case 6: return "STORE";
    case 0xFF: return "FAIL";
    default: return QString("UNKNOWN(%1)").arg(state);
  }
}

QString nackErrToString(uint16_t errCode) {
  switch (errCode) {
    case STC_ERR_NO_ERROR: return "No error";
    case STC_ERR_CLIENT_UNKNOWN: return "Client unknown";
    case STC_ERR_PROTOCOL_MISMATCH: return "Protocol mismatch";
    case STC_ERR_SYNTAX_ERROR: return "Syntax error";
    case STC_ERR_VALUE_OUT_OF_RANGE: return "Value out of range";
    case STC_ERR_NOT_ALLOWED_STATE: return "Command not allowed in current state";
    case STC_ERR_INSUFFICIENT_RES: return "Insufficient resources";
    case STC_ERR_TIMEOUT: return "Timeout";
    case STC_ERR_COMMAND_NOT_IMPLEMENTED: return "Command not implemented";
    case STC_ERR_GENERIC: return "Generic error";
    default: return QString("Unknown err_code=%1").arg(errCode);
  }
}

enum class UiCommand {
  CuState,
  SelfLev,
  Ibit,
  Storage,
  Laser,
  Turret,
  Mirror,
  Sut,
  Reset,
  DelLog,
  Factory,
  SelfTest,
  Shutdown,
  Config,
  ManualPos,
  PosMode,
  ReqSwVersion,
  ReqHwInfo,
  ReqWorkTime
};

struct CommandSpec {
  UiCommand ui;
  const char *label;
  uint16_t cmdId;
  bool requiresOper;
  bool enabled;
};

const std::array<CommandSpec, 19> kCommandSpecs{{
  {UiCommand::CuState, "CU_STATE", STC_CMD_CU_STATE, false, true},
  {UiCommand::SelfLev, "SELFLEV", STC_CMD_SELFLEV, false, true},
  {UiCommand::Ibit, "IBIT", STC_CMD_IBIT, false, true},
  {UiCommand::Storage, "STORAGE", STC_CMD_STORAGE, false, true},
  {UiCommand::Laser, "LASER", STC_CMD_LASER, false, true},
  {UiCommand::Turret, "TURRET", STC_CMD_TURRET, true, true},
  {UiCommand::Mirror, "MIRROR", STC_CMD_MIRROR, true, true},
  {UiCommand::Sut, "SUT", STC_CMD_SUT, true, true},
  {UiCommand::Reset, "RESET", STC_CMD_RESET, false, true},
  {UiCommand::DelLog, "DELLOG", STC_CMD_DELLOG, false, true},
  {UiCommand::Factory, "FACTORY", STC_CMD_FACTORY, false, true},
  {UiCommand::SelfTest, "SELFTEST", STC_CMD_SELFTEST, false, true},
  {UiCommand::Shutdown, "SHUTDOWN", STC_CMD_SHUTDOWN, false, true},
  {UiCommand::Config, "CONFIG (reserved)", STC_CMD_CONFIG, false, false},
  {UiCommand::ManualPos, "MANUALPOS", STC_CMD_MANUALPOS, false, true},
  {UiCommand::PosMode, "POSMODE", STC_CMD_POSMODE, false, true},
  {UiCommand::ReqSwVersion, "REQ_SW_VERSION", STC_CMD_REQ_SW_VERSION, false, true},
  {UiCommand::ReqHwInfo, "REQ_HW_INFO", STC_CMD_REQ_HW_INFO, false, true},
  {UiCommand::ReqWorkTime, "REQ_WORK_TIME", STC_CMD_REQ_WORK_TIME, false, true},
}};
}

class DiagnosticsWindow : public QMainWindow {
 public:
  DiagnosticsWindow() {
    setWindowTitle("STC Diagnostics Console");
    resize(1300, 840);

    tcpSocket_ = new QTcpSocket(this);
    udpHigh_ = new QUdpSocket(this);
    udpLow_ = new QUdpSocket(this);

    cfg_ = stc_icd_parse_cfg_default();

    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);
    root->addWidget(buildTopPanel(), 3);
    root->addWidget(buildLogPanel(), 1);
    setCentralWidget(central);

    initAxisTable();

    connect(sendButton_, &QPushButton::clicked, this, &DiagnosticsWindow::sendCommand);
    connect(commandBox_, qOverload<int>(&QComboBox::currentIndexChanged), this, &DiagnosticsWindow::onCommandSelectionChanged);
    connect(udpHigh_, &QUdpSocket::readyRead, this, &DiagnosticsWindow::processHighTelemetry);
    connect(udpLow_, &QUdpSocket::readyRead, this, &DiagnosticsWindow::processLowTelemetry);
    connect(simModeBox_, &QCheckBox::toggled, this, &DiagnosticsWindow::onSimModeChanged);
    connect(canBringupButton_, &QPushButton::clicked, this, &DiagnosticsWindow::bringUpCanAndConnect);
    connect(reconnectButton_, &QPushButton::clicked, this, [this]() {
      tcpSocket_->abort();
      tcpConnected_ = false;
      updateTcpStatusLabel();
      ensureTcpConnected();
    });
    connect(tlmHighPortSpin_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int) { bindTelemetrySockets(); });
    connect(tlmLowPortSpin_, qOverload<int>(&QSpinBox::valueChanged), this, [this](int) { bindTelemetrySockets(); });
    connect(tcpSocket_, &QTcpSocket::connected, this, &DiagnosticsWindow::onTcpConnected);
    connect(tcpSocket_, &QTcpSocket::disconnected, this, &DiagnosticsWindow::onTcpDisconnected);
    connect(tcpSocket_, &QTcpSocket::readyRead, this, &DiagnosticsWindow::onTcpReadyRead);
    connect(tcpSocket_, &QTcpSocket::errorOccurred, this, &DiagnosticsWindow::onTcpError);

    connectTimer_ = new QTimer(this);
    connectTimer_->setInterval(1000);
    connect(connectTimer_, &QTimer::timeout, this, &DiagnosticsWindow::ensureTcpConnected);

    bindTelemetrySockets();
    connectTimer_->start();

    simTimer_ = new QTimer(this);
    simTimer_->setInterval(20);
    connect(simTimer_, &QTimer::timeout, this, &DiagnosticsWindow::tickSimulation);

    onCommandSelectionChanged(commandBox_->currentIndex());
    appendLog("Diagnostics GUI initialized.");
    runCapabilityStartupCheck();
  }

 private:
  void onCommandSelectionChanged(int index) {
    payloadStack_->setCurrentIndex(index);
    updateCommandAvailability();
  }

  UiCommand currentUiCommand() const {
    return static_cast<UiCommand>(commandBox_->currentData().toInt());
  }

  void makeHeader(stc_icd_cmd_t *cmd) {
    std::memcpy(cmd->magic, cfg_.cmd_magic, sizeof(cmd->magic));
    cmd->version = cfg_.protocol_version_default;
    cmd->controller_id = static_cast<uint8_t>(controllerIdSpin_->value());
    cmd->seq = nextSeq_++;
    cmd->timestamp_ms = static_cast<uint64_t>(QDateTime::currentMSecsSinceEpoch());
  }

  bool buildCommandFromUi(stc_icd_cmd_t *cmd) {
    makeHeader(cmd);
    cmd->payload_decoded = 1;

    switch (currentUiCommand()) {
      case UiCommand::CuState:
        cmd->cmd_id = STC_CMD_CU_STATE;
        cmd->u.cu_state.state = static_cast<uint8_t>(cuStateBox_->currentData().toInt());
        cmd->payload_size = 1;
        return true;
      case UiCommand::SelfLev:
        cmd->cmd_id = STC_CMD_SELFLEV;
        cmd->u.selflev.v = static_cast<uint8_t>(selfLevActionBox_->currentData().toInt());
        cmd->payload_size = 1;
        return true;
      case UiCommand::Ibit:
        cmd->cmd_id = STC_CMD_IBIT;
        cmd->u.ibit.v = static_cast<uint8_t>(ibitActionBox_->currentData().toInt());
        cmd->payload_size = 1;
        return true;
      case UiCommand::Storage:
        cmd->cmd_id = STC_CMD_STORAGE;
        cmd->payload_size = 0;
        return true;
      case UiCommand::Laser:
        cmd->cmd_id = STC_CMD_LASER;
        cmd->u.laser.id = static_cast<uint8_t>(laserIdSpin_->value());
        cmd->u.laser.on_off = static_cast<uint8_t>(laserActionBox_->currentData().toInt());
        cmd->payload_size = 2;
        return true;
      case UiCommand::Turret:
        cmd->cmd_id = STC_CMD_TURRET;
        cmd->u.turret.motion_type = static_cast<uint8_t>(turretTypeBox_->currentData().toInt());
        cmd->u.turret.az = static_cast<float>(turretAzSpin_->value());
        cmd->u.turret.el = static_cast<float>(turretElSpin_->value());
        cmd->payload_size = 9;
        return true;
      case UiCommand::Mirror:
        cmd->cmd_id = STC_CMD_MIRROR;
        cmd->u.mirror.mirror_id = static_cast<uint8_t>(mirrorIdBox_->currentData().toInt());
        cmd->u.mirror.motion_type = static_cast<uint8_t>(mirrorTypeBox_->currentData().toInt());
        cmd->u.mirror.elevation = static_cast<float>(mirrorElevationSpin_->value());
        cmd->payload_size = 6;
        return true;
      case UiCommand::Sut:
        cmd->cmd_id = STC_CMD_SUT;
        cmd->u.sut.id = static_cast<uint8_t>(sutIdBox_->currentData().toInt());
        cmd->u.sut.motion_type = static_cast<uint8_t>(sutTypeBox_->currentData().toInt());
        cmd->u.sut.az = static_cast<float>(sutAzSpin_->value());
        cmd->u.sut.el = static_cast<float>(sutElSpin_->value());
        cmd->payload_size = 10;
        return true;
      case UiCommand::Reset:
        cmd->cmd_id = STC_CMD_RESET;
        cmd->payload_size = 0;
        return true;
      case UiCommand::DelLog:
        cmd->cmd_id = STC_CMD_DELLOG;
        cmd->payload_size = 0;
        return true;
      case UiCommand::Factory:
        cmd->cmd_id = STC_CMD_FACTORY;
        cmd->payload_size = 0;
        return true;
      case UiCommand::SelfTest:
        cmd->cmd_id = STC_CMD_SELFTEST;
        cmd->payload_size = 0;
        return true;
      case UiCommand::Shutdown:
        cmd->cmd_id = STC_CMD_SHUTDOWN;
        cmd->payload_size = 0;
        return true;
      case UiCommand::Config:
        cmd->cmd_id = STC_CMD_CONFIG;
        cmd->payload_decoded = 0;
        cmd->payload_size = 0;
        return true;
      case UiCommand::ManualPos:
        cmd->cmd_id = STC_CMD_MANUALPOS;
        cmd->u.manualpos.lat = manualLatSpin_->value();
        cmd->u.manualpos.lon = manualLonSpin_->value();
        cmd->u.manualpos.alt = manualAltSpin_->value();
        cmd->u.manualpos.heading = manualHeadingSpin_->value();
        cmd->u.manualpos.encoded_as_double = 1;
        cmd->payload_size = 32;
        return true;
      case UiCommand::PosMode:
        cmd->cmd_id = STC_CMD_POSMODE;
        cmd->u.posmode.v = static_cast<uint8_t>(posModeBox_->currentData().toInt());
        cmd->payload_size = 1;
        return true;
      case UiCommand::ReqSwVersion:
        cmd->cmd_id = STC_CMD_REQ_SW_VERSION;
        cmd->payload_size = 0;
        return true;
      case UiCommand::ReqHwInfo:
        cmd->cmd_id = STC_CMD_REQ_HW_INFO;
        cmd->payload_size = 0;
        return true;
      case UiCommand::ReqWorkTime:
        cmd->cmd_id = STC_CMD_REQ_WORK_TIME;
        cmd->payload_size = 0;
        return true;
    }
    return false;
  }

  void bindTelemetrySockets() {
    const quint16 highPort = static_cast<quint16>(tlmHighPortSpin_ ? tlmHighPortSpin_->value() : kDefaultTlmHighPort);
    const quint16 lowPort = static_cast<quint16>(tlmLowPortSpin_ ? tlmLowPortSpin_->value() : kDefaultTlmLowPort);

    if (udpHigh_->isOpen()) udpHigh_->close();
    if (udpLow_->isOpen()) udpLow_->close();

    const auto flags = QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint;
    if (!udpHigh_->bind(QHostAddress::AnyIPv4, highPort, flags)) {
      appendLog(QString("Failed binding high telemetry socket on %1: %2").arg(highPort).arg(udpHigh_->errorString()));
    }
    if (!udpLow_->bind(QHostAddress::AnyIPv4, lowPort, flags)) {
      appendLog(QString("Failed binding low telemetry socket on %1: %2").arg(lowPort).arg(udpLow_->errorString()));
    }
  }

  QString commandName(uint16_t cmdId) const {
    for (const auto &spec : kCommandSpecs) {
      if (spec.cmdId == cmdId) return QString::fromLatin1(spec.label);
    }
    return QString("0x%1").arg(cmdId, 4, 16, QChar('0'));
  }

  void updateTcpStatusLabel() {
    tcpStatusLabel_->setText(QString("CU TCP: %1").arg(tcpConnected_ ? "CONNECTED" : "DISCONNECTED"));
  }

  void ensureTcpConnected() {
    if (simModeBox_->isChecked() || tcpConnected_ || tcpSocket_->state() == QAbstractSocket::ConnectingState) {
      return;
    }
    const QString host = serverIpBox_->currentText().trimmed();
    if (host.isEmpty()) return;
    tcpSocket_->connectToHost(host, static_cast<quint16>(portSpin_->value()));
  }

  void onTcpConnected() {
    tcpConnected_ = true;
    updateTcpStatusLabel();
    appendLog(QString("CU TCP connected to %1:%2").arg(tcpSocket_->peerAddress().toString()).arg(tcpSocket_->peerPort()));
  }

  void onTcpDisconnected() {
    tcpConnected_ = false;
    updateTcpStatusLabel();
    appendLog("CU TCP disconnected.");
  }

  void onTcpError(QAbstractSocket::SocketError) {
    appendLog(QString("CU TCP error: %1").arg(tcpSocket_->errorString()));
    tcpConnected_ = false;
    updateTcpStatusLabel();
  }

  void onTcpReadyRead() {
    rspBuffer_.append(tcpSocket_->readAll());
    while (rspBuffer_.size() >= kRspHeaderSize) {
      const uint16_t payloadSize = static_cast<uint16_t>(static_cast<uint8_t>(rspBuffer_[12])) |
                                   (static_cast<uint16_t>(static_cast<uint8_t>(rspBuffer_[13])) << 8);
      const int frameLen = kRspHeaderSize + payloadSize;
      if (rspBuffer_.size() < frameLen) break;
      const QByteArray frame = rspBuffer_.left(frameLen);
      rspBuffer_.remove(0, frameLen);

      stc_icd_rsp_t rsp{};
      const stc_icd_rc_t rc = stc_icd_rsp_parse(&cfg_, reinterpret_cast<const uint8_t*>(frame.constData()), static_cast<size_t>(frame.size()), &rsp);
      if (rc != STC_ICD_OK) {
        appendLog("Received undecodable response frame.");
        continue;
      }

      const bool ack = rsp.result == STC_RSP_ACK;
      const QString errText = nackErrToString(rsp.err_code);
      ackLabel_->setText(QString("Last response: %1 | %2 (0x%3) seq=%4 | err_code=%5 (%6) | payload=%7B")
                           .arg(ack ? "ACK" : "NACK")
                           .arg(commandName(rsp.cmd_id))
                           .arg(rsp.cmd_id, 4, 16, QChar('0'))
                           .arg(rsp.seq)
                           .arg(rsp.err_code)
                           .arg(errText)
                           .arg(rsp.payload_size));
      const QString sentText = pendingCommands_.contains(rsp.seq)
                                ? pendingCommands_.take(rsp.seq)
                                : QString("cmd=0x%1").arg(rsp.cmd_id, 4, 16, QChar('0'));
      appendLog(QString("%1 seq=%2 (%3)").arg(ack ? "ACK" : "NACK").arg(rsp.seq).arg(sentText));
    }
  }

  void applySimulationCommand(const stc_icd_cmd_t &cmd) {
    if (cmd.cmd_id == STC_CMD_TURRET) {
      simTargetVel_[0] = cmd.u.turret.az;
      simTargetVel_[1] = cmd.u.turret.el;
    } else if (cmd.cmd_id == STC_CMD_SUT) {
      const int id = cmd.u.sut.id;
      if (id == 0xFF) {
        for (int s = 0; s < 8; ++s) {
          int base = 2 + s * 2;
          simTargetVel_[base] = cmd.u.sut.az;
          simTargetVel_[base + 1] = cmd.u.sut.el;
        }
      } else if (id >= 1 && id <= 8) {
        int base = 2 + (id - 1) * 2;
        simTargetVel_[base] = cmd.u.sut.az;
        simTargetVel_[base + 1] = cmd.u.sut.el;
      }
    } else if (cmd.cmd_id == STC_CMD_SELFLEV || cmd.cmd_id == STC_CMD_IBIT || cmd.cmd_id == STC_CMD_STORAGE) {
      simTargetVel_.fill(0.0);
    }
  }

  void sendCommand() {
    if (!sendButton_->isEnabled()) {
      appendLog("Command blocked: invalid in current state or unavailable in current mode.");
      return;
    }

    stc_icd_cmd_t cmd{};
    if (!buildCommandFromUi(&cmd)) {
      QMessageBox::warning(this, "Command error", "Unable to build command payload.");
      return;
    }

    if (simModeBox_->isChecked()) {
      applySimulationCommand(cmd);
      appendLog(QString("Simulation apply id=0x%1 seq=%2")
                  .arg(cmd.cmd_id, 4, 16, QChar('0'))
                  .arg(cmd.seq));
      return;
    }

    if (!tcpConnected_) {
      appendLog("Command not sent: CU TCP link is disconnected.");
      ensureTcpConnected();
      return;
    }

    uint8_t packet[20 + STC_ICD_MAX_PAYLOAD]{};
    size_t packetLen = 0;
    if (stc_icd_cmd_pack(&cfg_, &cmd, packet, sizeof(packet), &packetLen) != STC_ICD_OK) {
      QMessageBox::warning(this, "Serialize error", "Failed to encode ICD command.");
      return;
    }

    pendingCommands_.insert(cmd.seq, commandName(cmd.cmd_id));
    const qint64 wrote = tcpSocket_->write(reinterpret_cast<const char*>(packet), static_cast<qint64>(packetLen));
    if (wrote != static_cast<qint64>(packetLen)) {
      appendLog(QString("TCP write failed: %1").arg(tcpSocket_->errorString()));
      pendingCommands_.remove(cmd.seq);
      return;
    }

    appendLog(QString("Sent command %1 id=0x%2 seq=%3 bytes=%4")
                .arg(commandName(cmd.cmd_id))
                .arg(cmd.cmd_id, 4, 16, QChar('0'))
                .arg(cmd.seq)
                .arg(packetLen));
  }

  void processHighTelemetry() {
    while (udpHigh_->hasPendingDatagrams()) {
      QByteArray packet;
      packet.resize(static_cast<int>(udpHigh_->pendingDatagramSize()));
      udpHigh_->readDatagram(packet.data(), packet.size());
      stc_tlmh_t tlm{};
      if (stc_icd_tlmh_parse(&cfg_, reinterpret_cast<const uint8_t*>(packet.constData()), static_cast<size_t>(packet.size()), &tlm) != STC_ICD_OK) {
        appendLog("Dropped malformed TLMH datagram.");
        continue;
      }
      for (int i = 0; i < kAxisCount; ++i) {
        axisTable_->item(i, 1)->setText(QString::number(tlm.position[i], 'f', 3));
        axisTable_->item(i, 2)->setText(QString::number(tlm.rates[i], 'f', 3));
        axisTable_->item(i, 3)->setText(QString::number(tlm.acceleration[i], 'f', 3));
      }
      highSeqLabel_->setText(QString("High-rate seq: %1").arg(tlm.seq));
      tlmhPanel_->setPlainText(QString("seq=%1\nts=%2\naxis0 pos=%3 vel=%4 acc=%5\naxis1 pos=%6 vel=%7 acc=%8")
                                 .arg(tlm.seq)
                                 .arg(tlm.timestamp_ms)
                                 .arg(tlm.position[0], 0, 'f', 3)
                                 .arg(tlm.rates[0], 0, 'f', 3)
                                 .arg(tlm.acceleration[0], 0, 'f', 3)
                                 .arg(tlm.position[1], 0, 'f', 3)
                                 .arg(tlm.rates[1], 0, 'f', 3)
                                 .arg(tlm.acceleration[1], 0, 'f', 3));
    }
  }

  void processLowTelemetry() {
    while (udpLow_->hasPendingDatagrams()) {
      QByteArray packet;
      packet.resize(static_cast<int>(udpLow_->pendingDatagramSize()));
      udpLow_->readDatagram(packet.data(), packet.size());
      stc_tlml_t tlm{};
      if (stc_icd_tlml_parse(&cfg_, reinterpret_cast<const uint8_t*>(packet.constData()), static_cast<size_t>(packet.size()), &tlm) != STC_ICD_OK) {
        appendLog("Dropped malformed TLML datagram.");
        continue;
      }
      stateLabel_->setText(QString("State: %1").arg(stateToString(tlm.state)));
      currentState_ = tlm.state;
      lowSeqLabel_->setText(QString("Low-rate seq: %1").arg(tlm.seq));
      envLabel_->setText(QString("Temp %1 C, Hum %2 %%, Press %3 hPa")
                          .arg(tlm.temperature_c, 0, 'f', 1)
                          .arg(tlm.humidity_pct, 0, 'f', 1)
                          .arg(tlm.pressure_hpa, 0, 'f', 1));

      const int bitrateKbps = static_cast<int>(tlm.laser_status[0]) * 10 + static_cast<int>(tlm.laser_status[1]);
      const int connectedDevices = static_cast<int>(tlm.laser_status[2]);
      const int discoveredDevices = static_cast<int>(tlm.laser_status[3]);
      const int scanMaxNode = static_cast<int>(tlm.laser_status[4]);
      const int restartMs = (static_cast<int>(tlm.laser_status[5]) << 8) | static_cast<int>(tlm.laser_status[6]);
      const bool canOk = tlm.laser_status[7] != 0;

      connectionLabel_->setText(QString("CAN %1 | if-speed %2 kbps | restart %3 ms | connected %4 | discovered %5 | scan<=%6")
                                  .arg(canOk ? "UP" : "DOWN")
                                  .arg(bitrateKbps)
                                  .arg(restartMs)
                                  .arg(connectedDevices)
                                  .arg(discoveredDevices)
                                  .arg(scanMaxNode));

      updateDeviceSelectors(connectedDevices, discoveredDevices);
      updateCommandAvailability();

      QStringList errors;
      for (int i = 0; i < static_cast<int>(tlm.error_count); ++i) {
        errors << QString("0x%1").arg(tlm.errors[i], 8, 16, QChar('0'));
      }
      tlmlPanel_->setPlainText(QString("seq=%1\nts=%2\nstate=%3\nuptime=%4 s\nlat=%5 lon=%6 alt=%7 heading=%8\nerrors(%9): %10")
                                 .arg(tlm.seq)
                                 .arg(tlm.timestamp_ms)
                                 .arg(stateToString(tlm.state))
                                 .arg(tlm.uptime_sec)
                                 .arg(tlm.lat, 0, 'f', 6)
                                 .arg(tlm.lon, 0, 'f', 6)
                                 .arg(tlm.alt_m, 0, 'f', 2)
                                 .arg(tlm.heading_deg, 0, 'f', 2)
                                 .arg(tlm.error_count)
                                 .arg(errors.join(", ")));
    }
  }

  void updateDeviceSelectors(int connectedDevices, int discoveredDevices) {
    const int availableSut = qBound(0, connectedDevices > 2 ? (connectedDevices - 2) / 2 : 0, 8);
    if (sutIdBox_) {
      const int prev = sutIdBox_->currentData().toInt();
      sutIdBox_->clear();
      sutIdBox_->addItem("All", 0xFF);
      for (int id = 1; id <= availableSut; ++id) {
        sutIdBox_->addItem(QString::number(id), id);
      }
      int idx = sutIdBox_->findData(prev);
      if (idx < 0) idx = 0;
      sutIdBox_->setCurrentIndex(idx);
      sutIdBox_->setEnabled(availableSut > 0);
    }

    if (turretTypeBox_) {
      const bool turretAvailable = simModeBox_->isChecked() || (connectedDevices >= 2);
      turretTypeBox_->setEnabled(turretAvailable);
      turretAzSpin_->setEnabled(turretAvailable);
      turretElSpin_->setEnabled(turretAvailable);
    }

    if (laserIdSpin_) {
      laserIdSpin_->setMaximum(simModeBox_->isChecked() ? 8 : qMax(1, discoveredDevices));
    }

    if (mirrorIdBox_) {
      const int prev = mirrorIdBox_->currentData().toInt();
      mirrorIdBox_->clear();
      mirrorIdBox_->addItem("All", 0xFF);
      const int maxMirror = simModeBox_->isChecked() ? 8 : discoveredDevices;
      for (int id = 1; id <= maxMirror; ++id) {
        mirrorIdBox_->addItem(QString::number(id), id);
      }
      int idx = mirrorIdBox_->findData(prev);
      if (idx < 0) idx = 0;
      mirrorIdBox_->setCurrentIndex(idx);
      mirrorIdBox_->setEnabled(maxMirror > 0 || simModeBox_->isChecked());
    }

    availabilityLabel_->setText(QString("Selectable devices: turret %1, SUT %2")
                                  .arg(connectedDevices >= 2 ? "enabled" : "disabled")
                                  .arg(availableSut));
    inventoryLabel_->setText(QString("Inventory: discovered=%1, connected=%2, SUT=%3")
                               .arg(discoveredDevices)
                               .arg(connectedDevices)
                               .arg(availableSut));
  }

  void onSimModeChanged(bool enabled) {
    for (int i = 0; i < kAxisCount; ++i) {
      axisTable_->item(i, 4)->setText(enabled ? "SIM" : "HW");
    }
    if (enabled) {
      simTimer_->start();
      if (tcpSocket_->state() != QAbstractSocket::UnconnectedState) tcpSocket_->disconnectFromHost();
      appendLog("Simulation mode ON.");
    } else {
      simTimer_->stop();
      appendLog("Simulation mode OFF.");
      ensureTcpConnected();
    }
    modeLabel_->setText(QString("Server mode: %1").arg(enabled ? "SIM" : "HW"));
    updateCommandAvailability();
  }

  void bringUpCanAndConnect() {
    if (simModeBox_->isChecked()) {
      appendLog("CAN rediscovery skipped in simulation mode.");
      ackLabel_->setText("CAN rediscovery: skipped (simulation mode)");
      return;
    }

    appendLog("CAN rediscovery requested (SELFTEST exchange).");

    QStringList details;
    bool success = true;

    stc_icd_cmd_t cmd{};
    makeHeader(&cmd);
    cmd.cmd_id = STC_CMD_SELFTEST;
    cmd.payload_decoded = 1;
    cmd.payload_size = 0;

    uint8_t packet[20 + STC_ICD_MAX_PAYLOAD]{};
    size_t packetLen = 0;
    const stc_icd_rc_t packRc = stc_icd_cmd_pack(&cfg_, &cmd, packet, sizeof(packet), &packetLen);
    if (packRc != STC_ICD_OK) {
      success = false;
      details << "failed to encode SELFTEST rediscovery command";
    } else {
      tcpSocket_->abort();
      tcpSocket_->connectToHost(serverIpBox_->currentText(), static_cast<quint16>(portSpin_->value()));
      if (!tcpSocket_->waitForConnected(800)) {
        success = false;
        details << QString("TCP connect failed: %1").arg(tcpSocket_->errorString());
      } else {
        const qint64 wrote = tcpSocket_->write(reinterpret_cast<const char*>(packet), static_cast<qint64>(packetLen));
        if (wrote != static_cast<qint64>(packetLen) || !tcpSocket_->waitForBytesWritten(800)) {
          success = false;
          details << QString("rediscover command write failed: %1").arg(tcpSocket_->errorString());
        } else {
          const QByteArray rspMagic(reinterpret_cast<const char*>(cfg_.rsp_magic), 4);
          QByteArray rxBuf;
          int parsedFrames = 0;
          qint64 rawBytes = 0;
          stc_icd_rc_t finalRc = STC_ICD_EPROTO;
          stc_icd_rsp_t finalRsp{};
          bool gotFinalRsp = false;
          QElapsedTimer rspTimer;
          rspTimer.start();
          constexpr int kRediscoverTimeoutMs = 1200;

          while (rspTimer.elapsed() < kRediscoverTimeoutMs && !gotFinalRsp) {
            const int remainingMs = kRediscoverTimeoutMs - static_cast<int>(rspTimer.elapsed());
            if (!tcpSocket_->waitForReadyRead(qMax(1, remainingMs))) {
              continue;
            }

            const QByteArray chunk = tcpSocket_->readAll();
            if (chunk.isEmpty()) {
              continue;
            }
            rawBytes += chunk.size();
            rxBuf.append(chunk);

            while (true) {
              if (rxBuf.size() < kRspHeaderSize) {
                finalRc = STC_ICD_EPROTO;
                break;
              }

              if (std::memcmp(rxBuf.constData(), cfg_.rsp_magic, 4) != 0) {
                const int markerPos = rxBuf.indexOf(rspMagic, 1);
                if (markerPos >= 0) {
                  rxBuf.remove(0, markerPos);
                } else {
                  rxBuf.remove(0, 1);
                }
                finalRc = STC_ICD_EPROTO;
                continue;
              }

              const uint16_t payloadSize = static_cast<uint16_t>(static_cast<uint8_t>(rxBuf[12])) |
                                           (static_cast<uint16_t>(static_cast<uint8_t>(rxBuf[13])) << 8);
              if (payloadSize > STC_ICD_MAX_PAYLOAD) {
                rxBuf.remove(0, 1);
                finalRc = STC_ICD_EBUFS;
                continue;
              }

              const int frameLen = kRspHeaderSize + payloadSize;
              if (rxBuf.size() < frameLen) {
                finalRc = STC_ICD_EPROTO;
                break;
              }

              stc_icd_rsp_t rsp{};
              const stc_icd_rc_t rc = stc_icd_rsp_parse(&cfg_, reinterpret_cast<const uint8_t*>(rxBuf.constData()), static_cast<size_t>(frameLen), &rsp);
              if (rc != STC_ICD_OK) {
                rxBuf.remove(0, 1);
                finalRc = rc;
                continue;
              }

              rxBuf.remove(0, frameLen);
              parsedFrames++;
              finalRc = rc;
              if (rsp.cmd_id == STC_CMD_SELFTEST) {
                finalRsp = rsp;
                gotFinalRsp = true;
                break;
              }
            }
          }

          details << QString("rediscover parse stats: received %1 bytes, parsed %2 frame(s), final rc=%3")
                        .arg(rawBytes)
                        .arg(parsedFrames)
                        .arg(static_cast<int>(finalRc));

          if (!gotFinalRsp) {
            success = false;
            details << "timeout waiting for rediscover ACK";
          } else if (finalRsp.result != STC_RSP_ACK) {
            success = false;
            details << QString("rediscover NACK err=%1 (%2)").arg(finalRsp.err_code).arg(nackErrToString(finalRsp.err_code));
          } else {
            details << QString("rediscover ACK seq=%1").arg(finalRsp.seq);
          }
        }
        tcpSocket_->disconnectFromHost();
      }
    }

    const QString detailsJoined = details.join(" || ");
    if (success) {
      ackLabel_->setText("CAN rediscovery: SUCCESS");
      appendLog(QString("CAN rediscovery success: %1").arg(detailsJoined));
    } else {
      ackLabel_->setText("CAN rediscovery: FAILED");
      appendLog(QString("CAN rediscovery failed: %1").arg(detailsJoined));
    }
  }

  void runCapabilityStartupCheck() {
    if (startupCapabilityCheckDone_) return;
    startupCapabilityCheckDone_ = true;

    auto runCmd = [](const QString &program, const QStringList &args) {
      QProcess proc;
      proc.start(program, args);
      if (!proc.waitForStarted(1000)) {
        return QString("failed to start %1").arg(program);
      }
      if (!proc.waitForFinished(3000)) {
        proc.kill();
        proc.waitForFinished(500);
        return QString("timeout running %1 %2").arg(program, args.join(' '));
      }
      QString out = QString::fromUtf8(proc.readAllStandardOutput()).trimmed();
      const QString err = QString::fromUtf8(proc.readAllStandardError()).trimmed();
      if (!err.isEmpty()) {
        if (!out.isEmpty()) out += " | ";
        out += err;
      }
      return out.isEmpty() ? QString("ok") : out;
    };

    const QString canProbe = runCmd("/sbin/ip", {"link", "show", "can0"});
    appendLog(QString("Startup probe: /sbin/ip link show can0 => %1").arg(canProbe));

    bool hasNetAdmin = false;
#ifdef __linux__
    QFile statusFile("/proc/self/status");
    if (statusFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
      const QString status = QString::fromUtf8(statusFile.readAll());
      const QRegularExpression rx("^CapEff:\\s*([0-9A-Fa-f]+)$", QRegularExpression::MultilineOption);
      const QRegularExpressionMatch match = rx.match(status);
      if (match.hasMatch()) {
        bool ok = false;
        const quint64 capEff = match.captured(1).toULongLong(&ok, 16);
        if (ok) {
          hasNetAdmin = (capEff & (1ULL << CAP_NET_ADMIN)) != 0;
          appendLog(QString("Startup capability probe: CapEff=0x%1").arg(QString::number(capEff, 16)));
        }
      }
    }
#endif

    if (!hasNetAdmin) {
      appendLog("CAP_NET_ADMIN is not present in this process.");
    } else {
      appendLog("CAP_NET_ADMIN is present in this process.");
    }
  }

  void updateMotionUnits(QComboBox *motionTypeBox, QLabel *unitsLabel) {
    if (!motionTypeBox || !unitsLabel) return;
    switch (motionTypeBox->currentData().toInt()) {
      case STC_MOTION_RELATIVE: unitsLabel->setText("units: deg delta"); break;
      case STC_MOTION_ABSOLUTE: unitsLabel->setText("units: deg"); break;
      case STC_MOTION_SPEED: unitsLabel->setText("units: deg/s"); break;
      default: unitsLabel->setText("units: -"); break;
    }
  }

  void updateCommandAvailability() {
    const int idx = commandBox_->currentIndex();
    if (idx < 0) return;
    const UiCommand c = currentUiCommand();
    bool allowed = true;
    const bool isOper = currentState_ == 3;
    if (c == UiCommand::Turret || c == UiCommand::Mirror || c == UiCommand::Sut) {
      allowed = isOper;
    }
    if (c == UiCommand::Config) {
      allowed = false;
    }
    sendButton_->setEnabled(allowed);
  }

  void tickSimulation() {
    const double dt = 0.02;
    for (int i = 0; i < kAxisCount; ++i) {
      const double dv = simTargetVel_[i] - simVel_[i];
      simVel_[i] += dv * 0.25;
      simPos_[i] += simVel_[i] * dt;
      axisTable_->item(i, 1)->setText(QString::number(simPos_[i], 'f', 3));
      axisTable_->item(i, 2)->setText(QString::number(simVel_[i], 'f', 3));
      axisTable_->item(i, 3)->setText(QString::number(simTargetVel_[i], 'f', 3));
    }
  }

  QWidget* buildTopPanel() {
    auto *panel = new QWidget(this);
    auto *layout = new QHBoxLayout(panel);
    layout->setSpacing(10);

    auto *leftColumn = new QWidget(panel);
    auto *leftColumnLayout = new QVBoxLayout(leftColumn);
    leftColumnLayout->setContentsMargins(0, 0, 0, 0);
    leftColumnLayout->setSpacing(10);

    auto *cmdGroup = new QGroupBox("Command Input", leftColumn);
    auto *cmdForm = new QFormLayout(cmdGroup);
    cmdForm->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
    cmdForm->setHorizontalSpacing(12);
    cmdForm->setVerticalSpacing(8);

    constexpr int kInputWidth = 240;
    constexpr int kButtonWidth = 220;
    constexpr int kButtonHeight = 28;

    serverIpBox_ = new QComboBox(cmdGroup);
    serverIpBox_->setEditable(true);
    serverIpBox_->addItem("127.0.0.1");
    serverIpBox_->setMaximumWidth(kInputWidth);

    portSpin_ = new QSpinBox(cmdGroup);
    portSpin_->setRange(1, 65535);
    portSpin_->setValue(kDefaultControlPort);
    portSpin_->setMaximumWidth(kInputWidth);

    controllerIdSpin_ = new QSpinBox(cmdGroup);
    controllerIdSpin_->setRange(0, 255);
    controllerIdSpin_->setValue(11);
    controllerIdSpin_->setMaximumWidth(kInputWidth);

    commandBox_ = new QComboBox(cmdGroup);
    for (const auto &spec : kCommandSpecs) {
      commandBox_->addItem(spec.label, static_cast<int>(spec.ui));
      if (!spec.enabled) {
        commandBox_->setItemData(commandBox_->count() - 1, 0, Qt::UserRole - 1);
      }
    }
    commandBox_->setMaximumWidth(kInputWidth);

    payloadStack_ = new QStackedWidget(cmdGroup);
    payloadStack_->setMaximumWidth(kInputWidth);
    for (int i = 0; i < commandBox_->count(); ++i) {
      payloadStack_->addWidget(buildPayloadPage(static_cast<UiCommand>(commandBox_->itemData(i).toInt()), cmdGroup));
    }

    sendButton_ = new QPushButton("Send", cmdGroup);
    reconnectButton_ = new QPushButton("Reconnect CU link", cmdGroup);
    canBringupButton_ = new QPushButton("Rediscover/connect CAN devices", cmdGroup);
    for (QPushButton *button : {sendButton_, reconnectButton_, canBringupButton_}) {
      button->setMaximumWidth(kButtonWidth);
      button->setMinimumHeight(kButtonHeight);
      button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    }

    tlmHighPortSpin_ = new QSpinBox(cmdGroup);
    tlmHighPortSpin_->setRange(1, 65535);
    tlmHighPortSpin_->setValue(kDefaultTlmHighPort);
    tlmHighPortSpin_->setMaximumWidth(kInputWidth);
    tlmLowPortSpin_ = new QSpinBox(cmdGroup);
    tlmLowPortSpin_->setRange(1, 65535);
    tlmLowPortSpin_->setValue(kDefaultTlmLowPort);
    tlmLowPortSpin_->setMaximumWidth(kInputWidth);

    cmdForm->addRow("Server IP", serverIpBox_);
    cmdForm->addRow("Control Port", portSpin_);
    cmdForm->addRow("TLM high UDP", tlmHighPortSpin_);
    cmdForm->addRow("TLM low UDP", tlmLowPortSpin_);
    cmdForm->addRow("Controller ID", controllerIdSpin_);
    cmdForm->addRow("Command", commandBox_);
    cmdForm->addRow("Payload", payloadStack_);
    cmdForm->addRow(sendButton_);
    cmdForm->addRow(reconnectButton_);
    cmdForm->addRow(canBringupButton_);
    leftColumnLayout->addWidget(cmdGroup);

    auto *statusGroup = new QGroupBox("Status", leftColumn);
    auto *statusLayout = new QGridLayout(statusGroup);
    statusLayout->setHorizontalSpacing(20);
    statusLayout->setVerticalSpacing(8);
    stateLabel_ = new QLabel("State: UNKNOWN", statusGroup);
    modeLabel_ = new QLabel("Server mode: HW", statusGroup);
    inventoryLabel_ = new QLabel("Inventory: discovered -, connected -, SUT -", statusGroup);
    highSeqLabel_ = new QLabel("High-rate seq: -", statusGroup);
    lowSeqLabel_ = new QLabel("Low-rate seq: -", statusGroup);
    envLabel_ = new QLabel("Temp -, Hum -, Press -", statusGroup);
    tcpStatusLabel_ = new QLabel("CU TCP: DISCONNECTED", statusGroup);
    connectionLabel_ = new QLabel("CAN -", statusGroup);
    availabilityLabel_ = new QLabel("Selectable devices: turret -, SUT -", statusGroup);
    ackLabel_ = new QLabel("Last response: -", statusGroup);
    simModeBox_ = new QCheckBox("Simulation mode", statusGroup);

    for (QLabel *label : {stateLabel_, modeLabel_, inventoryLabel_, highSeqLabel_, lowSeqLabel_, envLabel_,
                          tcpStatusLabel_, connectionLabel_, availabilityLabel_, ackLabel_}) {
      label->setWordWrap(true);
    }

    statusLayout->addWidget(stateLabel_, 0, 0);
    statusLayout->addWidget(modeLabel_, 0, 1);
    statusLayout->addWidget(inventoryLabel_, 1, 0);
    statusLayout->addWidget(availabilityLabel_, 1, 1);
    statusLayout->addWidget(highSeqLabel_, 2, 0);
    statusLayout->addWidget(lowSeqLabel_, 2, 1);
    statusLayout->addWidget(envLabel_, 3, 0);
    statusLayout->addWidget(connectionLabel_, 3, 1);
    statusLayout->addWidget(tcpStatusLabel_, 4, 0);
    statusLayout->addWidget(ackLabel_, 4, 1);
    statusLayout->addWidget(simModeBox_, 5, 0, 1, 2);

    leftColumnLayout->addWidget(statusGroup);

    layout->addWidget(leftColumn, 2);
    layout->addWidget(buildAxisPanel(), 3);
    return panel;
  }

  QWidget* buildPayloadPage(UiCommand c, QWidget *parent) {
    auto *w = new QWidget(parent);
    auto *f = new QFormLayout(w);
    switch (c) {
      case UiCommand::CuState:
        cuStateBox_ = new QComboBox(w);
        cuStateBox_->addItem("INIT", 0);
        cuStateBox_->addItem("INIT_DONE", 1);
        cuStateBox_->addItem("SAFE", 2);
        cuStateBox_->addItem("OPER", 3);
        cuStateBox_->addItem("SELF_LEVELLING", 4);
        cuStateBox_->addItem("IBIT_RUNNING", 5);
        cuStateBox_->addItem("STORE", 6);
        cuStateBox_->addItem("FAIL", 0xFF);
        f->addRow("state", cuStateBox_);
        break;
      case UiCommand::SelfLev:
        selfLevActionBox_ = new QComboBox(w);
        selfLevActionBox_->addItem("Stop", 0);
        selfLevActionBox_->addItem("Start", 1);
        f->addRow("action", selfLevActionBox_);
        break;
      case UiCommand::Ibit:
        ibitActionBox_ = new QComboBox(w);
        ibitActionBox_->addItem("Stop", 0);
        ibitActionBox_->addItem("Start", 1);
        f->addRow("action", ibitActionBox_);
        break;
      case UiCommand::Laser:
        laserIdSpin_ = new QSpinBox(w);
        laserIdSpin_->setRange(1, 8);
        laserActionBox_ = new QComboBox(w);
        laserActionBox_->addItem("Off", 0);
        laserActionBox_->addItem("On", 1);
        f->addRow("id", laserIdSpin_);
        f->addRow("action", laserActionBox_);
        break;
      case UiCommand::Turret:
        turretTypeBox_ = new QComboBox(w);
        turretTypeBox_->addItem("Relative", 0);
        turretTypeBox_->addItem("Absolute", 1);
        turretTypeBox_->addItem("Speed", 2);
        turretAzSpin_ = new QDoubleSpinBox(w);
        turretAzSpin_->setRange(-720.0, 720.0);
        turretAzSpin_->setDecimals(3);
        turretElSpin_ = new QDoubleSpinBox(w);
        turretElSpin_->setRange(-720.0, 720.0);
        turretElSpin_->setDecimals(3);
        turretUnitsLabel_ = new QLabel("units: deg delta", w);
        connect(turretTypeBox_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
          updateMotionUnits(turretTypeBox_, turretUnitsLabel_);
        });
        f->addRow("type", turretTypeBox_);
        f->addRow("azimuth", turretAzSpin_);
        f->addRow("elevation", turretElSpin_);
        f->addRow("", turretUnitsLabel_);
        updateMotionUnits(turretTypeBox_, turretUnitsLabel_);
        break;
      case UiCommand::Mirror:
        mirrorIdBox_ = new QComboBox(w);
        mirrorIdBox_->addItem("All", 0xFF);
        for (int id = 1; id <= 8; ++id) mirrorIdBox_->addItem(QString::number(id), id);
        mirrorTypeBox_ = new QComboBox(w);
        mirrorTypeBox_->addItem("Relative", 0);
        mirrorTypeBox_->addItem("Absolute", 1);
        mirrorTypeBox_->addItem("Speed", 2);
        mirrorElevationSpin_ = new QDoubleSpinBox(w);
        mirrorElevationSpin_->setRange(-720.0, 720.0);
        mirrorElevationSpin_->setDecimals(3);
        mirrorUnitsLabel_ = new QLabel("units: deg delta", w);
        connect(mirrorTypeBox_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
          updateMotionUnits(mirrorTypeBox_, mirrorUnitsLabel_);
        });
        f->addRow("mirror id", mirrorIdBox_);
        f->addRow("motion type", mirrorTypeBox_);
        f->addRow("elevation", mirrorElevationSpin_);
        f->addRow("", mirrorUnitsLabel_);
        updateMotionUnits(mirrorTypeBox_, mirrorUnitsLabel_);
        break;
      case UiCommand::Sut:
        sutIdBox_ = new QComboBox(w);
        sutIdBox_->addItem("All", 0xFF);
        for (int id = 1; id <= 8; ++id) sutIdBox_->addItem(QString::number(id), id);
        sutTypeBox_ = new QComboBox(w);
        sutTypeBox_->addItem("Relative", 0);
        sutTypeBox_->addItem("Absolute", 1);
        sutTypeBox_->addItem("Speed", 2);
        sutAzSpin_ = new QDoubleSpinBox(w);
        sutAzSpin_->setRange(-720.0, 720.0);
        sutAzSpin_->setDecimals(3);
        sutElSpin_ = new QDoubleSpinBox(w);
        sutElSpin_->setRange(-720.0, 720.0);
        sutElSpin_->setDecimals(3);
        f->addRow("sut id", sutIdBox_);
        f->addRow("motion type", sutTypeBox_);
        f->addRow("SUT azimuth", sutAzSpin_);
        f->addRow("mirror elevation", sutElSpin_);
        break;
      case UiCommand::ManualPos:
        manualLatSpin_ = new QDoubleSpinBox(w);
        manualLatSpin_->setRange(-90.0, 90.0);
        manualLatSpin_->setDecimals(8);
        manualLonSpin_ = new QDoubleSpinBox(w);
        manualLonSpin_->setRange(-180.0, 180.0);
        manualLonSpin_->setDecimals(8);
        manualAltSpin_ = new QDoubleSpinBox(w);
        manualAltSpin_->setRange(-1000.0, 20000.0);
        manualHeadingSpin_ = new QDoubleSpinBox(w);
        manualHeadingSpin_->setRange(-360.0, 360.0);
        f->addRow("lat", manualLatSpin_);
        f->addRow("lon", manualLonSpin_);
        f->addRow("alt", manualAltSpin_);
        f->addRow("heading", manualHeadingSpin_);
        break;
      case UiCommand::PosMode:
        posModeBox_ = new QComboBox(w);
        posModeBox_->addItem("GPS", 0);
        posModeBox_->addItem("Manual", 1);
        f->addRow("mode", posModeBox_);
        break;
      default:
        f->addRow(new QLabel("No payload", w));
        break;
    }
    return w;
  }

  QWidget* buildAxisPanel() {
    auto *group = new QGroupBox("Telemetry", this);
    auto *layout = new QVBoxLayout(group);

    auto *split = new QVBoxLayout();
    auto *highGroup = new QGroupBox("TLMH (high-rate)", group);
    auto *highLayout = new QVBoxLayout(highGroup);
    tlmhPanel_ = new QPlainTextEdit(highGroup);
    tlmhPanel_->setReadOnly(true);
    tlmhPanel_->setMinimumHeight(100);
    highLayout->addWidget(tlmhPanel_);

    auto *lowGroup = new QGroupBox("TLML (low-rate)", group);
    auto *lowLayout = new QVBoxLayout(lowGroup);
    tlmlPanel_ = new QPlainTextEdit(lowGroup);
    tlmlPanel_->setReadOnly(true);
    tlmlPanel_->setMinimumHeight(100);
    lowLayout->addWidget(tlmlPanel_);

    split->addWidget(highGroup);
    split->addWidget(lowGroup);
    layout->addLayout(split);

    axisTable_ = new QTableWidget(group);
    axisTable_->setColumnCount(5);
    axisTable_->setHorizontalHeaderLabels({"Axis", "Position", "Velocity", "Target", "Type"});
    axisTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    layout->addWidget(axisTable_);
    return group;
  }

  QWidget* buildLogPanel() {
    auto *group = new QGroupBox("Log", this);
    auto *layout = new QVBoxLayout(group);
    logEdit_ = new QPlainTextEdit(group);
    logEdit_->setReadOnly(true);
    layout->addWidget(logEdit_);
    return group;
  }

  void initAxisTable() {
    axisTable_->setRowCount(kAxisCount);
    for (int i = 0; i < kAxisCount; ++i) {
      axisTable_->setItem(i, 0, new QTableWidgetItem(QString("Axis %1").arg(i)));
      axisTable_->setItem(i, 1, new QTableWidgetItem("0.000"));
      axisTable_->setItem(i, 2, new QTableWidgetItem("0.000"));
      axisTable_->setItem(i, 3, new QTableWidgetItem("0.000"));
      axisTable_->setItem(i, 4, new QTableWidgetItem("HW"));
    }
    simPos_.fill(0.0);
    simVel_.fill(0.0);
    simTargetVel_.fill(0.0);
    tlmhPanel_->setPlainText("Waiting for TLMH...");
    tlmlPanel_->setPlainText("Waiting for TLML...");
  }

  void appendLog(const QString &line) {
    logEdit_->appendPlainText(QString("[%1] %2")
                                .arg(QTime::currentTime().toString("HH:mm:ss.zzz"))
                                .arg(line));
  }

  QTcpSocket *tcpSocket_{};
  QUdpSocket *udpHigh_{};
  QUdpSocket *udpLow_{};
  QTimer *simTimer_{};
  QTimer *connectTimer_{};

  QComboBox *serverIpBox_{};
  QSpinBox *portSpin_{};
  QSpinBox *controllerIdSpin_{};
  QComboBox *commandBox_{};
  QStackedWidget *payloadStack_{};
  QPushButton *sendButton_{};
  QPushButton *reconnectButton_{};
  QPushButton *canBringupButton_{};

  QComboBox *cuStateBox_{};
  QComboBox *selfLevActionBox_{};
  QComboBox *ibitActionBox_{};
  QSpinBox *laserIdSpin_{};
  QComboBox *laserActionBox_{};
  QComboBox *turretTypeBox_{};
  QDoubleSpinBox *turretAzSpin_{};
  QDoubleSpinBox *turretElSpin_{};
  QLabel *turretUnitsLabel_{};
  QComboBox *mirrorIdBox_{};
  QComboBox *mirrorTypeBox_{};
  QDoubleSpinBox *mirrorElevationSpin_{};
  QLabel *mirrorUnitsLabel_{};
  QComboBox *sutIdBox_{};
  QComboBox *sutTypeBox_{};
  QDoubleSpinBox *sutAzSpin_{};
  QDoubleSpinBox *sutElSpin_{};
  QDoubleSpinBox *manualLatSpin_{};
  QDoubleSpinBox *manualLonSpin_{};
  QDoubleSpinBox *manualAltSpin_{};
  QDoubleSpinBox *manualHeadingSpin_{};
  QComboBox *posModeBox_{};

  QLabel *stateLabel_{};
  QLabel *modeLabel_{};
  QLabel *inventoryLabel_{};
  QLabel *highSeqLabel_{};
  QLabel *lowSeqLabel_{};
  QLabel *envLabel_{};
  QLabel *tcpStatusLabel_{};
  QLabel *connectionLabel_{};
  QLabel *availabilityLabel_{};
  QLabel *ackLabel_{};
  QCheckBox *simModeBox_{};

  QPlainTextEdit *tlmhPanel_{};
  QPlainTextEdit *tlmlPanel_{};
  QTableWidget *axisTable_{};
  QPlainTextEdit *logEdit_{};

  stc_icd_cfg_t cfg_{};
  uint16_t nextSeq_ = 1;
  uint8_t currentState_ = 0;
  bool tcpConnected_ = false;
  bool startupCapabilityCheckDone_ = false;
  QByteArray rspBuffer_;
  QHash<uint16_t, QString> pendingCommands_;

  QSpinBox *tlmHighPortSpin_{};
  QSpinBox *tlmLowPortSpin_{};

  std::array<double, kAxisCount> simPos_{};
  std::array<double, kAxisCount> simVel_{};
  std::array<double, kAxisCount> simTargetVel_{};
};

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  DiagnosticsWindow w;
  w.show();
  return app.exec();
}
