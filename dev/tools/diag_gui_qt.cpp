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
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QUdpSocket>
#include <QtCore/QDateTime>
#include <QtCore/QTime>
#include <QtCore/QTimer>

#include <array>
#include <cstdint>
#include <cstring>

extern "C" {
#include "stc_icd.h"
}

namespace {
constexpr uint16_t kDefaultControlPort = 7001;
constexpr uint16_t kDefaultTlmHighPort = 7002;
constexpr uint16_t kDefaultTlmLowPort = 7003;
constexpr int kAxisCount = static_cast<int>(STC_ICD_AXES_COUNT);

QString stateToString(uint8_t state) {
  switch (state) {
    case 0: return "OFF";
    case 1: return "SAFE";
    case 2: return "OPER";
    case 3: return "FAULT";
    default: return QString("UNKNOWN(%1)").arg(state);
  }
}

enum class UiCommand {
  CuState,
  SelfLev,
  Ibit,
  Storage,
  Laser,
  Turret,
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
    root->addWidget(buildTopPanel());
    root->addWidget(buildAxisPanel(), 1);
    root->addWidget(buildLogPanel(), 1);
    setCentralWidget(central);

    initAxisTable();

    connect(sendButton_, &QPushButton::clicked, this, &DiagnosticsWindow::sendCommand);
    connect(commandBox_, qOverload<int>(&QComboBox::currentIndexChanged), this, &DiagnosticsWindow::onCommandSelectionChanged);
    connect(udpHigh_, &QUdpSocket::readyRead, this, &DiagnosticsWindow::processHighTelemetry);
    connect(udpLow_, &QUdpSocket::readyRead, this, &DiagnosticsWindow::processLowTelemetry);
    connect(simModeBox_, &QCheckBox::toggled, this, &DiagnosticsWindow::onSimModeChanged);

    if (!udpHigh_->bind(QHostAddress::AnyIPv4, kDefaultTlmHighPort, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
      appendLog(QString("Failed binding high telemetry socket: %1").arg(udpHigh_->errorString()));
    }
    if (!udpLow_->bind(QHostAddress::AnyIPv4, kDefaultTlmLowPort, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
      appendLog(QString("Failed binding low telemetry socket: %1").arg(udpLow_->errorString()));
    }

    simTimer_ = new QTimer(this);
    simTimer_->setInterval(20);
    connect(simTimer_, &QTimer::timeout, this, &DiagnosticsWindow::tickSimulation);

    onCommandSelectionChanged(commandBox_->currentIndex());
    appendLog("Diagnostics GUI initialized.");
  }

 private:
  void onCommandSelectionChanged(int index) {
    payloadStack_->setCurrentIndex(index);
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

    uint8_t packet[20 + STC_ICD_MAX_PAYLOAD]{};
    size_t packetLen = 0;
    if (stc_icd_cmd_pack(&cfg_, &cmd, packet, sizeof(packet), &packetLen) != STC_ICD_OK) {
      QMessageBox::warning(this, "Serialize error", "Failed to encode ICD command.");
      return;
    }

    tcpSocket_->abort();
    tcpSocket_->connectToHost(serverIpBox_->currentText(), static_cast<quint16>(portSpin_->value()));
    if (!tcpSocket_->waitForConnected(500)) {
      appendLog(QString("TCP connect failed: %1").arg(tcpSocket_->errorString()));
      return;
    }

    const qint64 wrote = tcpSocket_->write(reinterpret_cast<const char*>(packet), static_cast<qint64>(packetLen));
    if (wrote != static_cast<qint64>(packetLen) || !tcpSocket_->waitForBytesWritten(500)) {
      appendLog(QString("TCP write failed: %1").arg(tcpSocket_->errorString()));
    } else {
      appendLog(QString("Sent command id=0x%1 seq=%2 bytes=%3")
                  .arg(cmd.cmd_id, 4, 16, QChar('0'))
                  .arg(cmd.seq)
                  .arg(packetLen));
    }
    tcpSocket_->disconnectFromHost();
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
      turretTypeBox_->setEnabled(connectedDevices >= 2);
      turretAzSpin_->setEnabled(connectedDevices >= 2);
      turretElSpin_->setEnabled(connectedDevices >= 2);
    }

    availabilityLabel_->setText(QString("Selectable devices: turret %1, SUT %2")
                                  .arg(connectedDevices >= 2 ? "enabled" : "disabled")
                                  .arg(availableSut));
    Q_UNUSED(discoveredDevices);
  }

  void onSimModeChanged(bool enabled) {
    for (int i = 0; i < kAxisCount; ++i) {
      axisTable_->item(i, 4)->setText(enabled ? "SIM" : "HW");
    }
    if (enabled) {
      simTimer_->start();
      appendLog("Simulation mode ON.");
    } else {
      simTimer_->stop();
      appendLog("Simulation mode OFF.");
    }
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

    auto *cmdGroup = new QGroupBox("Command Input", panel);
    auto *cmdForm = new QFormLayout(cmdGroup);

    serverIpBox_ = new QComboBox(cmdGroup);
    serverIpBox_->setEditable(true);
    serverIpBox_->addItem("127.0.0.1");

    portSpin_ = new QSpinBox(cmdGroup);
    portSpin_->setRange(1, 65535);
    portSpin_->setValue(kDefaultControlPort);

    controllerIdSpin_ = new QSpinBox(cmdGroup);
    controllerIdSpin_->setRange(0, 255);
    controllerIdSpin_->setValue(11);

    commandBox_ = new QComboBox(cmdGroup);
    commandBox_->addItem("CU State", static_cast<int>(UiCommand::CuState));
    commandBox_->addItem("SelfLev", static_cast<int>(UiCommand::SelfLev));
    commandBox_->addItem("IBIT", static_cast<int>(UiCommand::Ibit));
    commandBox_->addItem("Storage", static_cast<int>(UiCommand::Storage));
    commandBox_->addItem("Laser", static_cast<int>(UiCommand::Laser));
    commandBox_->addItem("Turret", static_cast<int>(UiCommand::Turret));
    commandBox_->addItem("SUT", static_cast<int>(UiCommand::Sut));
    commandBox_->addItem("Reset", static_cast<int>(UiCommand::Reset));
    commandBox_->addItem("DelLog", static_cast<int>(UiCommand::DelLog));
    commandBox_->addItem("Factory", static_cast<int>(UiCommand::Factory));
    commandBox_->addItem("SelfTest", static_cast<int>(UiCommand::SelfTest));
    commandBox_->addItem("Shutdown", static_cast<int>(UiCommand::Shutdown));
    commandBox_->addItem("Config", static_cast<int>(UiCommand::Config));
    commandBox_->addItem("ManualPos", static_cast<int>(UiCommand::ManualPos));
    commandBox_->addItem("PosMode", static_cast<int>(UiCommand::PosMode));
    commandBox_->addItem("ReqSwVersion", static_cast<int>(UiCommand::ReqSwVersion));
    commandBox_->addItem("ReqHwInfo", static_cast<int>(UiCommand::ReqHwInfo));
    commandBox_->addItem("ReqWorkTime", static_cast<int>(UiCommand::ReqWorkTime));

    payloadStack_ = new QStackedWidget(cmdGroup);
    for (int i = 0; i < commandBox_->count(); ++i) {
      payloadStack_->addWidget(buildPayloadPage(static_cast<UiCommand>(commandBox_->itemData(i).toInt()), cmdGroup));
    }

    sendButton_ = new QPushButton("Send", cmdGroup);

    cmdForm->addRow("Server IP", serverIpBox_);
    cmdForm->addRow("Control Port", portSpin_);
    cmdForm->addRow("Controller ID", controllerIdSpin_);
    cmdForm->addRow("Command", commandBox_);
    cmdForm->addRow("Payload", payloadStack_);
    cmdForm->addRow(sendButton_);

    auto *statusGroup = new QGroupBox("Status", panel);
    auto *statusLayout = new QVBoxLayout(statusGroup);
    stateLabel_ = new QLabel("State: UNKNOWN", statusGroup);
    highSeqLabel_ = new QLabel("High-rate seq: -", statusGroup);
    lowSeqLabel_ = new QLabel("Low-rate seq: -", statusGroup);
    envLabel_ = new QLabel("Temp -, Hum -, Press -", statusGroup);
    connectionLabel_ = new QLabel("CAN -", statusGroup);
    availabilityLabel_ = new QLabel("Selectable devices: turret -, SUT -", statusGroup);
    simModeBox_ = new QCheckBox("Simulation mode", statusGroup);
    statusLayout->addWidget(stateLabel_);
    statusLayout->addWidget(highSeqLabel_);
    statusLayout->addWidget(lowSeqLabel_);
    statusLayout->addWidget(envLabel_);
    statusLayout->addWidget(connectionLabel_);
    statusLayout->addWidget(availabilityLabel_);
    statusLayout->addWidget(simModeBox_);

    layout->addWidget(cmdGroup, 3);
    layout->addWidget(statusGroup, 1);
    return panel;
  }

  QWidget* buildPayloadPage(UiCommand c, QWidget *parent) {
    auto *w = new QWidget(parent);
    auto *f = new QFormLayout(w);
    switch (c) {
      case UiCommand::CuState:
        cuStateBox_ = new QComboBox(w);
        cuStateBox_->addItem("OFF", 0);
        cuStateBox_->addItem("SAFE", 1);
        cuStateBox_->addItem("OPER", 2);
        f->addRow("state", cuStateBox_);
        break;
      case UiCommand::SelfLev:
        selfLevActionBox_ = new QComboBox(w);
        selfLevActionBox_->addItem("Start", 1);
        selfLevActionBox_->addItem("Stop", 0);
        f->addRow("action", selfLevActionBox_);
        break;
      case UiCommand::Ibit:
        ibitActionBox_ = new QComboBox(w);
        ibitActionBox_->addItem("Start", 1);
        ibitActionBox_->addItem("Stop", 0);
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
        turretTypeBox_->addItem("Rate", 2);
        turretAzSpin_ = new QDoubleSpinBox(w);
        turretAzSpin_->setRange(-720.0, 720.0);
        turretAzSpin_->setDecimals(3);
        turretElSpin_ = new QDoubleSpinBox(w);
        turretElSpin_->setRange(-720.0, 720.0);
        turretElSpin_->setDecimals(3);
        f->addRow("type", turretTypeBox_);
        f->addRow("azimuth", turretAzSpin_);
        f->addRow("elevation", turretElSpin_);
        break;
      case UiCommand::Sut:
        sutIdBox_ = new QComboBox(w);
        sutIdBox_->addItem("All", 0xFF);
        for (int id = 1; id <= 8; ++id) sutIdBox_->addItem(QString::number(id), id);
        sutTypeBox_ = new QComboBox(w);
        sutTypeBox_->addItem("Relative", 0);
        sutTypeBox_->addItem("Absolute", 1);
        sutTypeBox_->addItem("Rate", 2);
        sutAzSpin_ = new QDoubleSpinBox(w);
        sutAzSpin_->setRange(-720.0, 720.0);
        sutAzSpin_->setDecimals(3);
        sutElSpin_ = new QDoubleSpinBox(w);
        sutElSpin_->setRange(-720.0, 720.0);
        sutElSpin_->setDecimals(3);
        f->addRow("id", sutIdBox_);
        f->addRow("type", sutTypeBox_);
        f->addRow("azimuth", sutAzSpin_);
        f->addRow("elevation", sutElSpin_);
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
        posModeBox_->addItem("0", 0);
        posModeBox_->addItem("1", 1);
        f->addRow("mode", posModeBox_);
        break;
      default:
        f->addRow(new QLabel("No payload", w));
        break;
    }
    return w;
  }

  QWidget* buildAxisPanel() {
    auto *group = new QGroupBox("Axes", this);
    auto *layout = new QVBoxLayout(group);
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

  QComboBox *serverIpBox_{};
  QSpinBox *portSpin_{};
  QSpinBox *controllerIdSpin_{};
  QComboBox *commandBox_{};
  QStackedWidget *payloadStack_{};
  QPushButton *sendButton_{};

  QComboBox *cuStateBox_{};
  QComboBox *selfLevActionBox_{};
  QComboBox *ibitActionBox_{};
  QSpinBox *laserIdSpin_{};
  QComboBox *laserActionBox_{};
  QComboBox *turretTypeBox_{};
  QDoubleSpinBox *turretAzSpin_{};
  QDoubleSpinBox *turretElSpin_{};
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
  QLabel *highSeqLabel_{};
  QLabel *lowSeqLabel_{};
  QLabel *envLabel_{};
  QLabel *connectionLabel_{};
  QLabel *availabilityLabel_{};
  QCheckBox *simModeBox_{};

  QTableWidget *axisTable_{};
  QPlainTextEdit *logEdit_{};

  stc_icd_cfg_t cfg_{};
  uint16_t nextSeq_ = 1;

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
