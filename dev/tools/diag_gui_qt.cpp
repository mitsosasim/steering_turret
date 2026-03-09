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
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QUdpSocket>
#include <QtCore/QDateTime>
#include <QtCore/QTime>
#include <QtCore/QRandomGenerator>
#include <QtCore/QTimer>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

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
}

class DiagnosticsWindow : public QMainWindow {

 public:
  DiagnosticsWindow() {
    setWindowTitle("STC Diagnostics Console");
    resize(1200, 800);

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
    simTimer_->setInterval(10);
    connect(simTimer_, &QTimer::timeout, this, &DiagnosticsWindow::tickSimulation);

    appendLog("Diagnostics GUI initialized.");
  }

 private:
  void sendCommand() {
    stc_icd_cmd_t cmd{};
    std::memcpy(cmd.magic, cfg_.cmd_magic, sizeof(cmd.magic));
    cmd.version = cfg_.protocol_version_default;
    cmd.controller_id = static_cast<uint8_t>(controllerIdSpin_->value());
    cmd.seq = nextSeq_++;
    cmd.timestamp_ms = static_cast<uint64_t>(QDateTime::currentMSecsSinceEpoch());

    const int cmdIdx = commandBox_->currentIndex();
    if (cmdIdx == 0 || cmdIdx == 1 || cmdIdx == 2) {
      cmd.cmd_id = STC_CMD_CU_STATE;
      cmd.payload_decoded = 1;
      cmd.u.cu_state.state = static_cast<uint8_t>(cmdIdx);  // OFF/SAFE/OPER
      cmd.payload_size = 1;
    } else if (cmdIdx == 3 || cmdIdx == 4) {
      cmd.cmd_id = STC_CMD_SELFLEV;
      cmd.payload_decoded = 1;
      cmd.u.selflev.v = static_cast<uint8_t>(cmdIdx == 3 ? 1 : 0);
      cmd.payload_size = 1;
    } else if (cmdIdx == 5 || cmdIdx == 6) {
      cmd.cmd_id = STC_CMD_IBIT;
      cmd.payload_decoded = 1;
      cmd.u.ibit.v = static_cast<uint8_t>(cmdIdx == 5 ? 1 : 0);
      cmd.payload_size = 1;
    } else {
      cmd.cmd_id = STC_CMD_TURRET;
      cmd.payload_decoded = 1;
      cmd.u.turret.motion_type = STC_MOTION_SPEED;
      const float speed = static_cast<float>(speedSpin_->value());
      cmd.u.turret.az = directionCwBox_->isChecked() ? speed : -speed;
      cmd.u.turret.el = static_cast<float>(elSpin_->value());
      cmd.payload_size = 9;
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
      envLabel_->setText(QString("Temp %1 C, Hum %2 %, Press %3 hPa")
                          .arg(tlm.temperature_c, 0, 'f', 1)
                          .arg(tlm.humidity_pct, 0, 'f', 1)
                          .arg(tlm.pressure_hpa, 0, 'f', 1));
      if (tlm.error_count > 0) {
        appendLog(QString("TLML errors count=%1 first=0x%2")
                    .arg(tlm.error_count)
                    .arg(tlm.errors[0], 8, 16, QChar('0')));
      }
    }
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
    simTimeSec_ += 0.01;
    for (int i = 0; i < kAxisCount; ++i) {
      const double phase = simTimeSec_ + (0.15 * static_cast<double>(i));
      const double pos = 10.0 * std::sin(phase);
      const double vel = 10.0 * std::cos(phase);
      const double tgt = speedSpin_->value();
      axisTable_->item(i, 1)->setText(QString::number(pos, 'f', 3));
      axisTable_->item(i, 2)->setText(QString::number(vel, 'f', 3));
      axisTable_->item(i, 3)->setText(QString::number(tgt, 'f', 3));
    }
  }

 private:
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
    commandBox_->addItems({"CU: OFF", "CU: SAFE", "CU: OPER", "SELFLEV Start", "SELFLEV Stop",
                           "IBIT Start", "IBIT Stop", "Move Turret"});

    speedSpin_ = new QDoubleSpinBox(cmdGroup);
    speedSpin_->setRange(0.0, 1000.0);
    speedSpin_->setDecimals(2);
    speedSpin_->setValue(10.0);

    elSpin_ = new QDoubleSpinBox(cmdGroup);
    elSpin_->setRange(-90.0, 90.0);
    elSpin_->setDecimals(2);

    directionCwBox_ = new QCheckBox("CW direction", cmdGroup);
    directionCwBox_->setChecked(true);

    sendButton_ = new QPushButton("Send", cmdGroup);

    cmdForm->addRow("Server IP", serverIpBox_);
    cmdForm->addRow("Control Port", portSpin_);
    cmdForm->addRow("Controller ID", controllerIdSpin_);
    cmdForm->addRow("Command", commandBox_);
    cmdForm->addRow("Speed", speedSpin_);
    cmdForm->addRow("Elevation", elSpin_);
    cmdForm->addRow("Direction", directionCwBox_);
    cmdForm->addRow(sendButton_);

    auto *statusGroup = new QGroupBox("Status", panel);
    auto *statusLayout = new QVBoxLayout(statusGroup);
    stateLabel_ = new QLabel("State: UNKNOWN", statusGroup);
    highSeqLabel_ = new QLabel("High-rate seq: -", statusGroup);
    lowSeqLabel_ = new QLabel("Low-rate seq: -", statusGroup);
    envLabel_ = new QLabel("Temp -, Hum -, Press -", statusGroup);
    simModeBox_ = new QCheckBox("Simulation mode", statusGroup);
    statusLayout->addWidget(stateLabel_);
    statusLayout->addWidget(highSeqLabel_);
    statusLayout->addWidget(lowSeqLabel_);
    statusLayout->addWidget(envLabel_);
    statusLayout->addWidget(simModeBox_);

    layout->addWidget(cmdGroup, 2);
    layout->addWidget(statusGroup, 1);
    return panel;
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
  QDoubleSpinBox *speedSpin_{};
  QDoubleSpinBox *elSpin_{};
  QCheckBox *directionCwBox_{};
  QPushButton *sendButton_{};

  QLabel *stateLabel_{};
  QLabel *highSeqLabel_{};
  QLabel *lowSeqLabel_{};
  QLabel *envLabel_{};
  QCheckBox *simModeBox_{};

  QTableWidget *axisTable_{};
  QPlainTextEdit *logEdit_{};

  stc_icd_cfg_t cfg_{};
  uint16_t nextSeq_ = 1;
  double simTimeSec_ = 0.0;
};

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  DiagnosticsWindow w;
  w.show();
  return app.exec();
}
