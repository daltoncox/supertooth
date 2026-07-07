#include "device_list_model.h"

#include <QVariantMap>
#include <QtMath>

DeviceListModel::DeviceListModel(QObject *parent)
    : QAbstractListModel(parent)
{
    // Seed a handful of fake rows so the framing can be exercised
    // without a live capture. Mix: one un-broken-out BR/EDR piconet,
    // two broken-out BR/EDR devices (same access address, clock known),
    // and three BLE devices with distinct access addresses.
    m_rows.reserve(6);

    m_rows.append(makeSeedRow(
        QStringLiteral("-48 dBm"), QStringLiteral("BR/EDR"),
        QStringLiteral("0xA1B2C3"), QStringLiteral("piconet"),
        QStringLiteral("12:04:17.221"), 1284, QStringLiteral("210/s"),
        -48.0, 3.5,
        { { QStringLiteral("UAP"),         QStringLiteral("0x9F") },
          { QStringLiteral("Clock known"), QStringLiteral("false") },
          { QStringLiteral("Channels"),    QStringLiteral("1..79") } }));

    m_rows.append(makeSeedRow(
        QStringLiteral("-42 dBm"), QStringLiteral("BR/EDR"),
        QStringLiteral("0xA1B2C3"), QStringLiteral("Master"),
        QStringLiteral("12:04:18.103"), 642, QStringLiteral("180/s"),
        -42.0, 2.0,
        { { QStringLiteral("UAP"),         QStringLiteral("0x9F") },
          { QStringLiteral("Clock known"), QStringLiteral("true") },
          { QStringLiteral("Clock"),       QStringLiteral("0x23C1") },
          { QStringLiteral("Role"),        QStringLiteral("Master") },
          { QStringLiteral("Channels"),    QStringLiteral("1..79") } }));

    m_rows.append(makeSeedRow(
        QStringLiteral("-61 dBm"), QStringLiteral("BR/EDR"),
        QStringLiteral("0xA1B2C3"), QStringLiteral("Slave A8"),
        QStringLiteral("12:04:18.090"), 518, QStringLiteral("158/s"),
        -61.0, 4.0,
        { { QStringLiteral("UAP"),         QStringLiteral("0x9F") },
          { QStringLiteral("Clock known"), QStringLiteral("true") },
          { QStringLiteral("Clock"),       QStringLiteral("0x2A4D") },
          { QStringLiteral("Role"),         QStringLiteral("Slave") },
          { QStringLiteral("LT_ADDR"),      QStringLiteral("3") },
          { QStringLiteral("Channels"),    QStringLiteral("1..79") } }));

    m_rows.append(makeSeedRow(
        QStringLiteral("-77 dBm"), QStringLiteral("BLE"),
        QStringLiteral("0xAA:BB:CC:11:22:33"),
        QStringLiteral("SensorTag-TI"),
        QStringLiteral("12:04:18.401"), 312, QStringLiteral("94/s"),
        -77.0, 6.0,
        { { QStringLiteral("Address type"), QStringLiteral("Public") },
          { QStringLiteral("Adv channels"),  QStringLiteral("37, 38, 39") },
          { QStringLiteral("Connectable"),    QStringLiteral("true") },
          { QStringLiteral("Last PDU type"),  QStringLiteral("ADV_IND") } }));

    m_rows.append(makeSeedRow(
        QStringLiteral("-68 dBm"), QStringLiteral("BLE"),
        QStringLiteral("0x8A4F2C0D5E"),
        QStringLiteral("Apple-TV-remote"),
        QStringLiteral("12:04:18.452"), 207, QStringLiteral("64/s"),
        -68.0, 5.0,
        { { QStringLiteral("Address type"), QStringLiteral("Random") },
          { QStringLiteral("Adv channels"),  QStringLiteral("37, 39") },
          { QStringLiteral("Connectable"),    QStringLiteral("true") },
          { QStringLiteral("Last PDU type"),  QStringLiteral("ADV_NONCONN_IND") } }));

    m_rows.append(makeSeedRow(
        QStringLiteral("-83 dBm"), QStringLiteral("BLE"),
        QStringLiteral("0x29D17B6F22"),
        QStringLiteral("Unknown-BLE"),
        QStringLiteral("12:04:18.510"), 19, QStringLiteral("6/s"),
        -83.0, 8.0,
        { { QStringLiteral("Address type"), QStringLiteral("Random") },
          { QStringLiteral("Adv channels"),  QStringLiteral("38") },
          { QStringLiteral("Connectable"),    QStringLiteral("false") },
          { QStringLiteral("Last PDU type"),  QStringLiteral("ADV_SCAN_IND") } }));

    emit countChanged();
}

int DeviceListModel::rowCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return 0;
    return m_rows.size();
}

QHash<int, QByteArray> DeviceListModel::roleNames() const
{
    return {
        {RssiRole,       "rssi"},
        {ProtoRole,      "proto"},
        {AddrRole,       "addr"},
        {DeviceRole,     "device"},
        {LastSeenRole,   "lastSeen"},
        {PacketsSeenRole,"packetsSeen"},
        {PacketRateRole, "packetRate"},
    };
}

QVariant DeviceListModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() < 0 || index.row() >= m_rows.size())
        return {};
    const Row &r = m_rows.at(index.row());
    switch (role)
    {
    case RssiRole:        return r.rssi;
    case ProtoRole:       return r.proto;
    case AddrRole:        return r.addr;
    case DeviceRole:      return r.device;
    case LastSeenRole:    return r.lastSeen;
    case PacketsSeenRole: return QVariant::fromValue(r.packetsSeen);
    case PacketRateRole:  return r.packetRate;
    }
    return {};
}

void DeviceListModel::appendRow(const QVariantMap &row)
{
    Row r;
    r.rssi        = row.value(QStringLiteral("rssi")).toString();
    r.proto       = row.value(QStringLiteral("proto")).toString();
    r.addr        = row.value(QStringLiteral("addr")).toString();
    r.device      = row.value(QStringLiteral("device")).toString();
    r.lastSeen    = row.value(QStringLiteral("lastSeen")).toString();
    r.packetsSeen = row.value(QStringLiteral("packetsSeen")).toULongLong();
    r.packetRate  = row.value(QStringLiteral("packetRate")).toString();
    r.detail      = row.value(QStringLiteral("detail")).toList();
    r.rssiSeries  = row.value(QStringLiteral("rssiSeries")).toList();

    beginInsertRows(QModelIndex(), m_rows.size(), m_rows.size());
    m_rows.append(std::move(r));
    endInsertRows();
    emit countChanged();
}

void DeviceListModel::clear()
{
    if (m_rows.isEmpty())
        return;
    beginResetModel();
    m_rows.clear();
    endResetModel();
    emit countChanged();
}

QVariantList DeviceListModel::detailFor(int index) const
{
    if (index < 0 || index >= m_rows.size())
        return {};
    return m_rows.at(index).detail;
}

QVariantList DeviceListModel::rssiSeriesFor(int index) const
{
    if (index < 0 || index >= m_rows.size())
        return {};
    return m_rows.at(index).rssiSeries;
}

DeviceListModel::Row DeviceListModel::makeSeedRow(
    const QString &rssi,
    const QString &proto,
    const QString &addr,
    const QString &device,
    const QString &lastSeen,
    unsigned long packets,
    const QString &rate,
    double baseRssi,
    double rssiJitter,
    const QList<QPair<QString, QString>> &extraDetail)
{
    Row r;
    r.rssi        = rssi;
    r.proto       = proto;
    r.addr        = addr;
    r.device      = device;
    r.lastSeen    = lastSeen;
    r.packetsSeen = packets;
    r.packetRate  = rate;

    // 30-point synthetic 1-second window sampled at ~33 ms.
    // x in seconds from 0.000..0.967; y a noisy walk around baseRssi.
    QVariantList series;
    series.reserve(30);
    double y = baseRssi;
    for (int i = 0; i < 30; ++i)
    {
        // Deterministic-ish pseudo-randomness so the trace differs per row.
        double seed = baseRssi * 0.13 + double(i) * 0.41 + rssiJitter;
        double n = qSin(seed) + qCos(seed * 1.7);
        y = baseRssi + n * rssiJitter;
        series.append(QPointF(double(i) / 30.0, y));
    }
    r.rssiSeries = series;

    // Build the info-pane key/value list. Always include the columns plus
    // whatever extra detail was supplied.
    QVariantList detail;
    auto kv = [&detail](const QString &k, const QVariant &v) {
        QVariantMap m;
        m.insert(QStringLiteral("field"), k);
        m.insert(QStringLiteral("value"), v);
        detail.append(m);
    };
    kv(QStringLiteral("Address"),       addr);
    kv(QStringLiteral("Protocol"),      proto);
    kv(QStringLiteral("Device"),        device);
    kv(QStringLiteral("RSSI (1s avg)"), rssi);
    kv(QStringLiteral("Packets seen"), QVariant::fromValue(packets));
    kv(QStringLiteral("Packet rate"),  rate);
    kv(QStringLiteral("Last seen"),     lastSeen);
    for (const auto &pair : extraDetail)
        kv(pair.first, pair.second);

    r.detail = detail;
    return r;
}