#include "device_list_model.h"

#include <QDateTime>
#include <QPointF>
#include <QSet>
#include <QVariantMap>

DeviceListModel::DeviceListModel(QObject *parent)
    : QAbstractListModel(parent)
{
    m_tick.setInterval(1000);
    connect(&m_tick, &QTimer::timeout, this, &DeviceListModel::tickRows);
    m_tick.start();
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
        {RssiRole,        "rssi"},
        {ProtoRole,       "proto"},
        {AddrRole,        "addr"},
        {DeviceRole,      "device"},
        {IdentifierRole,  "identifier"},
        {LastSeenRole,    "lastSeen"},
        {PacketsSeenRole, "packetsSeen"},
        {PacketRateRole,  "packetRate"},
        {DeviceIdRole,    "deviceId"},
    };
}

QVariant DeviceListModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() < 0 || index.row() >= m_rows.size())
        return {};
    const Row &r = m_rows.at(index.row());
    switch (role)
    {
    case RssiRole:
        // Numeric so a proxy can sort by signal; -999.0 = "no signal yet"
        // sentinel (always sorts to bottom). To be honest the legacy
        // "--" is reproduced on the renderer side.
        return QVariant::fromValue(qIsNaN(r.rssiDb) ? kRssiSentinel : r.rssiDb);
    case ProtoRole:       return r.proto;
    case AddrRole:       return r.addr;
    case DeviceRole:     return r.device;
    case IdentifierRole: return identifierLabelFor(r);
    case LastSeenRole:
        // QDateTime so a proxy can sort chronologically. An invalid
        // QDateTime means "never seen"; the renderer formats "--".
        return r.lastSeenMs == 0
                   ? QVariant()
                   : QVariant::fromValue(
                        QDateTime::fromMSecsSinceEpoch(r.lastSeenMs));
    case PacketsSeenRole: return QVariant::fromValue(r.packetsSeen);
    case PacketRateRole:  return QVariant::fromValue(r.lastRate);
    case DeviceIdRole:    return r.id;
    }
    return {};
}

void DeviceListModel::onFrameDecoded(const QVariantMap &frame)
{
    const QString proto = frame.value(QStringLiteral("proto")).toString();
    QString addr  = frame.value(QStringLiteral("addr")).toString();
    const QString src   = frame.value(QStringLiteral("src")).toString();
    const QString dst   = frame.value(QStringLiteral("dst")).toString();
    const float rssiDb  = frame.value(QStringLiteral("rssiDb")).toFloat();
    const qint64 now    = QDateTime::currentMSecsSinceEpoch();

    QString device = deviceLabelFor(proto, src, dst);

    // BR/EDR piconet handling: a single piconet (identified by LAP, the low
    // 24 bits of the address) may emit frames whose `addr` flips between
    // "0x??<LAP>" and "0x<UAP><LAP>" as UAP is recovered, and whose `src`
    // flips between "--" and "Central"/"LT_ADDR N" as the on-air header
    // becomes decodeable (clock found). To keep the device list stable
    // through these transitions and avoid a flickering duplicate piconet
    // row every time decode state toggles:
    //   - pre-break-out piconet frames are bucketed by LAP and stored under
    //     the normalized addr "0x??<LAP>" so UAP discovery never forks the
    //     row;
    //   - once a frame with a decodeable header arrives, the piconet is
    //     marked "broken out", any pre-break-out piconet row for that LAP
    //     is removed, and per-device rows replace it;
    //   - subsequent frames we can't decode (src="--") are dropped from the
    //     device view rather than recreating a stale piconet row.
    if (proto == QStringLiteral("BR/EDR"))
    {
        const QString lap = (addr.length() >= 6) ? addr.right(6) : addr;
        const QString picoKey = QStringLiteral("BR/EDR") + QChar(0x1F) + lap;
        const bool brokenOut = m_brokenOutPiconets.contains(picoKey);

        if (device == QStringLiteral("piconet"))
        {
            if (brokenOut)
                return;
            addr = QStringLiteral("0x??") + lap;
        }
        else
        {
            if (!brokenOut)
            {
                m_brokenOutPiconets.insert(picoKey);
                for (int i = m_rows.size() - 1; i >= 0; --i)
                {
                    const Row &r = m_rows.at(i);
                    if (r.proto == QStringLiteral("BR/EDR") &&
                        r.device == QStringLiteral("piconet") &&
                        (r.addr.length() >= 6 ? r.addr.right(6) : r.addr) == lap)
                    {
                        removeRow(i);
                    }
                }
            }
        }
    }

    const QString key = makeKey(proto, addr, device);
    int rowIdx = m_indexByKey.value(key, -1);

    // BLE advertising packets can carry a Complete/Shortened Local Name inside
    // their AD structures. When the backend surfaces one (as a "Device Name"
    // detail entry), adopt it as the row's display name so the Identifier
    // column can show a human-readable name instead of the raw AdvA. The most
    // recent non-empty authoritative name wins; a later frame without a name
    // keeps the previously discovered one.
    auto applyDeviceNameFromDetail = [](Row &r, const QVariantList &detail) {
        for (const QVariant &entry : detail)
        {
            const QVariantMap pair = entry.toMap();
            if (pair.value(QStringLiteral("field")).toString() ==
                QStringLiteral("Device Name"))
            {
                const QString name = pair.value(QStringLiteral("value")).toString();
                if (!name.isEmpty())
                    r.displayName = name;
                return;
            }
        }
    };

    if (rowIdx < 0)
    {
        Row r;
        r.id = m_nextId++;
        r.proto = proto;
        r.addr = addr;
        r.device = device;
        r.firstFrameMs = now;
        r.lastSeenMs = now;
        r.packetsSeen = 1;
        r.packetsThisSecond = 1;
        if (!qIsNaN(rssiDb))
        {
            r.samples.append({now, double(rssiDb)});
            r.lastFrameRssiDb = double(rssiDb);
            r.rssiDb = double(rssiDb);
            r.rawSeries.append(QPointF(0.0, double(rssiDb)));
            r.avgSeries.append(QPointF(0.0, double(rssiDb)));
        }
        r.lastFrameDetail = frame.value(QStringLiteral("detail")).toList();
        applyDeviceNameFromDetail(r, r.lastFrameDetail);

        rowIdx = m_rows.size();
        beginInsertRows(QModelIndex(), rowIdx, rowIdx);
        m_rows.append(std::move(r));
        endInsertRows();
        m_indexByKey.insert(key, rowIdx);
        emit countChanged();
        return;
    }

    Row &r = m_rows[rowIdx];
    r.packetsSeen++;
    r.packetsThisSecond++;
    r.lastSeenMs = now;

    if (!qIsNaN(rssiDb))
    {
        r.lastFrameRssiDb = double(rssiDb);
        r.samples.append({now, double(rssiDb)});

        const double x = double(now - r.firstFrameMs) / 1000.0;
        r.rawSeries.append(QPointF(x, double(rssiDb)));

        recomputeAverage(r, now);
        evictChartHistory(r, now);
    }

// Keep the latest frame's overflow detail so the info pane reflects the
    // most recent packet (PDU type, channel, AC errors, UAP, etc.).
    const QVariantList newDetail = frame.value(QStringLiteral("detail")).toList();
    if (!newDetail.isEmpty())
        r.lastFrameDetail = newDetail;
    applyDeviceNameFromDetail(r, newDetail);

    const QModelIndex idx = index(rowIdx, 0);
    emit dataChanged(idx, idx);
}

void DeviceListModel::recomputeAverage(Row &r, qint64 now)
{
    evictWindow(r, now);

    if (r.samples.isEmpty())
        return; // hold last rssiDb

    double sum = 0.0;
    int n = 0;
    for (const auto &s : r.samples)
    {
        if (!qIsNaN(s.second))
        {
            sum += s.second;
            n++;
        }
    }
    if (n > 0)
    {
        r.rssiDb = sum / double(n);

        const double x = double(now - r.firstFrameMs) / 1000.0;
        r.avgSeries.append(QPointF(x, r.rssiDb));
    }
}

void DeviceListModel::evictWindow(Row &r, qint64 now)
{
    // Evict only the 1-second averaging window. Chart-history series are
    // retained separately (see evictChartHistory) so a 5-minute trace stays
    // visible on the graph.
    const qint64 cutoff = now - kWindowMs;
    while (!r.samples.isEmpty() && r.samples.first().first < cutoff)
        r.samples.removeFirst();
}

void DeviceListModel::evictChartHistory(Row &r, qint64 now)
{
    // Drop chart-history points older than the maximum selectable window
    // so memory stays bounded during long captures. Also cap the total
    // point count per series as a backstop against very high packet rates.
    const double cutX = double(now - r.firstFrameMs - kChartHistoryMs) / 1000.0;
    auto evict = [cutX](QVector<QPointF> &v)
    {
        while (v.size() > 2 && v.first().x() < cutX && v.at(1).x() <= cutX)
            v.removeFirst();
        if (v.size() > kMaxSeriesPoints)
            v.remove(0, v.size() - kMaxSeriesPoints);
    };
    evict(r.avgSeries);
    evict(r.rawSeries);
}

void DeviceListModel::tickRows()
{
    if (m_rows.isEmpty())
        return;

    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    const int last = m_rows.size() - 1;

    for (int i = 0; i <= last; ++i)
    {
        Row &r = m_rows[i];
        r.lastRate = r.packetsThisSecond;
        r.packetsThisSecond = 0;
        recomputeAverage(r, now);
        evictChartHistory(r, now);
    }

    const QModelIndex topLeft = index(0, 0);
    const QModelIndex bottomRight = index(last, 0);
    emit dataChanged(topLeft, bottomRight);
}

void DeviceListModel::clear()
{
    if (m_rows.isEmpty())
        return;
    beginResetModel();
    m_rows.clear();
    m_indexByKey.clear();
    m_brokenOutPiconets.clear();
    m_nextId = 1;
    endResetModel();
    emit countChanged();
}

QVariantList DeviceListModel::detailFor(int index) const
{
    if (index < 0 || index >= m_rows.size())
        return {};

    const Row &r = m_rows.at(index);
    QVariantList out;

    auto add = [&out](const QString &field, const QVariant &value)
    {
        QVariantMap m;
        m.insert(QStringLiteral("field"), field);
        m.insert(QStringLiteral("value"), value);
        out.append(m);
    };

    add(QStringLiteral("Protocol"), r.proto);
    add(QStringLiteral("Address"), r.addr);
    add(QStringLiteral("Device"), r.device);
    if (!r.displayName.isEmpty())
        add(QStringLiteral("Device Name"), r.displayName);
    add(QStringLiteral("RSSI (1s avg)"),
        qIsNaN(r.rssiDb) ? QStringLiteral("--")
                         : QString::number(r.rssiDb, 'f', 1) + QStringLiteral(" dBm"));
    add(QStringLiteral("Packets seen"), QVariant::fromValue(r.packetsSeen));
    add(QStringLiteral("Packet rate"),
        QString::number(r.lastRate) + QStringLiteral("/s"));
    add(QStringLiteral("Last seen"), formatLastSeen(r.lastSeenMs));
    add(QStringLiteral("Last raw RSSI"),
        qIsNaN(r.lastFrameRssiDb)
            ? QStringLiteral("--")
            : QString::number(r.lastFrameRssiDb, 'f', 1) + QStringLiteral(" dBm"));

    // Merge in any per-frame detail from the most recent frame (PDU type,
    // channel, AC errors, UAP, etc.). Each entry is a {field, value} map.
    // Skip ones we already surface live above so the pane doesn't repeat.
    static const QSet<QString> s_liveReplaced{
        QStringLiteral("RSSI"),
        QStringLiteral("Channel"),
        QStringLiteral("Protocol"),
        QStringLiteral("Address"),
        QStringLiteral("Device"),
        QStringLiteral("Device Name"),
    };
    for (const QVariant &entry : r.lastFrameDetail)
    {
        const QVariantMap pair = entry.toMap();
        const QString field = pair.value(QStringLiteral("field")).toString();
        if (field.isEmpty() || s_liveReplaced.contains(field))
            continue;
        add(field, pair.value(QStringLiteral("value")));
    }

    return out;
}

QVariantList DeviceListModel::rssiSeriesFor(int index) const
{
    if (index < 0 || index >= m_rows.size())
        return {};
    const Row &r = m_rows.at(index);
    QVariantList out;
    out.reserve(r.avgSeries.size());
    for (const QPointF &p : r.avgSeries)
        out.append(QVariant::fromValue(p));
    return out;
}

QVariantList DeviceListModel::rssiRawSeriesFor(int index) const
{
    if (index < 0 || index >= m_rows.size())
        return {};
    const Row &r = m_rows.at(index);
    QVariantList out;
    out.reserve(r.rawSeries.size());
    for (const QPointF &p : r.rawSeries)
        out.append(QVariant::fromValue(p));
    return out;
}

qint64 DeviceListModel::firstFrameMsFor(int index) const
{
    if (index < 0 || index >= m_rows.size())
        return 0;
    return m_rows.at(index).firstFrameMs;
}

QString DeviceListModel::makeKey(const QString &proto, const QString &addr,
                                 const QString &device)
{
    static const QChar sep = QChar(0x1F); // unit-separator; never appears in these strings
    return proto + sep + addr + sep + device;
}

QString DeviceListModel::deviceLabelFor(const QString &proto, const QString &src,
                                        const QString &dst)
{
    if (proto == QStringLiteral("BR/EDR"))
    {
        // The C facade sets src="Central" on master slots, src="LT_ADDR N"
        // on slave slots, or src="--" while the piconet is unbroken-out.
        if (src.isEmpty() || src == QStringLiteral("--"))
            return QStringLiteral("piconet");
        return src;
    }
    // BLE: prefer AdvA/ScannerA/source addr; if not present, "Unknown".
    if (!src.isEmpty() && src != QStringLiteral("--"))
        return src;
    return QStringLiteral("Unknown");
}

QString DeviceListModel::identifierLabelFor(const Row &r)
{
    if (r.proto == QStringLiteral("BR/EDR"))
    {
        // Piconet rows (clock unknown, pre-break-out) expose only the
        // partially-recovered address — render that as-is, without a
        // role suffix.
        if (r.device == QStringLiteral("piconet"))
            return r.addr;

        // Broken-out piconet rows are tagged with their role relative to the
        // piconet: "Central" gets the short suffix "C" while each slave slot
        // carries its LT_ADDR number. The result reads like "0x12345678-C"
        // or "0x12345678-3".
        QString suffix;
        if (r.device == QStringLiteral("Central"))
            suffix = QStringLiteral("C");
        else if (r.device.startsWith(QStringLiteral("LT_ADDR ")))
            suffix = r.device.mid(QStringLiteral("LT_ADDR ").length());
        else
            suffix = r.device;

        if (suffix.isEmpty())
            return r.addr;
        return r.addr + QStringLiteral("-") + suffix;
    }

    // BLE: a learned local name (from an advertising packet's AD structure)
    // wins over the raw AdvA so the table reads naturally. Fall back to the
    // advertising/scan address if no name was ever observed.
    if (!r.displayName.isEmpty())
        return r.displayName;
    return r.device.isEmpty() ? QStringLiteral("Unknown") : r.device;
}

QString DeviceListModel::formatLastSeen(qint64 ms)
{
    if (ms == 0)
        return QStringLiteral("--");
    // Tenths of a second precision — Qt's date-format codes don't directly
    // clamp to that, so build the string from a HH:mm:ss component plus the
    // single most-significant digit of the millisecond residue.
    const QDateTime dt = QDateTime::fromMSecsSinceEpoch(ms);
    const QString base = dt.toString(QStringLiteral("HH:mm:ss"));
    const int tenth = int((ms % 1000) / 100);
    return base + QStringLiteral(".") + QString::number(tenth);
}

void DeviceListModel::removeRow(int index)
{
    if (index < 0 || index >= m_rows.size())
        return;

    const QString removedKey = makeKey(m_rows.at(index).proto,
                                       m_rows.at(index).addr,
                                       m_rows.at(index).device);
    m_indexByKey.remove(removedKey);

    beginRemoveRows(QModelIndex(), index, index);
    m_rows.removeAt(index);
    endRemoveRows();

    // Indexes after `index` shifted down by 1; the hash map needs to be re-keyed.
    m_indexByKey.clear();
    for (int i = 0; i < m_rows.size(); ++i)
    {
        const Row &r = m_rows.at(i);
        m_indexByKey.insert(makeKey(r.proto, r.addr, r.device), i);
    }

    emit countChanged();
}