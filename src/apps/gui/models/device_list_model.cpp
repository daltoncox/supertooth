#include "device_list_model.h"

#include <QDateTime>
#include <QPointF>
#include <QSet>
#include <QVariant>
#include <QVariantMap>

#include <algorithm>

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
        {TypeRole,        "type"},
        {AddrRole,        "addr"},
        {DeviceRole,      "device"},
        {IdentifierRole,  "identifier"},
        {FirstSeenRole,   "firstSeen"},
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
    case TypeRole:        return typeLabelFor(r.proto, r.device, r.addrType);
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
    case FirstSeenRole:
        // Frozen once at creation; mirrors core bredr_piconet (first_seen set on
        // the first packet, never touched again). Returned as a comparable
        // QDateTime so proxy sorting works identically to LastSeen.
        return r.firstSeenMs == 0
                   ? QVariant()
                   : QVariant::fromValue(
                        QDateTime::fromMSecsSinceEpoch(r.firstSeenMs));
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

    // BLE data-channel PDUs carry no device addresses: bucket them as a
    // "connection" identified by the random access address (the `addr`
    // field), mirroring how BR/EDR piconets get their own row.
    if (proto == QStringLiteral("LE") &&
        frame.value(QStringLiteral("type")).toString() == QStringLiteral("LL_DATA"))
        device = QStringLiteral("connection");

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

    // BLE advertising packets can carry a Manufacturer Specific Data AD
    // structure (0xFF) whose payload begins with a 16-bit Company Identifier.
    // When the backend surfaces one (as a "Manufacturer" detail entry), adopt
    // it on the row so the Device Info tab can show a human-readable vendor.
    // The most recent non-empty value wins; a later frame without it keeps the
    // previously discovered one.
    auto applyManufacturerFromDetail = [](Row &r, const QVariantList &detail) {
        for (const QVariant &entry : detail)
        {
            const QVariantMap pair = entry.toMap();
            if (pair.value(QStringLiteral("field")).toString() ==
                QStringLiteral("Manufacturer"))
            {
                const QString mfr = pair.value(QStringLiteral("value")).toString();
                if (!mfr.isEmpty())
                    r.manufacturer = mfr;
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
        r.addrType = frame.value(QStringLiteral("addrType")).toString();
        r.firstFrameMs = now;
        r.lastSeenMs   = now;
        r.firstSeenMs  = now;
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
        applyManufacturerFromDetail(r, r.lastFrameDetail);

        rowIdx = m_rows.size();
        beginInsertRows(QModelIndex(), rowIdx, rowIdx);
        m_rows.append(std::move(r));
        // Keep the lookups consistent before the proxy/views are notified.
        m_indexByKey.insert(key, rowIdx);
        m_rowById.insert(m_rows.at(rowIdx).id, rowIdx);
        endInsertRows();
        emit countChanged();
        // The new row must land in its sorted position (emits layoutChanged
        // if the order changed; no-op otherwise).
        maybeResort();
        return;
    }

    Row &r = m_rows[rowIdx];
    const QVariant sortKeyBefore = sortKey(r);
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
    applyManufacturerFromDetail(r, newDetail);

    const QModelIndex idx = index(rowIdx, 0);
    emit dataChanged(idx, idx);

    // Re-sort only when this row's sort key actually changed (a display name
    // discovery, a fresh RSSI/last-seen, ...). Cheap guard for the hot path.
    if (sortKey(r) != sortKeyBefore)
        maybeResort();
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
    maybeResort();
}

void DeviceListModel::clear()
{
    if (m_rows.isEmpty())
        return;
    beginResetModel();
    m_rows.clear();
    m_indexByKey.clear();
    m_rowById.clear();
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
    if (!r.manufacturer.isEmpty())
        add(QStringLiteral("Manufacturer"), r.manufacturer);
    add(QStringLiteral("RSSI (1s avg)"),
        qIsNaN(r.rssiDb) ? QStringLiteral("--")
                         : QString::number(r.rssiDb, 'f', 1) + QStringLiteral(" dBm"));
    add(QStringLiteral("Packets seen"), QVariant::fromValue(r.packetsSeen));
    add(QStringLiteral("Packet rate"),
        QString::number(r.lastRate) + QStringLiteral("/s"));
    add(QStringLiteral("First seen"), formatLastSeen(r.firstSeenMs));
    add(QStringLiteral("Last seen"),  formatLastSeen(r.lastSeenMs));
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
        QStringLiteral("Manufacturer"),
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

QString DeviceListModel::typeLabelFor(const QString &proto, const QString &device,
                                    const QString &addrType)
{
    if (proto == QStringLiteral("LE"))
    {
        // Data-channel PDUs are keyed by an access address (a link-layer
        // connection); advertising rows are keyed by a device address whose
        // subtype (PUBLIC / STATIC / RESOLVABLE / NONRESOLVABLE / RESERVED)
        // is carried in addrType when known.
        if (device == QStringLiteral("connection"))
            return QStringLiteral("CONN");
        if (!addrType.isEmpty())
            return addrType;
        return QStringLiteral("ADV_ADDR");
    }
    if (proto == QStringLiteral("BR/EDR"))
        // Pre-break-out piconet rows have no slave track yet; broken-out
        // rows are addressed by LT_ADDR (or the Central).
        return device == QStringLiteral("piconet") ? QStringLiteral("CONN")
                                                   : QStringLiteral("LT_ADDR");
    return QStringLiteral("--");
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

    // BLE connection rows (data-channel traffic) are identified by their
    // random access address; there is never a learned local name for one.
    if (r.device == QStringLiteral("connection"))
        return r.addr.isEmpty() ? QStringLiteral("Connection") : r.addr;

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

    beginRemoveRows(QModelIndex(), index, index);
    m_rows.removeAt(index);
    // Re-key the lookups before endRemoveRows so any view reselect that runs
    // synchronously on rowsRemoved reads a consistent row<->id mapping.
    rebuildLookup();
    endRemoveRows();
    emit countChanged();
}

int DeviceListModel::rowForDeviceId(int id) const
{
    return m_rowById.value(id, -1);
}

void DeviceListModel::setSortRoleName(const QString &name)
{
    if (m_sortRoleName == name)
        return;
    m_sortRoleName = name;
    emit sortRoleNameChanged();
    maybeResort();
}

void DeviceListModel::setSortOrder(Qt::SortOrder order)
{
    if (m_sortOrder == order)
        return;
    m_sortOrder = order;
    emit sortOrderChanged();
    maybeResort();
}

void DeviceListModel::sortBy(const QString &roleName, Qt::SortOrder order)
{
    const bool roleChanged = (m_sortRoleName != roleName);
    const bool orderChanged = (m_sortOrder != order);
    if (!roleChanged && !orderChanged)
        return;

    m_sortRoleName = roleName;
    m_sortOrder = order;
    if (roleChanged) emit sortRoleNameChanged();
    if (orderChanged) emit sortOrderChanged();
    maybeResort();
}

bool DeviceListModel::lessThan(const Row &a, const Row &b) const
{
    const bool asc = (m_sortOrder == Qt::AscendingOrder);
    const auto cmpInt = [asc](qint64 l, qint64 r) { return asc ? (l < r) : (l > r); };
    const auto cmpStr = [asc](const QString &l, const QString &r) { return asc ? (l < r) : (l > r); };

    if (m_sortRoleName == QStringLiteral("rssi"))
    {
        // Pin the "no signal yet" sentinel to the bottom in either direction,
        // mirroring the old proxy's lessThan.
        const double l = qIsNaN(a.rssiDb) ? kRssiSentinel : a.rssiDb;
        const double r = qIsNaN(b.rssiDb) ? kRssiSentinel : b.rssiDb;
        const bool lSentinel = (l <= kRssiSentinel);
        const bool rSentinel = (r <= kRssiSentinel);
        if (lSentinel && rSentinel) return false;
        if (lSentinel) return false;   // a sorts to the bottom in both orders
        if (rSentinel) return true;    // b sorts to the bottom
        return asc ? (l < r) : (l > r);
    }

    if (m_sortRoleName == QStringLiteral("identifier"))
    {
        // Group by protocol first (a 0x??<LAP> BR/EDR address never
        // interleaves with a short BLE advertising address), then by type,
        // then by the rendered identifier. Each level honors the direction.
        if (a.proto != b.proto)
            return cmpStr(a.proto, b.proto);
        const QString ta = typeLabelFor(a.proto, a.device, a.addrType);
        const QString tb = typeLabelFor(b.proto, b.device, b.addrType);
        if (ta != tb)
            return cmpStr(ta, tb);
        return cmpStr(identifierLabelFor(a), identifierLabelFor(b));
    }

    if (m_sortRoleName == QStringLiteral("type"))
        return cmpStr(typeLabelFor(a.proto, a.device, a.addrType),
                      typeLabelFor(b.proto, b.device, b.addrType));
    if (m_sortRoleName == QStringLiteral("proto"))
        return cmpStr(a.proto, b.proto);
    if (m_sortRoleName == QStringLiteral("lastSeen"))
        return cmpInt(a.lastSeenMs, b.lastSeenMs);
    if (m_sortRoleName == QStringLiteral("firstSeen"))
        return cmpInt(a.firstSeenMs, b.firstSeenMs);
    if (m_sortRoleName == QStringLiteral("packetRate"))
        return cmpInt(a.lastRate, b.lastRate);

    // Unknown/unsortable role: preserve current (stable insertion) order.
    return false;
}

QVariant DeviceListModel::sortKey(const Row &r) const
{
    if (m_sortRoleName == QStringLiteral("rssi"))
        return qIsNaN(r.rssiDb) ? kRssiSentinel : r.rssiDb;
    if (m_sortRoleName == QStringLiteral("lastSeen"))
        return r.lastSeenMs;
    if (m_sortRoleName == QStringLiteral("firstSeen"))
        return r.firstSeenMs;
    if (m_sortRoleName == QStringLiteral("packetRate"))
        return r.lastRate;
    if (m_sortRoleName == QStringLiteral("proto"))
        return r.proto;
    if (m_sortRoleName == QStringLiteral("type"))
        return typeLabelFor(r.proto, r.device, r.addrType);
    if (m_sortRoleName == QStringLiteral("identifier"))
        return identifierLabelFor(r);
    return {};
}

void DeviceListModel::maybeResort()
{
    if (m_rows.size() < 2)
        return;

    QVector<int> before;
    before.reserve(m_rows.size());
    for (const Row &r : m_rows)
        before.append(r.id);

    std::stable_sort(m_rows.begin(), m_rows.end(),
                     [this](const Row &a, const Row &b) { return lessThan(a, b); });

    // Emit layoutChanged only when the order actually changed, so views and
    // the selection resolver are not churned on every frame/tick.
    for (int i = 0; i < m_rows.size(); ++i)
    {
        if (m_rows.at(i).id != before.at(i))
        {
            rebuildLookup();
            emit layoutAboutToBeChanged();
            emit layoutChanged();
            return;
        }
    }
}

void DeviceListModel::rebuildLookup()
{
    m_indexByKey.clear();
    m_rowById.clear();
    for (int i = 0; i < m_rows.size(); ++i)
    {
        const Row &r = m_rows.at(i);
        m_indexByKey.insert(makeKey(r.proto, r.addr, r.device), i);
        m_rowById.insert(r.id, i);
    }
}