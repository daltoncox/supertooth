#include "device_list_model.h"

#include <QDateTime>
#include <QVariant>
#include <QVariantMap>

#include <algorithm>

DeviceListModel::DeviceListModel(QObject *parent)
    : QAbstractListModel(parent)
{
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
        return QVariant::fromValue(r.rssiValid
                                       ? r.rssiDb
                                       : kRssiSentinel);
    case ProtoRole:       return r.proto;
    case TypeRole:        return typeLabelFor(r.proto, r.device, r.addrType);
    case AddrRole:        return r.addr;
    case DeviceRole:      return r.device;
    case IdentifierRole:  return identifierLabelFor(r);
    case LastSeenRole:
        return r.lastSeenMs == 0
                   ? QVariant()
                   : QVariant::fromValue(
                         QDateTime::fromMSecsSinceEpoch(r.lastSeenMs));
    case FirstSeenRole:
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

void DeviceListModel::setRows(const QVariantList &rows)
{
    /* Preserve per-entity RSSI history across polls (rows are rebuilt each
     * poll from scratch, so carry the series forward by stable id). */
    QHash<int, QVector<QPointF>> oldSeries;
    for (const Row &r : m_rows)
        oldSeries.insert(r.id, r.avgSeries);

    const qint64 now = QDateTime::currentMSecsSinceEpoch();

    QVector<Row> next;
    next.reserve(rows.size());
    for (const QVariant &v : rows)
    {
        const QVariantMap m = v.toMap();
        Row r;
        r.id = m.value(QStringLiteral("id")).toInt();
        r.proto = m.value(QStringLiteral("proto")).toString();
        r.device = m.value(QStringLiteral("device")).toString();
        r.addr = m.value(QStringLiteral("addr")).toString();
        r.addrType = m.value(QStringLiteral("addrType")).toString();
        r.displayName = m.value(QStringLiteral("displayName")).toString();
        r.manufacturer = m.value(QStringLiteral("manufacturer")).toString();
        r.rssiValid = m.value(QStringLiteral("rssiValid")).toBool() ? 1 : 0;
        r.rssiDb = r.rssiValid
                       ? m.value(QStringLiteral("rssiDb")).toDouble()
                       : qQNaN();
        r.firstSeenMs = m.value(QStringLiteral("firstSeenMs")).toLongLong();
        r.lastSeenMs = m.value(QStringLiteral("lastSeenMs")).toLongLong();
        r.firstFrameMs = r.firstSeenMs;
        const qulonglong pk =
            m.value(QStringLiteral("packetsSeen")).toULongLong();
        const qulonglong prev = m_prevPackets.value(r.id, pk);
        const qulonglong delta = (pk >= prev) ? (pk - prev) : 0;
        r.lastRate = (int)qMin(delta * 4ULL, 100000ULL);
        m_prevPackets[r.id] = pk;
        r.packetsSeen = pk;

        r.crcInit = (uint32_t)m.value(QStringLiteral("crcInit")).toUInt();
        r.crcInitConfirmed = m.value(QStringLiteral("crcInitConfirmed")).toBool() ? 1 : 0;
        r.crcInitCandidates = m.value(QStringLiteral("crcInitCandidates")).toInt();

        QVector<QPointF> series = oldSeries.value(r.id);
        if (r.rssiValid)
        {
            const double x = (now - r.firstFrameMs) / 1000.0;
            series.append(QPointF(x, r.rssiDb));
            if (series.size() > kMaxSeriesPoints)
                series.removeFirst();
        }
        r.avgSeries = series;

        next.append(r);
    }

    /* Keep the active column sort stable across polls: re-sort the incoming
     * rows the same way the previous snapshot was sorted, so a user-initiated
     * sort doesn't make every later poll look "structural" (which would force
     * a full reset again). */
    if (!m_sortRoleName.isEmpty() && next.size() >= 2)
        std::stable_sort(next.begin(), next.end(),
                         [this](const Row &a, const Row &b) { return lessThan(a, b); });

    /* Decide how much of the model changed and emit the lightest signal that
     * is correct:
     *   - same ids, same order  -> dataChanged      (delegates untouched)
     *   - same ids, new order   -> layoutChanged     (delegates reused, reordered)
     *   - ids added/removed     -> full reset        (rare)
     * beginResetModel()/endResetModel() must be avoided for the common
     * data-refresh case: it destroys and recreates every delegate on the GUI
     * thread 4x/second and starves input handling during a live capture. */
    bool sameSet = (next.size() == m_rows.size());
    if (sameSet && next.size() > 0)
    {
        QSet<int> curIds;
        curIds.reserve(m_rows.size());
        for (const Row &r : m_rows)
            curIds.insert(r.id);
        for (const Row &r : next)
        {
            if (!curIds.contains(r.id))
            {
                sameSet = false;
                break;
            }
        }
    }

    if (sameSet)
    {
        bool sameOrder = (next.size() == m_rows.size());
        for (int i = 0; sameOrder && i < next.size(); ++i)
        {
            if (next.at(i).id != m_rows.at(i).id)
                sameOrder = false;
        }

        m_rows = next;
        rebuildLookup();

        if (sameOrder)
        {
            if (!m_rows.isEmpty())
                emit dataChanged(index(0, 0), index(m_rows.size() - 1, 0));
        }
        else
        {
            emit layoutAboutToBeChanged();
            emit layoutChanged();
        }
    }
    else
    {
        beginResetModel();
        m_rows = next;
        rebuildLookup();
        endResetModel();
        emit countChanged();
    }
}

void DeviceListModel::clear()
{
    if (m_rows.isEmpty() && m_prevPackets.isEmpty())
        return;
    beginResetModel();
    m_rows.clear();
    m_rowById.clear();
    m_prevPackets.clear();
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
        r.rssiValid ? QString::number(r.rssiDb, 'f', 1) + QStringLiteral(" dBm")
                    : QStringLiteral("--"));
    add(QStringLiteral("Packets seen"), QVariant::fromValue(r.packetsSeen));
    add(QStringLiteral("Packet rate"),
        QString::number(r.lastRate) + QStringLiteral("/s"));
    add(QStringLiteral("First seen"), formatLastSeen(r.firstSeenMs));
    add(QStringLiteral("Last seen"),  formatLastSeen(r.lastSeenMs));

    if (r.proto == QStringLiteral("LE") && r.device == QStringLiteral("connection"))
    {
        if (r.crcInitConfirmed)
            add(QStringLiteral("CRC Init"),
                QStringLiteral("0x%1").arg(r.crcInit, 6, 16, QChar('0')).toUpper());
        else
            add(QStringLiteral("CRC Init candidates"),
                QVariant::fromValue(r.crcInitCandidates));
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
    Q_UNUSED(index);
    return {};
}

qint64 DeviceListModel::firstFrameMsFor(int index) const
{
    if (index < 0 || index >= m_rows.size())
        return 0;
    return m_rows.at(index).firstFrameMs;
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

QString DeviceListModel::typeLabelFor(const QString &proto, const QString &device,
                                      const QString &addrType)
{
    if (proto == QStringLiteral("LE"))
    {
        if (device == QStringLiteral("connection"))
            return QStringLiteral("CONN");
        if (!addrType.isEmpty())
            return addrType;
        return QStringLiteral("ADV_ADDR");
    }
    if (proto == QStringLiteral("BR/EDR"))
    {
        if (device == QStringLiteral("piconet"))
            return QStringLiteral("CONN");
        if (device == QStringLiteral("INQUIRY"))
            return QStringLiteral("INQUIRY");
        return QStringLiteral("LT_ADDR");
    }
    return QStringLiteral("--");
}

QString DeviceListModel::identifierLabelFor(const Row &r)
{
    if (r.proto == QStringLiteral("BR/EDR"))
    {
        if (r.device == QStringLiteral("piconet") ||
            r.device == QStringLiteral("INQUIRY"))
            return r.addr;
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

    if (r.device == QStringLiteral("connection"))
        return r.addr.isEmpty() ? QStringLiteral("Connection") : r.addr;

    if (!r.displayName.isEmpty())
        return r.displayName;
    return r.addr.isEmpty() ? QStringLiteral("Unknown") : r.addr;
}

QString DeviceListModel::formatLastSeen(qint64 ms)
{
    if (ms == 0)
        return QStringLiteral("--");
    const QDateTime dt = QDateTime::fromMSecsSinceEpoch(ms);
    const QString base = dt.toString(QStringLiteral("HH:mm:ss"));
    const int tenth = int((ms % 1000) / 100);
    return base + QStringLiteral(".") + QString::number(tenth);
}

bool DeviceListModel::lessThan(const Row &a, const Row &b) const
{
    const bool asc = (m_sortOrder == Qt::AscendingOrder);
    const auto cmpInt = [asc](qint64 l, qint64 r) { return asc ? (l < r) : (l > r); };
    const auto cmpStr = [asc](const QString &l, const QString &r) { return asc ? (l < r) : (l > r); };

    if (m_sortRoleName == QStringLiteral("rssi"))
    {
        const double l = a.rssiValid ? a.rssiDb : kRssiSentinel;
        const double r = b.rssiValid ? b.rssiDb : kRssiSentinel;
        const bool lS = (l <= kRssiSentinel);
        const bool rS = (r <= kRssiSentinel);
        if (lS && rS) return false;
        if (lS) return false;
        if (rS) return true;
        return asc ? (l < r) : (l > r);
    }

    if (m_sortRoleName == QStringLiteral("identifier"))
    {
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

    return false;
}

QVariant DeviceListModel::sortKey(const Row &r) const
{
    if (m_sortRoleName == QStringLiteral("rssi"))
        return r.rssiValid ? r.rssiDb : kRssiSentinel;
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
    m_rowById.clear();
    for (int i = 0; i < m_rows.size(); ++i)
        m_rowById.insert(m_rows.at(i).id, i);
}
