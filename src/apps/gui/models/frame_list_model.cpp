#include "frame_list_model.h"

#include <QVariantMap>

#include <algorithm>

FrameListModel::FrameListModel(QObject *parent)
    : QAbstractListModel(parent)
{
}

int FrameListModel::rowCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return 0;
    return m_rows.size();
}

QHash<int, QByteArray> FrameListModel::roleNames() const
{
    return {
        {NoRole,    "no"},
        {TimeRole,  "time"},
        {RssiRole,  "rssi"},
        {ProtoRole, "proto"},
        {ChIdxRole, "chIdx"},
        {AddrRole,  "addr"},
        {SrcRole,   "src"},
        {DstRole,   "dst"},
        {TypeRole,  "type"},
        {InfoRole,  "info"},
    };
}

QVariant FrameListModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() < 0 || index.row() >= m_rows.size())
        return {};
    const Row &r = m_rows.at(index.row());
    switch (role)
    {
    case NoRole:    return QVariant::fromValue(r.no);
    case TimeRole:  return r.time;
    case RssiRole:  return r.rssi;
    case ProtoRole: return r.proto;
    case ChIdxRole: return QVariant::fromValue(r.chIdx);
    case AddrRole:  return r.addr;
    case SrcRole:   return r.src;
    case DstRole:   return r.dst;
    case TypeRole:  return r.type;
    case InfoRole:  return r.info;
    }
    return {};
}

void FrameListModel::appendRow(const QVariantMap &row)
{
    Row r;
    r.no = row.value(QStringLiteral("no")).toULongLong();
    if (r.no == 0)
        r.no = m_nextNo++;
    else
        m_nextNo = std::max(m_nextNo, r.no + 1);
    r.time = row.value(QStringLiteral("time")).toString();
    {
        const float rssi = row.value(QStringLiteral("rssiDb")).toFloat();
        if (qIsNaN(rssi))
            r.rssi = QStringLiteral("--");
        else
            r.rssi = QString::number(rssi, 'f', 1);
    }
    r.proto = row.value(QStringLiteral("proto")).toString();
    r.chIdx = (unsigned int)row.value(QStringLiteral("chIdx")).toUInt();
    r.addr = row.value(QStringLiteral("addr")).toString();
    r.src = row.value(QStringLiteral("src")).toString();
    r.dst = row.value(QStringLiteral("dst")).toString();
    r.type = row.value(QStringLiteral("type")).toString();
    r.info = row.value(QStringLiteral("info")).toString();
    r.raw = row.value(QStringLiteral("rawBytes")).toByteArray();
    r.detail = row.value(QStringLiteral("detail")).toList();

    if (m_rows.size() >= s_capacity)
    {
        beginRemoveRows(QModelIndex(), 0, 0);
        m_rows.removeFirst();
        endRemoveRows();
    }
    beginInsertRows(QModelIndex(), m_rows.size(), m_rows.size());
    m_rows.append(std::move(r));
    endInsertRows();
    emit countChanged();
}

void FrameListModel::clear()
{
    if (m_rows.isEmpty())
        return;
    beginResetModel();
    m_rows.clear();
    m_nextNo = 1;
    endResetModel();
    emit countChanged();
}

int FrameListModel::indexForNo(qulonglong no) const
{
    for (int i = 0; i < m_rows.size(); i++)
        if (m_rows.at(i).no == no)
            return i;
    return -1;
}

QVariantList FrameListModel::detailFor(int index) const
{
    if (index < 0 || index >= m_rows.size())
        return {};
    return m_rows.at(index).detail;
}

QVariantList FrameListModel::buildHex(const QByteArray &raw)
{
    static constexpr int kBytesPerLine = 16;

    QVariantList out;
    const int total = raw.size();
    if (total <= 0)
        return out;

    for (int off = 0; off < total; off += kBytesPerLine)
    {
        int chunk = std::min(kBytesPerLine, total - off);

        QString hex;
        QString ascii;
        hex.reserve(chunk * 3);
        ascii.reserve(chunk);
        for (int i = 0; i < chunk; i++)
        {
            unsigned char b = (unsigned char)raw.at(off + i);
            hex.append(QString::asprintf("%02X ", b));
            ascii.append((b >= 0x20 && b < 0x7F) ? QChar::fromLatin1(b)
                                                 : QLatin1Char('.'));
        }
        for (int i = chunk; i < kBytesPerLine; i++)
            hex.append(QStringLiteral("   "));

        QVariantMap line;
        line.insert(QStringLiteral("offset"),
                    QString::asprintf("%04X", off));
        // Keep the padding (do not trim) so every row's hex column has the
        // same width and the ascii column stays left-aligned across rows.
        line.insert(QStringLiteral("hex"), hex);
        line.insert(QStringLiteral("ascii"), ascii);
        out.append(line);
    }
    return out;
}

QVariantList FrameListModel::hexFor(int index) const
{
    if (index < 0 || index >= m_rows.size())
        return {};
    return buildHex(m_rows.at(index).raw);
}
