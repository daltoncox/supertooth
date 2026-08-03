#include "device_list_sort_proxy_model.h"

#include "device_list_model.h"

#include <QDateTime>
#include <QVariant>

DeviceListSortProxyModel::DeviceListSortProxyModel(QObject *parent)
    : QSortFilterProxyModel(parent)
{
    // Re-sort whenever the source model emits dataChanged for the sort
    // role (e.g. DeviceListModel's 1 Hz tick updates), and whenever rows
    // are inserted/removed. This keeps the displayed order stable-by-value
    // for live RSSI/Rate decay rather than only re-sorting on user clicks.
    setDynamicSortFilter(true);

    // Forward row-count changes so QML can bind to `.count` exactly the same
    // way it does on DeviceListModel.
    connect(this, &QAbstractItemModel::rowsInserted, this,
            &DeviceListSortProxyModel::countChanged);
    connect(this, &QAbstractItemModel::rowsRemoved, this,
            &DeviceListSortProxyModel::countChanged);
    connect(this, &QAbstractItemModel::modelReset, this,
            &DeviceListSortProxyModel::countChanged);
    connect(this, &QAbstractItemModel::layoutChanged, this,
            &DeviceListSortProxyModel::countChanged);
}

void DeviceListSortProxyModel::setSourceModel(QAbstractItemModel *model)
{
    QSortFilterProxyModel::setSourceModel(model);
    applySort();
}

void DeviceListSortProxyModel::setSortRoleName(const QString &name)
{
    if (m_sortRoleName == name)
        return;
    m_sortRoleName = name;
    emit sortRoleNameChanged();
    applySort();
}

void DeviceListSortProxyModel::setSortOrder(Qt::SortOrder order)
{
    if (m_sortOrder == order)
        return;
    m_sortOrder = order;
    emit sortOrderChanged();
    applySort();
}

void DeviceListSortProxyModel::sortBy(const QString &roleName,
                                       Qt::SortOrder order)
{
    const bool roleChanged = (m_sortRoleName != roleName);
    const bool orderChanged = (m_sortOrder != order);
    if (!roleChanged && !orderChanged)
        return;

    m_sortRoleName = roleName;
    m_sortOrder = order;
    if (roleChanged) emit sortRoleNameChanged();
    if (orderChanged) emit sortOrderChanged();
    applySort();
}

void DeviceListSortProxyModel::applySort()
{
    if (!sourceModel())
        return;

    // Resolve the role name (e.g. "rssi") back to a role id via the source
    // model's roleNames hash. If the name is unknown leave the previous
    // sort role in place rather than silently clearing the sort.
    const QHash<int, QByteArray> names = sourceModel()->roleNames();
    for (auto it = names.constBegin(); it != names.constEnd(); ++it)
    {
        if (QString::fromUtf8(it.value()) == m_sortRoleName)
        {
            setSortRole(it.key());
            sort(0, m_sortOrder);
            return;
        }
    }
}

int DeviceListSortProxyModel::mapToSource(int proxyRow) const
{
    if (proxyRow < 0 || proxyRow >= rowCount())
        return -1;
    const QModelIndex proxyIdx = index(proxyRow, 0);
    const QModelIndex srcIdx = QSortFilterProxyModel::mapToSource(proxyIdx);
    return srcIdx.isValid() ? srcIdx.row() : -1;
}

int DeviceListSortProxyModel::rowForDeviceId(int id) const
{
    if (!sourceModel() || id <= 0)
        return -1;

    // Resolve "deviceId" to a role id once, then scan the proxy rows.
    int roleId = -1;
    const auto names = roleNames();
    for (auto it = names.constBegin(); it != names.constEnd(); ++it)
    {
        if (it.value() == "deviceId")
        {
            roleId = it.key();
            break;
        }
    }
    if (roleId < 0)
        return -1;

    for (int i = 0; i < rowCount(); ++i)
    {
        if (data(index(i, 0), roleId).toInt() == id)
            return i;
    }
    return -1;
}

QVariantList DeviceListSortProxyModel::detailFor(int proxyRow) const
{
    if (auto *src = qobject_cast<DeviceListModel *>(sourceModel()))
        return src->detailFor(mapToSource(proxyRow));
    return {};
}

QVariantList DeviceListSortProxyModel::rssiSeriesFor(int proxyRow) const
{
    if (auto *src = qobject_cast<DeviceListModel *>(sourceModel()))
        return src->rssiSeriesFor(mapToSource(proxyRow));
    return {};
}

QVariantList DeviceListSortProxyModel::rssiRawSeriesFor(int proxyRow) const
{
    if (auto *src = qobject_cast<DeviceListModel *>(sourceModel()))
        return src->rssiRawSeriesFor(mapToSource(proxyRow));
    return {};
}

qint64 DeviceListSortProxyModel::firstFrameMsFor(int proxyRow) const
{
    if (auto *src = qobject_cast<DeviceListModel *>(sourceModel()))
        return src->firstFrameMsFor(mapToSource(proxyRow));
    return 0;
}

bool DeviceListSortProxyModel::lessThan(const QModelIndex &left,
                                         const QModelIndex &right) const
{
    // Only the RSSI role needs special handling: pin the -999.0 sentinel to
    // the bottom in both directions. For everything else fall back to Qt's
    // default QVariant comparison, which is correct for double / QDateTime /
    // int / QString roles.
    if (sortRole() >= Qt::UserRole && m_sortRoleName == QStringLiteral("rssi"))
    {
        const double l = sourceModel()->data(left, sortRole()).toDouble();
        const double r = sourceModel()->data(right, sortRole()).toDouble();
        const bool lSentinel = (l <= DeviceListModel::kRssiSentinel);
        const bool rSentinel = (r <= DeviceListModel::kRssiSentinel);
        if (lSentinel && rSentinel) return false;   // equal — keep source order
        if (lSentinel) return false;                // left is "greatest"
        if (rSentinel) return true;                 // right is "greatest"
        return l < r;
    }

    // When sorting by identifier, group rows by protocol first (so a
    // 0x??<LAP> BR/EDR address never interleaves with a short BLE advertising
    // address), then by type, then apply Qt's default comparison for the
    // identifier within each group. Each level honors the sort direction.
    if (sortRole() >= Qt::UserRole && m_sortRoleName == QStringLiteral("identifier")) {
        const QString lp = sourceModel()->data(left, DeviceListModel::ProtoRole).toString();
        const QString rp = sourceModel()->data(right, DeviceListModel::ProtoRole).toString();
        if (lp != rp)
            return (m_sortOrder == Qt::AscendingOrder) ? (lp < rp) : (lp > rp);

        const QString lt = sourceModel()->data(left, DeviceListModel::TypeRole).toString();
        const QString rt = sourceModel()->data(right, DeviceListModel::TypeRole).toString();
        if (lt != rt)
            return (m_sortOrder == Qt::AscendingOrder) ? (lt < rt) : (lt > rt);
    }

    return QSortFilterProxyModel::lessThan(left, right);
}