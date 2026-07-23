#ifndef DEVICE_LIST_SORT_PROXY_MODEL_H
#define DEVICE_LIST_SORT_PROXY_MODEL_H

#include <QSortFilterProxyModel>
#include <qqmlintegration.h>

/**
 * @brief Sort proxy sitting between DeviceListView and DeviceListModel.
 *
 * Exposes QML-settable sortRoleName + sortOrder so a click on a column
 * header can re-sort the table without touching the underlying aggregator.
 * setDynamicSortFilter(true) is on so the 1 Hz DeviceListModel::tickRows
 * dataChanged refresh re-orders idle-decaying RSSI/Rate rows automatically.
 *
 * lessThan() is overridden only to pin the RSSI "no signal" sentinel
 * (DeviceListModel::kRssiSentinel) to the bottom in either sort direction
 * — every other role compares via Qt's default QVariant ordering, which is
 * correct for the comparable primitive roles (double rssi, QDateTime
 * lastSeen, int packetRate, QString proto/identifier) exposed by
 * DeviceListModel. The sentinel value itself is shared from
 * DeviceListModel (see device_list_model.h) so the two can never drift.
 *
 * mapToSource(int) is exposed for parity, but QML mostly calls the
 * forwarding accessors (detailFor / rssiSeriesFor / rssiRawSeriesFor /
 * firstFrameMsFor) declared below, which accept proxy-row indices and
 * translate internally — the view does not need to know row spaces.
 */
class DeviceListSortProxyModel : public QSortFilterProxyModel
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(QString sortRoleName READ sortRoleName WRITE setSortRoleName
                   NOTIFY sortRoleNameChanged)
    Q_PROPERTY(Qt::SortOrder sortOrder READ sortOrder WRITE setSortOrder
                   NOTIFY sortOrderChanged)
    Q_PROPERTY(int count READ count NOTIFY countChanged)

public:
    explicit DeviceListSortProxyModel(QObject *parent = nullptr);
    int count() const { return rowCount(); }

    QString sortRoleName() const { return m_sortRoleName; }
    void setSortRoleName(const QString &name);

    Qt::SortOrder sortOrder() const { return m_sortOrder; }
    void setSortOrder(Qt::SortOrder order);

    /// Convenience for QML: set both role and order in one call (one
    /// re-sort rather than two). Tolerates an unknown role name by leaving
    /// the previous sort state intact.
    Q_INVOKABLE void sortBy(const QString &roleName, Qt::SortOrder order);

    /// Map a proxy row index to the corresponding DeviceListModel row so
    /// the chart/info accessors (which take source rows) get the right
    /// device after a re-sort. Returns -1 for an invalid proxy row.
    Q_INVOKABLE int mapToSource(int proxyRow) const;

    /// Resolve a stable device ID to the proxy row currently displaying it.
    /// Returns -1 if the device is not in the model (removed, cleared, or
    /// never existed). Used by the view to keep the highlighted/selected row
    /// following a device across sort-induced re-orderings rather than being
    /// tied to a row index that drifts when rows move.
    Q_INVOKABLE int rowForDeviceId(int id) const;

    // Forwarding wrappers for the DeviceListModel accessors. QML only ever
    // holds a handle to this proxy, so invokables declared on the source
    // model are invisible to it; these re-expose the same API and translate
    // the row index from proxy-space to source-space on the way through.
    // The view therefore never needs to know that a proxy sits in between.
    Q_INVOKABLE QVariantList detailFor(int proxyRow) const;
    Q_INVOKABLE QVariantList rssiSeriesFor(int proxyRow) const;
    Q_INVOKABLE QVariantList rssiRawSeriesFor(int proxyRow) const;
    Q_INVOKABLE qint64 firstFrameMsFor(int proxyRow) const;

signals:
    void sortRoleNameChanged();
    void sortOrderChanged();
    void countChanged();

protected:
    bool lessThan(const QModelIndex &left,
                  const QModelIndex &right) const override;

private:
    void applySort();   // resolves role name -> role id, then sort(0, order)

    QString m_sortRoleName = QStringLiteral("rssi");
    Qt::SortOrder m_sortOrder = Qt::DescendingOrder;
};

#endif // DEVICE_LIST_SORT_PROXY_MODEL_H