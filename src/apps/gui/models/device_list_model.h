#ifndef DEVICE_LIST_MODEL_H
#define DEVICE_LIST_MODEL_H

#include <QAbstractListModel>
#include <QHash>
#include <QPointF>
#include <QSet>
#include <QString>
#include <QTimer>
#include <QVariantList>
#include <QVector>

#include <qqmlintegration.h>

/**
 * @brief QML-facing list model backing the DeviceListView table.
 *
 * Each row is a single observed radio entity: a BLE advertiser, a BR/EDR
 * piconet member (master or a specific slave slot), a BR/EDR piconet, or an
 * LE connection. Rows are produced by the core trackers and delivered via
 * setRows() on a 4 Hz poll driven by ReceiverController (which calls
 * backend_session_poll_entities). The core supplies a stable entity id, a
 * 1-second average RSSI, and first/last-seen timestamps.
 *
 * The displayed RSSI (rssiDb) is the core's rolling 1-second average. The
 * RSSI chart shows that average sampled at the poll rate (the faint raw
 * per-frame series is no longer collected).
 *
 * roles exposed via data() return comparable primitive values so a
 * QSortFilterProxyModel can sort them numerically/lexically:
 *   - RssiRole      double; -999.0 sentinel means "no signal yet"
 *   - FirstSeenRole QDateTime; invalid means "never seen"
 *   - LastSeenRole  QDateTime; invalid means "never seen"
 *   - PacketRateRole int (packets/second, derived from the poll delta)
 */
class DeviceListModel : public QAbstractListModel
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(int count READ count NOTIFY countChanged)
    Q_PROPERTY(QString sortRoleName READ sortRoleName WRITE setSortRoleName
                   NOTIFY sortRoleNameChanged)
    Q_PROPERTY(Qt::SortOrder sortOrder READ sortOrder WRITE setSortOrder
                   NOTIFY sortOrderChanged)

public:
    enum Roles {
        RssiRole = Qt::UserRole + 1,
        ProtoRole,
        TypeRole,
        AddrRole,
        DeviceRole,
        IdentifierRole,
        FirstSeenRole,
        LastSeenRole,
        PacketsSeenRole,
        PacketRateRole,
        DeviceIdRole,
    };
    Q_ENUM(Roles)

    explicit DeviceListModel(QObject *parent = nullptr);
    ~DeviceListModel() override = default;

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override;

    int count() const { return m_rows.size(); }

    QString sortRoleName() const { return m_sortRoleName; }
    void setSortRoleName(const QString &name);

    Qt::SortOrder sortOrder() const { return m_sortOrder; }
    void setSortOrder(Qt::SortOrder order);

    Q_INVOKABLE void sortBy(const QString &roleName, Qt::SortOrder order);

    Q_INVOKABLE int rowForDeviceId(int id) const;

    /// Sentinel for "no RSSI yet" (pinned to bottom of any RSSI sort).
    static constexpr double kRssiSentinel = -999.0;

    /// Replace the model contents with the polled entity list (4 Hz).
    Q_INVOKABLE void setRows(const QVariantList &rows);

    /// Clear all rows (called when a new capture starts).
    Q_INVOKABLE void clear();

    Q_INVOKABLE QVariantList detailFor(int index) const;
    /// Smoothed-average RSSI time series (1 s average sampled per poll).
    Q_INVOKABLE QVariantList rssiSeriesFor(int index) const;
    /// Raw per-frame RSSI series (no longer collected; returns empty).
    Q_INVOKABLE QVariantList rssiRawSeriesFor(int index) const;
    Q_INVOKABLE qint64 firstFrameMsFor(int index) const;

signals:
    void countChanged();
    void sortRoleNameChanged();
    void sortOrderChanged();

private:
    struct Row
    {
        int id = 0;                     // stable per-session identity (from core)
        QString proto;                  // "BR/EDR" / "LE"
        QString addr;                   // core addr_str
        QString device;                 // core label ("Central"/"LT_ADDR N"/...)
        QString addrType;               // BLE adv subtype / ""
        QString displayName;            // BLE local name (if any)
        QString manufacturer;           // BLE manufacturer (if any)

        double rssiDb = qQNaN();       // 1 s average
        int    rssiValid = 0;
        qint64 firstFrameMs = 0;       // chart x=0 anchor
        qint64 lastSeenMs = 0;
        qint64 firstSeenMs = 0;
        unsigned long packetsSeen = 0;
        int lastRate = 0;               // packets/sec (poll delta)

        uint32_t crcInit = 0;           // BLE connection CRCInit
        int crcInitConfirmed = 0;       // 0 while still unconfirmed
        int crcInitCandidates = 0;      // distinct CRCInit candidates accumulated

        QVector<QPointF> avgSeries;     // (seconds, dB)
    };

    static QString typeLabelFor(const QString &proto, const QString &device,
                                const QString &addrType);
    static QString identifierLabelFor(const Row &r);
    static QString formatLastSeen(qint64 ms);
    bool lessThan(const Row &a, const Row &b) const;
    QVariant sortKey(const Row &r) const;
    void maybeResort();
    void rebuildLookup();

    static constexpr int kMaxSeriesPoints = 600;   // ~150 s at 4 Hz

    QVector<Row> m_rows;
    QHash<int, int> m_rowById;          // stable device id -> display row
    QHash<int, qulonglong> m_prevPackets; // id -> last total_packets (for rate)

    QString m_sortRoleName = QStringLiteral("identifier");
    Qt::SortOrder m_sortOrder = Qt::AscendingOrder;
};

#endif // DEVICE_LIST_MODEL_H
