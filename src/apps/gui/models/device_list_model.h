#ifndef DEVICE_LIST_MODEL_H
#define DEVICE_LIST_MODEL_H

#include <QAbstractListModel>
#include <QHash>
#include <QPair>
#include <QPointF>
#include <QString>
#include <QVariantList>
#include <QVector>

#include <qqmlintegration.h>

/**
 * @brief QML-facing list model backing the DeviceListView table.
 *
 * Each row represents a single observed radio entity: a BLE device, a
 * broken-out BR/EDR device (clock known), or an un-broken-out BR/EDR
 * piconet (clock unknown) — in which case the device column reads
 * "piconet".
 *
 * The model is structural-only for now: the constructor seeds a handful
 * of fake rows so the Devices view framing can be exercised without a
 * live capture. The shape mirrors FrameListModel so wiring it to the
 * real pipeline later is mechanical.
 */
class DeviceListModel : public QAbstractListModel
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(int count READ count NOTIFY countChanged)

public:
    enum Roles {
        RssiRole = Qt::UserRole + 1,        // averaged over the last second
        ProtoRole,                          // "BR/EDR" or "BLE"
        AddrRole,                            // piconet address or access address
        DeviceRole,                          // source equivalent; "piconet" if un-broken-out
        LastSeenRole,
        PacketsSeenRole,
        PacketRateRole,                      // packets in the last second
    };
    Q_ENUM(Roles)

    explicit DeviceListModel(QObject *parent = nullptr);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override;

    int count() const { return m_rows.size(); }

    /// Placeholder parity with FrameListModel — append/replace a device row.
    Q_INVOKABLE void appendRow(const QVariantMap &row);
    /// Clear all rows.
    Q_INVOKABLE void clear();

    /// Detail key/value pairs for the selected device (Device Info pane).
    Q_INVOKABLE QVariantList detailFor(int index) const;
    /// Averaged-RSSI time series for the selected device, as a list of
    /// QPointF so it can be fed straight to a QtGraphs LineSeries.
    Q_INVOKABLE QVariantList rssiSeriesFor(int index) const;

signals:
    void countChanged();

private:
    struct Row
    {
        QString rssi;            // e.g. "-42 dBm" (1s average)
        QString proto;           // "BR/EDR" or "BLE"
        QString addr;            // piconet address or BLE access address
        QString device;          // source name; "piconet" if un-broken-out
        QString lastSeen;
        unsigned long packetsSeen = 0;
        QString packetRate;      // e.g. "12/s"
        QVariantList detail;             // key/value pairs for the info pane
        QVariantList rssiSeries;         // list of QPointF for the chart
    };

    static Row makeSeedRow(const QString &rssi,
                           const QString &proto,
                           const QString &addr,
                           const QString &device,
                           const QString &lastSeen,
                           unsigned long packets,
                           const QString &rate,
                           double baseRssi,
                           double rssiJitter,
                           const QList<QPair<QString, QString>> &extraDetail);

    QVector<Row> m_rows;
};

#endif // DEVICE_LIST_MODEL_H