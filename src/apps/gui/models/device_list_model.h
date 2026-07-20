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
 * Each row represents a single observed radio entity: a BLE device, a
 * broken-out BR/EDR device (clock known), or an un-broken-out BR/EDR
 * piconet (clock unknown) — in which case the device column reads
 * "piconet".
 *
 * The model aggregates frames pushed via onFrameDecoded() by (proto, addr,
 * device). For each device it tracks:
 *   - a 1-second trailing window of per-frame RSSI samples (floats),
 *   - a longer-lived smoothed-average series for the chart,
 *   - cumulative packet count, periodic packet rate, last-seen time.
 *
 * A 1 Hz QTimer tick re-renders the displayed average/rate/last-seen; this
 * is independent of frame arrivals so an idle device's "packet rate"
 * correctly decays to 0. The displayed RSSI is the mean of the 1s trailing
 * window; when the window empties out (no frames in the last second) the
 * last computed average is held until new frames arrive.
 *
 * RSSI is carried as a float all the way through from rx_metadata_t to
 * this model; it is only formatted to a string at data() read time.
 */
class DeviceListModel : public QAbstractListModel
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(int count READ count NOTIFY countChanged)

public:
    enum Roles {
        RssiRole = Qt::UserRole + 1,
        ProtoRole,
        AddrRole,
        DeviceRole,
        LastSeenRole,
        PacketsSeenRole,
        PacketRateRole,
    };
    Q_ENUM(Roles)

    explicit DeviceListModel(QObject *parent = nullptr);
    ~DeviceListModel() override = default;

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override;

    int count() const { return m_rows.size(); }

    /// Feed a decoded frame (proto/addr/src/dst/rssiDb/detail) into the
    /// aggregator. Finds-or-creates the relevant device row and folds the
    /// frame's sample into the trailing 1s window. Called from the
    /// controller's queued slot, always on the main thread.
    Q_INVOKABLE void onFrameDecoded(const QVariantMap &frame);

    /// Clear all rows (called when a new capture starts).
    Q_INVOKABLE void clear();

    /// Detail key/value pairs for the selected device (Device Info pane),
    /// rebuilt live from current row state.
    Q_INVOKABLE QVariantList detailFor(int index) const;
    /// Smoothed-average RSSI time series for the selected device, as a list
    /// of QPointF that can be fed straight to a QtGraphs LineSeries.
    Q_INVOKABLE QVariantList rssiSeriesFor(int index) const;
    /// Raw per-frame RSSI samples still inside the chart history window, for
    /// the faint scatter series.
    Q_INVOKABLE QVariantList rssiRawSeriesFor(int index) const;
    /// Wall-clock ms of the first frame for this device — the chart x=0
    /// anchor. Returns 0 if the index is invalid.
    Q_INVOKABLE qint64 firstFrameMsFor(int index) const;

signals:
    void countChanged();

private:
    struct Row
    {
        QString proto;
        QString addr;
        QString device;                 // resolved: BLE src / "Central" / "LT_ADDR N" / "piconet" / "Unknown"

        double rssiDb = qQNaN();        // displayed 1s avg; held when idle
        double lastFrameRssiDb = qQNaN();// most recent raw sample (used for series tail)

        qint64 firstFrameMs = 0;        // anchor for chart x-axis (t=0)
        qint64 lastSeenMs = 0;
        unsigned long packetsSeen = 0;
        int packetsThisSecond = 0;      // reset each tick
        int lastRate = 0;               // rate reported by the most recent tick

        QVector<QPair<qint64, double>> samples; // (t_ms, rssi_db) in trailing 1s
        QVector<QPointF> avgSeries;     // smooth line: (seconds, dB)
        QVector<QPointF> rawSeries;     // faint per-frame points: (seconds, dB)

        QVariantList lastFrameDetail;   // {field, value} pairs from the most recent frame
    };

    void tickRows();
    static QString makeKey(const QString &proto, const QString &addr,
                           const QString &device);
    static QString deviceLabelFor(const QString &proto, const QString &src,
                           const QString &dst);
    void recomputeAverage(Row &r, qint64 now);
    void evictWindow(Row &r, qint64 now);
    void evictChartHistory(Row &r, qint64 now);
    void removeRow(int index);

    static constexpr int kWindowMs = 1000;
    static constexpr int kChartHistoryMs = 300 * 1000;   // 5 min (max range)
    static constexpr int kMaxSeriesPoints = 12000;       // backstop cap

    QVector<Row> m_rows;
    QHash<QString, int> m_indexByKey;
    QSet<QString> m_brokenOutPiconets;   // LAP-keyed; "BR/EDR\x1F<lap>"
    QTimer m_tick;
};

#endif // DEVICE_LIST_MODEL_H