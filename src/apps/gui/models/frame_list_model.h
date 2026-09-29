#ifndef FRAME_LIST_MODEL_H
#define FRAME_LIST_MODEL_H

#include <QAbstractListModel>
#include <QByteArray>
#include <QHash>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>
#include <QVector>

#include <qqmlintegration.h>

/**
 * @brief QML-facing list model backing the FrameListView table.
 *
 * Holds decoded-frame rows in a ring buffer capped at s_capacity entries
 * (oldest evicted FIFO). Roles mirror the existing FrameListView columns; the
 * raw bytes and a detail key/value list are retained per row so the Frame Info
 * and Hex panes can be populated when a row is selected.
 *
 * Incoming rows are batched: appendRow() only enqueues into m_pending and
 * flushPending() (on a ~150 ms coalescing timer) moves them into m_rows with
 * a single remove-range + single insert-range. Without this, a packet burst
 * emits begin/endInsertRows per frame and saturates the GUI event loop,
 * starving input handling in the sibling device list.
 */
class FrameListModel : public QAbstractListModel
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(int count READ count NOTIFY countChanged)

public:
    enum Roles {
        NoRole = Qt::UserRole + 1,
        TimeRole,
        RssiRole,
        ProtoRole,
        ChIdxRole,
        AddrRole,
        SrcRole,
        DstRole,
        TypeRole,
        InfoRole,
    };
    Q_ENUM(Roles)

    explicit FrameListModel(QObject *parent = nullptr);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    QHash<int, QByteArray> roleNames() const override;

    int count() const { return m_rows.size(); }

    /// Append a decoded row (called from the controller's queued slot).
    /// Batched: enqueued and flushed on a coalescing timer (see flushPending).
    Q_INVOKABLE void appendRow(const QVariantMap &row);
    /// Clear all rows (including queued-but-unflushed rows).
    Q_INVOKABLE void clear();

    /// Current row index for a frame number, or -1 if it has been evicted.
    /// Frame numbers are stable across ring-buffer evictions; indices are not.
    Q_INVOKABLE int indexForNo(qulonglong no) const;

    /// Detail key/value pairs for the selected row (Frame Info pane).
    Q_INVOKABLE QVariantList detailFor(int index) const;
    /// Hex-dump rows {offset, hex, ascii} for the selected row, built from raw bytes.
    Q_INVOKABLE QVariantList hexFor(int index) const;

signals:
    void countChanged();

private:
    struct Row
    {
        unsigned long no = 0;
        QString time;
        QString rssi;
        QString proto;
        unsigned int chIdx = 0;
        QString addr;
        QString src;
        QString dst;
        QString type;
        QString info;
        QByteArray raw;
        QVariantList detail;
    };

    static constexpr int s_capacity = 5000;
    // Coalescing window for incoming rows (ms) and cap on the pending queue.
    // Pending rows are invisible to rowCount/data until flushed (≤1 window of
    // delay); the cap keeps a sustained burst from growing memory unboundedly
    // (the frame list is a view — oldest unflushed rows are dropped first).
    static constexpr int s_flushIntervalMs = 150;
    static constexpr int s_pendingCap = 1000;

    QVector<Row> m_rows;
    QVector<Row> m_pending;
    QTimer m_flushTimer;
    unsigned long m_nextNo = 1;

    static QVariantList buildHex(const QByteArray &raw);

  private slots:
    void flushPending();
};

#endif // FRAME_LIST_MODEL_H
