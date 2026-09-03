#include "receiver_controller.h"

#include "bredr_bitstream_decoder.h"

#include <QByteArray>
#include <QDebug>
#include <QLoggingCategory>
#include <QMetaObject>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>

#include <cstring>

Q_LOGGING_CATEGORY(lcSession, "supertooth.session")

ReceiverController::ReceiverController(QObject *parent)
    : QObject(parent)
{
    m_session = backend_session_create();
    if (!m_session)
    {
        qCWarning(lcSession) << "Failed to create receiver session";
        emit errorOccurred(tr("Failed to create receiver session."));
    }
    else
    {
        qCInfo(lcSession) << "Receiver session created";
    }

    m_poll.setInterval(250);   /* 4 Hz device-list poll */
    connect(&m_poll, &QTimer::timeout, this, &ReceiverController::pollDevices);
}

ReceiverController::~ReceiverController()
{
    stop();
    /* Join the worker rather than detaching it. A detached thread keeps
     * calling back into `this` (rowTrampoline -> handleRow, and the queued
     * onFinish) long after the controller is destroyed, which segfaulted on
     * app exit. stop() was just requested, so session_run()'s 50 ms stop
     * poll returns promptly; the blocking radio teardown runs on the worker
     * thread inside session_run(), so this join never blocks the GUI thread
     * for long. */
    if (m_thread && m_thread->joinable())
        m_thread->join();
    m_thread.reset();
    if (m_session)
        backend_session_destroy(m_session);
    m_session = nullptr;
}

void ReceiverController::setRunning(bool running)
{
    if (m_running == running)
        return;
    m_running = running;
    emit runningChanged();
}

bool ReceiverController::start(int inputType, const QString &deviceId,
                                int sessionType, bool enforceCrc,
                                int channelCount, int bottomChannel,
                                int leGrid, int bleChannel, int acErrors)
{
    qCInfo(lcSession).nospace().noquote()
        << "start() requested: inputType=" << inputType
        << " deviceId=\"" << deviceId << "\""
        << " sessionType=" << sessionType
        << " enforceCrc=" << enforceCrc
        << " channelCount=" << channelCount
        << " bottomChannel=" << bottomChannel
        << " leGrid=" << leGrid
        << " bleChannel=" << bleChannel
        << " acErrors=" << acErrors
        << " running=" << m_running;

    if (m_running)
    {
        qCWarning(lcSession) << "start() ignored: already running";
        return true;
    }

    if (!m_session)
    {
        /* A previous run's onFinish() cleanup thread releases the old session;
         * recreate a fresh one for this run. */
        m_session = backend_session_create();
        if (!m_session)
        {
            qCWarning(lcSession) << "start() aborted: no session";
            emit errorOccurred(tr("No receiver session available."));
            return false;
        }
    }

    if (inputType == BACKEND_INPUT_FILE)
    {
        qCWarning(lcSession) << "start() aborted: file replay not supported";
        emit errorOccurred(tr("File replay is not yet supported."));
        return false;
    }

    /* Apply the global access-code error tolerance before streaming begins.
     * The BR/EDR bitstream decoder is the sole access-code acceptance gate. */
    bredr_bitstream_decoder_set_global_max_ac_errors((uint8_t)acErrors);

    /* A prior run's worker is always joined and released by onFinish() before
     * running() flips back to false, so nothing should be pending here. Guard
     * defensively: if a thread is somehow still joinable, session_run() has
     * already returned (otherwise running() would be true and start() would
     * have bailed), so this join is immediate. */
    if (m_thread && m_thread->joinable())
        m_thread->join();
    m_thread.reset();

    QString idStr = deviceId;
    backend_session_t *session = m_session;
    ReceiverController *self = this;

    const char *modeName =
        (sessionType == BACKEND_SESSION_BLE)    ? "ble"
        : (sessionType == BACKEND_SESSION_BREDR) ? "bredr"
        : "hybrid";
    qCInfo(lcSession).nospace()
        << "spawning receiver worker thread (mode=" << modeName
        << ", device=" << (idStr.isEmpty() ? QString("<default>") : idStr)
        << ", channels=" << channelCount
        << ", bottom=" << bottomChannel
        << ", grid=" << (leGrid == BACKEND_GRID_LE ? "LE" : "BR/EDR")
        << ", bleCh=" << bleChannel << ")";

    m_thread = std::make_unique<std::thread>(
        [session, sessionType, inputType, idStr, enforceCrc,
         channelCount, bottomChannel, leGrid, bleChannel, self]() {
            QByteArray idBytes = idStr.toUtf8();
            const char *idPtr = idBytes.isEmpty() ? nullptr
                                                  : idBytes.constData();
            /* Note: no stopped-callback is registered. running() stays true
             * until onFinish() runs (which is queued only after session_run()
             * has torn the radio down), so a new start() can never race a
             * still-finishing worker/session. */
            int result;
            if (sessionType == BACKEND_SESSION_BLE)
            {
                /* LE-window session: channelCount/bottomChannel arrive in
                 * LE RF units (count 1..10, bottom 0..39). */
                qCInfo(lcSession) << "worker: entering backend_session_run_ble";
                result = backend_session_run_ble(session,
                                                 (unsigned int)bottomChannel,
                                                 (unsigned int)channelCount,
                                                 inputType, idPtr,
                                                 enforceCrc ? 1 : 0,
                                                 &rowTrampoline, self);
                qCInfo(lcSession) << "worker: backend_session_run_ble returned"
                                  << result;
            }
            else if (sessionType == BACKEND_SESSION_BREDR)
            {
                qCInfo(lcSession) << "worker: entering backend_session_run_bredr";
                result = backend_session_run_bredr(session,
                                                   (unsigned int)channelCount,
                                                   (unsigned int)bottomChannel,
                                                   inputType, idPtr,
                                                   &rowTrampoline, self);
                qCInfo(lcSession) << "worker: backend_session_run_bredr returned"
                                  << result;
            }
            else
            {
                qCInfo(lcSession) << "worker: entering backend_session_run_hybrid";
                result = backend_session_run_hybrid(session,
                                                    (unsigned int)channelCount,
                                                    (unsigned int)bottomChannel,
                                                    leGrid,
                                                    (uint8_t)bleChannel,
                                                    inputType, idPtr,
                                                    enforceCrc ? 1 : 0,
                                                    &rowTrampoline, self);
                qCInfo(lcSession) << "worker: backend_session_run_hybrid returned"
                                  << result;
            }
            QMetaObject::invokeMethod(self, [self, result]() {
                self->onFinish(result);
            }, Qt::QueuedConnection);
        });

    setRunning(true);
    m_poll.start();
    return true;
}

void ReceiverController::stop()
{
    qCInfo(lcSession) << "stop() requested; running=" << m_running;
    if (m_session)
        backend_session_request_stop(m_session);
    /* The worker observes the stop request within its 50 ms poll, tears the
     * radio down inside session_run(), then posts onFinish(); running() stays
     * true until that queued onFinish() runs on the main thread. */
}

void ReceiverController::onFinish(int result)
{
    qCInfo(lcSession) << "onFinish: result=" << result;

    m_poll.stop();

    /* Capture the session/thread handles we own so a subsequent start() that
     * reassigns m_session/m_thread cannot be destroyed by this cleanup. */
    backend_session_t *old_session = m_session;
    std::unique_ptr<std::thread> old_thread = std::move(m_thread);
    m_session = nullptr;
    m_thread.reset();

    /* The worker has already returned from backend_session_run_*(), which
     * performed the blocking radio teardown on its own thread before posting
     * this callback. Joining is therefore immediate, and doing it here on the
     * main thread keeps `this` alive until every worker callback has drained —
     * no detached cleanup thread, so nothing outlives the controller. */
    if (old_thread && old_thread->joinable())
        old_thread->join();
    if (old_session)
        backend_session_destroy(old_session);

    setRunning(false);
    if (result != 0)
    {
        qCWarning(lcSession) << "receiver failed with code" << result;
        emit errorOccurred(tr("Receiver failed (code %1).").arg(result));
    }
    else
    {
        qCInfo(lcSession) << "receiver session stopped cleanly";
    }
}

/* static */ void ReceiverController::rowTrampoline(const backend_row_t *row,
                                                    void *user)
{
    auto *self = static_cast<ReceiverController *>(user);
    if (self)
        self->handleRow(row);
}

void ReceiverController::handleRow(const backend_row_t *row)
{
    QVariantMap map;
    map.insert(QStringLiteral("no"),    QVariant::fromValue(row->no));
    map.insert(QStringLiteral("time"),  QString::fromUtf8(row->time));
    map.insert(QStringLiteral("rssiDb"), QVariant::fromValue(row->rssi_db));
    map.insert(QStringLiteral("proto"), QString::fromUtf8(row->proto));
    map.insert(QStringLiteral("chIdx"), QVariant::fromValue(row->ch_idx));
    map.insert(QStringLiteral("addr"),  QString::fromUtf8(row->addr));
    map.insert(QStringLiteral("src"),   QString::fromUtf8(row->src));
    map.insert(QStringLiteral("dst"),   QString::fromUtf8(row->dst));
    map.insert(QStringLiteral("type"),  QString::fromUtf8(row->type));
    map.insert(QStringLiteral("addrType"), QString::fromUtf8(row->addr_type));
    map.insert(QStringLiteral("info"),  QString::fromUtf8(row->info));

    QByteArray raw(reinterpret_cast<const char *>(row->raw),
                   (int)row->raw_len);
    map.insert(QStringLiteral("rawBytes"), raw);

    QVariantList detail;
    detail.reserve((int)row->detail_count);
    for (unsigned int i = 0u; i < row->detail_count; i++)
    {
        QVariantMap pair;
        pair.insert(QStringLiteral("field"),
                    QString::fromUtf8(row->detail_keys[i]));
        pair.insert(QStringLiteral("value"),
                    QString::fromUtf8(row->detail_vals[i]));
        detail.append(pair);
    }
    map.insert(QStringLiteral("detail"), detail);

    emit frameDecoded(map);
}

void ReceiverController::pollDevices()
{
    if (!m_session || !m_running)
        return;

    /* The promoted piconet store can now hold 512 entries; combined with the
     * other device/piconet registries the total entity count can exceed the
     * old 256 buffer, so allocate on the heap to avoid truncation. */
    const size_t cap = 4096u;
    backend_entity_t *buf = (backend_entity_t *)calloc(cap, sizeof(backend_entity_t));
    if (!buf)
        return;
    size_t count = backend_session_poll_entities(m_session, buf, cap);
    if (count == 0u)
    {
        emit devicesUpdated(QVariantList());
        free(buf);
        return;
    }

    QVariantList list;
    list.reserve((int)count);
    for (size_t i = 0u; i < count; i++)
    {
        const backend_entity_t *e = &buf[i];
        QVariantMap m;
        m.insert(QStringLiteral("id"), (int)e->id);
        m.insert(QStringLiteral("proto"), QString::fromUtf8(e->proto));
        m.insert(QStringLiteral("device"), QString::fromUtf8(e->device));
        m.insert(QStringLiteral("addr"), QString::fromUtf8(e->addr));
        m.insert(QStringLiteral("addrType"), QString::fromUtf8(e->addr_type));
        m.insert(QStringLiteral("displayName"), QString::fromUtf8(e->name));
        m.insert(QStringLiteral("manufacturer"), QString::fromUtf8(e->manufacturer));
        m.insert(QStringLiteral("rssiDb"), (double)e->rssi_db);
        m.insert(QStringLiteral("rssiValid"), e->rssi_valid ? true : false);
        m.insert(QStringLiteral("firstSeenMs"), (qlonglong)e->first_seen_ms);
        m.insert(QStringLiteral("lastSeenMs"), (qlonglong)e->last_seen_ms);
        m.insert(QStringLiteral("packetsSeen"), (qulonglong)e->total_packets);
        m.insert(QStringLiteral("packetRate"), (int)e->packet_rate);
        m.insert(QStringLiteral("crcInit"), (qulonglong)e->crc_init);
        m.insert(QStringLiteral("crcInitConfirmed"), e->crc_init_confirmed ? true : false);
        m.insert(QStringLiteral("crcInitCandidates"), (int)e->crc_init_candidates);
        list.append(m);
    }
    emit devicesUpdated(list);
    free(buf);
}
