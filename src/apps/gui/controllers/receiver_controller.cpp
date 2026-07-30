#include "receiver_controller.h"

#include <QByteArray>
#include <QDebug>
#include <QLoggingCategory>
#include <QMetaObject>
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
}

ReceiverController::~ReceiverController()
{
    stop();
    /* Detach rather than join: the worker thread tears the session down on its
     * own (session_run() calls session_destroy()), so joining here would block
     * the GUI thread on device shutdown during app teardown. The wrapper is
     * released by the running onFinish() cleanup thread (if any); if the
     * controller is destroyed before a stop was requested, the worker keeps
     * the session valid until it finishes. */
    if (m_thread && m_thread->joinable())
        m_thread->detach();
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
                               int leGrid, int bleChannel)
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

    /* Detach any previous (still-finishing) worker thread rather than joining
     * it on the GUI thread; its teardown is owned by the prior onFinish()
     * cleanup thread. The new run gets a fresh thread below. */
    if (m_thread && m_thread->joinable())
        m_thread->detach();
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
            /* Flip the UI to "stopped" the instant capture ends, before the
             * (potentially blocking) radio teardown runs on this thread. */
            backend_session_set_stopped_callback(session, &stopTrampoline, self);
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
    return true;
}

void ReceiverController::stop()
{
    qCInfo(lcSession) << "stop() requested; running=" << m_running;
    if (m_session)
        backend_session_request_stop(m_session);
    /* The UI flips to "stopped" via stopTrampoline (queued from the worker
     * thread) before the blocking radio teardown; onFinish() then performs the
     * join off the GUI thread so the UI never freezes. */
}

void ReceiverController::onFinish(int result)
{
    qCInfo(lcSession) << "onFinish: result=" << result;

    /* Capture the session/thread handles we own so a subsequent start() that
     * reassigns m_session/m_thread cannot be destroyed by this cleanup. */
    backend_session_t *old_session = m_session;
    std::unique_ptr<std::thread> old_thread = std::move(m_thread);
    m_session = nullptr;
    m_thread.reset();

    /* Perform teardown join + session destruction on a separate thread so the
     * GUI/main thread is never blocked waiting on device shutdown. The worker
     * thread has already torn the session down inside session_run(); here we
     * only join it and free the wrapper. */
    std::thread cleanup([old_session, old_thread = std::move(old_thread), this, result]() mutable {
        if (old_thread && old_thread->joinable())
            old_thread->join();
        backend_session_destroy(old_session);
        QMetaObject::invokeMethod(this, [this, result]() {
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
        }, Qt::QueuedConnection);
    });
    cleanup.detach();
}

/* static */ void ReceiverController::stopTrampoline(void *user)
{
    auto *self = static_cast<ReceiverController *>(user);
    if (self)
    {
        /* Runs on the session worker thread, just before radio teardown.
         * Queue the UI update to the main thread so the icon flips
         * immediately and does not wait on device shutdown. */
        QMetaObject::invokeMethod(self, &ReceiverController::setRunning,
                                  Qt::QueuedConnection, false);
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
