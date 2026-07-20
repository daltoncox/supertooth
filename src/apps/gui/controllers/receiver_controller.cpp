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
    if (m_thread && m_thread->joinable())
        m_thread->join();
    backend_session_destroy(m_session);
}

void ReceiverController::setRunning(bool running)
{
    if (m_running == running)
        return;
    m_running = running;
    emit runningChanged();
}

bool ReceiverController::start(int inputType, const QString &deviceId, int sessionType)
{
    qCInfo(lcSession).nospace().noquote()
        << "start() requested: inputType=" << inputType
        << " deviceId=\"" << deviceId << "\""
        << " sessionType=" << sessionType
        << " running=" << m_running;

    if (m_running)
    {
        qCWarning(lcSession) << "start() ignored: already running";
        return true;
    }

    if (!m_session)
    {
        qCWarning(lcSession) << "start() aborted: no session";
        emit errorOccurred(tr("No receiver session available."));
        return false;
    }

    if (inputType == BACKEND_INPUT_FILE)
    {
        qCWarning(lcSession) << "start() aborted: file replay not supported";
        emit errorOccurred(tr("File replay is not yet supported."));
        return false;
    }

    /* Join any previous (finished) thread before starting a new one. */
    if (m_thread && m_thread->joinable())
        m_thread->join();

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
        << ")";

    m_thread = std::make_unique<std::thread>(
        [session, sessionType, inputType, idStr, self]() {
            QByteArray idBytes = idStr.toUtf8();
            const char *idPtr = idBytes.isEmpty() ? nullptr
                                                  : idBytes.constData();
            int result;
            if (sessionType == BACKEND_SESSION_BLE)
            {
                qCInfo(lcSession) << "worker: entering backend_session_run_ble";
                result = backend_session_run_ble(session, BACKEND_BLE_CH37,
                                                 inputType, idPtr,
                                                 &rowTrampoline, self);
                qCInfo(lcSession) << "worker: backend_session_run_ble returned"
                                  << result;
            }
            else if (sessionType == BACKEND_SESSION_BREDR)
            {
                qCInfo(lcSession) << "worker: entering backend_session_run_bredr";
                result = backend_session_run_bredr(session, inputType, idPtr,
                                                   &rowTrampoline, self);
                qCInfo(lcSession) << "worker: backend_session_run_bredr returned"
                                  << result;
            }
            else
            {
                qCInfo(lcSession) << "worker: entering backend_session_run_hybrid";
                result = backend_session_run_hybrid(session, inputType, idPtr,
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
    /* The worker's onFinish() handler (queued to the main thread) joins the
     * thread once run_ble returns. */
}

void ReceiverController::onFinish(int result)
{
    qCInfo(lcSession) << "onFinish: result=" << result;
    if (m_thread && m_thread->joinable())
        m_thread->join();
    m_thread.reset();
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
