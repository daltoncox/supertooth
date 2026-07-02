#ifndef RECEIVER_CONTROLLER_H
#define RECEIVER_CONTROLLER_H

#include <QObject>
#include <QString>
#include <QVariantMap>

#include <memory>
#include <thread>

#include <qqmlintegration.h>

#include "../backend/backend_api.h"

/**
 * @brief QML-facing controller that owns a receiver session lifecycle and
 *        bridges decoded frames from the C worker thread into Qt signals.
 *
 * start() spawns a std::thread running the blocking backend_session_run_ble().
 * The C trampoline calls handleRow() on the worker thread; emitting
 * frameDecoded() from a non-Qt thread is delivered to Qt slots via a queued
 * connection (the controller lives in the main thread).
 */
class ReceiverController : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(bool running READ running NOTIFY runningChanged)

public:
    explicit ReceiverController(QObject *parent = nullptr);
    ~ReceiverController() override;

    bool running() const { return m_running; }

    /**
     * Start a BLE capture session.
     * @param inputType  BACKEND_INPUT_HACKRF (0) or BACKEND_INPUT_FILE (1).
     * @param deviceId   HackRF identifier, or empty for default device.
     * @param bleChannel BLE advertising channel (37/38/39). Defaults to 37.
     * @return true if the session was started (or already running).
     */
    Q_INVOKABLE bool start(int inputType, const QString &deviceId,
                           int bleChannel = 37);
    /** Request the running session to stop and join the worker thread. */
    Q_INVOKABLE void stop();

signals:
    void frameDecoded(const QVariantMap &row);
    void runningChanged();
    void errorOccurred(const QString &message);

private:
    static void rowTrampoline(const backend_ble_row_t *row, void *user);
    // Runs on the worker thread; builds a QVariantMap and emits frameDecoded().
    void handleRow(const backend_ble_row_t *row);
    void onFinish(int result);
    void setRunning(bool running);

    backend_session_t *m_session = nullptr;
    std::unique_ptr<std::thread> m_thread;
    bool m_running = false;
};

#endif // RECEIVER_CONTROLLER_H

