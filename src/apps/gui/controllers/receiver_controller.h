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
 * start() spawns a std::thread running the blocking backend_session_run_*
 * function selected by @p sessionType (hybrid / BLE / BR/EDR). The C
 * trampoline calls handleRow() on the worker thread; emitting frameDecoded()
 * from a non-Qt thread is delivered to Qt slots via a queued connection (the
 * controller lives in the main thread).
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
     * Start a capture session.
     * @param inputType   BACKEND_INPUT_HACKRF (0) or BACKEND_INPUT_FILE (1).
     * @param deviceId    HackRF identifier, or empty for default device.
     * @param sessionType BACKEND_SESSION_HYBRID (0), _BLE (1) or _BREDR (2).
     * @param enforceCrc  true to drop BLE frames whose CRC fails (applies to
     *                   BLE and hybrid sessions; ignored for BR/EDR-only).
     * @param channelCount  Channel processors covering the capture window,
     *                   in the session's native grid: BR/EDR channels for
     *                   BR/EDR/hybrid sessions (even 2..20 BR/EDR-grid, odd
     *                   1..19 LE-grid), LE RF channels for BLE sessions
     *                   (1..10). Ignored units aside, always >= 1.
     * @param bottomChannel Lowest channel of the window in the session's
     *                   native grid: BR/EDR channel 0..78 for BR/EDR/hybrid,
     *                   LE RF channel 0..39 for BLE.
     * @param leGrid      BACKEND_GRID_BREDR (0) or BACKEND_GRID_LE (1);
     *                   hybrid only.
     * @param bleChannel  Advertising channel 37/38/39, or 0 = none inside
     *                   the window (hybrid BLE worker idles). Hybrid only;
     *                   BLE sessions decode whichever advertising channels
     *                   fall inside their LE window.
     * @return true if the session was started (or already running).
     */
    Q_INVOKABLE bool start(int inputType, const QString &deviceId,
                           int sessionType, bool enforceCrc,
                           int channelCount, int bottomChannel,
                           int leGrid, int bleChannel);
    /** Request the running session to stop and join the worker thread. */
    Q_INVOKABLE void stop();

signals:
    void frameDecoded(const QVariantMap &row);
    void runningChanged();
    void errorOccurred(const QString &message);

private:
    static void rowTrampoline(const backend_row_t *row, void *user);
    // Runs on the worker thread; builds a QVariantMap and emits frameDecoded().
    void handleRow(const backend_row_t *row);
    // Runs on the session worker thread the moment capture ends, before the
    // blocking radio teardown; flips the UI to "stopped" without blocking.
    static void stopTrampoline(void *user);
    void onFinish(int result);
    void setRunning(bool running);

    backend_session_t *m_session = nullptr;
    std::unique_ptr<std::thread> m_thread;
    bool m_running = false;
};

#endif // RECEIVER_CONTROLLER_H

