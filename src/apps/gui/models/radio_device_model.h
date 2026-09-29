#ifndef RADIO_DEVICE_MODEL_H
#define RADIO_DEVICE_MODEL_H

#include <QStringListModel>

#include <qqmlintegration.h>

/**
 * QML-facing model exposing the identifiers of available radio devices
 * for a given BACKEND_INPUT_* value (see backend_api.h).
 *
 * Device enumeration and per-device window sizing go through the shared
 * backend facade (which wraps radio_list_devices() and the session default
 * helpers), so this model never mirrors radio_device_type_t or hardcoded
 * channel limits — the same logic the CLI uses.
 */
class RadioDeviceModel : public QStringListModel
{
    Q_OBJECT
    QML_ELEMENT

public:
    explicit RadioDeviceModel(QObject *parent = nullptr);

    /**
     * Repopulate the model for the given BACKEND_INPUT_* value. Safe to
     * call repeatedly; a no-op when the input type has not changed unless
     * @p force is true (e.g. replugged hardware).
     */
    Q_INVOKABLE void refresh(int inputType, bool force = false);

    /**
     * Return the row whose identifier matches @p identifier, or -1.
     */
    Q_INVOKABLE int indexFromIdentifier(const QString &identifier) const;

    /**
     * Live radio input types compiled into this build, as a list of maps:
     * { "inputType": BACKEND_INPUT_*, "label": "HackRF"/"bladeRF" }.
     * Radios disabled via -DENABLE_HACKRF=OFF / -DENABLE_BLADERF=OFF are
     * excluded. File replay is never included.
     */
    Q_INVOKABLE QVariantList availableInputTypes() const;

    /** Default/max capture-window sizes for an input type (shared with
     *  the CLI via session_default_*_count). */
    Q_INVOKABLE unsigned int defaultBredrCount(int inputType) const;
    Q_INVOKABLE unsigned int defaultBleCount(int inputType) const;
    Q_INVOKABLE unsigned int maxBredrCount(int inputType) const;
    Q_INVOKABLE unsigned int maxBleCount(int inputType) const;

    /**
     * Valid capture-window channel counts for an input type, ascending
     * (same validity the CLI enforces — the lane-split set clipped to the
     * radio's ceiling). The spectrum snaps resize/drag counts to these so
     * the GUI can never select a count the session would reject or
     * silently snap away from.
     */
    Q_INVOKABLE QVariantList supportedBredrCounts(int inputType) const;
    Q_INVOKABLE QVariantList supportedBleCounts(int inputType) const;

private:
    int m_inputType = -1;
};

#endif // RADIO_DEVICE_MODEL_H
