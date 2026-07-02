#ifndef RADIO_DEVICE_MODEL_H
#define RADIO_DEVICE_MODEL_H

#include <QStringListModel>

#include <qqmlintegration.h>

/**
 * QML-facing model exposing the identifiers of available radio devices
 * for a given input type (HackRF, File, ...).
 *
 * The model is backed by QStringListModel. Call refresh() when the user
 * switches input type or replugs hardware; the selected identifier can
 * be read from the bound ComboBox's currentText.
 *
 * Input-type indices mirror the Header.qml inputTypeSelector ComboBox:
 *   0 = HackRF
 *   1 = File  (no enumeration; model is cleared)
 */
class RadioDeviceModel : public QStringListModel
{
    Q_OBJECT
    QML_ELEMENT

public:
    explicit RadioDeviceModel(QObject *parent = nullptr);

    /**
     * Repopulate the model for the given input-type index. Safe to call
     * repeatedly; a no-op when the input type has not changed.
     */
    Q_INVOKABLE void refresh(int inputTypeIndex);

    /**
     * Return the row whose identifier matches @p identifier, or -1.
     */
    Q_INVOKABLE int indexFromIdentifier(const QString &identifier) const;

private:
    int m_inputTypeIndex = -1;
};

#endif // RADIO_DEVICE_MODEL_H