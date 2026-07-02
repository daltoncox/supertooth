#include "radio_device_model.h"

#include <QDebug>

/* Forward declarations of the C core API. The full radio_common.h header
 * pulls in C-only DSP headers (C99 `float complex`) that are not valid
 * C++, so we declare just the symbols we need here. */
extern "C" {
    enum radio_device_type_t_c { RADIO_DEVICE_HACKRF_C = 0 };
    int  radio_list_devices(int device_type, char ***out_identifiers,
                            size_t *out_count);
    void radio_free_device_list(char ***identifiers, size_t count);
}

namespace {
    // Mirror of Header.qml inputTypeSelector indices.
    constexpr int kInputTypeHackRF = 0;
    constexpr int kInputTypeFile   = 1;
}

RadioDeviceModel::RadioDeviceModel(QObject *parent)
    : QStringListModel(parent)
{
}

void RadioDeviceModel::refresh(int inputTypeIndex, bool force)
{
    if (!force && inputTypeIndex == m_inputTypeIndex)
        return;

    m_inputTypeIndex = inputTypeIndex;

    QStringList identifiers;

    if (inputTypeIndex == kInputTypeHackRF)
    {
        char **raw = nullptr;
        size_t count = 0u;
        int result = radio_list_devices(RADIO_DEVICE_HACKRF_C, &raw, &count);
        if (result == 0)
        {
            for (size_t i = 0u; i < count; i++)
                identifiers.append(QString::fromUtf8(raw[i] ? raw[i] : ""));
            radio_free_device_list(&raw, count);
        }
        else
        {
            qWarning() << "RadioDeviceModel: hackrf_list_devices failed:"
                       << result;
        }
    }
    // File input has no device enumeration; leave identifiers empty.

    setStringList(identifiers);
}

int RadioDeviceModel::indexFromIdentifier(const QString &identifier) const
{
    if (identifier.isEmpty())
        return -1;

    const QStringList rows = stringList();
    for (int i = 0; i < rows.size(); ++i)
    {
        if (rows[i] == identifier)
            return i;
    }
    return -1;
}