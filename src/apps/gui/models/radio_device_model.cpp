#include "radio_device_model.h"

#include "../backend/backend_api.h"

#include <QDebug>
#include <QVariantMap>

RadioDeviceModel::RadioDeviceModel(QObject *parent)
    : QStringListModel(parent)
{
}

void RadioDeviceModel::refresh(int inputType, bool force)
{
    if (!force && inputType == m_inputType)
        return;

    m_inputType = inputType;

    QStringList identifiers;

    char **raw = nullptr;
    size_t count = 0u;
    int result = backend_list_devices(inputType, &raw, &count);
    if (result == 0)
    {
        for (size_t i = 0u; i < count; i++)
            identifiers.append(QString::fromUtf8(raw[i] ? raw[i] : ""));
        backend_free_device_list(&raw, count);
    }
    else
    {
        qWarning() << "RadioDeviceModel: backend_list_devices failed:"
                   << result;
    }

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

QVariantList RadioDeviceModel::availableInputTypes() const
{
    QVariantList out;
    int types[BACKEND_MAX_LIVE_INPUTS];
    int n = backend_live_input_types(types, BACKEND_MAX_LIVE_INPUTS);

    for (int i = 0; i < n; i++)
    {
        char label[BACKEND_INPUT_LABEL_LEN] = {0};
        if (backend_input_type_label(types[i], label, sizeof(label)) != 0)
            continue;
        QVariantMap m;
        m.insert(QStringLiteral("inputType"), types[i]);
        m.insert(QStringLiteral("label"), QString::fromUtf8(label));
        out.append(m);
    }
    return out;
}

unsigned int RadioDeviceModel::defaultBredrCount(int inputType) const
{
    return backend_default_bredr_count(inputType);
}

unsigned int RadioDeviceModel::defaultBleCount(int inputType) const
{
    return backend_default_ble_count(inputType);
}

unsigned int RadioDeviceModel::maxBredrCount(int inputType) const
{
    return backend_max_bredr_count(inputType);
}

unsigned int RadioDeviceModel::maxBleCount(int inputType) const
{
    return backend_max_ble_count(inputType);
}

static QVariantList supportedCounts(int inputType, int leGrid)
{
    QVariantList out;
    /* BR/EDR grid: at most 2..79; LE grid: at most 1..40. */
    unsigned buf[80];
    int n = backend_supported_counts(inputType, leGrid, buf,
                                     (int)(sizeof(buf) / sizeof(buf[0])));
    for (int i = 0; i < n; i++)
        out.append(buf[i]);
    return out;
}

QVariantList RadioDeviceModel::supportedBredrCounts(int inputType) const
{
    return supportedCounts(inputType, 0);
}

QVariantList RadioDeviceModel::supportedBleCounts(int inputType) const
{
    return supportedCounts(inputType, 1);
}
