#include <QDir>
#include <QFont>
#include <QFontDatabase>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QLoggingCategory>
#include <QIcon>

int main(int argc, char *argv[])
{
    /* Ensure session lifecycle logging is visible regardless of build type. */
    QLoggingCategory::setFilterRules(
        QStringLiteral("supertooth.session.debug=true\n"
                       "supertooth.session.info=true\n"
                       "supertooth.session.warning=true"));

    QGuiApplication app(argc, argv);

app.setWindowIcon(QIcon(":/assets/icons/Supertooth_Spaced_512_2x.png"));

    const QDir fontsDir(":/assets/fonts");
    for (const QFileInfo &fi : fontsDir.entryInfoList({"*.ttf", "*.otf"}, QDir::Files)) {
        const int id = QFontDatabase::addApplicationFont(fi.absoluteFilePath());
        if (id < 0)
            qWarning("Failed to load font: %s", qPrintable(fi.absoluteFilePath()));
    }

    QFont appFont("Google Sans Code");
    appFont.setStyleHint(QFont::Monospace);
    app.setFont(appFont);

    QQmlApplicationEngine engine;
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);
    engine.loadFromModule("Supertooth", "Main");

    return app.exec();
}
