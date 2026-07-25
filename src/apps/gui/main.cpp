#include <QColor>
#include <QDir>
#include <QFont>
#include <QFontDatabase>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QPalette>
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
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(45, 45, 45));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(30, 30, 30));
    darkPalette.setColor(QPalette::AlternateBase, QColor(45, 45, 45));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(45, 45, 45));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);

    app.setPalette(darkPalette);

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
