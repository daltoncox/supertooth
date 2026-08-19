#include <cstdio>
#include <cstring>
#include <QColor>
#include <QDir>
#include <QFont>
#include <QFontDatabase>
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QPalette>
#include <QLoggingCategory>
#include <QIcon>
#include "version.h"
#include "backend_api.h"

static void print_usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s [-h | --help] [-V | --version] [--debug]\n"
            "  %-30s Print this help and exit\n"
            "  %-30s Print version and exit\n"
            "  %-30s Enable verbose backend/session debug logging\n",
            argv0,
            "-h, --help",
            "-V, --version",
            "--debug");
}

int main(int argc, char *argv[])
{
    /* Parse flags before touching Qt. */
    int debug = 0;
    for (int i = 1; i < argc; i++) {
        if (argv[i][0] != '-')
            continue;
        if (strcmp(argv[i], "--version") == 0 || strcmp(argv[i], "-V") == 0) {
            printf("supertooth %s\n", supertooth_get_version());
            return 0;
        }
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        }
        if (strcmp(argv[i], "--debug") == 0) {
            debug = 1;
            continue;
        }
        /* macOS process serial number — let Qt handle it. */
        if (strncmp(argv[i], "-psn", 4) == 0)
            continue;
        /* Unrecognized flag. */
        fprintf(stderr, "Unrecognized option: %s\n\n", argv[i]);
        print_usage(argv[0]);
        return 1;
    }

    if (debug)
        backend_set_debug(1);

    /* Ensure session lifecycle logging is visible regardless of build type. */
    QLoggingCategory::setFilterRules(
        QStringLiteral("supertooth.session.debug=true\n"
                       "supertooth.session.info=true\n"
                       "supertooth.session.warning=true"));

    QGuiApplication app(argc, argv);

    app.setApplicationVersion(QString::fromUtf8(supertooth_get_version()));
    app.setWindowIcon(QIcon(":/assets/icons/Supertooth_Squircle_1024x1024.png"));
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
