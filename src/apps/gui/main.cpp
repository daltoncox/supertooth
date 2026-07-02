#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QLoggingCategory>

int main(int argc, char *argv[])
{
    /* Ensure session lifecycle logging is visible regardless of build type. */
    QLoggingCategory::setFilterRules(
        QStringLiteral("supertooth.session.debug=true\n"
                       "supertooth.session.info=true\n"
                       "supertooth.session.warning=true"));

    QGuiApplication app(argc, argv);

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
