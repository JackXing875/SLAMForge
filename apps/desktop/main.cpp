#include <QApplication>
#include <QCoreApplication>
#include <QIcon>
#include <QTimer>

#include "main_window.h"

int main(int argc, char* argv[]) {
    QApplication application(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("SLAMForge"));
    QCoreApplication::setOrganizationDomain(QStringLiteral("github.com/JackXing875"));
    QCoreApplication::setApplicationName(QStringLiteral("SLAMForge Desktop"));
    QCoreApplication::setApplicationVersion(QStringLiteral(SLAMFORGE_DESKTOP_VERSION));
    application.setWindowIcon(QIcon(QStringLiteral(":/slamforge/app-icon.svg")));

    slamforge::desktop::MainWindow window;
    window.show();

    // A headless construction-and-event-loop smoke test for release CI. This
    // exercises Qt platform initialization without starting a SLAM job.
    const QStringList arguments = QCoreApplication::arguments();
    const qsizetype result_index = arguments.indexOf(QStringLiteral("--smoke-result"));
    if (result_index >= 0) {
        if (result_index + 1 >= arguments.size()) {
            return 2;
        }
        QString result_error;
        if (!window.LoadResultDirectory(arguments[result_index + 1], &result_error)) {
            return 3;
        }
    }
    if (arguments.contains(QStringLiteral("--smoke-test")) || result_index >= 0) {
        QTimer::singleShot(0, &application, &QCoreApplication::quit);
    }
    return application.exec();
}
