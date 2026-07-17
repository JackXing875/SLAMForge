#include "main_window.h"

#include <QAction>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDesktopServices>
#include <QDir>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMimeData>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QSettings>
#include <QStandardPaths>
#include <QTabWidget>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>
#include <QWidget>
#include <algorithm>

#include "map_view.h"

namespace slamforge::desktop {
namespace {

constexpr auto kVideoFilter = "Video files (*.mp4 *.mov *.avi *.mkv *.m4v);;All files (*)";
constexpr auto kConfigFilter = "SLAMForge configuration (*.yaml *.yml);;All files (*)";

QString NativePath(const QString& path) {
    return QDir::toNativeSeparators(QDir::cleanPath(path));
}

}  // namespace

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), process_(new QProcess(this)), log_file_(new QFile(this)) {
    setAcceptDrops(true);
    setMinimumSize(820, 600);
    resize(980, 700);
    setWindowTitle(tr("SLAMForge Desktop %1").arg(QCoreApplication::applicationVersion()));

    BuildUi();
    ConnectSignals();
    RestoreSettings();
    SetRunning(false);
}

bool MainWindow::LoadResultDirectory(const QString& directory, QString* error) {
    const QDir result_directory(directory);
    const QString map_path = result_directory.filePath(QStringLiteral("map.ply"));
    const QString trajectory_path = result_directory.filePath(QStringLiteral("trajectory.txt"));
    QString viewer_error;
    if (!map_view_->LoadResult(map_path, trajectory_path, &viewer_error)) {
        if (error != nullptr) {
            *error = viewer_error;
        }
        return false;
    }

    result_directory_ = NativePath(result_directory.absolutePath());
    map_path_ = map_path;
    trajectory_path_ = trajectory_path;
    view_tabs_->setCurrentWidget(map_view_);
    open_result_button_->setEnabled(true);
    if (error != nullptr) {
        *error = viewer_error;
    }
    return true;
}

void MainWindow::BuildUi() {
    auto* central = new QWidget(this);
    auto* root_layout = new QVBoxLayout(central);
    root_layout->setContentsMargins(24, 20, 24, 20);
    root_layout->setSpacing(14);

    auto* title = new QLabel(tr("Build a map from a monocular video"), central);
    QFont title_font = title->font();
    title_font.setPointSize(title_font.pointSize() + 6);
    title_font.setBold(true);
    title->setFont(title_font);
    root_layout->addWidget(title);

    auto* introduction = new QLabel(
        tr("Drop a video anywhere in this window, choose a calibrated camera configuration, "
           "and start the local SLAM analysis. The result view displays a fused dense surface and "
           "camera trajectory without uploading your video."),
        central);
    introduction->setWordWrap(true);
    root_layout->addWidget(introduction);

    auto* help_menu = menuBar()->addMenu(tr("&Help"));
    auto* about_action = help_menu->addAction(tr("&About SLAMForge Desktop"));
    connect(about_action, &QAction::triggered, this, [this] {
        QMessageBox::about(
            this, tr("About SLAMForge Desktop"),
            tr("<h3>SLAMForge Desktop %1</h3>"
               "<p>Monocular visual SLAM with offline dense surface reconstruction.</p>"
               "<p><b>Important:</b> results use relative scale and require accurate camera "
               "calibration.</p>"
               "<p>Licensed under GPL-3.0-only.<br>"
               "github.com/JackXing875/SLAMForge</p>")
                .arg(QCoreApplication::applicationVersion()));
    });

    input_group_ = new QGroupBox(tr("Analysis input"), central);
    auto* input_layout = new QGridLayout(input_group_);

    input_edit_ = new QLineEdit(input_group_);
    input_edit_->setReadOnly(true);
    input_edit_->setPlaceholderText(tr("Drop or choose an MP4, MOV, AVI, MKV, or M4V video"));
    auto* input_button = new QPushButton(tr("Choose video…"), input_group_);

    config_edit_ = new QLineEdit(input_group_);
    config_edit_->setReadOnly(true);
    config_edit_->setPlaceholderText(tr("Choose a YAML camera configuration"));
    auto* config_button = new QPushButton(tr("Choose config…"), input_group_);

    output_edit_ = new QLineEdit(input_group_);
    output_edit_->setPlaceholderText(tr("Directory for trajectory and logs"));
    auto* output_button = new QPushButton(tr("Choose folder…"), input_group_);

    input_layout->addWidget(new QLabel(tr("Video:"), input_group_), 0, 0);
    input_layout->addWidget(input_edit_, 0, 1);
    input_layout->addWidget(input_button, 0, 2);
    input_layout->addWidget(new QLabel(tr("Camera config:"), input_group_), 1, 0);
    input_layout->addWidget(config_edit_, 1, 1);
    input_layout->addWidget(config_button, 1, 2);
    input_layout->addWidget(new QLabel(tr("Results:"), input_group_), 2, 0);
    input_layout->addWidget(output_edit_, 2, 1);
    input_layout->addWidget(output_button, 2, 2);
    input_layout->setColumnStretch(1, 1);
    root_layout->addWidget(input_group_);

    auto* status_row = new QHBoxLayout();
    status_label_ = new QLabel(tr("Ready — drop a video to begin"), central);
    progress_bar_ = new QProgressBar(central);
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(0);
    progress_bar_->setTextVisible(true);
    status_row->addWidget(status_label_);
    status_row->addStretch(1);
    status_row->addWidget(progress_bar_, 1);
    root_layout->addLayout(status_row);

    view_tabs_ = new QTabWidget(central);
    map_view_ = new MapView(view_tabs_);
    view_tabs_->addTab(map_view_, tr("3D result"));

    log_edit_ = new QPlainTextEdit(view_tabs_);
    log_edit_->setReadOnly(true);
    log_edit_->setPlaceholderText(tr("SLAM analysis output will appear here."));
    log_edit_->setMaximumBlockCount(10000);
    QFont log_font = QFont(QStringLiteral("monospace"));
    log_font.setStyleHint(QFont::Monospace);
    log_edit_->setFont(log_font);
    view_tabs_->addTab(log_edit_, tr("Run log"));
    root_layout->addWidget(view_tabs_, 1);

    auto* action_row = new QHBoxLayout();
    open_result_button_ = new QPushButton(tr("Open results"), central);
    start_button_ = new QPushButton(tr("Start mapping"), central);
    cancel_button_ = new QPushButton(tr("Cancel"), central);
    start_button_->setDefault(true);
    action_row->addWidget(open_result_button_);
    action_row->addStretch(1);
    action_row->addWidget(cancel_button_);
    action_row->addWidget(start_button_);
    root_layout->addLayout(action_row);

    setCentralWidget(central);

    connect(input_button, &QPushButton::clicked, this, [this] { ChooseInput(); });
    connect(config_button, &QPushButton::clicked, this, [this] { ChooseConfig(); });
    connect(output_button, &QPushButton::clicked, this, [this] { ChooseOutputDirectory(); });
}

void MainWindow::ConnectSignals() {
    connect(start_button_, &QPushButton::clicked, this, [this] { StartAnalysis(); });
    connect(cancel_button_, &QPushButton::clicked, this, [this] { CancelAnalysis(); });
    connect(open_result_button_, &QPushButton::clicked, this, [this] { OpenResultDirectory(); });

    connect(process_, &QProcess::readyReadStandardOutput, this, [this] {
        AppendProcessOutput(QString::fromLocal8Bit(process_->readAllStandardOutput()));
    });
    connect(process_, &QProcess::readyReadStandardError, this, [this] {
        const QString output = QString::fromLocal8Bit(process_->readAllStandardError());
        UpdateProgressFromOutput(output);
        AppendProcessOutput(output);
    });
    connect(process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
        if (error == QProcess::FailedToStart) {
            AppendProcessOutput(
                tr("Unable to start slamforge_cli. Ensure it is installed next "
                   "to SLAMForge Desktop."));
            progress_bar_->setRange(0, 100);
            progress_bar_->setValue(0);
            status_label_->setText(tr("SLAM engine failed to start"));
            SetRunning(false);
            log_file_->close();
        } else {
            AppendProcessOutput(tr("SLAM process error: %1").arg(process_->errorString()));
        }
    });
    connect(process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
            [this](int exit_code, QProcess::ExitStatus exit_status) {
                ProcessFinished(exit_code, exit_status);
            });
}

void MainWindow::RestoreSettings() {
    QSettings settings;
    input_edit_->setText(settings.value(QStringLiteral("input/video")).toString());
    config_edit_->setText(settings.value(QStringLiteral("input/config")).toString());
    output_edit_->setText(settings.value(QStringLiteral("output/directory")).toString());

    if (!input_edit_->text().isEmpty() && !QFileInfo::exists(input_edit_->text())) {
        input_edit_->clear();
    }
    if (!config_edit_->text().isEmpty() && !QFileInfo::exists(config_edit_->text())) {
        config_edit_->clear();
    }
    if (!output_edit_->text().isEmpty()) {
        result_directory_ = output_edit_->text();
    }
}

void MainWindow::SaveSettings() const {
    QSettings settings;
    settings.setValue(QStringLiteral("input/video"), input_edit_->text());
    settings.setValue(QStringLiteral("input/config"), config_edit_->text());
    settings.setValue(QStringLiteral("output/directory"), output_edit_->text());
}

void MainWindow::ChooseInput() {
    const QString initial = input_edit_->text().isEmpty()
                                ? QStandardPaths::writableLocation(QStandardPaths::MoviesLocation)
                                : QFileInfo(input_edit_->text()).absolutePath();
    const QString path =
        QFileDialog::getOpenFileName(this, tr("Choose a video"), initial, tr(kVideoFilter));
    if (!path.isEmpty()) {
        SetInputPath(path);
    }
}

void MainWindow::ChooseConfig() {
    const QString initial = config_edit_->text().isEmpty()
                                ? FindConfigDirectory()
                                : QFileInfo(config_edit_->text()).absolutePath();
    const QString path = QFileDialog::getOpenFileName(this, tr("Choose camera configuration"),
                                                      initial, tr(kConfigFilter));
    if (!path.isEmpty()) {
        config_edit_->setText(NativePath(path));
        SaveSettings();
    }
}

void MainWindow::ChooseOutputDirectory() {
    const QString initial =
        output_edit_->text().isEmpty() ? QDir::homePath() : output_edit_->text();
    const QString path =
        QFileDialog::getExistingDirectory(this, tr("Choose results directory"), initial);
    if (!path.isEmpty()) {
        output_edit_->setText(NativePath(path));
        result_directory_ = output_edit_->text();
        SaveSettings();
    }
}

void MainWindow::SetInputPath(const QString& path) {
    if (!IsSupportedVideo(path)) {
        QMessageBox::warning(this, tr("Unsupported input"),
                             tr("Choose an MP4, MOV, AVI, MKV, or M4V video file."));
        return;
    }

    input_edit_->setText(NativePath(path));
    if (output_edit_->text().isEmpty()) {
        const QFileInfo video(path);
        output_edit_->setText(
            NativePath(video.absoluteDir().filePath(video.completeBaseName() + "-slamforge")));
        result_directory_ = output_edit_->text();
    }
    status_label_->setText(tr("Video selected — verify the camera configuration"));
    SaveSettings();
}

void MainWindow::StartAnalysis() {
    const QString input_path = input_edit_->text();
    const QString config_path = config_edit_->text();
    const QString requested_output = output_edit_->text().trimmed();

    if (!IsSupportedVideo(input_path) || !QFileInfo(input_path).isFile()) {
        QMessageBox::warning(this, tr("Video required"),
                             tr("Select a readable supported video before starting."));
        return;
    }
    if (!QFileInfo(config_path).isFile()) {
        QMessageBox::warning(this, tr("Configuration required"),
                             tr("Select a readable SLAMForge YAML camera configuration."));
        return;
    }
    if (requested_output.isEmpty()) {
        QMessageBox::warning(this, tr("Invalid results directory"),
                             tr("Choose a results directory before starting."));
        return;
    }
    const QString output_directory = NativePath(QDir(requested_output).absolutePath());
    if (!QDir().mkpath(output_directory)) {
        QMessageBox::warning(this, tr("Invalid results directory"),
                             tr("The selected results directory could not be created."));
        return;
    }
    output_edit_->setText(output_directory);

    const QString cli = FindCliExecutable();
    if (cli.isEmpty()) {
        QMessageBox::critical(
            this, tr("SLAM engine not found"),
            tr("slamforge_cli was not found next to the desktop application or on PATH."));
        return;
    }
    const QString depth_model = FindDepthModel();
    if (depth_model.isEmpty()) {
        QMessageBox::critical(
            this, tr("Dense model not found"),
            tr("The bundled Depth Anything V2 model was not found. Reinstall the complete "
               "SLAMForge Desktop package."));
        return;
    }

    result_directory_ = NativePath(output_directory);
    trajectory_path_ = QDir(output_directory).filePath(QStringLiteral("trajectory.txt"));
    map_path_ = QDir(output_directory).filePath(QStringLiteral("map.ply"));
    sparse_map_path_ = QDir(output_directory).filePath(QStringLiteral("sparse_map.ply"));
    const QString log_path = QDir(output_directory).filePath(QStringLiteral("run.log"));
    if (QFileInfo::exists(trajectory_path_) || QFileInfo::exists(map_path_) ||
        QFileInfo::exists(sparse_map_path_) || QFileInfo::exists(log_path)) {
        const auto answer = QMessageBox::question(
            this, tr("Replace existing results"),
            tr("This folder already contains SLAMForge result files. trajectory.txt, map.ply, "
               "sparse_map.ply, and run.log will be replaced. Continue?"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (answer != QMessageBox::Yes) {
            return;
        }
    }
    log_file_->close();
    log_file_->setFileName(log_path);
    if (!log_file_->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        QMessageBox::warning(this, tr("Log file unavailable"),
                             tr("The analysis can continue, but run.log could not be created: %1")
                                 .arg(log_file_->errorString()));
    }
    const QStringList arguments = {QStringLiteral("run"),
                                   QStringLiteral("--config"),
                                   config_path,
                                   QStringLiteral("--input"),
                                   input_path,
                                   QStringLiteral("--output"),
                                   trajectory_path_,
                                   QStringLiteral("--map-output"),
                                   sparse_map_path_,
                                   QStringLiteral("--dense-output"),
                                   map_path_,
                                   QStringLiteral("--depth-model"),
                                   depth_model,
                                   QStringLiteral("--verbose")};

    log_edit_->clear();
    map_view_->Clear();
    view_tabs_->setCurrentWidget(log_edit_);
    AppendProcessOutput(tr("Starting SLAMForge analysis…"));
    AppendProcessOutput(tr("Engine: %1").arg(NativePath(cli)));
    AppendProcessOutput(tr("Video:  %1").arg(input_path));
    AppendProcessOutput(tr("Config: %1").arg(config_path));
    AppendProcessOutput(tr("Trajectory: %1").arg(NativePath(trajectory_path_)));
    AppendProcessOutput(tr("Dense map:  %1").arg(NativePath(map_path_)));
    AppendProcessOutput(tr("Sparse map: %1").arg(NativePath(sparse_map_path_)));

    cancelling_ = false;
    progress_bar_->setRange(0, 0);
    status_label_->setText(tr("Starting SLAM engine…"));
    SetRunning(true);
    SaveSettings();
    process_->setWorkingDirectory(output_directory);
    process_->start(cli, arguments);
}

void MainWindow::CancelAnalysis() {
    if (process_->state() == QProcess::NotRunning) {
        return;
    }

    cancelling_ = true;
    status_label_->setText(tr("Cancelling safely…"));
    AppendProcessOutput(tr("Cancellation requested."));
    const qint64 process_id = process_->processId();
    process_->terminate();
    QTimer::singleShot(3000, this, [this, process_id] {
        if (process_->state() != QProcess::NotRunning && process_->processId() == process_id) {
            AppendProcessOutput(tr("The SLAM process did not stop in time; forcing it to exit."));
            process_->kill();
        }
    });
}

void MainWindow::OpenResultDirectory() {
    if (result_directory_.isEmpty() || !QFileInfo(result_directory_).isDir()) {
        QMessageBox::information(this, tr("No results yet"),
                                 tr("Run an analysis or choose a results directory first."));
        return;
    }
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(result_directory_))) {
        QMessageBox::warning(this, tr("Unable to open results"),
                             tr("The system file manager could not open the results directory."));
    }
}

void MainWindow::AppendProcessOutput(const QString& text) {
    if (text.isEmpty()) {
        return;
    }
    QString normalized = text;
    normalized.replace(QLatin1Char('\r'), QLatin1Char('\n'));
    while (normalized.endsWith(QLatin1Char('\n'))) {
        normalized.chop(1);
    }
    if (!normalized.isEmpty()) {
        log_edit_->appendPlainText(normalized);
        if (log_file_->isOpen()) {
            log_file_->write(normalized.toUtf8());
            log_file_->write("\n");
            log_file_->flush();
        }
    }
}

void MainWindow::UpdateProgressFromOutput(const QString& text) {
    if (text.contains(QStringLiteral("Finalizing map and checking loop closures"))) {
        progress_bar_->setRange(0, 0);
        status_label_->setText(tr("Finalizing map and checking loop closures"));
    }
    static const QRegularExpression frame_pattern(QStringLiteral(R"(Frame\s+(\d+)/(\d+))"));
    auto matches = frame_pattern.globalMatch(text);
    while (matches.hasNext()) {
        const auto match = matches.next();
        const int current = match.captured(1).toInt();
        const int total = match.captured(2).toInt();
        if (total > 0) {
            progress_bar_->setRange(0, total);
            progress_bar_->setValue(std::clamp(current, 0, total));
            status_label_->setText(tr("Mapping frame %1 of %2").arg(current).arg(total));
        } else {
            progress_bar_->setRange(0, 0);
            status_label_->setText(tr("Mapping frame %1").arg(current));
        }
    }
    static const QRegularExpression dense_pattern(
        QStringLiteral(R"(Dense keyframe\s+(\d+)/(\d+))"));
    auto dense_matches = dense_pattern.globalMatch(text);
    while (dense_matches.hasNext()) {
        const auto match = dense_matches.next();
        const int current = match.captured(1).toInt();
        const int total = match.captured(2).toInt();
        if (total > 0) {
            progress_bar_->setRange(0, total);
            progress_bar_->setValue(std::clamp(current, 0, total));
            status_label_->setText(
                tr("Reconstructing dense surface %1 of %2").arg(current).arg(total));
        }
    }
}

void MainWindow::SetRunning(bool running) {
    start_button_->setEnabled(!running);
    cancel_button_->setEnabled(running);
    input_group_->setEnabled(!running);
    open_result_button_->setEnabled(!running && !result_directory_.isEmpty());
}

void MainWindow::ProcessFinished(int exit_code, QProcess::ExitStatus exit_status) {
    SetRunning(false);
    progress_bar_->setRange(0, 100);

    if (cancelling_) {
        progress_bar_->setValue(0);
        status_label_->setText(tr("Analysis cancelled"));
        AppendProcessOutput(tr("Analysis cancelled by the user."));
    } else if (exit_status == QProcess::NormalExit && exit_code == 0) {
        progress_bar_->setValue(100);
        AppendProcessOutput(tr("Analysis completed successfully."));
        QString viewer_error;
        if (LoadResultDirectory(result_directory_, &viewer_error)) {
            status_label_->setText(tr("Mapping complete — result ready"));
            view_tabs_->setCurrentWidget(map_view_);
            if (!viewer_error.isEmpty()) {
                AppendProcessOutput(tr("Result viewer note: %1").arg(viewer_error));
            }
        } else {
            status_label_->setText(tr("Mapping complete — no viewable result"));
            AppendProcessOutput(tr("Result viewer: %1").arg(viewer_error));
        }
    } else {
        progress_bar_->setValue(0);
        status_label_->setText(tr("Analysis failed — review the log"));
        AppendProcessOutput(tr("Analysis failed with exit code %1.").arg(exit_code));
    }
    cancelling_ = false;
    open_result_button_->setEnabled(!result_directory_.isEmpty());
    log_file_->close();
}

QString MainWindow::FindCliExecutable() const {
#ifdef Q_OS_WIN
    constexpr auto executable_name = "slamforge_cli.exe";
#else
    constexpr auto executable_name = "slamforge_cli";
#endif
    const QString adjacent =
        QDir(QCoreApplication::applicationDirPath()).filePath(QString::fromLatin1(executable_name));
    if (QFileInfo(adjacent).isFile()) {
        return QFileInfo(adjacent).absoluteFilePath();
    }
    return QStandardPaths::findExecutable(QString::fromLatin1(executable_name));
}

QString MainWindow::FindDepthModel() const {
    const QStringList candidates = {
        QDir(QCoreApplication::applicationDirPath())
            .filePath(QStringLiteral("models/depth_anything_v2_vits_dynamic.onnx")),
        QDir(QCoreApplication::applicationDirPath())
            .filePath(QStringLiteral("../share/slamforge/models/"
                                     "depth_anything_v2_vits_dynamic.onnx")),
        QDir::current().filePath(QStringLiteral("models/depth_anything_v2_vits_dynamic.onnx"))};
    for (const QString& candidate : candidates) {
        if (QFileInfo(candidate).isFile()) {
            return QFileInfo(candidate).absoluteFilePath();
        }
    }
    return {};
}

QString MainWindow::FindConfigDirectory() const {
    const QStringList candidates = {
        QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config")),
        QDir(QCoreApplication::applicationDirPath())
            .filePath(QStringLiteral("../share/slamforge/config")),
        QDir::current().filePath(QStringLiteral("config")), QDir::homePath()};
    for (const QString& candidate : candidates) {
        if (QFileInfo(candidate).isDir()) {
            return QFileInfo(candidate).absoluteFilePath();
        }
    }
    return QDir::homePath();
}

bool MainWindow::IsSupportedVideo(const QString& path) const {
    static const QStringList supported = {QStringLiteral("mp4"), QStringLiteral("mov"),
                                          QStringLiteral("avi"), QStringLiteral("mkv"),
                                          QStringLiteral("m4v")};
    return supported.contains(QFileInfo(path).suffix().toLower());
}

void MainWindow::dragEnterEvent(QDragEnterEvent* event) {
    if (process_->state() != QProcess::NotRunning) {
        return;
    }
    const QList<QUrl> urls = event->mimeData()->urls();
    if (urls.size() == 1 && urls.front().isLocalFile() &&
        IsSupportedVideo(urls.front().toLocalFile())) {
        event->acceptProposedAction();
    }
}

void MainWindow::dropEvent(QDropEvent* event) {
    if (process_->state() != QProcess::NotRunning) {
        return;
    }
    const QList<QUrl> urls = event->mimeData()->urls();
    if (urls.size() == 1 && urls.front().isLocalFile()) {
        SetInputPath(urls.front().toLocalFile());
        event->acceptProposedAction();
    }
}

void MainWindow::closeEvent(QCloseEvent* event) {
    if (process_->state() != QProcess::NotRunning) {
        const auto answer = QMessageBox::question(
            this, tr("Analysis is running"),
            tr("Closing SLAMForge Desktop will cancel the current analysis. Close anyway?"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
        if (answer != QMessageBox::Yes) {
            event->ignore();
            return;
        }
        process_->terminate();
        if (!process_->waitForFinished(3000)) {
            process_->kill();
            process_->waitForFinished(1000);
        }
    }
    log_file_->close();
    SaveSettings();
    event->accept();
}

}  // namespace slamforge::desktop
