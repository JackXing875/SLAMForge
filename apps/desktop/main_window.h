#pragma once

#include <QMainWindow>
#include <QProcess>
#include <QString>

class QCloseEvent;
class QDragEnterEvent;
class QDropEvent;
class QFile;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QTabWidget;

namespace slamforge::desktop {

class MapView;

/// First product-facing desktop shell for SLAMForge.
///
/// This milestone deliberately launches the already-tested CLI in a child
/// process. It provides crash isolation and a complete drag-to-result workflow
/// while the reusable in-process SlamSession application layer is developed.
class MainWindow final : public QMainWindow {
public:
    explicit MainWindow(QWidget* parent = nullptr);

    /// Load an existing result directory containing map.ply and trajectory.txt.
    /// Used by release smoke tests and the future open-project workflow.
    bool LoadResultDirectory(const QString& directory, QString* error = nullptr);

protected:
    void closeEvent(QCloseEvent* event) override;
    void dragEnterEvent(QDragEnterEvent* event) override;
    void dropEvent(QDropEvent* event) override;

private:
    void BuildUi();
    void ConnectSignals();
    void RestoreSettings();
    void SaveSettings() const;

    void ChooseInput();
    void ChooseConfig();
    void ChooseOutputDirectory();
    void SetInputPath(const QString& path);
    void StartAnalysis();
    void CancelAnalysis();
    void OpenResultDirectory();

    void AppendProcessOutput(const QString& text);
    void UpdateProgressFromOutput(const QString& text);
    void SetRunning(bool running);
    void ProcessFinished(int exit_code, QProcess::ExitStatus exit_status);

    [[nodiscard]] QString FindCliExecutable() const;
    [[nodiscard]] QString FindConfigDirectory() const;
    [[nodiscard]] bool IsSupportedVideo(const QString& path) const;

    QLineEdit* input_edit_ = nullptr;
    QLineEdit* config_edit_ = nullptr;
    QLineEdit* output_edit_ = nullptr;
    QGroupBox* input_group_ = nullptr;
    QPushButton* start_button_ = nullptr;
    QPushButton* cancel_button_ = nullptr;
    QPushButton* open_result_button_ = nullptr;
    QProgressBar* progress_bar_ = nullptr;
    QLabel* status_label_ = nullptr;
    QPlainTextEdit* log_edit_ = nullptr;
    QTabWidget* view_tabs_ = nullptr;
    MapView* map_view_ = nullptr;
    QProcess* process_ = nullptr;
    QFile* log_file_ = nullptr;

    QString result_directory_;
    QString trajectory_path_;
    QString map_path_;
    bool cancelling_ = false;
};

}  // namespace slamforge::desktop
