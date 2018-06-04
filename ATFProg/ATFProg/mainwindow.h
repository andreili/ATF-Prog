#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <hidapi/hidapi.h>
#include <QTimer>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow  *ui;
    hid_device  *m_dev;
    uint8_t     m_buf[100];
    QTimer      m_hid_timer;
private slots:
    void update_hid();
};

#endif // MAINWINDOW_H
