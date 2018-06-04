#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>

#define REPORT_SET_CHIP     0x01
#define REPORT_PING         0x02
#define REPORT_GET_VOLTAGE  0x03
#define REPORT_SET_FUSE     0x04
#define REPORT_GET_FUSE     0x05

uint8_t data_ping[2] = {REPORT_PING, 0xdf};

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_dev = nullptr;

    connect(&m_hid_timer, SIGNAL(timeout()), this, SLOT(update_hid()));
    m_hid_timer.setInterval(1000);
    m_hid_timer.start();
}

MainWindow::~MainWindow()
{
    hid_close(m_dev);
    delete ui;
}

void MainWindow::update_hid()
{
    if (m_dev == nullptr)
    {
        m_dev = hid_open(0x483, 0x1CB2, nullptr);

        if (m_dev == nullptr)
            return;
    }

    if (m_dev != nullptr)
    {
        // ping packet
        if (hid_write(m_dev, data_ping, 2) < 0)
        {
            hid_close(m_dev);
            m_dev = nullptr;
            qDebug() << "hid_write Error";
            return;
        }

        uint8_t ping_reply[10];
        hid_set_nonblocking(m_dev, 0);
        if (hid_read_timeout(m_dev, ping_reply, 3, 1000) <= 0)
        {
            hid_close(m_dev);
            m_dev = nullptr;
            qDebug() << "hid_read Error";
            return;
        }
    }
}
