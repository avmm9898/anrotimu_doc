#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QByteArray>
#include <QTimer>
#include <QTableWidget>

#include "include/imu_data_decode.h"
#include "include/packet.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


private slots:
    void on_btn_serial_init_clicked();
    void read_serial();
    void get_data();
    void on_Com_Refresh_btn_clicked();
    void on_btn_serial_stop_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPortInfo com_info;
    QSerialPort m_reader;
    QTimer display_timer;
    void ShowOnTable(receive_imusol_packet_t);

};
#endif // MAINWINDOW_H
