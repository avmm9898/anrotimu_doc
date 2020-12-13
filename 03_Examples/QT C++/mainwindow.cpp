#include "mainwindow.h"
#include "ui_mainwindow.h"

/*

C++ QT example.Here are 3 steps to follow,
 1. "imu_data_decode_init()" -> is for the RFreceiver and the node to initiate
 2. "packet_decode(c)" -> to decode every character from serial port.
 3. while decoding, "receive_imusol", the data received from IMU, which will update itself in real-time.
 4. If you are using wireless receiver, "receive_gwsol" is the data received, and it's array of multiple receive_imusol.

 C++ QT 範例:
 1."imu_data_decode_init（）"->用於RFreceiver和要啟動的節點
 2."packet_decode(c)"->從序列端口解碼每個字符。
 3.在解碼的同時，"receive_imusol"會根據IMU接收的數據進行即時更新。
 4.如果使用的是無線接收器，則"receive_gwsol"將會是接收到的數據，它是多個"receive_imusol"的組合。

*/
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        ui->Com_combo->addItem(info.portName()+" : "+info.description());
    }
    ui->rf_id_combobox->setVisible(false);

    /********setting table**********/

    ui->tableWidget->setRowCount(10);
    ui->tableWidget->setColumnCount(5);
    ui->tableWidget->verticalHeader()->setVisible(false);
    ui->tableWidget->horizontalHeader()->setVisible(false);

    QString title1="Quaternion,W,X,Y,Z";
    QString title2="Euler Angle,Roll,Pitch,Yaw";
    QString title3="Acceleration,X,Y,Z";
    QString title4="Gyroscope,X,Y,Z";
    QString title5="Magnetic field,X,Y,Z";

    foreach(QString item,  title1.split(",")){
        static int i=0;
        QTableWidgetItem * protoitem = new QTableWidgetItem(item);
        protoitem->setFlags(protoitem->flags() & ~Qt::ItemIsEnabled & ~Qt::ItemIsSelectable);
        protoitem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(0,i,protoitem);
        i++;
    }
    foreach(QString item,  title2.split(",")){
        static int i=0;
        QTableWidgetItem * protoitem = new QTableWidgetItem(item);
        protoitem->setFlags(protoitem->flags() & ~Qt::ItemIsEnabled & ~Qt::ItemIsSelectable);
        protoitem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(2,i,protoitem);
        i++;
    }
    foreach(QString item,  title3.split(",")){
        static int i=0;
        QTableWidgetItem * protoitem = new QTableWidgetItem(item);
        protoitem->setFlags(protoitem->flags() & ~Qt::ItemIsEnabled & ~Qt::ItemIsSelectable);
        protoitem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(4,i,protoitem);
        i++;
    }
    foreach(QString item,  title4.split(",")){
        static int i=0;
        QTableWidgetItem * protoitem = new QTableWidgetItem(item);
        protoitem->setFlags(protoitem->flags() & ~Qt::ItemIsEnabled & ~Qt::ItemIsSelectable);
        protoitem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(6,i,protoitem);
        i++;
    }
    foreach(QString item,  title5.split(",")){
        static int i=0;
        QTableWidgetItem * protoitem = new QTableWidgetItem(item);
        protoitem->setFlags(protoitem->flags() & ~Qt::ItemIsEnabled & ~Qt::ItemIsSelectable);
        protoitem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(8,i,protoitem);
        i++;
    }
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
}


MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_btn_serial_init_clicked()
{

    //finding usable port on your computer
    QString m_PortName = ui->Com_combo->currentText().split(" : ").at(0);
    m_reader.setPortName(m_PortName);

    if(m_reader.open(QIODevice::ReadWrite))
    {

        m_reader.setBaudRate(QSerialPort::Baud115200);
        m_reader.setParity(QSerialPort::NoParity);
        m_reader.setDataBits(QSerialPort::Data8);
        m_reader.setStopBits(QSerialPort::OneStop);
        m_reader.setFlowControl(QSerialPort::NoFlowControl);
        m_reader.clearError();
        m_reader.clear();
        connect(&m_reader, SIGNAL(readyRead()), this, SLOT(read_serial()));

    }

    //initiaate hi221/226/229 at the begining
    imu_data_decode_init();

    //set a timer to display data from port
    connect(&display_timer, SIGNAL(timeout()), this, SLOT(get_data()));
    display_timer.start(50);



}
void MainWindow::on_btn_serial_stop_clicked()
{
    ui->rf_id_combobox->setVisible(false);
    ui->rf_id_combobox->clear();
    m_reader.disconnect();
    display_timer.disconnect();
    m_reader.close();

}
void MainWindow::read_serial()
{
    
    auto NumberOfBytesToRead = m_reader.bytesAvailable();

    if(NumberOfBytesToRead > 0 && m_reader.isReadable())
    {
        QByteArray arr = m_reader.readAll();

        for (int i=0;i<NumberOfBytesToRead;i++) {
            uint8_t c=arr[i];
            packet_decode(c);
        }
    }

}

void MainWindow::get_data()
{

    
    qDebug()<<receive_gwsol.tag;
    if(receive_gwsol.tag != KItemGWSOL)
    {
        /* imu data packet */

        ShowOnTable(receive_imusol);
        if(ui->rf_id_combobox->isVisible()){
            ui->rf_id_combobox->setVisible(false);
        }
        ui->imu_id_label->setText("IMU(ID = "+QString::number(receive_imusol.id)+")");
    }
    else
    {
        if(!ui->rf_id_combobox->isVisible()){
            ui->rf_id_combobox->setVisible(true);
            for (int i=0;i<receive_gwsol.n;i++) {
                ui->rf_id_combobox->addItem(QString::number(i));
            }
        }
        int slave_id=ui->rf_id_combobox->currentIndex();

        /* wireless data packet */
        for(int i = 0; i < receive_gwsol.n; i++)
        {
            if(QString::number(receive_gwsol.gw_id)!="")
                ui->imu_id_label->setText("Wireless IMU, ID = ");
            ShowOnTable(receive_gwsol.receive_imusol[slave_id]);


        }
    }


}

void MainWindow::on_Com_Refresh_btn_clicked()
{
    ui->Com_combo->clear();
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        com_info = info;
        qDebug() << "Name : "<< com_info.portName();
        qDebug() << "Description : "  << com_info.description();
        qDebug() << "Serial Number: " << com_info.serialNumber();
        ui->Com_combo->addItem(com_info.portName()+" : "+com_info.description());
    }
}

void MainWindow::ShowOnTable(receive_imusol_packet_t imu)
{
    for(int i=0;i<4;i++){
        QString item= QString::number(imu.quat[i],'f',3);
        QTableWidgetItem * protoitem = new QTableWidgetItem(item);
        protoitem->setTextAlignment(Qt::AlignCenter);

        ui->tableWidget->setItem(1,i+1,protoitem);
    }

    for(int i=0;i<3;i++){
        QString item= QString::number(imu.eul[i],'f',3);
        QTableWidgetItem * protoitem = new QTableWidgetItem(item);
        protoitem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(3,i+1,protoitem);
        if(i==2){
            ui->tableWidget->setItem(3,4,new QTableWidgetItem("degree"));
        }
    }

    for(int i=0;i<3;i++){
        QString item= QString::number(imu.acc[i]);
        QTableWidgetItem * protoitem = new QTableWidgetItem(item);
        protoitem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(5,i+1,protoitem);
        if(i==2){
            ui->tableWidget->setItem(5,4,new QTableWidgetItem("1G"));
        }
    }

    for(int i=0;i<3;i++){
        QString item= QString::number(imu.gyr[i]);
        QTableWidgetItem * protoitem = new QTableWidgetItem(item);
        protoitem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(7,i+1,protoitem);
        if(i==2){
            ui->tableWidget->setItem(7,4,new QTableWidgetItem("1°/s"));
        }
    }

    for(int i=0;i<3;i++){
        QString item= QString::number(imu.mag[i]);
        QTableWidgetItem * protoitem = new QTableWidgetItem(item);
        protoitem->setTextAlignment(Qt::AlignCenter);
        ui->tableWidget->setItem(9,i+1,protoitem);
        if(i==2){
            ui->tableWidget->setItem(9,4,new QTableWidgetItem("0.01Gauss"));
        }
    }
}




