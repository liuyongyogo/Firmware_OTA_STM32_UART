#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QObject>
#include <QtSerialPort/QSerialPortInfo>
#include <QtSerialPort/qserialport.h>
#include <QDebug>
#include <QTimer>
#include <QTime>
#include <QMessageBox>
#include <QObject>
#include <QUdpSocket>
#include <QDebug>
#include <QTimer>

#define YOGO_REMOTE_PORT    9527

#define CABIN_CMD_OPEN       0x04
#define CABIN_CMD_CLOSE      0x01
#define CABIN_CMD_PAUSE      0x05
#define CABIN_CMD_NULL       0x00
#define CABIN_CMD_PUSH       0x0C
#define CABIN_CMD_PULL       0x09
#define CABIN_CMD_MASK       0x0F


//status form driver
#define CABINET_DOOR_CLOSED    0x01
#define CABINET_DOOR_OPENED    0x02
#define CABINET_DOOR_CLOSING   0x03
#define CABINET_DOOR_OPENING   0x04
#define CABINET_DOOR_MASK      0x0F


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QSerialPort  *  m_MotionPort;

private slots:

    void netDatacome(void);

    void on_btn_Open_clicked();

    void rx_data_come();

    void on_btn_File_clicked();

    void on_btn_Download_clicked();


    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_btn_Open_2_clicked();

    void on_btn_Close_clicked();

    void on_btn_first_open_clicked();

    void on_btn_first_close_clicked();

    void on_btn_push_first_clicked();

    void on_btn_pull_first_clicked();

    void on_btn_second_open_clicked();

    void on_btn_second_close_clicked();

    void on_btn_second_push_clicked();

    void on_btn_second_pull_clicked();

    void on_btn_LED_clicked();

private:
    bool portOpen();
    bool portClose();
    unsigned char checkSum_Calc(QByteArray buf);
    void sendDownloadHeader(QString fileName);
    void sendDownloadPackage(int offset);
    unsigned char* mySendPackageStd(unsigned char iID,unsigned char  iCMD,unsigned char  Length,unsigned char  *data);

    void TimeUpdate_COM();
    //int Rxd_Comm(unsigned char *buff);

  //  void sleep(unsigned int msec);

    void sendtonet();

    Ui::MainWindow *ui;

    bool m_bOpend = false;

    QString m_FileRead;
    QString m_FileWrite;

    QByteArray DownloadCache;
    uint8_t    DownloadTar=0;
    uint32_t    DownloadOffset=0;

    QTimer *mytimer_comm;
    QByteArray myDate;
    QByteArray myReaddata;

    QByteArray rx;



    QUdpSocket    * udpSocket;
    QTimer        *m_RunTimer;
    unsigned char inbuf[4096];
    unsigned char outbuf[4096];
    QHostAddress    robotAddr;
    QHostAddress    m_RemoteFrom;
    //UART_RAWDATA_RX uart_rx;
    float pitch,roll,yaw;

    int        nSendIndex;

    uint8_t door_cmd[3];
    uint8_t door_status[3];

    uint8_t led_cmd[20];

};

#endif // MAINWINDOW_H
