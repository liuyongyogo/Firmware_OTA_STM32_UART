#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QFile>


unsigned char calc_checksum(unsigned char * buf,int len)
{
    int sum = 0;
    for(int m=0;m<len;m++)
    {
        sum += buf[m];
    }
    return (~sum) & 0xFF;
}



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    m_MotionPort = NULL;
    ui->setupUi(this);
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        QString s = QObject::tr("Port: ") + info.portName() + "\n"
                + QObject::tr("Location: ") + info.systemLocation() + "\n"
                + QObject::tr("Description: ") + info.description() + "\n"
                + QObject::tr("Manufacturer: ") + info.manufacturer() + "\n"
                + QObject::tr("Serial number: ") + info.serialNumber() + "\n"
                + QObject::tr("Vendor Identifier: ") + (info.hasVendorIdentifier() ? QString::number(info.vendorIdentifier(), 16) : QString()) + "\n"
                + QObject::tr("Product Identifier: ") + (info.hasProductIdentifier() ? QString::number(info.productIdentifier(), 16) : QString()) + "\n"
                + QObject::tr("Busy: ") + (info.isBusy() ? QObject::tr("Yes") : QObject::tr("No")) + "\n";
        ui->comboBox_Port->addItem(info.portName());
        qDebug()<<s;
     }


    robotAddr = QHostAddress(QString("192.168.80.201"));
    //robotAddr = QHostAddress(QString("192.168.79.200"));
    //robotAddr = QHostAddress(QString("192.168.12.1"));

    udpSocket = new QUdpSocket;
    udpSocket->bind(YOGO_REMOTE_PORT, QAbstractSocket::ShareAddress);//,QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    qDebug() <<"Start Listen UDP PORT:"<< YOGO_REMOTE_PORT;
    connect(udpSocket,SIGNAL(readyRead()),this,SLOT(netDatacome()));

    m_RunTimer = new QTimer(this);
    int heartBeatRate = 10;//100ms
    m_RunTimer->start(heartBeatRate);
    //connect(m_RunTimer, SIGNAL(timeout()), this, SLOT(robot_monitor()));

   nSendIndex = 0;
   door_cmd[0]=0;
   door_cmd[1]=0;
   door_cmd[2]=0;

   ui->edit_target->setText("10");

}

MainWindow::~MainWindow()
{
    delete ui;
    if( m_MotionPort != NULL )
    {
        m_MotionPort->close();
        delete m_MotionPort;
    }
}

void MainWindow::TimeUpdate_COM()
{
    mytimer_comm->stop();
    qDebug()<<"Recv Data:"<<myReaddata.toHex();
}

bool MainWindow::portOpen()
{
    qDebug()<<"Port Open";
    if( m_MotionPort != NULL )
    {
        m_MotionPort->close();
        delete m_MotionPort;
    }
    m_bOpend    = false;

    QString portName = ui->comboBox_Port->currentText();
    if(portName.length()<1)return false;
    QSerialPortInfo info(portName);
    m_MotionPort = new QSerialPort(info);
    m_MotionPort->setPortName(portName);

    m_MotionPort->setDataBits(QSerialPort::Data8);
    m_MotionPort->setParity(QSerialPort::NoParity);
    m_MotionPort->setFlowControl(QSerialPort::NoFlowControl);
    //m_MotionPort->setReadBufferSize(4096);

    //m_MotionPort->setBaudRate(QSerialPort::Baud115200);
    m_MotionPort->setBaudRate(115200);
   // m_MotionPort->setBaudRate(1000000);      // 1000000   // 115200

    connect(m_MotionPort,SIGNAL(readyRead()),this,SLOT(rx_data_come())); //setto block mode

    mytimer_comm = new QTimer(this);
    connect(mytimer_comm,SIGNAL(timeout()),this,SLOT(TimeUpdate_COM()));

    if(!m_MotionPort->open(QIODevice::ReadWrite))
    {
        qDebug() << "Failed to open port %1, error: %2" << portName << m_MotionPort->errorString() << endl;
        return false;
    }
    else
    {
        m_bOpend    = true;
        qDebug()<<"Serial Open OK!";
        //m_pRobot->Robot_nDeviceReadyFlags |= FLAGS_UART_READY;
        //m_bRun  = true;
        return true;
    }

    //connect(m_MotionPort, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(error(QSerialPort::SerialPortError)), Qt::DirectConnection);

    //https://www.cnblogs.com/lvchaoshun/p/5911903.html  ubuntu 串口权限

}

bool MainWindow::portClose()
{
    //return port open status
    if( m_MotionPort == NULL )
    {
        return false;
    }
    if(m_MotionPort->isOpen())
    {
        m_MotionPort->close();
        qDebug()<<"Port Closed!";
        return false;
    }
    return m_MotionPort->isOpen();
}


void MainWindow::on_btn_Open_clicked()
{
    if(m_bOpend == false)
    {
       m_bOpend = portOpen();
    }else
    {
        m_bOpend = portClose();
    }
    if(m_bOpend)
    {
        ui->lab_open_state->setText("Opened");
        ui->btn_Open->setText("Close");
    }else
    {
    ui->lab_open_state->setText("Closed");
    ui->btn_Open->setText("Open");
    }
}


void MainWindow::rx_data_come()
{
    QByteArray rx2 = m_MotionPort->readAll();
    rx.append(rx2);
    //qDebug()<<"RX2:"<<rx.toHex();

    do{
    if( rx.length()<8) return;
    if((rx.at(0)==0x68)&&(rx.at(1)==0x6C))
    {
        int needLen = (uint8_t) (rx.at(5))+8;
        if(rx.length()< needLen )
        {
            return;
        }
        qDebug()<<"Exec:"<<rx.toHex();
        if(rx.at(4)==(char)(0x32|0x80))
        {
            qDebug()<<"wait OTA Cmd";
            if(DownloadCache.length()>100)
            {
                sendDownloadHeader("333");
            }
        }
        else  if(rx.at(4)==(char)(0x30|0x80))
        {
            //mytimer_comm->stop();
            sendDownloadPackage(0);
        }
        else if(rx.at(4)==(char)(0x31|0x80))
        {
            mytimer_comm->stop();
            uint32_t offset = 0;
            offset += (uint32_t) ((uint8_t)rx.at(6));
            offset += (((uint32_t) ((uint8_t)rx.at(7)))<<8);
            offset += (((uint32_t) ((uint8_t)rx.at(8)))<<16);
            offset += (((uint32_t) ((uint8_t)rx.at(9)))<<24);

            uint32_t sent = offset + ((uint8_t)rx.at(5)) - 4; //len

            if(sent < DownloadCache.length())
            {
               sendDownloadPackage(sent);
            }else
            {
                qDebug()<<"Finish"<<sent;
                on_pushButton_2_clicked();//run app
            }
            ui->progressBar->setValue(sent*100/DownloadCache.length());
            ui->lab_percent->setText(QString().sprintf("%d / %d",sent,DownloadCache.length()));
        }
        else
        {
            qDebug()<<"RX"<<rx.toHex();
        }
        rx.remove(0,needLen);
    }
    else
    {
        //qDebug()<<"RX3:"<<rx.toHex();
        //rx.clear();
        rx.remove(0,1);
    }
    }
    while(rx.length()>64);

}

void MainWindow::on_btn_File_clicked()
{
    //QString myfilter = "*.bin *.fw";

    QFileDialog *fileDialog = new QFileDialog(this);//创建一个QFileDialog对象，构造函数中的参数可以有所添加???    fileDialog->setWindowTitle(tr("Save As"));//设置文件保存对话框的标题
    fileDialog->setAcceptMode(QFileDialog::AcceptOpen);//设置文件对话框为保存模式
    fileDialog->setFileMode(QFileDialog::ExistingFile);//设置文件对话框弹出的时候显示任何文件，不论是文件夹还是文件
    fileDialog->setViewMode(QFileDialog::Detail);//文件以详细的形式显示，显示文件名，大小，创建日期等信息；

    //还有另一种形式QFileDialog::List，这个只是把文件的文件名以列表的形式显示出来
    fileDialog->setGeometry(10,30,300,200);//设置文件对话框的显示位置
    fileDialog->setDirectory(".");//设置文件对话框打开时初始打开的位???    //fileDialog->setFilter(myfilter);//(tr("FrameWare Files(*.bin *.fw)"));//设置文件类型过滤???
    if(fileDialog->exec() == QDialog::Accepted)
    {
        //注意使用的是QFileDialog::Accepted或者QDialog::Accepted,不是QFileDialog::Accept
        QString path = fileDialog->selectedFiles()[0];//得到用户选择的文件名
        qDebug()<<"File"<<path;
        ui->edit_file->setText(path);
        int size = QFile(path).size();
        ui->lab_downloadFileInfo->setText(QString().sprintf("FileSize: %d Bytes",size));
        ui->lab_percent->setText(QString().sprintf("0 / %d",size));
        ui->progressBar->setValue(0);
        if(size>0)
        {
            m_FileRead = path;
            on_btn_Download_clicked();
        }else
        {
            m_FileRead = "";
        }
    }

}
// downloading
#define CMD_DownAppHead		0x30		//下载开始命令  版本号+文件大小+文件名
#define CMD_DownAppData		0x31		//下载文件内容

#define CMD_Ping			0x71		//ping从机
#define CMD_RunApp			0x72		//运行应用程序，可以加参数
#define CMD_ReBoot			0x73		//重启


void sleep(unsigned int msec)
{

    QTime dieTime = QTime::currentTime().addMSecs(msec);

    while( QTime::currentTime() < dieTime )

    QCoreApplication::processEvents(QEventLoop::AllEvents, 100);

}
void MainWindow::on_btn_Download_clicked()
{
    uint8_t Filebuff[256*1024]; // file buffer,max 256k
    if(m_FileRead.length()>2)
    {
        bool ok;
        uint8_t tar = ui->edit_target->text().toInt(&ok,16);
        if(ok)
        {
            DownloadTar = tar;
            QFile file(m_FileRead);
            if(!file.open(QIODevice::ReadOnly))return;

            DownloadCache = file.readAll();
            if(file.size()<256*1024)
                file.read((char *)Filebuff,file.size());
            else
            {
                QMessageBox::warning(this,"File size warning","File size overlaod...",QMessageBox::Ok,QMessageBox::Ok);
                return;
            }

            file.close();
            DownloadOffset =0;
            QMessageBox::warning(this,"start","Downloading...",QMessageBox::Ok,QMessageBox::Ok);
            sendDownloadHeader(m_FileRead.section('/',-1));
            //sendDownloadPackage(DownloadOffset);
            mytimer_comm->setInterval(1000);
            mytimer_comm->start();
            return ;
        }
    }
}


void MainWindow::sendDownloadHeader(QString fileName)
{
    qDebug()<<"sendDownloadHeader"<<fileName;
    QByteArray dat;
    dat.clear();
    dat.append((char )0x68);
    dat.append((char )0x6C);
    dat.append((char )0x50);
    dat.append((char )(DownloadTar));
    dat.append((char )0x30);    //header
    //uint8_t len = 8 + 4 +fileName.length(); //version + size + name
    dat.append((char )64);
    dat.append(64+4,'0');
    dat.append((char)checkSum_Calc(dat));
    dat.append((char )0x16);
    qDebug()<<"TX:"<<dat.toHex();
    m_MotionPort->write(dat);
    mytimer_comm->start();
}

void MainWindow::sendDownloadPackage(int offset)
{
    QByteArray dat;
    dat.clear();
    dat.append((char )0x68);
    dat.append((char )0x6C);
    dat.append((char )0x50);
    dat.append((char )(DownloadTar));
    dat.append((char )0x31);//file
    uint8_t len = 0;
    len = 64;
    dat.append((char )len + 4);
    dat.append(char(offset&0xFF));
    dat.append(char((offset>>8)&0xFF));
    dat.append(char((offset>>16)&0xFF));
    dat.append(char((offset>>24)&0xFF));
    QByteArray append = DownloadCache.mid(offset,len);
    dat.append(append);
    if(append.length() !=64) dat.append(64-append.length(),'0');
    dat.append((char)checkSum_Calc(dat));
    dat.append((char )0x16);
    m_MotionPort->write(dat);
    qDebug()<<"TX:"<<offset<<dat.toHex();
    mytimer_comm->start();
}


unsigned char MainWindow::checkSum_Calc(QByteArray buf)
{

    int sum = 0;

    for(int i = 0; i < buf.size(); i++)

    {

        sum += buf.at(i) & 0x000000FF;

    }

    return   (sum & 0xFF);

}
unsigned char  CheckSumCreat(unsigned char  *s,unsigned char  length)
{
    unsigned char  sum = 0;
    while(length--)
    {
        sum += *s++;
    }
    return sum;
}

unsigned char* MainWindow::mySendPackageStd(unsigned char iID,unsigned char  iCMD,unsigned char  Length,unsigned char  *data)
{
    int i;
    unsigned char iSendBuffer[256] = {0};

    iSendBuffer[0] = 0x68;
    iSendBuffer[1] = 0x6C;
    iSendBuffer[2] = 0x50;
    iSendBuffer[4] = iID;
    iSendBuffer[4] = iCMD;
    iSendBuffer[5] = Length;

    for(i=0;i<Length;i++)
            iSendBuffer[i+6] = data[i];

    iSendBuffer[Length+6] = CheckSumCreat(iSendBuffer,(Length+6));
    iSendBuffer[Length+7] = 0x16;

    //RS485_Send(iSendBuffer,(comm.LENGTH+8));
    qDebug()<<"TX:"<< QByteArray((char *)iSendBuffer,Length+8).toHex();

    m_MotionPort->write((char *)iSendBuffer,(Length+8));
    return iSendBuffer;
}


void MainWindow::on_pushButton_clicked()
{
    bool ok;
    unsigned char ID = ui->edit_target->text().toInt(&ok,16);
    // reset target
    mySendPackageStd(ID,0x73,0,0);
}

void MainWindow::on_pushButton_2_clicked()
{
    bool ok;
    unsigned char ID = ui->edit_target->text().toInt(&ok,16);
    // reset target
    unsigned char buff[64+4];
    mySendPackageStd(ID,CMD_RunApp,64+4,buff);
}

void MainWindow::on_pushButton_3_clicked()
{
    bool ok;
    unsigned char buff[32];
    unsigned char ID = ui->edit_target->text().toInt(&ok,16);
    int recv_cnt = 0;
    // reset target
    mySendPackageStd(ID,CMD_Ping,0,0);

}


void MainWindow::netDatacome(void)
{
   qint64 pendLen = udpSocket->bytesAvailable();
   int length = udpSocket->readDatagram(((char *)inbuf),pendLen,&m_RemoteFrom);
   qDebug()<<"RobotRX:"<<QByteArray((char *)inbuf,length).toHex();
   if(length<8)return;
  if((inbuf[0]==0x55)&&(inbuf[1]==0xaa))
  {
      //UART_RAWDATA_RX * urx = (UART_RAWDATA_RX *) inbuf;
      //qDebug()<<"PRY:"<<urx->pitch*0.01f<<urx->roll*0.01f<<urx->yaw*0.01f;
      //emit pitchRollYaw(urx->pitch*0.01f,urx->roll*0.01f,urx->yaw*0.01f);
  }
  else if((inbuf[0]==0x68)&&(inbuf[1]==0x6C))
  {
      if(inbuf[4]==(0x71|0x80))
      {
          qDebug()<<"PING CMD reply from 0x"<<QByteArray(1, inbuf[2]).toHex();
      }
      else if(inbuf[4]==(0x04|0x80))
      {
          qDebug()<<"carbinet: "<<QByteArray((char *)&inbuf[8],3).toHex();
          ui->lab_door->setText(QByteArray((char *)&inbuf[8],1).toHex());
          ui->lab_cabinet1->setText( QByteArray((char *)&inbuf[9],1).toHex());
          ui->lab_cabinet2->setText( QByteArray((char *)&inbuf[10],1).toHex());
      }
  }
}

void MainWindow::sendtonet()
{
    outbuf[0] = 0x68;
    outbuf[1] = 0x6C;
    outbuf[2] = 0x80;//from
    outbuf[3] = 0x50;// 0x20;//to
    outbuf[4] = 0x04;
    outbuf[5] = 6;//tx_bodySize;
    //memcpy(&(outbuf[6]),&m_pRobot->tx_raw,tx_bodySize);
    outbuf[6] = 0x20;
    outbuf[7] = 0x04;
    outbuf[8] = door_cmd[0]&0x0F;
    outbuf[9] = (door_cmd[1]&0x0F);
    outbuf[10] = door_cmd[2]&0x0F;
    outbuf[11] = 0;
    outbuf[12] = ~ calc_checksum(&(outbuf[0]),12);
    outbuf[13] = 0x16;
    udpSocket->writeDatagram((char *)outbuf,14,robotAddr,YOGO_REMOTE_PORT);
    qDebug()<<"TXRobot:"<<QByteArray((char *)outbuf,14).toHex();
}

void MainWindow::on_btn_Open_2_clicked()
{
    door_cmd[0]= CABIN_CMD_OPEN;
    sendtonet();
}

void MainWindow::on_btn_Close_clicked()
{
    door_cmd[0]= CABIN_CMD_CLOSE;
    sendtonet();

}

void MainWindow::on_btn_first_open_clicked()
{
    door_cmd[1]= CABIN_CMD_OPEN;
    sendtonet();

}

void MainWindow::on_btn_first_close_clicked()
{
    door_cmd[1]= CABIN_CMD_CLOSE;
    sendtonet();

}

void MainWindow::on_btn_push_first_clicked()
{
    door_cmd[1]= CABIN_CMD_PUSH;
    sendtonet();

}

void MainWindow::on_btn_pull_first_clicked()
{
    door_cmd[1]= CABIN_CMD_PULL;
    sendtonet();

}

void MainWindow::on_btn_second_open_clicked()
{
    door_cmd[2]= CABIN_CMD_OPEN;
    sendtonet();

}

void MainWindow::on_btn_second_close_clicked()
{
    door_cmd[2]= CABIN_CMD_CLOSE;
    sendtonet();

}

void MainWindow::on_btn_second_push_clicked()
{
    door_cmd[2]= CABIN_CMD_PUSH;
    sendtonet();

}

void MainWindow::on_btn_second_pull_clicked()
{
    door_cmd[2]= CABIN_CMD_PULL;
    sendtonet();

}

void MainWindow::on_btn_LED_clicked()
{
    led_cmd[0] = ui->lineEdit_1->text().toInt();
    led_cmd[1] = ui->lineEdit_2->text().toInt();
    led_cmd[2] = ui->lineEdit_3->text().toInt();
    led_cmd[3] = ui->lineEdit_4->text().toInt();
    led_cmd[4] = ui->lineEdit_5->text().toInt();
    led_cmd[5] = ui->lineEdit_6->text().toInt();

    outbuf[0] = 0x68;
    outbuf[1] = 0x6C;
    outbuf[2] = 0x80;//from
    outbuf[3] = 0x50;// 0x40;//to
    outbuf[4] = 0x05;
    outbuf[5] = 18;//tx_bodySize;
    //memcpy(&(outbuf[6]),&m_pRobot->tx_raw,tx_bodySize);
    outbuf[6] = 0x20;
    outbuf[7] = 16;
    outbuf[8] = led_cmd[0];
    outbuf[9] = 0;
    outbuf[10] = led_cmd[1];
    outbuf[11] = 0;
    outbuf[12] = led_cmd[2];
    outbuf[13] = 0;
    outbuf[14] = led_cmd[3];
    outbuf[15] = 0;
    outbuf[16] = led_cmd[4];
    outbuf[17] = 0;
    outbuf[18] = led_cmd[5];
    outbuf[19] = 0;
    outbuf[20] = 0;
    outbuf[21] = 0;
    outbuf[22] = 0;
    outbuf[23] = 0;

    outbuf[24] = ~ calc_checksum(&(outbuf[0]),24);
    outbuf[25] = 0x16;
    udpSocket->writeDatagram((char *)outbuf,26,robotAddr,YOGO_REMOTE_PORT);
    qDebug()<<"TXRobot:"<<QByteArray((char *)outbuf,26).toHex();
}
