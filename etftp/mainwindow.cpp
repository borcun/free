#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  init();
}

MainWindow::~MainWindow()
{
  sock->close();
  delete sock;

  delete ui;
}

void MainWindow::init() {
  sock = new QUdpSocket( this );

  if( !sock->bind( QHostAddress::LocalHost, 20000 ) ) {
    qDebug() << "Bind failed";
  }

  return;
}

void MainWindow::on_send_btn_clicked()
{
  ETFTPRRQ rrq;
  char *stream = new char[ rrq.len ];

  rrq.filename_len = 9;
  memcpy( rrq.filename, "hello.txt", rrq.filename_len );
  rrq.mode_len = 8;
  memcpy( rrq.mode, MODE_NETASCII, rrq.mode_len );

  rrq.serialize( stream );

  if( -1 == sock->writeDatagram( stream, rrq.len, QHostAddress::LocalHost, 69 ) ) {
    qDebug() << "Socket write failed";
  }
  else {
    QThread::msleep( 100 );

    char rcv_data[ 512 ] = { 0x00 };
    qint64 rcv_len = 512;

    if( -1 == ( rcv_len = sock->readDatagram( rcv_data, rcv_len ) ) ) {
      qDebug() << "Socket read failed";
    }
    else {
      ETFTPData data;
      ETFTPOAck ack;

      char *start = data.getData( rcv_data );
      QString str( start );

      ui->answer->setText( str );
      ack.serialize( stream );

      QThread::msleep( 100 );

      sock->writeDatagram( stream, ack.len, QHostAddress::LocalHost, 69 );
    }
  }

  delete [] stream;

  return;
}
