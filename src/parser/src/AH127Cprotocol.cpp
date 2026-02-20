#include "AH127Cprotocol.h"
#include <stdint.h>
#include <QTextCodec>

AH127Cprotocol::AH127Cprotocol(QString portName, int baudRate, QObject *parent)
{
    m_port.setBaudRate(QSerialPort::Baud9600);
    m_port.setPortName(portName);
    m_port.open(QIODevice::ReadWrite);


    char cmd_1[6]; //задание формата посылки и частоты выдачи данных, 2.15 и 2.17
    cmd_1[0] = 0x77;
    cmd_1[1] = 0x05;
    cmd_1[2] = 0x00;
    cmd_1[3] = 0x56;
    cmd_1[4] = 0x05;
    cmd_1[5] = 0x60;
    m_port.write(cmd_1, 6);
    if (m_port.waitForBytesWritten(500)) {
    qDebug() << "cmd_1 отправлена успешно (0x56 0x05)";
} else {
    qDebug() << "Ошибка отправки cmd_1:" << m_port.errorString();
}

    char cmd_2[6];
    cmd_2[0] = 0x77;
    cmd_2[1] = 0x05;
    cmd_2[2] = 0x00;
    cmd_2[3] = 0x0C;
    cmd_2[4] = 0x05;
    cmd_2[5] = 0x16;
    m_port.write(cmd_2, 6);
    if (m_port.waitForBytesWritten(500)) {
    qDebug() << "cmd_2 отправлена успешно (0x0C 0x05)";
} else {
    qDebug() << "Ошибка отправки cmd_2:" << m_port.errorString();
}

    QTimer *timer = new QTimer(this);
    connect(&m_port, &QSerialPort::readyRead, this, &AH127Cprotocol::readData);
    connect(&m_port, &QSerialPort::readyRead, this, &AH127Cprotocol::readyReadForTimer);
    connect(timer, &QTimer::timeout, this, &AH127Cprotocol::timeoutSlot);
    timer->start(10000);
}

unsigned short AH127Cprotocol::calculateCRC(unsigned char data[], unsigned int length) {
     unsigned int i;
     unsigned short crc = 0;
     for(i=0; i<length; i++){
         crc += data[i];
     }
    return crc & 0xFF;
}

bool AH127Cprotocol::correctChecksum (QByteArray const &ba) {
    if (calculateCRC((uchar*)ba.data(), 55) == ba[55]) {
        return true;
    }
    return false;
}

void AH127Cprotocol::readData() {
    QByteArray newData = m_port.readAll();
    if (!newData.isEmpty()) {
        m_buffer.append(newData);
    } else {
        qDebug() << "readAll() вернул 0 байт";
    }
    parseBuffer();
}

void AH127Cprotocol::readyReadForTimer() {
    time.restart();
}

void AH127Cprotocol::timeoutSlot(){
 double deltaTMax = 2000;
    if (time.elapsed()>deltaTMax) {
        char cmd_1[6]; //задание формата посылки, 2.15
        cmd_1[0] = 0x77;
        cmd_1[1] = 0x05;
        cmd_1[2] = 0x00;
        cmd_1[3] = 0x0C;
        cmd_1[4] = 0x05;
        cmd_1[5] = 0x16;
        m_port.write(cmd_1, 6);
        m_port.waitForBytesWritten();

        char cmd_2[6]; //задание частоты выдачи данных, 2.17
        cmd_2[0] = 0x77;
        cmd_2[1] = 0x05;
        cmd_2[2] = 0x00;
        cmd_2[3] = 0x56;
        cmd_2[4] = 0x05;
        cmd_2[5] = 0x60;
        m_port.write(cmd_2, 6);
        m_port.waitForBytesWritten();
    }
}

float ThreeBytesToFloat(QByteArray const buf) {
    float result = 0.0;
    result += int(buf[0] & 0x0F) * 100;
    result += int((buf[1] & 0xF0) >> 4) * 10;
    result += int(buf[1] & 0x0F);
    result += int((buf[2] & 0xF0) >> 4) * 0.1;
    result += int(buf[2] & 0x0F) * 0.01;
    result *= (buf[0] & 0xF0) ? (-1) : (1);
    return result;
}

float ThreeBytesToFloatAccel(QByteArray const buf) {
    float result = 0.0;
    result += int(buf[0] & 0x0F);
    result += int((buf[1] & 0xF0) >> 4) * 0.1;
    result += int(buf[1] & 0x0F)*0.01;
    result += int((buf[2] & 0xF0) >> 4) * 0.001;
    result += int(buf[2] & 0x0F) * 0.0001;
    result *= (buf[0] & 0xF0) ? (-1) : (1);
    return result*9.81;
}

float ThreeBytesToFloatMagn(QByteArray const buf) {
    float result = 0.0;
    result += int(buf[0] & 0x0F)*0.1;
    result += int((buf[1] & 0xF0) >> 4) * 0.01;
    result += int(buf[1] & 0x0F)*0.001;
    result += int((buf[2] & 0xF0) >> 4) * 0.0001;
    result += int(buf[2] & 0x0F) * 0.00001;
    result *= (buf[0] & 0xF0) ? (-1) : (1);
    return result;
}

float FourBytesToFloatQvat(QByteArray const buf) {
    float result = 0.0;
    result += int(buf[0] & 0x0F);
    result += int((buf[1] & 0xF0) >> 4) * 0.1;
    result += int(buf[1] & 0x0F)*0.01;
    result += int((buf[2] & 0xF0) >> 4) * 0.001;
    result += int(buf[2] & 0x0F) * 0.0001;
    result += int(buf[3] & 0x0F) * 0.00001;
    result += int(buf[3] & 0x0F) * 0.000001;
    result *= (buf[0] & 0xF0) ? (-1) : (1);
    return result;
}

void PrintMsg(DataFromAH127C const& msg) {
    qDebug() << "yaw: " <<msg.yaw;
    qDebug() << "pitch: " <<msg.pitch;
    qDebug() << "roll: " <<msg.roll;
    qDebug() << "X_accel: " <<msg.X_accel;
    qDebug() << "Y_accel: " <<msg.Y_accel;
    qDebug() << "Z_accel: " <<msg.Z_accel;
    qDebug() << "X_rate: " <<msg.X_rate;
    qDebug() << "Y_rate: " <<msg.Y_rate;
    qDebug() << "Z_rate: " <<msg.Z_rate;
    qDebug() << "X_magn: " <<msg.X_magn;
    qDebug() << "Y_magn: " <<msg.Y_magn;
    qDebug() << "Z_magn: " <<msg.Z_magn;
    qDebug() << "first_qvat " <<msg.first_qvat;
    qDebug() << "second_qvat " <<msg.second_qvat;
    qDebug() << "third_qvat " <<msg.third_qvat;
    qDebug() << "four_qvat " <<msg.four_qvat;
}

void AH127Cprotocol::parseBuffer() {
    if (m_buffer.size() <= 4 ) {
        return;
    }

    if (flag_start_cal == 1) {
    QByteArray header_calibration_start((char*) &(calibr_start),sizeof(Header_AH_calibration_start));
    int index_calibr_start = m_buffer.indexOf(header_calibration_start);
    if (index_calibr_start == -1) {
        qDebug() << "команда начала калибровки не распознана";
        return;
    } else {
        flag_calibration_start = 1;
        flag_calibration_end = 0;
        qDebug() << "команда калибровки дошла до датчика, можно начинать калибровку";
    }
    flag_start_cal = 0;
    }

    if (flag_finish_cal == 1) {
    QByteArray header_calibration_end((char*) &(calibr_end),sizeof(Header_AH_calibration_end));
    int index_calibr_end = m_buffer.indexOf(header_calibration_end);
    if (index_calibr_end == -1) {
        qDebug() << "команда окончания калибровки не распознана";
        return;
    } else {
        flag_calibration_start = 0;
        flag_calibration_end = 1;
        qDebug() << "команда окончания калибровки дошла до датчика, результат калибровки записан";
        //и тогда возобоновляем посылку данных
        char cmd_1[6]; //задание формата посылки, 2.15
        cmd_1[0] = 0x77;
        cmd_1[1] = 0x05;
        cmd_1[2] = 0x00;
        cmd_1[3] = 0x0C;
        cmd_1[4] = 0x05;
        cmd_1[5] = 0x16;
        m_port.write(cmd_1, 6);
        m_port.waitForBytesWritten();

        char cmd_2[6]; //задание частоты выдачи данных, 2.17
        cmd_2[0] = 0x77;
        cmd_2[1] = 0x05;
        cmd_2[2] = 0x00;
        cmd_2[3] = 0x56;
        cmd_2[4] = 0x05;
        cmd_2[5] = 0x60;
        m_port.write(cmd_2, 6);
        m_port.waitForBytesWritten();

    }
    flag_finish_cal = 0;
    }

    QByteArray header((char*) &(data.header),sizeof(Header_AH));
    int index = m_buffer.indexOf(header);
    
    if (index == -1) {
        qDebug() << "no message";
        return;
    }

    if (index > 0) {
        m_buffer.remove(0, index);
        index = 0;
    }

    if (m_buffer.size() < (index + 57)) {
        return;
    }

    QByteArray packetData = m_buffer.mid(1, 56);
    if (correctChecksum(packetData)) {
        DataFromAH127C msg;
        QByteArray tmp = m_buffer.mid(0, 57);

        msg.pitch = ThreeBytesToFloat(tmp.mid(4,3));
        msg.roll = ThreeBytesToFloat(tmp.mid(7, 3));
        msg.yaw = ThreeBytesToFloat(tmp.mid(10, 3));
        msg.X_accel = ThreeBytesToFloatAccel(tmp.mid(13, 3));
        msg.Y_accel = ThreeBytesToFloatAccel(tmp.mid(16, 3));
        msg.Z_accel = ThreeBytesToFloatAccel(tmp.mid(19, 3));
        msg.X_rate = ThreeBytesToFloat(tmp.mid(22, 3));
        msg.Y_rate = ThreeBytesToFloat(tmp.mid(25, 3));
        msg.Z_rate = ThreeBytesToFloat(tmp.mid(28, 3));
        msg.X_magn = ThreeBytesToFloatMagn(tmp.mid(31, 3));
        msg.Y_magn = ThreeBytesToFloatMagn(tmp.mid(34, 3));
        msg.Z_magn = ThreeBytesToFloatMagn(tmp.mid(37, 3));
        msg.first_qvat = FourBytesToFloatQvat(tmp.mid(40, 4));
        msg.second_qvat = FourBytesToFloatQvat(tmp.mid(44, 4));
        msg.third_qvat = FourBytesToFloatQvat(tmp.mid(48, 4));
        msg.four_qvat = FourBytesToFloatQvat(tmp.mid(52, 4));
        
        data = msg;
        PrintMsg(msg); 
        m_buffer.remove(0, index+57);
    } else {
        m_buffer.remove(0, index+1);
    }
}

