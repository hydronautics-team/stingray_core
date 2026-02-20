#ifndef AH127CPROTOCOL_H
#define AH127CPROTOCOL_H

#include <QObject>
#include <QSerialPort>
#include <QDebug>
#include <QTimer>
#include <QTime>
;
//класс протокола
#pragma pack(push,1)
//заглушка для заголовка послыки
struct Header_AH {
    uint8_t identif = 0x77;
    uint8_t lenght = 0x38;
    uint8_t adress = 0x00;
    uint8_t command = 0x59;
};
//структура, которая приходит от датчика
struct DataFromAH127C {
    Header_AH header;

    float yaw;
    float pitch=0;
    float roll=0;
    float X_accel=0;
    float Y_accel=0;
    float Z_accel=0;
    float X_rate=0;
    float Y_rate=0;
    float Z_rate=0;
    float X_magn=0;
    float Y_magn=0;
    float Z_magn=0;
    float first_qvat=0;
    float second_qvat=0;
    float third_qvat=0;
    float four_qvat=0;
    float crc;
};

//заглушка для ответа от команды начала калибровки
struct Header_AH_calibration_start {
    uint8_t identif = 0x77;
    uint8_t lenght = 0x05;
    uint8_t adress = 0x00;
    uint8_t command = 0x91;
    uint8_t datafield = 0x00;
    uint8_t checksum = 0x96;
};

//заглушка для ответа от команды начала калибровки
struct Header_AH_calibration_end {
    uint8_t identif = 0x77;
    uint8_t lenght = 0x05;
    uint8_t adress = 0x00;
    uint8_t command = 0x92;
    uint8_t datafield = 0x00;
    uint8_t checksum = 0x97;
};
#pragma pack(pop)

class AH127Cprotocol : public QObject
{
    Q_OBJECT
public:
    explicit AH127Cprotocol(QString portName, int baudRate = 115200, QObject *parent = 0);
    DataFromAH127C data;//выходная структура
    Header_AH_calibration_start calibr_start;
    Header_AH_calibration_end calibr_end;
    quint8 flag_calibration_start = false;
    quint8 flag_calibration_end = false;

    bool correctChecksum (QByteArray const &ba);//это метод, который проверяет корректность чексуммы
public slots:
    void readData(); //слот, который будет вызываться в ответ на readyRead
    void readyReadForTimer();
    void timeoutSlot();

protected:
    unsigned short calculateCRC(unsigned char data[], unsigned int length);
    void parseBuffer();
    QByteArray m_buffer;
    int baudRate = 115200; //бодрейт
    QTime time;
    QTimer *timer;
public:
    QSerialPort m_port; //объект COM-порта
    quint8 flag_start_cal = 0;
    quint8 flag_finish_cal = 0;
};

#endif // AH127CPROTOCOL_H
