#ifndef ENUMCODE_H
#define ENUMCODE_H
enum ERRORCODE
{
    NORMAL = 0,
    NOINSTALLSDK,
    NODEVICES,
    OUTOFRANGE,
    PARAMETERERR,
    COLLECTIONCYCLEDATALOSS, //采集周期太长导致数据丢失
    CREATEFFECTERR,
    ENCODINGFAILED,
    FFBERR,
    FIRMWARETOOOLD, //固件版本过低
    PITHOUSENOTREADY, // PitHouse未准备好
};

enum PRODUCTTYPE
{
    PRODUCT_WHEELBASE = 0,
    PRODUCT_STEERINGWHEEL,
    PRODUCT_DISPLAYSCREEN,
    PRODUCT_PEDALS,
    PRODUCT_METER,
    PRODUCT_ADAPTER,
    PRODUCT_HANDBRAKE,
    PRODUCT_GEARSHIFTER,
    PRODUCT_UNKNOWDEVICE,
};
#endif // BASESINGLETON_H
