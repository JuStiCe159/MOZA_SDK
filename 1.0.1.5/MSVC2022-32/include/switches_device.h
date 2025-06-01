#pragma once

#include "./hid_device.h"

enum ERRORCODE;

namespace moza {

/**
 * @if english
 * @brief Enumeration of switch indices for MOZA Switches devices
 * @note The enumeration values can be directly used as indices for the switch state array
 * @note Toggle Switch is a latching switch (it remains triggered after being pressed);
 *       Rotary Switch is a position switch (it stays at different positions or settings).
 * @else
 * @brief MOZA Switches（多功能组合开关）设备的开关索引枚举
 * @note 枚举值可直接用作开关状态数组的下标
 * @note Toggle Switch是保持型开关（按下触发后一直保持）；Rotary Switch是拨动型开关（保持在不同挡位/位置）
 * @endif
 */
enum SwitchesIndex
{
    HeadlightOff = 0,   /* Rotary Switch 1 */
    HeadlightPark,      /* Rotary Switch 1 */
    HeadlightHigh,      /* Rotary Switch 1 */
    HighBeam,           /* Rotary Switch 2 special */
    FlasherOff,         /* Rotary Switch 2 special */
    Flasher,            /* Rotary Switch 2 special */
    FogLight,           /* Toggle Switch */
    TurnRight,          /* Rotary Switch 3 */
    TurnSignalOff,      /* Rotary Switch 3 */
    TurnLeft,           /* Rotary Switch 3 */
    RearWiperOff,       /* Rotary Switch 4 */
    RearWiperSpray,     /* Rotary Switch 4 */
    RearWiperWash,      /* Rotary Switch 4 toggle */
    WiperSensitivity1,  /* Rotary Switch 5 */
    WiperSensitivity2,  /* Rotary Switch 5 */
    WiperSensitivity3,  /* Rotary Switch 5 */
    WiperSensitivity4,  /* Rotary Switch 5 */
    WiperSensitivity5,  /* Rotary Switch 5 */
    FrontWiperWash,     /* Toggle Switch */
    FrontWiperSingle,   /* Rotary Switch 6 toggle */
    FrontWiperOff,      /* Rotary Switch 6 */
    FrontWiperInterval, /* Rotary Switch 6 */
    FrontWiperLow,      /* Rotary Switch 6 */
    FrontWiperHigh,     /* Rotary Switch 6 */
    CruiseOnOff,        /* Toggle Switch */
    CruiseDecrease,     /* Toggle Switch */
    CruiseIncrease,     /* Toggle Switch */
    CruiseCancel,       /* Toggle Switch */

    MAX_SWITCHES_INDEX
};

/**
 * @if english
 * @brief Group of switches, specifically for Rotary Switches. Each group corresponds to the different positions of a single Rotary Switch
 * @else
 * @brief 开关组，仅针对Rotary Switch旋转型开关，每组就是一个Rotary Switch对应在不同位置
 * @endif
 */
const static std::vector<SwitchesIndex> SWITCHES_GROUPS[] = {
    {HeadlightOff, HeadlightPark, HeadlightHigh},                                                    /* Rotary Switch Group 1 */
    {HighBeam, FlasherOff, Flasher},                                                                 /* Rotary Switch Group 2 */
    {TurnRight, TurnSignalOff, TurnLeft},                                                            /* Rotary Switch Group 3 */
    {RearWiperOff, RearWiperSpray, RearWiperWash},                                                   /* Rotary Switch Group 4 */
    {WiperSensitivity1, WiperSensitivity2, WiperSensitivity3, WiperSensitivity4, WiperSensitivity5}, /* Rotary Switch Group 5 */
    {FrontWiperSingle, FrontWiperOff, FrontWiperInterval, FrontWiperLow, FrontWiperHigh},            /* Rotary Switch Group 6 */
};

/**
 * @if english
 * @brief MOZA Switches device class
 * @else
 * @brief MOZA Switches（多功能组合开关）设备类
 * @endif
 */
class SwitchesDevice : public HidDevice
{
    friend class SwitchesDevicePrivate;

public:
    /**
     * @if english
     * @brief Default constructor
     * @else
     * @brief 无参构造
     * @endif
     */
    SwitchesDevice();

    /**
     * @if english
     * @brief Constructs with a device instance path
     * @param path device instance path
     * @else
     * @brief 从一个设备实例路径构造
     * @param path 设备实例路径
     * @endif
     */
    explicit SwitchesDevice(const std::string& path);

    ~SwitchesDevice() override;

    SwitchesDevice(const SwitchesDevice&)            = delete;
    SwitchesDevice& operator=(const SwitchesDevice&) = delete;

    SwitchesDevice(SwitchesDevice&& other) noexcept;
    SwitchesDevice& operator=(SwitchesDevice&& other) noexcept;

    /**
     * @if english
     * @brief Open the device
     * @return True if the device is successfully opened; false otherwise
     * @else
     * @brief 打开设备
     * @return 设备是否成功打开
     * @endif
     */
    bool open() override;

    /**
     * @if english
     * @brief Close the device
     * @else
     * @brief 关闭设备
     * @endif
     */
    void close() override;

    /**
     * @if english
     * @brief Check if the rotary switch states are ready
     * @return Returns true if the states are ready; otherwise, returns false
     * @note This function is non-blocking when reading data
     * @else
     * @brief 检查旋转型开关的状态是否准备完毕
     * @return 如果准备完毕则返回true；否则返回false
     * @note 该函数在读取数据是非阻塞的
     * @endif
     */
    bool isRotarySwitchStateReady() const;

    /**
     * @if english
     * @brief Get the current switches state of the device
     * @param err Error code
     * @return A vector of device switches state
     * @retval Each element in the vector represents the state of a switch.
     *         The index starts from 0 and corresponds to the switch numbers in MOZA Pit House minus 1.
     *         Alternatively, refer to the moza::SwitchesIndex enum values.
     * @note Some switch state data needs to be retrieved from Pit House. Establishing communication between the SDK and Pit House takes time to complete.
     *       Therefore, the switch states obtained by this function may be inaccurate before the communication is fully established.
     *       You can check whether the preparation is complete by using the return value of the `isRotarySwitchStateReady` function or the error code `err`.
     *       This function is non-blocking when reading data
     * @else
     * @brief 获取设备当前的开关状态
     * @param err 错误码
     * @return 设备的开关状态数组
     * @retval 数组的每一项都表示一个开关的状态。下标从0开始，对应MOZA Pit House中的开关编号-1，也可以参考moza::SwitchesIndex枚举值
     * @note 开关状态部分数据需要从PitHouse获取，而SDK与PitHouse的通信建立需要一定的时间才能完成，所以在通信未完成时该函数获取到的开关状态不准确；
     *       可以通过`isRotarySwitchStateReady`函数返回值或者错误码`err`判断是否准备完毕；
     *       该函数在读取数据是非阻塞的。
     * @endif
     */
    std::vector<uint8_t> getStateInfo(ERRORCODE& err) const;

    /**
     * @if english
     * @brief Get the current switches state of the device (retrieved solely from the HID reports sent by the device) 
     * @return A vector of device switches state
     * @retval Each element in the vector represents the state of a switch.
     *         The index starts from 0 and corresponds to the switch numbers in MOZA Pit House minus 1.
     *         Alternatively, refer to the moza::SwitchesIndex enum values.
     * @note It is generally recommended to use the `getStateInfo` function to obtain the current switch state of the device.
     *       This function retrieves the switch state solely by reading the HID report, which does not include the device's initial state.
     *       It can be used when the device firmware is not supported or when PitHouse cannot be connected.
     *       This function is non-blocking when reading data.
     * @else
     * @brief 获取设备当前的开关状态（仅通过设备发送的HID报告来获取）
     * @return 设备的开关状态数组
     * @retval 数组的每一项都表示一个开关的状态。下标从0开始，对应MOZA Pit House中的开关编号-1，也可以参考moza::SwitchesIndex枚举值
     * @note 通常情况下建议使用`getStateInfo`函数来获取当前设备的开关状态。该函数仅通过读取HID报告来获取开关状态，这将无法获取到设备的初始状态。可以在设备固件不支持时或无法连接PitHouse时使用该函数。
     *       该函数在读取数据是非阻塞的。
     * @endif
     */
    std::vector<uint8_t> getStateInfoByHid() const;

private:
    SwitchesDevicePrivate* m_impl;
};

}
