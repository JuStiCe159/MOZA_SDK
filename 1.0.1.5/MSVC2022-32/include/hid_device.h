#pragma once

#include <string>
#include <vector>

namespace moza {

/**
 * @if english
 * @brief HID device class
 * @else
 * @brief HID设备类
 * @endif
 */
class HidDevice
{
public:
    /**
     * @if english
     * @brief Default constructor
     * @else
     * @brief 无参构造
     * @endif
     */
    HidDevice();

    /**
     * @if english
     * @brief Constructs with a device instance path
     * @param path device instance path
     * @else
     * @brief 从一个设备实例路径构造
     * @param path 设备实例路径
     * @endif
     */
    explicit HidDevice(const std::string& path);

    /**
     * @if english
     * @brief Destructor, automatically closes the device and releases resources
     * @else
     * @brief 析构函数，内部会自动关闭设备并释放资源
     * @endif
     */
    virtual ~HidDevice();

    // Copying is disabled
    HidDevice(const HidDevice&)            = delete;
    HidDevice& operator=(const HidDevice&) = delete;

    // Move only
    HidDevice(HidDevice&& other) noexcept;
    HidDevice& operator=(HidDevice&& other) noexcept;

    /**
     * @if english
     * @brief Get the device instance path
     * @return device instance path
     * @else
     * @brief 获取设备实例路径
     * @return 设备实例路径
     * @endif
     */
    const std::string& path() const;

    /**
     * @if english
     * @brief Set the device instance path
     * @param path device instance path
     * @else
     * @brief 设置设备实例路径
     * @param path 设备实例路径
     * @endif
     */
    void setPath(const std::string& path);

    /**
     * @if english
     * @brief Check if the device is already opened (only indicates the opened state, does not guarantee the connection state; use isConnected function to check the connection state)
     * @return Returns true if the device is opened; otherwise, returns false
     * @else
     * @brief 判断设备是否已被打开（仅表示打开状态，不保证连接状态，若需检查连接状态请使用isConnected函数）
     * @return 如果设备已经打开，则返回true；否则返回false
     * @endif
     */
    bool isOpen() const;

    /**
     * @if english
     * @brief Check if the device is in a connected state, useful for determining if the device is disconnected
     * @note Internally, this is determined based on the error code of the last device operation.
     *       It is recommended to use this function if a call to the button state retrieval function returns empty,
     *       to confirm whether the device has been disconnected
     * @return Returns true if the device is connected; otherwise, returns false
     * @else
     * @brief 判断设备是否处于已连接状态，可用于判断设备是否断连
     * @note 内部通过上一次设备操作的错误码判断。建议在执行获取按钮状态函数后返回值为空时，通过此函数加以判断设备是否已经断开连接
     * @return 如果设备已连接，则返回true；否则返回false
     * @endif
     */
    bool isConnected() const;

    /**
     * @if english
     * @brief Open the device using the device instance path
     * @return Returns true if the device was successfully opened; otherwise, returns false
     * @else
     * @brief 通过设备实例路径打开设备
     * @return 如果设备打开成功，则返回true；否则返回false
     * @endif
     */
    virtual bool open();

    /**
     * @if english
     * @brief Close the device
     * @else
     * @brief 关闭设备
     * @endif
     */
    virtual void close();

private:
    class HidDevicePrivate* m_impl;

protected:
    size_t               getReportSize() const;
    size_t               getNumInputReports() const;
    std::vector<uint8_t> read(size_t size) const;
    std::vector<uint8_t> readLatestReport() const;
};

inline HidDevice::HidDevice(HidDevice&& other) noexcept : m_impl() { std::swap(m_impl, other.m_impl); }

inline HidDevice& HidDevice::operator=(HidDevice&& other) noexcept
{
    if (this != &other)
    {
        std::swap(m_impl, other.m_impl);
    }
    return *this;
}

}
