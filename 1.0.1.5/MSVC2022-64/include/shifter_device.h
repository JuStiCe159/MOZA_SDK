#pragma once

#include "./hid_device.h"

namespace moza {

class ShifterDevice : public HidDevice
{
public:
    using HidDevice::HidDevice;

    /**
     * @if english
     * @brief Get the current gear of the shifter
     * @return The current gear
     * @retval -1 for reverse gear, 0 for neutral gear, and 1-7 for forward gears.
     *         During gear shifting, since the neutral position is passed through, a value of 0 may appear during this process.
     * @warning This function waits for the HID report while reading data, until valid data is received or an error occurs.
                This means the execution time of this function may be relatively long, and it is not recommended to call it in the main thread.
     * @else
     * @brief 获取换挡器的当前档位
     * @return 当前档位
     * @retval -1为倒档，0为空档，1-7为前进档。在切换档位时，因为中途会经过空档，所以在这个过程中会出现0的值
     * @warning 该函数在读取数据时会等待HID报文，直到读取到有效数据或者发生错误。这意味着该函数的执行时间可能会比较长，不适合在主线程中调用
     * @endif
     */
    int getCurrentGear() const;
};

}
