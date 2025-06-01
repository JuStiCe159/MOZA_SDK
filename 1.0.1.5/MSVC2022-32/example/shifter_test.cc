#include "mozaAPI.h"
#include "shifter_device.h"
#include <conio.h>
#include <atomic>
#include <thread>
#include <functional>

/**
 * @brief shifter device wrapper
 */
class DeviceWrapper
{
public:
    // Pass the current gear (after the change) to the callback function
    using Callback = std::function<void(int)>;

    // Default constructor
    DeviceWrapper() = default;

    // Constructor, constructs with a device object
    DeviceWrapper(moza::ShifterDevice&& device)
        : device(std::make_unique<moza::ShifterDevice>(std::move(device))) {}

    // Destructor, automatic close the device
    ~DeviceWrapper()
    {
        stopWorkerThread();
        close();
    }

    // Copying and moving are disabled
    DeviceWrapper(const DeviceWrapper&)            = delete;
    DeviceWrapper& operator=(const DeviceWrapper&) = delete;
    DeviceWrapper(DeviceWrapper&&)                 = delete;
    DeviceWrapper& operator=(DeviceWrapper&&)      = delete;

    // Set the operating device
    void setDevice(moza::ShifterDevice&& device) { this->device = std::make_unique<moza::ShifterDevice>(std::move(device)); }

    // Check if the worker thread is running
    bool isRunning() const { return thread && flag; }

    // Set the callback function for the current gear changed event
    void setOnCurrentGearChanged(Callback callback) { this->callback = std::move(callback); }

    // Get the most recent gear obtained by the worker thread
    int getCurrentGear() const { return currentGear; }

    // Check if the device is open
    bool isOpen() const { return device && device->isOpen(); }

    // Open the device
    bool open() { return device && device->open(); }

    // Close the device
    void close()
    {
        if (device)
        {
            device->close();
        }
    }

    // Start the device state retrieval worker thread
    void startWorkerThread()
    {
        if (device && !thread)
        {
            flag = true;
            thread.reset(new std::thread(&DeviceWrapper::mainLoop, this));
        }
    }

    // Stop the device state retrieval worker thread
    void stopWorkerThread()
    {
        flag = false;
        if (thread && thread->joinable())
        {
            thread->join();
        }
    }

private:
    std::unique_ptr<moza::ShifterDevice> device;      // device pointer
    std::atomic<bool>                    flag;        // worker thread running flag
    std::unique_ptr<std::thread>         thread;      // worker thread
    Callback                             callback;    // current gear changed callback
    std::atomic<int>                     currentGear; // the latest gear

    // worker thread main loop
    void mainLoop()
    {
        while (flag && device->isConnected())
        {
            const int gear = device->getCurrentGear(); // the current button state
            if (gear != currentGear)
            {
                if (callback)
                {
                    callback(gear);
                }
                currentGear = gear;
            }
        }
        if (!device->isConnected()) // check device connection
        {
            // TODO: Handle device disconnected
            std::cerr << "The device has been disconnected.\n";
            flag = false;
        }
    }
};

int main(int argc, char* argv[])
{
    ERRORCODE err;
    auto      devices = moza::enumShifterDevices(err);
    if (devices.empty() || err != NORMAL)
    {
        std::cerr << "MOZA shifter device not found.\n";
        return -1;
    }

    std::cout << "number of MOZA shifter devices: " << devices.size() << '\n';
    // Example for using the DeviceWrapper class
    // You can also directly use the moza::ShifterDevice class and refer to the implementation of the DeviceWrapper class
    DeviceWrapper wrapper(std::move(devices[0]));
    if (!wrapper.isOpen() && !wrapper.open())
    {
        std::cerr << "Device open failed!\n";
        return -1;
    }
    // The callback function is executed in the worker thread, so it is recommended to avoid performing time-consuming operations within the callback to prevent impacting the reading and updating of device current gear
    wrapper.setOnCurrentGearChanged([](int gear) {
        std::cout << "current gear changes: " << gear << '\n';
    });
    wrapper.startWorkerThread();
    std::cout << "Start monitoring gear changes. Press any key to exit:\n";
    while (!_kbhit() && wrapper.isRunning())
    {
        Sleep(100);
    }
    wrapper.stopWorkerThread();
    wrapper.close();
    return 0;
}
