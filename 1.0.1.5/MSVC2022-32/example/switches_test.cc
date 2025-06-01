#include "mozaAPI.h"
#include "switches_device.h"
#include <conio.h>
#include <mutex>
#include <thread>
#include <functional>

/**
 * @brief Button state
 */
enum ButtonState : uint8_t
{
    Pressed = 0,
    Released
};

/**
 * @brief Compares two device button state arrays for equality
 * @param bs1 The first button state array
 * @param bs2 The second button state array
 * @return Returns true if both button state arrays are of the same size and all elements are equal; otherwise, returns false
 */
bool buttonStateEquals(const std::vector<uint8_t>& bs1, const std::vector<uint8_t>& bs2)
{
    if (bs1.size() != bs2.size())
        return false;

    for (size_t i = 0; i < bs1.size(); ++i)
    {
        if (bs1[i] != bs2[i])
            return false;
    }
    return true;
}

/**
 * @brief Switches device wrapper
 */
class DeviceWrapper
{
public:
    // Pass the previous device state (before the change) and the current device state (after the change) as function parameters
    using DeviceStateCallback = std::function<void(const std::vector<uint8_t>&, const std::vector<uint8_t>&)>;

    // Pass the changed button's enum value and its updated state as function parameters
    using ButtonStateCallback = std::function<void(moza::SwitchesIndex, ButtonState)>;

    // Default constructor
    DeviceWrapper() = default;

    // Constructor, constructs with a device object
    DeviceWrapper(moza::SwitchesDevice&& device)
        : device(std::make_unique<moza::SwitchesDevice>(std::move(device))) {}

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
    void setDevice(moza::SwitchesDevice&& device) { this->device = std::make_unique<moza::SwitchesDevice>(std::move(device)); }

    // Check if the worker thread is running
    bool isRunning() const { return thread && flag; }

    // Callback setters
    // Device state changes include all button state changes
    void setOnDeviceStateChanged(DeviceStateCallback callback) { deviceCallback = std::move(callback); }
    // Button state changes include individual button state updates
    void setOnButtonStateChanged(ButtonStateCallback callback) { buttonCallback = std::move(callback); }

    // Get the most recent button state obtained by the worker thread
    std::vector<uint8_t> getLatestButtonState() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return lastState;
    }

    // Check if the device is open
    bool isOpen() const
    {
        return device && device->isOpen();
    }

    // Open the device
    bool open()
    {
        return device && device->open();
    }

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
    std::unique_ptr<moza::SwitchesDevice> device;         // device pointer
    std::atomic<bool>                     flag;           // worker thread running flag
    std::unique_ptr<std::thread>          thread;         // worker thread
    DeviceStateCallback                   deviceCallback; // device state changed callback
    ButtonStateCallback                   buttonCallback; // button state changed callback
    std::vector<uint8_t>                  lastState;      // the latest device state
    mutable std::mutex                    mutex;          // async mutex

    // worker thread main loop
    void mainLoop()
    {
        std::vector<uint8_t> previousState; // Record the previous button state
        while (flag)
        {
            ERRORCODE err;
            auto state = device->getStateInfo(err); // the current button state
            if (!state.empty() && (previousState.empty() || !buttonStateEquals(state, previousState)))
            {
                // ensure the size of previousState and state are consistent
                if (previousState.size() < state.size())
                    previousState.resize(state.size(), 0);

                // update the last state
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    lastState = state;
                }

                // execute device callback
                if (deviceCallback)
                {
                    deviceCallback(previousState, state);
                }

                // execute button callback
                if (buttonCallback)
                {
                    for (size_t i = 0; i < state.size(); ++i)
                    {
                        if (previousState[i] != state[i])
                        {
                            // 0 -> 1 Pressed
                            // 1 -> 0 Released
                            buttonCallback(static_cast<moza::SwitchesIndex>(i), state[i] ? Pressed : Released);
                        }
                    }
                }

                // update previous state
                previousState = std::move(state);
            }
            else if (!device->isConnected()) // check device connection
            {
                // TODO: Handle device disconnected
                std::cerr << "The device has been disconnected.\n";
                flag = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // avoid busy waiting
        }
    }
};

// print all button states of the device
void printDeviceState(const std::vector<uint8_t>& state)
{
    std::cout << "device state: ";
    for (const uint8_t button : state)
    {
        std::cout << static_cast<int>(button) << ' ';
    }
    std::cout << '\n';
}

int main(int argc, char* argv[])
{
    moza::installMozaSDK();
    ERRORCODE err;
    auto      devices = moza::enumSwitchesDevices(err);
    if (devices.empty() || err != NORMAL)
    {
        std::cerr << "MOZA Switches device not found.\n";
        return -1;
    }

    std::cout << "number of the MOZA Switches devices: " << devices.size() << '\n';
    // Example for using the DeviceWrapper class
    // You can also directly use the moza::HidDevice class and refer to the implementation of the DeviceWrapper class
    DeviceWrapper wrapper(std::move(devices[0]));
    if (!wrapper.isOpen() && !wrapper.open())
    {
        std::cerr << "Device open failed!\n";
        return -1;
    }
    // Below are two types of callback events, you can choose one
    // The callback function is executed in the worker thread, so it is recommended to avoid performing time-consuming operations within the callback to prevent impacting the reading and updating of device button states
    wrapper.setOnDeviceStateChanged([](const std::vector<uint8_t>& previousState, const std::vector<uint8_t>& currentState) {
        std::cout << "device state changes, ";
        printDeviceState(currentState);
    });
    /*
    wrapper.setOnButtonStateChanged([](moza::SwitchesIndex button, ButtonState state) {
        std::cout << "button state change: " << static_cast<int>(button) << (state == Pressed ? " pressed" : " released") << '\n';
    });
    */
    wrapper.startWorkerThread();
    std::cout << "Start monitoring device state changes. Press any key to exit:\n";
    while (!_kbhit() && wrapper.isRunning())
    {
        Sleep(300);
    }
    wrapper.stopWorkerThread();
    wrapper.close();
    moza::removeMozaSDK();
    return 0;
}
