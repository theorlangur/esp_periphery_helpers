#ifndef PH_I2C_HPP_
#define PH_I2C_HPP_

#include "lib_expected_results.hpp"
#include "lib_misc_helpers.hpp"
#include "driver/i2c_master.h"
#include <expected>
#include "lib_thread_lock.hpp"
#include "lib_type_traits.hpp"

namespace i2c
{
    using duration_t = ::duration_ms_t;
    //inline static constexpr const duration_t kForever = ::kForever;

    class SDAType: public StrongType<gpio_num_t, struct SDATag>//, Comparable
    {
        using StrongType::StrongType;
    };

    class SCLType: public StrongType<gpio_num_t, struct SCLTag>//, Comparable
    {
        using StrongType::StrongType;
    };

    enum class I2CPort: int
    {
        Auto = -1,
        Port0 = 0,
        Port1 = 1,
    };

    class I2CDevice;

    class I2CBusMaster
    {
    public:
        using BusRef = std::reference_wrapper<I2CBusMaster>;
        using ExpectedResult = std::expected<BusRef, Err>;

        I2CBusMaster(SDAType sda, SCLType sdc, I2CPort port = I2CPort::Auto);
        I2CBusMaster(const I2CBusMaster &rhs) = delete;
        I2CBusMaster(I2CBusMaster &&rhs);
        ~I2CBusMaster();

        I2CBusMaster& SetAccessLock(thread::ILockable *pLock){ m_pLock = pLock; return *this; }
        thread::ILockable* GetAccessLock() const { return m_pLock; }

        I2CBusMaster& SetSDAPin(SDAType sda);
        SDAType GetSDAPin() const;

        I2CBusMaster& SetSCLPin(SCLType sdc);
        SCLType GetSCLPin() const;

        I2CBusMaster& SetPort(I2CPort p);
        I2CPort GetPort() const;

        I2CBusMaster& SetGlitchIgnoreCount(uint8_t c = 7);
        uint8_t GetGlitchIgnoreCount() const;

        I2CBusMaster& SetInterruptPriority(int p = 0);
        int GetInterruptPriority() const;

        I2CBusMaster& SetEnableInternalPullup(bool enable);
        bool GetEnableInternalPullup() const;

        ExpectedResult Open();
        ExpectedResult Close();

        std::expected<I2CDevice, Err> Add(uint16_t addr) const;
    private:
        i2c_master_bus_config_t m_Config;
        i2c_master_bus_handle_t m_Handle = nullptr;
        thread::ILockable              *m_pLock = nullptr;

        friend class I2CDevice;
    };

    class I2CDevice
    {
    public:
        using DevRef = std::reference_wrapper<I2CDevice>;
        using ExpectedResult = std::expected<DevRef, Err>;
        template<typename V>
        using RetValue = RetValT<DevRef, V>;
        template<class V>
        using ExpectedValue = std::expected<RetValue<V>, Err>;

        I2CDevice(const I2CBusMaster &bus, uint16_t addr = 0xff, uint32_t speed_hz = 100'000);
        I2CDevice(const I2CDevice &rhs) = delete;
        I2CDevice(I2CDevice &&rhs);

        I2CDevice& SetAddress(uint16_t addr);
        uint16_t GetAddress() const;

        I2CDevice& SetSpeedHz(uint32_t hz);
        uint32_t GetSpeedHz() const;

        ExpectedResult Open();
        ExpectedResult Close();

        ExpectedResult Send(const uint8_t *pBuf, std::size_t len, duration_t d = kForever);
        ExpectedResult Recv(uint8_t *pBuf, std::size_t len, duration_t d = kForever);
        ExpectedResult SendRecv(const uint8_t *pSendBuf, std::size_t sendLen, uint8_t *pRecvBuf, std::size_t recvLen, duration_t d = kForever);

        ExpectedResult WriteReg8(uint8_t reg, uint8_t data, duration_t d = kForever);
        ExpectedResult WriteReg16(uint8_t reg, uint16_t data, duration_t d = kForever);

        ExpectedValue<uint8_t> ReadReg8(uint8_t reg, duration_t d = kForever);
        ExpectedValue<uint16_t> ReadReg16(uint8_t reg, duration_t d = kForever);
    private:
        const I2CBusMaster &m_Bus;
        i2c_master_dev_handle_t m_Handle = nullptr;
        i2c_device_config_t m_Config;
    };

    namespace helpers
    {
        constexpr static const duration_t kTimeout = duration_t(500);
        enum class RegAccess: uint8_t
        {
            Read = 0x01,
            Write = 0x02,
            RW = 0x03
        };

        template<class Val>
        using ExpectedValue = std::expected<Val, ::Err>;

        using ExpectedRes = std::expected<void, ::Err>;

        template<typename V, auto r, RegAccess access> requires (!std::is_polymorphic_v<V>)
        struct Register
        {

            i2c::I2CDevice &d;

            ExpectedValue<V> Read() const requires (access == RegAccess::Read || access == RegAccess::RW)
            {
                V res{};
                uint8_t *pDst = reinterpret_cast<uint8_t *>(&res);
                for(size_t i = 0; i < sizeof(V); ++i)
                {
                    if (auto res = d.ReadReg8(uint8_t(r) + i, kTimeout); !res)
                        return std::unexpected(res.error());
                    else
                        pDst[i] = res->v;
                }
                return res;
            }

            ExpectedRes Write(V const& v) const requires (access == RegAccess::Write || access == RegAccess::RW)
            {
                const uint8_t *pSrc = reinterpret_cast<const uint8_t *>(&v);
                for(size_t i = 0; i < sizeof(V); ++i)
                {
                    if (auto res = d.WriteReg8(uint8_t(r) + i, pSrc[i], kTimeout); !res)
                        return std::unexpected(res.error());
                }
                return {};
            }
        };
    }
}

#endif
