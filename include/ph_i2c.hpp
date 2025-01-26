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
        using multi_data_to_send_t = std::span<i2c_master_transmit_multi_buffer_info_t>;
        ExpectedResult SendMulti(multi_data_to_send_t, duration_t d = kForever);
        ExpectedResult Recv(uint8_t *pBuf, std::size_t len, duration_t d = kForever);
        ExpectedResult SendRecv(const uint8_t *pSendBuf, std::size_t sendLen, uint8_t *pRecvBuf, std::size_t recvLen, duration_t d = kForever);

        ExpectedResult WriteReg8(uint8_t reg, uint8_t data, duration_t d = kForever);
        ExpectedResult WriteReg16(uint8_t reg, uint16_t data, duration_t d = kForever);

        ExpectedValue<uint8_t> ReadReg8(uint8_t reg, duration_t d = kForever);
        ExpectedValue<uint16_t> ReadReg16(uint8_t reg, duration_t d = kForever);
        ExpectedResult ReadRegMulti(uint8_t reg, std::span<uint8_t> dst, duration_t d = kForever);
        ExpectedResult WriteRegMulti(uint8_t reg, std::span<const uint8_t> src, duration_t d = kForever);

#ifndef NDEBUG
        void dbg_on_send(bool v) { m_Dbg.print_send = v; }
        void dbg_on_recv(bool v) { m_Dbg.print_recv = v; }
private:
        struct
        {
            uint8_t print_send: 1 = 0;
            uint8_t print_recv: 1 = 0;
        }m_Dbg;
public:
#endif
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

        enum class ByteOrder
        {
            LE,
            BE
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

        struct ByteCfg
        {
            uint8_t offset;
            uint8_t bit_off = 0;
            uint8_t bit_len = 8;
        };
        template<uint8_t N>
        struct RegConfig
        {
            using reg_config_tag = void;
            uint8_t addr;
            RegAccess access;
            ByteCfg bytes[N];
        };

        template<class... T> requires (std::is_same_v<T, ByteCfg> && ...)
        auto ConfigBytes(uint8_t baseAddr, RegAccess access, T... bytes)
        {
            RegConfig<sizeof...(T)> res{baseAddr, access, {bytes...}};
            return res;
        }

        template<typename V, auto r, RegAccess access, ByteOrder bo = ByteOrder::LE, size_t word_size = 0> 
            requires (!std::is_polymorphic_v<V>) && ((word_size == 0) || ((sizeof(V) % word_size == 0) && (word_size % 2 == 0)))
        struct RegisterMultiByte
        {
            i2c::I2CDevice &d;

            ExpectedRes Read(V &res) const requires (access == RegAccess::Read || access == RegAccess::RW)
            {
                uint8_t *pDst = (uint8_t*)&res;
                auto ret = d.ReadRegMulti(uint8_t(r), {pDst, sizeof(V)}, kTimeout);
                if (ret)
                {
                    if constexpr (bo == ByteOrder::BE)
                    {
                        if constexpr (word_size == 0)
                        {
                            for(size_t i = 0; i < (sizeof(V) / 2); ++i)
                                std::swap(pDst[i], pDst[sizeof(V) - i - 1]);
                        }else
                        {
                            for(size_t w = 0; w < (sizeof(V) / word_size); ++w)
                            {
                                if constexpr (word_size == 2)
                                    std::swap(pDst[w * word_size], pDst[w * word_size + 1]);
                                else if constexpr (word_size == 4)
                                {
                                    std::swap(pDst[w * word_size], pDst[w * word_size + 3]);
                                    std::swap(pDst[w * word_size + 1], pDst[w * word_size + 2]);
                                }
                                else
                                {
                                    for(size_t i = 0; i < (word_size / 2); ++i)
                                        std::swap(pDst[w * word_size + i], pDst[w * word_size + word_size - i - 1]);
                                }
                            }
                        }
                    }
                    return {};
                }
                else
                    return std::unexpected(ret.error());
            }

            ExpectedRes Write(V const& v) const requires (access == RegAccess::Write || access == RegAccess::RW)
            {
                uint8_t buf[sizeof(V)];
                const uint8_t *pSrc = reinterpret_cast<const uint8_t *>(&v);
                if constexpr (bo == ByteOrder::BE)
                {
                    if constexpr (word_size == 0)
                    {
                        for(size_t i = 0; i < sizeof(V); ++i)
                            buf[i] = pSrc[sizeof(V) - i - 1];
                    }else
                    {
                        for(size_t w = 0; w < (sizeof(V) / word_size); ++w)
                        {
                            if constexpr (word_size == 2)
                            {
                                buf[w * word_size] = pSrc[w * word_size + 1];
                                buf[w * word_size + 1] = pSrc[w * word_size];
                            }
                            else if constexpr (word_size == 4)
                            {
                                buf[w * word_size] = pSrc[w * word_size + 3];
                                buf[w * word_size + 1] = pSrc[w * word_size + 2];
                                buf[w * word_size + 2] = pSrc[w * word_size + 1];
                                buf[w * word_size + 3] = pSrc[w * word_size];
                            }
                            else
                            {
                                for(size_t i = 0; i < (word_size / 2); ++i)
                                    buf[w * word_size + i] = pSrc[w * word_size + word_size - i - 1];
                            }
                        }
                    }
                    pSrc = buf;
                }
                auto ret = d.WriteRegMulti(uint8_t(r), {pSrc, sizeof(V)}, kTimeout);
                if (ret)
                    return {};
                else
                    return std::unexpected(ret.error());
            }
        };

        template<typename V, auto regCfg> requires (!std::is_polymorphic_v<V> && requires { typename decltype(regCfg)::reg_config_tag; })
        struct RegisterCustomBytes
        {
            i2c::I2CDevice &d;

            ExpectedRes Read(V &res) const requires (regCfg.access == RegAccess::Read || regCfg.access == RegAccess::RW)
            {
                uint8_t *pDst = reinterpret_cast<uint8_t*>(&res);
                size_t dstBitOffset = 0;
                uint8_t tempDst[std::size(regCfg.bytes)];
                if (auto r = d.ReadRegMulti(regCfg.addr, tempDst); !r)
                    return std::unexpected(r.error());

                for(int i = 0; i < std::size(tempDst); ++i)
                {
                    auto b = regCfg.bytes[i];
                    uint8_t v = tempDst[i];
                    uint8_t bits_src_left = b.bit_len;
                    v = (v >> b.bit_off) & ((1 << b.bit_len) - 1);
                    uint8_t bits_dst_off = dstBitOffset % 8;
                    uint8_t *pT = pDst + dstBitOffset / 8;
                    uint8_t bits_dst_eff = std::min(uint8_t(8 - bits_dst_off), bits_src_left);
                    uint8_t maskToClear = (1 << bits_dst_eff) - 1;
                    *pT &= ~(maskToClear << bits_dst_off);//clear the bits
                    *pT |= v << bits_dst_off;
                    dstBitOffset += bits_dst_eff;
                    bits_src_left -= bits_dst_eff;
                    if (bits_src_left)
                    {
                        ++pT;
                        v = tempDst[i];
                        *pT = (v >> (b.bit_off + b.bit_len - bits_src_left)) & ((1 << bits_src_left) - 1);
                        dstBitOffset += bits_src_left;
                    }
                }
            }
        };
    }
}

#endif
