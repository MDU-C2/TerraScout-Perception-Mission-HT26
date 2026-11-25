#include "bms_interface.hpp"
#include <array>
#include <cassert>
#include <cstring>

using Frames = std::array<Can::Frame, 8>;

// Independent bitwise reference, including the published CCITT check vector.
static uint16_t crc(const uint8_t *bytes, size_t count) {
    uint32_t value = 0xFFFF;
    for (size_t i = 0; i < count; ++i) {
        for (int bit = 7; bit >= 0; --bit) {
            const bool feedback = ((value >> 15) ^ (bytes[i] >> bit)) & 1;
            value = (value << 1) & 0xFFFF;
            if (feedback) value ^= 0x1021;
        }
    }
    return static_cast<uint16_t>(value);
}

static Frames fixture() {
    uint8_t bytes[50]{};
    auto put = [&](size_t offset, uint16_t value) {
        bytes[offset] = value & 255; bytes[offset + 1] = value >> 8;
    };
    put(2, 12); put(4, 34); put(6, 46600);
    put(8, static_cast<uint16_t>(-123)); put(10, static_cast<uint16_t>(-5));
    put(12, 62); put(14, 6); put(16, 100);
    for (size_t i=0; i<12; ++i) put(18+2*i, 3800+i);
    put(42, 22000); put(44, 13640);
    bytes[47]=1; bytes[49]=0x80; // cell undervoltage plus reserved bit 31
    put(0, crc(bytes+2, 48));
    Frames frames{};
    for (size_t i=0; i<8; ++i) {
        auto &f=frames[i]; f.id=0x01109216; f.extended=true;
        const size_t n=i==7 ? 1:7; f.dlc=n+1;
        std::memcpy(f.data, bytes+7*i, n);
        f.data[n]=(i==0?0x80:0)|(i==7?0x40:0)|((i%2)*0x20)|i;
    }
    return frames;
}

static BmsInterface::Result feed(BmsInterface &b, const Frames &frames, int64_t now=1000) {
    BmsInterface::Result result{};
    for (size_t i=0; i<frames.size(); ++i) result=b.processFrame(frames[i], now+i);
    return result;
}

int main() {
    const uint8_t check[]="123456789";
    assert(crc(check, 9)==0x29B1);
    Can can;
    BmsInterface b(can);
    assert(b.state(0)==BmsInterface::State::Missing && b.latest()==nullptr);
    const auto good=fixture();
    assert(feed(b,good)==BmsInterface::Result::Updated);
    const auto &s=*b.latest();
    assert(s.pack_voltage_mv==46600 && s.current_ma==-1230 && s.temperature_c==-5);
    assert(s.charge_pct==62 && s.cycle_count==6 && s.health_pct==100);
    assert(s.cell_voltages_mv[11]==3811 && s.remaining_capacity_mah==13640);
    assert(s.design_capacity_mah==22000 && s.payload_crc_matches);
    assert(s.fault_flags==0x80000100U && s.hasFault(BmsInterface::Fault::CellUndervoltage));
    assert(!s.hasFault(BmsInterface::Fault::OverTemperature) && s.hasAnyFault());
    assert(b.state(1001006)==BmsInterface::State::Fresh);
    assert(b.state(1001007)==BmsInterface::State::Stale);
    assert(b.state(1)==BmsInterface::State::Stale);

    auto bad=good; bad[2].data[0]^=1;
    assert(feed(b,bad,2000)==BmsInterface::Result::CrcMismatch);
    assert(b.latest()->received_us==1007); // rejected traffic never refreshes data
    bad=good; bad[1].data[7]^=0x20;
    assert(feed(b,bad,3000)==BmsInterface::Result::Rejected);
    assert(b.processFrame(good[0],4000)==BmsInterface::Result::Incomplete);
    assert(b.processFrame(good[1],104000)==BmsInterface::Result::Rejected); // timeout
    assert(b.processFrame(good[0],5000)==BmsInterface::Result::Incomplete);
    assert(b.processFrame(good[2],5001)==BmsInterface::Result::Rejected); // lost frame
    bad=good; bad[0].dlc=9;
    assert(b.processFrame(bad[0],6000)==BmsInterface::Result::Rejected);
    bad=good; bad[0].rtr=true;
    assert(b.processFrame(bad[0],6000)==BmsInterface::Result::Rejected);
    bad=good; bad[0].id=123;
    assert(b.processFrame(bad[0],6000)==BmsInterface::Result::Ignored);
    assert(feed(b,good,7000)==BmsInterface::Result::Updated); // recover after bad traffic
    BmsInterface::Config config; config.sequence=BmsInterface::Sequence::ConstantTransferId;
    BmsInterface constant(can,config);
    auto fixed=good;
    for(auto &f:fixed) f.data[f.dlc-1]=(f.data[f.dlc-1]&0xE0)|5;
    assert(feed(constant,fixed)==BmsInterface::Result::Updated);
    b.reset(); assert(b.latest()==nullptr && b.state(9000)==BmsInterface::State::Missing);

    auto invalid=good[0]; invalid.dlc=9;
    assert(b.processFrame(invalid,10000)==BmsInterface::Result::Rejected);
    assert(feed(b,good,11000)==BmsInterface::Result::Updated);
}
