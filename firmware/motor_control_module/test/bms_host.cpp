#include "bms_interface.hpp"
#include "bms.hpp"
#include <array>
#include <cassert>
#include <cstring>

using Bms = BmsInterface;
using Frames = std::array<Can::Frame, 8>;
int64_t bms_test_now = 1000;

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

static Bms::Result feed(Bms &b, const Frames &frames, int64_t now=1000) {
    Bms::Result result{};
    for (size_t i=0; i<frames.size(); ++i) result=b.processFrame(frames[i], now+i);
    return result;
}

int main() {
    const uint8_t check[]="123456789";
    assert(crc(check, 9)==0x29B1);
    Can can;
    Bms b(can);
    assert(b.state(0)==Bms::State::Missing && b.latest()==nullptr);
    const auto good=fixture();
    assert(feed(b,good)==Bms::Result::Updated);
    const auto &s=*b.latest();
    assert(s.pack_voltage_mv==46600 && s.current_ma==-1230 && s.temperature_c==-5);
    assert(s.charge_pct==62 && s.cycle_count==6 && s.health_pct==100);
    assert(s.cell_voltages_mv[11]==3811 && s.remaining_capacity_mah==13640);
    assert(s.design_capacity_mah==22000 && s.payload_crc_matches);
    assert(s.fault_flags==0x80000100U && s.hasFault(Bms::Fault::CellUndervoltage));
    assert(!s.hasFault(Bms::Fault::OverTemperature) && s.hasAnyFault());
    assert(b.state(1001006)==Bms::State::Fresh);
    assert(b.state(1001007)==Bms::State::Stale);
    assert(b.state(1)==Bms::State::Stale);

    auto bad=good; bad[2].data[0]^=1;
    assert(feed(b,bad,2000)==Bms::Result::CrcMismatch);
    assert(b.latest()->received_us==1007); // rejected traffic never refreshes data
    bad=good; bad[1].data[7]^=0x20;
    assert(feed(b,bad,3000)==Bms::Result::Rejected);
    assert(b.processFrame(good[0],4000)==Bms::Result::Incomplete);
    assert(b.processFrame(good[1],104000)==Bms::Result::Rejected); // timeout
    assert(b.processFrame(good[0],5000)==Bms::Result::Incomplete);
    assert(b.processFrame(good[2],5001)==Bms::Result::Rejected); // lost frame
    bad=good; bad[0].dlc=9;
    assert(b.processFrame(bad[0],6000)==Bms::Result::Rejected);
    bad=good; bad[0].rtr=true;
    assert(b.processFrame(bad[0],6000)==Bms::Result::Rejected);
    bad=good; bad[0].id=123;
    assert(b.processFrame(bad[0],6000)==Bms::Result::Ignored);
    assert(feed(b,good,7000)==Bms::Result::Updated); // recover after bad traffic
    Bms::Config config; config.sequence=Bms::Sequence::ConstantTransferId;
    Bms constant(can,config);
    auto fixed=good;
    for(auto &f:fixed) f.data[f.dlc-1]=(f.data[f.dlc-1]&0xE0)|5;
    assert(feed(constant,fixed)==Bms::Result::Updated);
    b.reset(); assert(b.latest()==nullptr && b.state(9000)==Bms::State::Missing);

    TattuBmsDecoder decoder;
    BmsData result;
    for (size_t i=0;i<8;++i)
        assert(decoder.process(good[i],result)==(i==7));
    assert(result.voltage_mv==46600 && result.current_ma==-1230);
    auto invalid=good[0];invalid.dlc=9;
    assert(!decoder.process(invalid,result));
    invalid=good[0];invalid.rtr=true;
    assert(!decoder.process(invalid,result));
    assert(!decoder.process(good[0],result));
    bms_test_now+=100000;
    for(size_t i=1;i<8;++i) assert(!decoder.process(good[i],result));
    for(size_t i=0;i<8;++i) assert(decoder.process(good[i],result)==(i==7));
}
