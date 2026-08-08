#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_ENABLE_BENCHMARKING

#include <array>

#include "catch2/catch.hpp"
#include "cereal/messaging/messaging.h"
#include "common/util.h"
#include "selfdrive/pandad/panda.h"

struct PandaTest : public Panda {
  PandaTest(uint32_t bus_offset, int can_list_size, cereal::PandaState::PandaType hw_type);
  void test_can_send();
  void test_can_recv(uint32_t chunk_size = 0);
  void test_chunked_can_recv();

  std::map<int, std::string> test_data;
  int can_list_size = 0;
  int total_pakets_size = 0;
  MessageBuilder msg;
  capnp::List<cereal::CanData>::Reader can_data_list;
};

struct FlexRayPandaTest : public Panda {
  explicit FlexRayPandaTest(uint32_t bus_offset) : Panda(bus_offset) {}

  std::vector<std::vector<uint8_t>> pack(const capnp::List<cereal::CanData>::Reader &can_data_list) {
    std::vector<std::vector<uint8_t>> chunks;
    pack_flexray_buffer(can_data_list, [&](uint8_t *data, size_t size) {
      chunks.emplace_back(data, data + size);
    });
    return chunks;
  }

  long map_source(uint8_t source) const {
    return flexray_bus_from_source(source);
  }
};

static uint8_t crc8_1d(const uint8_t *data, size_t len, uint8_t init = 0xF1) {
  uint8_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x1DU) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

PandaTest::PandaTest(uint32_t bus_offset_, int can_list_size, cereal::PandaState::PandaType hw_type) : can_list_size(can_list_size), Panda(bus_offset_) {
  this->hw_type = hw_type;
  int data_limit = ((hw_type == cereal::PandaState::PandaType::RED_PANDA) ? std::size(dlc_to_len) : 8);
  // prepare test data
  for (int i = 0; i < data_limit; ++i) {
    std::random_device rd;
    std::independent_bits_engine<std::default_random_engine, CHAR_BIT, unsigned char> rbe(rd());

    int data_len = dlc_to_len[i];
    std::string bytes(data_len, '\0');
    std::generate(bytes.begin(), bytes.end(), std::ref(rbe));
    test_data[data_len] = bytes;
  }

  // generate can messages for this panda
  auto can_list = msg.initEvent().initSendcan(can_list_size);
  for (uint8_t i = 0; i < can_list_size; ++i) {
    auto can = can_list[i];
    uint32_t id = util::random_int(0, std::size(dlc_to_len) - 1);
    const std::string &dat = test_data[dlc_to_len[id]];
    can.setAddress(i);
    can.setSrc(util::random_int(0, 2) + bus_offset);
    can.setDat(kj::ArrayPtr((uint8_t *)dat.data(), dat.size()));
    total_pakets_size += sizeof(can_header) + dat.size();
  }

  can_data_list = can_list.asReader();
  INFO("test " << can_list_size << " packets, total size " << total_pakets_size);
}

void PandaTest::test_can_send() {
  std::vector<uint8_t> unpacked_data;
  this->pack_can_buffer(can_data_list, [&](uint8_t *chunk, size_t size) {
    unpacked_data.insert(unpacked_data.end(), chunk, &chunk[size]);
  });
  REQUIRE(unpacked_data.size() == total_pakets_size);

  int cnt = 0;
  INFO("test can message integrity");
  for (int pos = 0, pckt_len = 0; pos < unpacked_data.size(); pos += pckt_len) {
    can_header header;
    memcpy(&header, &unpacked_data[pos], sizeof(can_header));
    const uint8_t data_len = dlc_to_len[header.data_len_code];
    pckt_len = sizeof(can_header) + data_len;

    REQUIRE(header.addr == cnt);
    REQUIRE(test_data.find(data_len) != test_data.end());
    const std::string &dat = test_data[data_len];
    REQUIRE(memcmp(dat.data(), &unpacked_data[pos + sizeof(can_header)], dat.size()) == 0);
    ++cnt;
  }
  REQUIRE(cnt == can_list_size);
}

void PandaTest::test_can_recv(uint32_t rx_chunk_size) {
  std::vector<can_frame> frames;
  this->pack_can_buffer(can_data_list, [&](uint8_t *data, uint32_t size) {
    if (rx_chunk_size == 0) {
      REQUIRE(this->unpack_can_buffer(data, size, frames));
    } else {
      this->receive_buffer_size = 0;
      uint32_t pos = 0;

      while (pos < size) {
        uint32_t chunk_size = std::min(rx_chunk_size, size - pos);
        memcpy(&this->receive_buffer[this->receive_buffer_size], &data[pos], chunk_size);
        this->receive_buffer_size += chunk_size;
        pos += chunk_size;

        REQUIRE(this->unpack_can_buffer(this->receive_buffer, this->receive_buffer_size, frames));
      }
    }
  });

  REQUIRE(frames.size() == can_list_size);
  for (int i = 0; i < frames.size(); ++i) {
    REQUIRE(frames[i].address == i);
    REQUIRE(test_data.find(frames[i].dat.size()) != test_data.end());
    const std::string &dat = test_data[frames[i].dat.size()];
    REQUIRE(memcmp(dat.data(), frames[i].dat.data(), dat.size()) == 0);
  }
}

TEST_CASE("send/recv CAN 2.0 packets") {
  auto bus_offset = GENERATE(0, 4);
  auto can_list_size = GENERATE(1, 3, 5, 10, 30, 60, 100, 200);
  PandaTest test(bus_offset, can_list_size, cereal::PandaState::PandaType::DOS);

  SECTION("can_send") {
    test.test_can_send();
  }
  SECTION("can_receive") {
    test.test_can_recv();
  }
  SECTION("chunked_can_receive") {
    test.test_can_recv(0x40);
  }
}

TEST_CASE("send/recv CAN FD packets") {
  auto bus_offset = GENERATE(0, 4);
  auto can_list_size = GENERATE(1, 3, 5, 10, 30, 60, 100, 200);
  PandaTest test(bus_offset, can_list_size, cereal::PandaState::PandaType::RED_PANDA);

  SECTION("can_send") {
    test.test_can_send();
  }
  SECTION("can_receive") {
    test.test_can_recv();
  }
  SECTION("chunked_can_receive") {
    test.test_can_recv(0x40);
  }
}

TEST_CASE("FlexRay override records are exact and atomic") {
  FlexRayPandaTest test(4);

  SECTION("19-byte C238 command becomes one exact 25-byte USB record") {
    MessageBuilder msg;
    auto can_list = msg.initEvent().initSendcan(1);
    auto can = can_list[0];
    const std::array<uint8_t, 19> dat = {
      0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x08, 0x80, 0x32, 0x08, 0x00,
    };
    can.setAddress(0x08);
    can.setSrc(14);
    can.setDat(kj::arrayPtr(dat.data(), dat.size()));

    auto chunks = test.pack(can_list.asReader());
    REQUIRE(chunks.size() == 1);
    REQUIRE(chunks[0].size() == 25);
    REQUIRE(chunks[0][0] == 0x90);
    REQUIRE(chunks[0][1] == 0x08);
    REQUIRE(chunks[0][2] == 0x00);
    REQUIRE(chunks[0][3] == 0x02);
    REQUIRE(chunks[0][4] == 19);
    REQUIRE(chunks[0][5] == 0);
    REQUIRE(chunks[0][6] == crc8_1d(dat.data() + 1, dat.size() - 1));
    REQUIRE(chunks[0][6] == 0x92);
    REQUIRE(memcmp(chunks[0].data() + 7, dat.data() + 1, dat.size() - 1) == 0);
  }


  SECTION("two-channel source IDs are independent of Panda index") {
    REQUIRE(test.map_source(13) == 13);
    REQUIRE(test.map_source(14) == 14);
    REQUIRE(test.map_source(24) == 24);

    // Keep the legacy one-channel mapping for existing FlexRay ports.
    REQUIRE(test.map_source(1) == 5);
  }

  SECTION("all explicit two-channel source IDs route to the FlexRay Panda") {
    MessageBuilder msg;
    auto can_list = msg.initEvent().initSendcan(3);
    const std::array<uint8_t, 1> dat = {0x02};
    const std::array<uint8_t, 3> sources = {13, 14, 24};
    for (size_t i = 0; i < can_list.size(); ++i) {
      auto can = can_list[i];
      can.setAddress((uint16_t)(0x08 + i));
      can.setSrc(sources[i]);
      can.setDat(kj::arrayPtr(dat.data(), dat.size()));
    }

    auto chunks = test.pack(can_list.asReader());
    REQUIRE(chunks.size() == 1);
    REQUIRE(chunks[0].size() == 3 * 7);
  }

  SECTION("records flush before the soft boundary and are never truncated") {
    MessageBuilder msg;
    auto can_list = msg.initEvent().initSendcan(11);
    std::array<uint8_t, 19> dat = {};
    dat[0] = 0x02;
    for (size_t i = 0; i < can_list.size(); ++i) {
      auto can = can_list[i];
      can.setAddress((uint16_t)(0x08 + i));
      can.setSrc(5);
      dat[1] = (uint8_t)i;
      can.setDat(kj::arrayPtr(dat.data(), dat.size()));
    }

    auto chunks = test.pack(can_list.asReader());
    REQUIRE(chunks.size() == 2);
    REQUIRE(chunks[0].size() == 10 * 25);
    REQUIRE(chunks[1].size() == 25);

    size_t record_count = 0;
    for (const auto &chunk : chunks) {
      for (size_t pos = 0; pos < chunk.size(); pos += 25) {
        REQUIRE(chunk[pos] == 0x90);
        REQUIRE(chunk[pos + 4] == 19);
        REQUIRE(chunk[pos + 5] == 0);
        ++record_count;
      }
    }
    REQUIRE(record_count == 11);
  }

  SECTION("the full three high frame-id bits are retained") {
    MessageBuilder msg;
    auto can_list = msg.initEvent().initSendcan(1);
    auto can = can_list[0];
    const std::array<uint8_t, 1> dat = {0x02};
    can.setAddress(0x708);
    can.setSrc(5);
    can.setDat(kj::arrayPtr(dat.data(), dat.size()));

    auto chunks = test.pack(can_list.asReader());
    REQUIRE(chunks.size() == 1);
    REQUIRE(chunks[0][1] == 0x08);
    REQUIRE(chunks[0][2] == 0x07);
  }
}
