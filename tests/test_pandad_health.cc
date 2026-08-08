#define CATCH_CONFIG_MAIN

#include "catch2/catch.hpp"
#include "cereal/messaging/messaging.h"
#include "selfdrive/pandad/pandad.h"

TEST_CASE("current Panda health permissions are published independently") {
  health_t health = {};
  health.controls_allowed_pkt = 0;
  health.sound_output_level_pkt = 321;
  health.controls_allowed_lateral_pkt = 1;
  health.controls_allowed_longitudinal_pkt = 0;

  MessageBuilder msg;
  auto ps = msg.initEvent().initPandaStates(1)[0];
  fill_panda_state(ps, cereal::PandaState::PandaType::RED_PANDA, health);

  REQUIRE_FALSE(ps.getControlsAllowed());
  REQUIRE(ps.getSoundOutputLevel() == 321);
  REQUIRE(ps.getControlsAllowedLateral());
  REQUIRE_FALSE(ps.getControlsAllowedLongitudinal());
}
