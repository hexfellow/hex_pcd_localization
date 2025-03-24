/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-11-21
 ****************************************************************/

#include "hex_pcd_localization/data_interface/data_interface.h"
#include "hex_pcd_localization/pcd_localization.h"

using hex::localization::DataInterface;
using hex::localization::HexLogLevel;
using hex::localization::PcdLocalization;

const char kNodeName[] = "pcd_localization";

void TimeHandle() {
  enum class FiniteState { kInitState = 0, kWorkState };
  static FiniteState finite_state_machine_state = FiniteState::kInitState;
  static PcdLocalization& pcd_localization = PcdLocalization::GetSingleton();
  static DataInterface& data_interface = DataInterface::GetSingleton();

  switch (finite_state_machine_state) {
    case FiniteState::kInitState: {
      if (pcd_localization.Init()) {
        data_interface.Log(HexLogLevel::kInfo, "%s : Init Succeded", kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(HexLogLevel::kWarn, "%s : Init Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    case FiniteState::kWorkState: {
      if (pcd_localization.Work()) {
        // data_interface.Log(HexLogLevel::kInfo, "%s : Work Succeded",
        // kNodeName);
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(HexLogLevel::kWarn, "%s : Work Failed", kNodeName);
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    default: {
      data_interface.Log(HexLogLevel::kError, "%s : Unknown State", kNodeName);
      finite_state_machine_state = FiniteState::kInitState;
      break;
    }
  }
}

int main(int argc, char** argv) {
  DataInterface& data_interface = DataInterface::GetSingleton();
  data_interface.Init(argc, argv, "pcd_localization", 50.0, TimeHandle);

  data_interface.Work();

  data_interface.Deinit();

  return 0;
}
