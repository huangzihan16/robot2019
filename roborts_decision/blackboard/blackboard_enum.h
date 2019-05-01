#ifndef BLACKBOARD_ENUM_H
#define BLACKBOARD_ENUM_H

namespace roborts_decision {
  enum class Identity {
	  MASTER = 0,
	  SLAVE,
	  SAME
  };
  enum class PartnerStatus {
    SUPPLY = 0,
    GAINBUFF,
    BATTLE,
    GUARD,
    PATROL
  };
  enum class GameStatus{
    PRE_MATCH = 0,
    SETUP = 1,
    INIT = 2,
    FIVE_SEC_CD = 3,
    ROUND = 4,
    CALCULATION = 5
  };
  enum class DamageSource{
    FORWARD = 0,
    BACKWARD = 1,
    LEFT = 2,
    RIGHT = 3,
    NONE = 4
  };
  enum class DamageType{
    ARMOR = 0,
    OFFLINE = 1,
    EXCEED_HEAT = 2,
    EXCEED_POWER = 3,
  };
  enum class BonusStatus{
    UNOCCUPIED = 0,
    BEING_OCCUPIED = 1,
    OCCUPIED = 2
  };
  enum class SupplierStatus{
    CLOSE = 0,
    PREPARING = 1,
    SUPPLYING = 2
  };
  enum class EnemyStatus{
    NONE = 0,
    FRONT = 1,
    BACK = 2,
    BOTH = 3
  };
  enum class GameResult{
    DRAW = 0,
    RED_WIN = 1,
    BLUE_WIN = 2
  };
}

#endif