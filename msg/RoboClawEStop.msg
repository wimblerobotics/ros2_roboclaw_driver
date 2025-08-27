# MIT License
# Author: Michael Wimble <mike@wimblerobotics.com>
# External / internal E-Stop state or command message.
# When published with estop_active=true it requests or reports an estop from the given source.
# When estop_active=false it requests clearing that source. A master clear can be represented by source="*".

bool   estop_active
string source        # Identifier of estop source (e.g. "safety_overcurrent", "user_button") or "*" for master clear
string reason        # Human readable reason (ignored on clear if empty)
uint8  state         # 0=OK 1=DECEL 2=ESTOP (optional mirror)
string[] all_active_sources  # Optional list populated when publishing state (empty when used as a command)
