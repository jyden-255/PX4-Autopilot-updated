# HumanApprovalResponse
# Published by GCS plugin or RC switch handler.

uint64 timestamp
uint8  action_id        # Must match the request action_id
bool   approved         # True = proceed, False = abort/hold

# TOPICS human_approval_response
