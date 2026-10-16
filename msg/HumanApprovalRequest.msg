# HumanApprovalRequest
# Published by autonomous_navigator before executing any autonomous action.
# GCS displays this to operator; operator responds via human_approval_response.

uint64 timestamp        # [us]
uint8  action_id        # Unique ID for this action (matches response)
float32 timeout_s       # Seconds before auto-execute (full-auto mode)
char[64] description    # Human-readable description of the action

# TOPICS human_approval_request
