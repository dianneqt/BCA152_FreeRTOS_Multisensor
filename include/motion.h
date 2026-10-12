#pragma once

// MotionTask: PIR interrupt + inactivity timeout.
// Owns the ACTIVE/INACTIVE state (sets/clears EVENT_ACTIVE, EVENT_MOTION).
void motionTask(void *parameter);