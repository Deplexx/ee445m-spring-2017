// OS.h

// ***** OS_AddPeriodicThread *****
// Sets thread to run every periodically every ms
// Input: task - function to run
//        period - recurring thread every period in ms
//        priority - NVIC priority
// Output: ??
int OS_AddPeriodicThread(void(*task)(void), uint32_t period, uint32_t priority);

// ***** OS_ClearPeriodicTime *****
// Resets global counter to 0
// Input: none
// Output: none
void OS_ClearPeriodicTime(void);

// ***** OS_AddPeriodicThread *****
// Returns global counter value in units of period initialized by
// OS_AddPeriodicThread
// Input: none
// Output: counter value
uint32_t OS_ReadPeriodicTime(void);
