#ifndef FSM_H
#define FSM_H

typedef enum {
    START = 0,
    SENDH,
    SENDE,
    SENDL1,
    SENDL2,
    SENDO,
    SENDHC, //\r
    SENDHH, //\n
} fsm_t;

//! \name finit state machine state
//! @{
typedef enum {
    fsm_rt_err          = -1,    //!< fsm error, error code can be get from other interface
    fsm_rt_cpl          = 0,     //!< fsm complete
    fsm_rt_on_going     = 1,     //!< fsm on-going
} fsm_rt_t;
//! @}
#endif