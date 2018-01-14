#ifndef SBP2_H
#define SBP2_H
#include "sbp.h"
#include "navigation.h"
  /*
   * State of the SBP message parser.
   * Must be statically allocated.
   */
  extern sbp_state_t sbp_state;
  /* SBP structs that messages from Piksi will feed. */
  extern msg_pos_llh_t      pos_llh;
  extern msg_baseline_ned_t baseline_ned;
  extern msg_vel_ned_t      vel_ned;
  extern msg_dops_t         dops;
  extern msg_gps_time_t     gps_time;
  /*
   * SBP callback nodes must be statically allocated. Each message ID / callback
   * pair must have a unique sbp_msg_callbacks_node_t associated with it.
   */
  extern sbp_msg_callbacks_node_t pos_llh_node;
  extern sbp_msg_callbacks_node_t baseline_ned_node;
  extern sbp_msg_callbacks_node_t vel_ned_node;
  extern sbp_msg_callbacks_node_t dops_node;
  extern sbp_msg_callbacks_node_t gps_time_node;
  /*
   * Callback functions to interpret SBP messages.
   * Every message ID has a callback associated with it to
   * receive and interpret the message payload.
   */
  void sbp_pos_llh_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
  void sbp_baseline_ned_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
  void sbp_vel_ned_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
  void sbp_dops_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
  void sbp_gps_time_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context);
  void sbp_setup(void);
  #endif /* LIBSBP_SBP_H */