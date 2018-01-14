#include "sbp2.h"


sbp_state_t sbp_state;
/* SBP structs that messages from Piksi will feed. */
  msg_pos_llh_t      pos_llh;
  msg_baseline_ned_t baseline_ned;
  msg_vel_ned_t      vel_ned;
  msg_dops_t         dops;
  msg_gps_time_t     gps_time;
  /*
   * SBP callback nodes must be statically allocated. Each message ID / callback
   * pair must have a unique sbp_msg_callbacks_node_t associated with it.
   */
  sbp_msg_callbacks_node_t pos_llh_node;
  sbp_msg_callbacks_node_t baseline_ned_node;
  sbp_msg_callbacks_node_t vel_ned_node;
  sbp_msg_callbacks_node_t dops_node;
  sbp_msg_callbacks_node_t gps_time_node;

void sbp_pos_llh_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context)
{
  pos_llh = *(msg_pos_llh_t *)msg;
}
void sbp_baseline_ned_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context)
{
  baseline_ned = *(msg_baseline_ned_t *)msg;
}
void sbp_vel_ned_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context)
{
  vel_ned = *(msg_vel_ned_t *)msg;
}
void sbp_dops_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context)
{
  dops = *(msg_dops_t *)msg;
}
void sbp_gps_time_callback(uint16_t sender_id, uint8_t len, uint8_t msg[], void *context)
{
  gps_time = *(msg_gps_time_t *)msg;
}

/*
 * Set up SwiftNav Binary Protocol (SBP) nodes; the sbp_process function will
 * search through these to find the callback for a particular message ID.
 *
 * Example: sbp_pos_llh_callback is registered with sbp_state, and is associated
 * with both a unique sbp_msg_callbacks_node_t and the message ID SBP_POS_LLH.
 * When a valid SBP message with the ID SBP_POS_LLH comes through the UART, written
 * to the FIFO, and then parsed by sbp_process, sbp_pos_llh_callback is called
 * with the data carried by that message.
 */
void sbp_setup(void)
{
  /* SBP parser state must be initialized before sbp_process is called. */
  sbp_state_init(&sbp_state);

  /* Register a node and callback, and associate them with a specific message ID. */
  sbp_register_callback(&sbp_state, SBP_MSG_GPS_TIME, &sbp_gps_time_callback,
                        NULL, &gps_time_node);
  sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &sbp_pos_llh_callback,
                        NULL, &pos_llh_node);
  sbp_register_callback(&sbp_state, SBP_MSG_BASELINE_NED, &sbp_baseline_ned_callback,
                        NULL, &baseline_ned_node);
  sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED, &sbp_vel_ned_callback,
                        NULL, &vel_ned_node);
  sbp_register_callback(&sbp_state, SBP_MSG_DOPS, &sbp_dops_callback,
                        NULL, &dops_node);

}