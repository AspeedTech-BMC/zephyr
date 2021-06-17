/*
 * Copyright (c) 2020-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_VCS_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_VCS_H_

/**
 * @brief Volume Control Service (VCS)
 *
 * @defgroup bt_gatt_vcs Volume Control Service (VCS)
 *
 * @ingroup bluetooth
 * @{
 *
 * [Experimental] Users should note that the APIs can change
 * as a part of ongoing development.
 */

#include <zephyr/types.h>
#include <bluetooth/audio/aics.h>
#include <bluetooth/audio/vocs.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_BT_VCS)
#define BT_VCS_VOCS_CNT CONFIG_BT_VCS_VOCS_INSTANCE_COUNT
#define BT_VCS_AICS_CNT CONFIG_BT_VCS_AICS_INSTANCE_COUNT
#else
#define BT_VCS_VOCS_CNT 0
#define BT_VCS_AICS_CNT 0
#endif /* CONFIG_BT_VCS */

/** Volume Control Service Error codes */
#define BT_VCS_ERR_INVALID_COUNTER             0x80
#define BT_VCS_ERR_OP_NOT_SUPPORTED            0x81

/** Register structure for Volume Control Service */
struct bt_vcs_register_param {
	/** Register parameters for Volume Offset Control Services */
	struct bt_vocs_register_param vocs_param[BT_VCS_VOCS_CNT];

	/** Register parameters  for Audio Input Control Services */
	struct bt_aics_register_param aics_param[BT_VCS_AICS_CNT];

	/** Volume Control Service callback structure. */
	struct bt_vcs_cb *cb;
};

/**
 * @brief Volume Control Service service instance
 *
 * Used for to represent a Volume Control Service instance, for either a client
 * or a server. The instance pointers either represent local server instances,
 * or remote service instances.
 */
struct bt_vcs {
	/** Number of Volume Offset Control Service instances */
	uint8_t vocs_cnt;
	/** Array of pointers to Volume Offset Control Service instances */
	struct bt_vocs **vocs;

	/** Number of Audio Input Control Service instances */
	uint8_t aics_cnt;
	/** Array of pointers to Audio Input Control Service instances */
	struct bt_aics **aics;
};

/**
 * @brief Register the Volume Control Service.
 *
 * This will register and enable the service and make it discoverable by
 * clients.
 *
 * @param param     Volume Control Service register parameters.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_register(struct bt_vcs_register_param *param);

/**
 * @brief Get Volume Control Service service pointer.
 *
 * Returns a pointer to a struct that contains information about the
 * Volume Control Service instance, such as pointers to the
 * Volume Offset Control Service (Volume Offset Control Service) or
 * Audio Input Control Service (AICS) instances.
 *
 * @param conn          Connection to peer device, or NULL to get server value.
 * @param[out] service  Pointer to store the result in.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_get(struct bt_conn *conn, struct bt_vcs *service);

/**
 * @brief Callback function for bt_vcs_discover.
 *
 * This callback is only used for the client.
 *
 * @param conn         The connection that was used to discover
 *                     Volume Control Service.
 * @param err          Error value. 0 on success, GATT error on positive value
 *                     or errno on negative value.
 * @param vocs_count   Number of Volume Offset Control Service instances
 *                     on peer device.
 * @param aics_count   Number of Audio Input Control Service instances on
 *                     peer device.
 */
typedef void (*bt_vcs_discover_cb)(struct bt_conn *conn, int err,
				   uint8_t vocs_count, uint8_t aics_count);

/**
 * @brief Callback function for Volume Control Service volume state.
 *
 * Called when the value is locally read as the server.
 * Called when the value is remotely read as the client.
 * Called if the value is changed by either the server or client.
 *
 * @param conn    NULL if local server read or write, otherwise the connection
 *                to the peer device if remotely read or written.
 * @param err     Error value. 0 on success, GATT error on positive value
 *                or errno on negative value.
 * @param volume  The volume of the Volume Control Service server.
 * @param mute    The mute setting of the Volume Control Service server.
 */
typedef void (*bt_vcs_state_cb)(struct bt_conn *conn, int err, uint8_t volume,
				uint8_t mute);

/**
 * @brief Callback function for Volume Control Service flags.
 *
 * Called when the value is locally read as the server.
 * Called when the value is remotely read as the client.
 * Called if the value is changed by either the server or client.
 *
 * @param conn    NULL if local server read or write, otherwise the connection
 *                to the peer device if remotely read or written.
 * @param err     Error value. 0 on success, GATT error on positive value
 *                or errno on negative value.
 * @param flags   The flags of the Volume Control Service server.
 */
typedef void (*bt_vcs_flags_cb)(struct bt_conn *conn, int err, uint8_t flags);

/**
 * @brief Callback function for writes.
 *
 * This callback is only used for the client.
 *
 * @param conn    NULL if local server read or write, otherwise the connection
 *                to the peer device if remotely read or written.
 * @param err     Error value. 0 on success, GATT error on fail.
 */
typedef void (*bt_vcs_write_cb)(struct bt_conn *conn, int err);

struct bt_vcs_cb {
	/* Volume Control Service */
	bt_vcs_state_cb               state;
	bt_vcs_flags_cb               flags;
#if defined(CONFIG_BT_VCS_CLIENT)
	bt_vcs_discover_cb            discover;
	bt_vcs_write_cb               vol_down;
	bt_vcs_write_cb               vol_up;
	bt_vcs_write_cb               mute;
	bt_vcs_write_cb               unmute;
	bt_vcs_write_cb               vol_down_unmute;
	bt_vcs_write_cb               vol_up_unmute;
	bt_vcs_write_cb               vol_set;

	/* Volume Offset Control Service */
	struct bt_vocs_cb             vocs_cb;

	/* Audio Input Control Service */
	struct bt_aics_cb             aics_cb;
#endif /* CONFIG_BT_VCS_CLIENT */
};

/**
 * @brief Discover Volume Control Service and included services.
 *
 * This will start a GATT discovery and setup handles and subscriptions.
 * This shall be called once before any other actions can be
 * executed for the peer device.
 *
 * This shall only be done as the client,
 *
 * @param conn    The connection to discover Volume Control Service for.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_discover(struct bt_conn *conn);

/**
 * @brief Set the Volume Control Service volume step size.
 *
 * Set the value that the volume changes, when changed relatively with e.g.
 * @ref bt_vcs_vol_down or @ref bt_vcs_vol_up.
 *
 * This can only be done as the server.
 *
 * @param volume_step  The volume step size (1-255).
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_vol_step_set(uint8_t volume_step);

/**
 * @brief Read the Volume Control Service volume state.
 *
 * @param conn   Connection to the peer device,
 *               or NULL to read local server value.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_vol_get(struct bt_conn *conn);

/**
 * @brief Read the Volume Control Service flags.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_flags_get(struct bt_conn *conn);

/**
 * @brief Turn the volume down by one step on the server.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_vol_down(struct bt_conn *conn);

/**
 * @brief Turn the volume up by one step on the server.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_vol_up(struct bt_conn *conn);

/**
 * @brief Turn the volume down and unmute the server.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_unmute_vol_down(struct bt_conn *conn);

/**
 * @brief Turn the volume up and unmute the server.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_unmute_vol_up(struct bt_conn *conn);

/**
 * @brief Set the volume on the server
 *
 * @param conn   Connection to peer device, or NULL to set local server value.
 * @param volume The absolute volume to set.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_vol_set(struct bt_conn *conn, uint8_t volume);

/**
 * @brief Unmute the server.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_unmute(struct bt_conn *conn);

/**
 * @brief Mute the server.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_mute(struct bt_conn *conn);

/**
 * @brief Read the Volume Offset Control Service offset state.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 * @param inst   Pointer to the Volume Offset Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_vocs_state_get(struct bt_conn *conn, struct bt_vocs *inst);

/**
 * @brief Read the Volume Offset Control Service location.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 * @param inst   Pointer to the Volume Offset Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_vocs_location_get(struct bt_conn *conn, struct bt_vocs *inst);

/**
 * @brief Set the Volume Offset Control Service location.
 *
 * @param conn       Connection to peer device, or NULL to set local server
 *                   value.
 * @param inst       Pointer to the Volume Offset Control Service instance.
 * @param location   The location to set.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_vocs_location_set(struct bt_conn *conn, struct bt_vocs *inst,
			     uint8_t location);

/**
 * @brief Set the Volume Offset Control Service offset state.
 *
 * @param conn    Connection to peer device, or NULL to set local server value.
 * @param inst    Pointer to the Volume Offset Control Service instance.
 * @param offset  The offset to set (-255 to 255).
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_vocs_state_set(struct bt_conn *conn, struct bt_vocs *inst,
			  int16_t offset);

/**
 * @brief Read the Volume Offset Control Service output description.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 * @param inst   Pointer to the Volume Offset Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_vocs_description_get(struct bt_conn *conn, struct bt_vocs *inst);

/**
 * @brief Set the Volume Offset Control Service description.
 *
 * @param conn          Connection to peer device, or NULL to set local server
 *                      value.
 * @param inst          Pointer to the Volume Offset Control Service instance.
 * @param description   The description to set.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_vocs_description_set(struct bt_conn *conn, struct bt_vocs *inst,
				const char *description);

/**
 * @brief Deactivates an Audio Input Control Service instance.
 *
 * Audio Input Control Services are activated by default, but this will allow
 * the server to deactivate an Audio Input Control Service.
 *
 * @param inst   Pointer to the Audio Input Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_deactivate(struct bt_aics *inst);

/**
 * @brief Activates an Audio Input Control Service instance.
 *
 * Audio Input Control Services are activated by default, but this will allow
 * the server to reactivate an Audio Input Control Service instance after it has
 * been deactivated with @ref bt_vcs_aics_deactivate.
 *
 * @param inst   Pointer to the Audio Input Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_activate(struct bt_aics *inst);

/**
 * @brief Read the Audio Input Control Service input state.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 * @param inst   Pointer to the Audio Input Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_state_get(struct bt_conn *conn, struct bt_aics *inst);

/**
 * @brief Read the Audio Input Control Service gain settings.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 * @param inst   Pointer to the Audio Input Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_gain_setting_get(struct bt_conn *conn, struct bt_aics *inst);

/**
 * @brief Read the Audio Input Control Service input type.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 * @param inst   Pointer to the Audio Input Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_type_get(struct bt_conn *conn, struct bt_aics *inst);

/**
 * @brief Read the Audio Input Control Service input status.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 * @param inst   Pointer to the Audio Input Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_status_get(struct bt_conn *conn, struct bt_aics *inst);

/**
 * @brief Mute the Audio Input Control Service input.
 *
 * @param conn   Connection to peer device, or NULL to set local server value.
 * @param inst   Pointer to the Audio Input Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_mute(struct bt_conn *conn, struct bt_aics *inst);

/**
 * @brief Unmute the Audio Input Control Service input.
 *
 * @param conn   Connection to peer device, or NULL to set local server value.
 * @param inst   Pointer to the Audio Input Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_unmute(struct bt_conn *conn, struct bt_aics *inst);

/**
 * @brief Set input gain to manual.
 *
 * @param conn   Connection to peer device, or NULL to set local server value.
 * @param inst   Pointer to the Audio Input Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_manual_gain_set(struct bt_conn *conn, struct bt_aics *inst);

/**
 * @brief Set the input gain to automatic.
 *
 * @param conn   Connection to peer device, or NULL to set local server value.
 * @param inst   Pointer to the Audio Input Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_automatic_gain_set(struct bt_conn *conn, struct bt_aics *inst);

/**
 * @brief Set the input gain.
 *
 * @param conn   Connection to peer device, or NULL to set local server value.
 * @param inst   Pointer to the Audio Input Control Service instance.
 * @param gain   The gain in dB to set (-128 to 127).
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_gain_set(struct bt_conn *conn, struct bt_aics *inst,
			 int8_t gain);

/**
 * @brief Read the Audio Input Control Service description.
 *
 * @param conn   Connection to peer device, or NULL to read local server value.
 * @param inst   Pointer to the Audio Input Control Service instance.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_description_get(struct bt_conn *conn, struct bt_aics *inst);

/**
 * @brief Set the Audio Input Control Service description.
 *
 * @param conn          Connection to peer device, or NULL to set local server
 *                      value.
 * @param inst          Pointer to the Audio Input Control Service instance.
 * @param description   The description to set.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_aics_description_set(struct bt_conn *conn, struct bt_aics *inst,
				const char *description);

/**
 * @brief Registers the callbacks used by the Volume Control Service client.
 *
 * @param cb   The callback structure.
 *
 * @return 0 if success, errno on failure.
 */
int bt_vcs_client_cb_register(struct bt_vcs_cb *cb);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_VCS_H_ */
