/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <init.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <sys/check.h>

#include <bluetooth/services/ots.h>
#include "ots_internal.h"
#include "ots_obj_manager_internal.h"
#include "ots_dir_list_internal.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(bt_ots, CONFIG_BT_OTS_LOG_LEVEL);

#if defined(CONFIG_BT_OTS_OACP_READ_SUPPORT)
#define OACP_FEAT_BIT_READ BIT(BT_OTS_OACP_FEAT_READ)
#else
#define OACP_FEAT_BIT_READ 0
#endif

/* OACP features supported by Kconfig */
#define OACP_FEAT OACP_FEAT_BIT_READ

#if defined(CONFIG_BT_OTS_OLCP_GO_TO_SUPPORT)
#define OLCP_FEAT_BIT_GOTO BIT(BT_OTS_OLCP_FEAT_GO_TO)
#else
#define OLCP_FEAT_BIT_GOTO 0
#endif

/* OLCP features supported by Kconfig */
#define OLCP_FEAT OLCP_FEAT_BIT_GOTO

static ssize_t ots_feature_read(struct bt_conn *conn,
				const struct bt_gatt_attr *attr, void *buf,
				uint16_t len, uint16_t offset)
{
	struct bt_ots *ots = (struct bt_ots *) attr->user_data;

	LOG_DBG("OTS Feature GATT Read Operation");

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &ots->features,
		sizeof(ots->features));
}

static ssize_t ots_obj_name_read(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr, void *buf,
				 uint16_t len, uint16_t offset)
{
	struct bt_ots *ots = (struct bt_ots *) attr->user_data;

	LOG_DBG("OTS Object Name GATT Read Operation");

	if (!ots->cur_obj) {
		LOG_DBG("No Current Object selected in OTS!");
		return BT_GATT_ERR(BT_GATT_OTS_OBJECT_NOT_SELECTED);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 ots->cur_obj->metadata.name,
				 strlen(ots->cur_obj->metadata.name));
}

static ssize_t ots_obj_type_read(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr, void *buf,
				 uint16_t len, uint16_t offset)
{
	struct bt_ots *ots = (struct bt_ots *) attr->user_data;
	struct bt_ots_obj_metadata *obj_meta;

	LOG_DBG("OTS Object Type GATT Read Operation");

	if (!ots->cur_obj) {
		LOG_DBG("No Current Object selected in OTS!");
		return BT_GATT_ERR(BT_GATT_OTS_OBJECT_NOT_SELECTED);
	}

	obj_meta = &ots->cur_obj->metadata;
	if (obj_meta->type.uuid.type == BT_UUID_TYPE_128) {
		return bt_gatt_attr_read(conn, attr, buf, len, offset,
					 obj_meta->type.uuid_128.val,
					 sizeof(obj_meta->type.uuid_128.val));
	} else {
		return bt_gatt_attr_read(conn, attr, buf, len, offset,
					 &obj_meta->type.uuid_16.val,
					 sizeof(obj_meta->type.uuid_16.val));
	}
}

static ssize_t ots_obj_size_read(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr, void *buf,
				 uint16_t len, uint16_t offset)
{
	struct bt_ots *ots = (struct bt_ots *) attr->user_data;

	LOG_DBG("OTS Object Size GATT Read Operation");

	if (!ots->cur_obj) {
		LOG_DBG("No Current Object selected in OTS!");
		return BT_GATT_ERR(BT_GATT_OTS_OBJECT_NOT_SELECTED);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &ots->cur_obj->metadata.size,
				 sizeof(ots->cur_obj->metadata.size));
}

static ssize_t ots_obj_id_read(struct bt_conn *conn,
			       const struct bt_gatt_attr *attr, void *buf,
			       uint16_t len, uint16_t offset)
{
	struct bt_ots *ots = (struct bt_ots *) attr->user_data;
	uint8_t id[BT_OTS_OBJ_ID_SIZE];
	char id_str[BT_OTS_OBJ_ID_STR_LEN];

	LOG_DBG("OTS Object ID GATT Read Operation");

	if (!ots->cur_obj) {
		LOG_DBG("No Current Object selected in OTS!");
		return BT_GATT_ERR(BT_GATT_OTS_OBJECT_NOT_SELECTED);
	}

	sys_put_le48(ots->cur_obj->id, id);

	bt_ots_obj_id_to_str(ots->cur_obj->id, id_str,
				      sizeof(id_str));
	LOG_DBG("Current Object ID: %s", log_strdup(id_str));

	return bt_gatt_attr_read(conn, attr, buf, len, offset, id, sizeof(id));
}

static ssize_t ots_obj_prop_read(struct bt_conn *conn,
				 const struct bt_gatt_attr *attr, void *buf,
				 uint16_t len, uint16_t offset)
{
	struct bt_ots *ots = (struct bt_ots *) attr->user_data;

	LOG_DBG("OTS Object Properties GATT Read Operation");

	if (!ots->cur_obj) {
		LOG_DBG("No Current Object selected in OTS!");
		return BT_GATT_ERR(BT_GATT_OTS_OBJECT_NOT_SELECTED);
	}

	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 &ots->cur_obj->metadata.props,
				 sizeof(ots->cur_obj->metadata.props));
}

int bt_ots_obj_add(struct bt_ots *ots,
			    struct bt_ots_obj_metadata *obj_init)
{
	int err;
	struct bt_gatt_ots_object *obj;
	size_t name_len;

	if (IS_ENABLED(CONFIG_BT_OTS_DIR_LIST_OBJ) && ots->dir_list &&
	    ots->dir_list->dir_list_obj->state.type != BT_GATT_OTS_OBJECT_IDLE_STATE) {
		LOG_DBG("Directory Listing Object is being read");
		return -EBUSY;
	}

	name_len = strlen(obj_init->name);

	CHECKIF(name_len == 0 || name_len > BT_OTS_OBJ_MAX_NAME_LEN) {
		LOG_DBG("Invalid name length %zu", name_len);
		return -EINVAL;
	}

	err = bt_gatt_ots_obj_manager_obj_add(ots->obj_manager, &obj);
	if (err) {
		LOG_ERR("No space available in the object manager");
		return err;
	}

	/* Initialize object. */
	memcpy(&obj->metadata, obj_init, sizeof(obj->metadata));

	if (IS_ENABLED(CONFIG_BT_OTS_DIR_LIST_OBJ)) {
		bt_ots_dir_list_obj_add(ots->dir_list, ots->obj_manager, ots->cur_obj, obj);
	}

	/* Request object data. */
	if (ots->cb->obj_created) {
		err = ots->cb->obj_created(ots, NULL, obj->id, obj_init);
		if (err) {
			bt_gatt_ots_obj_manager_obj_delete(obj);

			if (IS_ENABLED(CONFIG_BT_OTS_DIR_LIST_OBJ)) {
				bt_ots_dir_list_obj_remove(ots->dir_list, ots->obj_manager,
							   ots->cur_obj, obj);
			}

			return err;
		}
	}

	return 0;
}

int bt_ots_obj_delete(struct bt_ots *ots, uint64_t id)
{
	int err;
	struct bt_gatt_ots_object *obj;

	err = bt_gatt_ots_obj_manager_obj_get(ots->obj_manager, id, &obj);
	if (err) {
		return err;
	}

	if (ots->cur_obj == obj) {
		if (obj->state.type != BT_GATT_OTS_OBJECT_IDLE_STATE) {
			return -EBUSY;
		}
		ots->cur_obj = NULL;
	}

	if (IS_ENABLED(CONFIG_BT_OTS_DIR_LIST_OBJ) && ots->dir_list &&
	    ots->dir_list->dir_list_obj->state.type != BT_GATT_OTS_OBJECT_IDLE_STATE) {
		LOG_DBG("Directory Listing Object is being read");
		return -EBUSY;
	}

	err = bt_gatt_ots_obj_manager_obj_delete(obj);
	if (err) {
		return err;
	}

	if (IS_ENABLED(CONFIG_BT_OTS_DIR_LIST_OBJ)) {
		bt_ots_dir_list_obj_remove(ots->dir_list, ots->obj_manager, ots->cur_obj, obj);
	}

	if (ots->cb->obj_deleted) {
		ots->cb->obj_deleted(ots, NULL, obj->id);
	}

	return 0;
}

#if defined(CONFIG_BT_OTS_SECONDARY_SVC)
void *bt_ots_svc_decl_get(struct bt_ots *ots)
{
	return ots->service->attrs;
}
#endif

int bt_ots_init(struct bt_ots *ots,
		     struct bt_ots_init *ots_init)
{
	int err;

	if (!ots || !ots_init || !ots_init->cb) {
		return -EINVAL;
	}

	__ASSERT(ots_init->cb->obj_created,
		 "Callback for object creation is not set");
	__ASSERT(ots_init->cb->obj_read ||
		 !BT_OTS_OACP_GET_FEAT_READ(ots_init->features.oacp),
		 "Callback for object reading is not set");

	/* Set callback structure. */
	ots->cb = ots_init->cb;

	/* Check OACP supported features against Kconfig. */
	if (ots_init->features.oacp & (~((uint32_t) OACP_FEAT))) {
		return -ENOTSUP;
	}

	ots->features.oacp = ots_init->features.oacp;
	LOG_DBG("OACP features: 0x%04X", ots->features.oacp);

	/* Check OLCP supported features against Kconfig. */
	if (ots_init->features.olcp & (~((uint32_t) OLCP_FEAT))) {
		return -ENOTSUP;
	}
	ots->features.olcp = ots_init->features.olcp;
	LOG_DBG("OLCP features: 0x%04X", ots->features.olcp);

	/* Register L2CAP context. */
	err = bt_gatt_ots_l2cap_register(&ots->l2cap);
	if (err) {
		return err;
	}

	err = bt_gatt_service_register(ots->service);
	if (err) {
		bt_gatt_ots_l2cap_unregister(&ots->l2cap);

		return err;
	}

	if (IS_ENABLED(CONFIG_BT_OTS_DIR_LIST_OBJ)) {
		bt_ots_dir_list_init(&ots->dir_list, ots->obj_manager);
	}

	LOG_DBG("Initialized OTS");

	return 0;
}

#if defined(CONFIG_BT_OTS_SECONDARY_SVC)
	#define BT_GATT_OTS_SERVICE	BT_GATT_SECONDARY_SERVICE
#else
	#define BT_GATT_OTS_SERVICE	BT_GATT_PRIMARY_SERVICE
#endif

#define BT_GATT_OTS_ATTRS(_ots) {					\
	BT_GATT_OTS_SERVICE(BT_UUID_OTS),				\
	BT_GATT_CHARACTERISTIC(BT_UUID_OTS_FEATURE,			\
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,			\
		ots_feature_read, NULL, &_ots),				\
	BT_GATT_CHARACTERISTIC(BT_UUID_OTS_NAME,			\
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,			\
		ots_obj_name_read, NULL, &_ots),			\
	BT_GATT_CHARACTERISTIC(BT_UUID_OTS_TYPE,			\
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,			\
		ots_obj_type_read, NULL, &_ots),			\
	BT_GATT_CHARACTERISTIC(BT_UUID_OTS_SIZE,			\
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,			\
		ots_obj_size_read, NULL, &_ots),			\
	BT_GATT_CHARACTERISTIC(BT_UUID_OTS_ID,				\
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,			\
		ots_obj_id_read, NULL, &_ots),				\
	BT_GATT_CHARACTERISTIC(BT_UUID_OTS_PROPERTIES,			\
		BT_GATT_CHRC_READ, BT_GATT_PERM_READ,			\
		ots_obj_prop_read, NULL, &_ots),			\
	BT_GATT_CHARACTERISTIC(BT_UUID_OTS_ACTION_CP,			\
		BT_GATT_CHRC_WRITE | BT_GATT_CHRC_INDICATE,		\
		BT_GATT_PERM_WRITE, NULL,				\
		bt_gatt_ots_oacp_write, &_ots),				\
	BT_GATT_CCC_MANAGED(&_ots.oacp_ind.ccc,				\
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),		\
	BT_GATT_CHARACTERISTIC(BT_UUID_OTS_LIST_CP,			\
		BT_GATT_CHRC_WRITE | BT_GATT_CHRC_INDICATE,		\
		BT_GATT_PERM_WRITE, NULL,				\
		bt_gatt_ots_olcp_write, &_ots),				\
	BT_GATT_CCC_MANAGED(&_ots.olcp_ind.ccc,				\
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)			\
}

#define BT_GATT_OTS_INSTANCE_LIST_SIZE	(ARRAY_SIZE(ots_instances))
#define BT_GATT_OTS_INSTANCE_LIST_START	ots_instances
#define BT_GATT_OTS_INSTANCE_LIST_END	\
	(&ots_instances[BT_GATT_OTS_INSTANCE_LIST_SIZE])

#define BT_GATT_OTS_SERVICE_LIST_START	ots_service_list

static struct bt_ots ots_instances[CONFIG_BT_OTS_MAX_INST_CNT];
static uint32_t instance_cnt;
BT_GATT_SERVICE_INSTANCE_DEFINE(ots_service_list, ots_instances,
				CONFIG_BT_OTS_MAX_INST_CNT,
				BT_GATT_OTS_ATTRS);

struct bt_ots *bt_ots_free_instance_get(void)
{
	if (instance_cnt >= BT_GATT_OTS_INSTANCE_LIST_SIZE) {
		return NULL;
	}

	return &BT_GATT_OTS_INSTANCE_LIST_START[instance_cnt++];
}

static int bt_gatt_ots_instances_prepare(const struct device *dev)
{
	uint32_t index;
	struct bt_ots *instance;

	for (instance = BT_GATT_OTS_INSTANCE_LIST_START, index = 0;
	     instance != BT_GATT_OTS_INSTANCE_LIST_END;
	     instance++, index++) {
		/* Assign an object pool to the OTS instance. */
		instance->obj_manager = bt_gatt_ots_obj_manager_assign();

		if (!instance->obj_manager) {
			LOG_ERR("OTS Object manager instance not available");
			return -ENOMEM;
		}

		/* Assign pointer to the service descriptor. */
		instance->service = &BT_GATT_OTS_SERVICE_LIST_START[index];

		/* Initialize CCC descriptors for characteristics with
		 * indication properties.
		 */
		instance->oacp_ind.ccc.cfg_changed =
			bt_gatt_ots_oacp_cfg_changed;
		instance->olcp_ind.ccc.cfg_changed =
			bt_gatt_ots_olcp_cfg_changed;
	}

	return 0;
}

SYS_INIT(bt_gatt_ots_instances_prepare, APPLICATION,
	 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
