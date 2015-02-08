/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <mhi_sys.h>
#include <mhi.h>
extern mhi_pcie_devices mhi_devices;
int mhi_ssr_notify_cb(struct notifier_block *nb,
			unsigned long action, void *data);


static struct notifier_block mhi_ssr_nb = {
	.notifier_call = mhi_ssr_notify_cb,
};

static void esoc_parse_link_type(mhi_device_ctxt* mhi_dev_ctxt)
{
	int ret_val;
	ret_val = strncmp(mhi_dev_ctxt->esoc_handle->link,
				"HSIC+PCIe",
				sizeof("HSIC+PCIe"));
	mhi_log(MHI_MSG_VERBOSE, "Link type is %s as indicated by ESOC\n",
					mhi_dev_ctxt->esoc_handle->link);
	if (ret_val)
		mhi_dev_ctxt->base_state = STATE_TRANSITION_BHI;
	else
		mhi_dev_ctxt->base_state = STATE_TRANSITION_RESET;
}

int mhi_esoc_register(mhi_device_ctxt* mhi_dev_ctxt)
{
	int ret_val = 0;
	struct device_node *np;
	struct pci_driver *mhi_driver;
	struct device *dev = &mhi_dev_ctxt->dev_info->pcie_device->dev;

	mhi_driver = mhi_dev_ctxt->dev_info->mhi_pcie_driver;
	np = dev->of_node;
	mhi_dev_ctxt->esoc_handle = devm_register_esoc_client(dev, "mdm");
	mhi_log(MHI_MSG_VERBOSE,
		"Of table of pcie device property is dev->of_node %p \n",
		np);
	if (IS_ERR_OR_NULL(mhi_dev_ctxt->esoc_handle)) {
		mhi_log(MHI_MSG_CRITICAL,
			"Failed to register for SSR, ret %lx\n",
			(uintptr_t)mhi_dev_ctxt->esoc_handle);
		return -EIO;
	}

	esoc_parse_link_type(mhi_dev_ctxt);

	mhi_dev_ctxt->esoc_ssr_handle = subsys_notif_register_notifier(
					mhi_dev_ctxt->esoc_handle->name,
					&mhi_ssr_nb);
	if (IS_ERR_OR_NULL(mhi_dev_ctxt->esoc_ssr_handle)) {
		ret_val = PTR_RET(mhi_dev_ctxt->esoc_ssr_handle);
		mhi_log(MHI_MSG_CRITICAL,
			"Can't find esoc desc ret 0x%lx\n",
			(uintptr_t)mhi_dev_ctxt->esoc_ssr_handle);
	}

	return ret_val;
}


void mhi_notify_clients(mhi_device_ctxt *mhi_dev_ctxt, MHI_CB_REASON reason)
{
	u32 i;
	mhi_client_handle *client_handle = NULL;
	mhi_cb_info cb_info = {0};
	mhi_result result = {0};
	for (i = 0; i < MHI_MAX_CHANNELS; ++i) {
		if (VALID_CHAN_NR(i)) {
			client_handle =
				mhi_dev_ctxt->client_handle_list[i];
			cb_info.result = NULL;
			cb_info.cb_reason = reason;
			if (NULL != client_handle &&
					NULL != client_handle->client_info.mhi_client_cb) {
				result.user_data = client_handle->user_data;
				cb_info.result = &result;
				client_handle->client_info.mhi_client_cb(&cb_info);
			}
		}
	}
}

MHI_STATUS mhi_process_link_down(mhi_device_ctxt *mhi_dev_ctxt)
{
	unsigned long flags;
	int r;

	mhi_log(MHI_MSG_INFO, "Entered.\n");
	if (NULL == mhi_dev_ctxt)
		return MHI_STATUS_ERROR;
	switch(hrtimer_try_to_cancel(&mhi_dev_ctxt->inactivity_tmr))
	{
		case 0:
			mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
					"Timer was not active\n");
			break;
		case 1:
			mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
					"Timer was active\n");
			break;
		case -1:
			mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
					"Timer executing and can't stop\n");
	}
	mhi_log(MHI_MSG_CRITICAL,
			"Informing clients of MHI that link is down\n");
	write_lock_irqsave(&mhi_dev_ctxt->xfer_lock, flags);
	mhi_dev_ctxt->link_up = 0;
	mhi_dev_ctxt->mhi_state = MHI_STATE_RESET;
	mhi_deassert_device_wake(mhi_dev_ctxt);
	write_unlock_irqrestore(&mhi_dev_ctxt->xfer_lock, flags);
	//mhi_notify_clients(mhi_dev_ctxt, MHI_CB_MHI_DISABLED);
	mhi_dev_ctxt->mhi_initialized = 0;
	r =
		msm_bus_scale_client_update_request(mhi_dev_ctxt->bus_client, 0);
	if (r)
		mhi_log(MHI_MSG_INFO,
			"Failed to scale bus request to sleep set.\n");
	mhi_dev_ctxt->dev_info->link_down_cntr++;
	atomic_set(&mhi_dev_ctxt->data_pending, 0);
	mhi_log(MHI_MSG_INFO, "Exited.\n");
	return MHI_STATUS_SUCCESS;
}

void mhi_link_state_cb(struct msm_pcie_notify *notify)
{
	int r;
	mhi_pcie_dev_info *mhi_pcie_dev;
	mhi_device_ctxt *mhi_dev_ctxt = NULL;
	mhi_log(MHI_MSG_INFO, "Entered\n");
	if (NULL == notify || NULL == notify->data) {
		mhi_log(MHI_MSG_CRITICAL,
				"Incomplete handle received\n");
		return;
	}
	mhi_pcie_dev = notify->data;
	mhi_dev_ctxt = mhi_pcie_dev->mhi_ctxt;
	switch (notify->event){
		case MSM_PCIE_EVENT_NO_ACCESS:
			mhi_log(MHI_MSG_CRITICAL,
					"Received NO_ACCESS event\n");
		case MSM_PCIE_EVENT_LINKDOWN:
			mhi_log(MHI_MSG_CRITICAL,
					"Received MSM_PCIE_EVENT_LINKDOWN\n");
			if (MHI_STATUS_SUCCESS !=
					mhi_process_link_down(mhi_dev_ctxt)){
				mhi_log(MHI_MSG_CRITICAL,
						"Failed to process link down\n");
			}
			break;
		case MSM_PCIE_EVENT_LINKUP:
			mhi_log(MHI_MSG_CRITICAL,
					"Received MSM_PCIE_EVENT_LINKUP\n");
			if (0 == mhi_pcie_dev->link_up_cntr) {
				mhi_log(MHI_MSG_INFO,
						"Initializing MHI for the first time\n");
				mhi_startup_thread(mhi_pcie_dev);
				mhi_dev_ctxt = mhi_pcie_dev->mhi_ctxt;
				pci_set_master(mhi_pcie_dev->pcie_device);
			} else {
				mhi_log(MHI_MSG_INFO,
						"Received Link Up Callback\n");
			}

			mhi_pcie_dev->link_up_cntr++;
			break;
		case MSM_PCIE_EVENT_WAKEUP:
			mhi_log(MHI_MSG_CRITICAL,
					"Received MSM_PCIE_WAKEUP_EVENT\n");
			if (!atomic_cmpxchg(&mhi_dev_ctxt->link_ops_flag, 0, 1)) {
				mutex_lock(&mhi_dev_ctxt->mhi_link_state);
				r = msm_pcie_pm_control(MSM_PCIE_RESUME,
						mhi_pcie_dev->pcie_device->bus->number,
						mhi_pcie_dev->pcie_device, NULL,
						MSM_PCIE_CONFIG_NO_CFG_RESTORE | MSM_PCIE_CONFIG_LINKDOWN);
				if (r) {
					mhi_log(MHI_MSG_CRITICAL,
							"Failed to restore PCIe link ret: %d\n", r);
					mhi_dev_ctxt->link_up = 0;
				} else {
					r = msm_pcie_recover_config(mhi_pcie_dev->pcie_device);
					if (r) {
						mhi_log(MHI_MSG_CRITICAL,
								"Failed to restore PCIe config space ret: %d\n", r);
					} else {
						r = pci_set_power_state(mhi_dev_ctxt->dev_info->pcie_device, PCI_D0);
						if (r) {
							mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
									"Failed to set power state 0x%x\n", r);
						} else {
							mhi_dev_ctxt->link_up = 1;
						}
					}
				}
				mutex_unlock(&mhi_dev_ctxt->mhi_link_state);
				atomic_set(&mhi_dev_ctxt->link_ops_flag, 0);
			}
			break;
		default:
			mhi_log(MHI_MSG_INFO,
					"Received bad link event\n");
			break;
	}
	mhi_log(MHI_MSG_INFO, "%s","Exited\n");
}
int mhi_ssr_notify_cb(struct notifier_block *nb,
		unsigned long action, void *data)
{
	unsigned long flags;
	int ret_val = 0;
	int max_retries = 5;
	mhi_device_ctxt *mhi_dev_ctxt = mhi_devices.device_list[0].mhi_ctxt;
	mhi_pcie_dev_info *mhi_pcie_dev = NULL;
	mhi_pcie_dev = &mhi_devices.device_list[mhi_devices.nr_of_devices];
	mhi_log(MHI_MSG_VERBOSE,
		"Received Subsystem event 0x%lx, from esoc %p\n",
		action, data);
	if (NULL != mhi_dev_ctxt)
		mhi_dev_ctxt->esoc_notif = action;
	switch (action) {
		case SUBSYS_AFTER_POWERUP:
			mhi_log(MHI_MSG_VERBOSE,
					"Received Subsystem event AFTER_POWERUP\n");
			ret_val = init_mhi_base_state(mhi_dev_ctxt);
			if (ret_val != MHI_STATUS_SUCCESS)
				mhi_log(MHI_MSG_INFO,
						"Could not reset MHI, ret: %d\n", ret_val);

			break;
		case SUBSYS_BEFORE_SHUTDOWN:
			mhi_log(MHI_MSG_INFO,
					"Received Subsystem event BEFORE_SHUTDOWN\n");
			break;
		case SUBSYS_AFTER_SHUTDOWN:
			mhi_log(MHI_MSG_INFO,
					"Received Subsystem event AFTER_SHUTDOWN\n");

			write_lock_irqsave(&mhi_dev_ctxt->xfer_lock, flags);
			mhi_dev_ctxt->mhi_state = MHI_STATE_RESET;
			ret_val = mhi_assert_device_wake(mhi_dev_ctxt);
			write_unlock_irqrestore(&mhi_dev_ctxt->xfer_lock, flags);
			mhi_notify_clients(mhi_dev_ctxt, MHI_CB_MHI_DISABLED);
			mhi_dev_ctxt->mhi_initialized = 0;
			mhi_pcie_dev->link_down_cntr++;

			while (--max_retries) {
				if (!atomic_cmpxchg(&mhi_dev_ctxt->link_ops_flag, 0, 1)) {
					mhi_log(MHI_MSG_CRITICAL,
							"Acquired critical section\n");

					ret_val = msm_pcie_pm_control(MSM_PCIE_SUSPEND,
							mhi_dev_ctxt->dev_info->pcie_device->bus->number,
							mhi_dev_ctxt->dev_info->pcie_device, NULL,
							MSM_PCIE_CONFIG_NO_CFG_RESTORE);
					if (ret_val)
						mhi_log(MHI_MSG_CRITICAL,
								"Failed to suspend link\n");
					mhi_dev_ctxt->link_up = 0;
					return 0;
				} else {
					msleep(10);
					mhi_log(MHI_MSG_CRITICAL,
							"Could not acquire critical section, waiting.\n");
				}
			}
			break;
		case SUBSYS_BEFORE_POWERUP:
			mhi_log(MHI_MSG_VERBOSE,
					"Received Subsystem event BEFORE_POWERUP\n");
			atomic_set(&mhi_dev_ctxt->link_ops_flag, 0);
			break;
		case SUBSYS_RAMDUMP_NOTIFICATION:
			mhi_log(MHI_MSG_VERBOSE,
					"Received Subsystem event RAMDUMP_NOTIFICATION\n");
			break;
		case SUBSYS_PROXY_VOTE:
			mhi_log(MHI_MSG_VERBOSE,
					"Received Subsystem event SYBSYS_PROXY_VOTE\n");
			break;
		case SUBSYS_PROXY_UNVOTE:
			mhi_log(MHI_MSG_VERBOSE,
					"Received Subsystem event SYBSYS_PROXY_UNVOTE\n");
			break;
		case SUBSYS_POWERUP_FAILURE:
			mhi_log(MHI_MSG_VERBOSE,
					"Received Subsystem event SYBSYS_POWERUP_FAILURE\n");
			break;
	}
	return NOTIFY_OK;
}

MHI_STATUS init_mhi_base_state(mhi_device_ctxt* mhi_dev_ctxt)
{
	int r = 0;
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;

	mhi_assert_device_wake(mhi_dev_ctxt);
	mhi_dev_ctxt->link_up = 1;
	r =
	msm_bus_scale_client_update_request(mhi_dev_ctxt->bus_client, 1);
	if (r)
	mhi_log(MHI_MSG_INFO,
		"Failed to scale bus request to active set.\n");
	ret_val = mhi_init_state_transition(mhi_dev_ctxt,
			mhi_dev_ctxt->base_state);
	if (MHI_STATUS_SUCCESS != ret_val) {
		mhi_log(MHI_MSG_CRITICAL,
		"Failed to start state change event, to %d\n",
		mhi_dev_ctxt->base_state);
	}
	return ret_val;
}

