/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#include "msm_mhi.h"
#include "mhi_sys.h"
#include "mhi.h"
#include "mhi_hwio.h"

/* Write only sysfs attributes */
static DEVICE_ATTR(MHI_M3, S_IWUSR, NULL, sysfs_init_M3);
static DEVICE_ATTR(MHI_M0, S_IWUSR, NULL, sysfs_init_M0);
static DEVICE_ATTR(MHI_M1, S_IWUSR, NULL, sysfs_init_M1);

/* Read only sysfs attributes */
static DEVICE_ATTR(MHI_STATE, S_IRUSR, sysfs_get_mhi_state, NULL);

static struct attribute *mhi_attributes[] = {
	&dev_attr_MHI_M3.attr,
	&dev_attr_MHI_M0.attr,
	&dev_attr_MHI_M1.attr,
	&dev_attr_MHI_STATE.attr,
	NULL,
};

static struct attribute_group mhi_attribute_group = {
	.attrs = mhi_attributes,
};

int mhi_suspend(struct pci_dev *pcie_dev, pm_message_t state)
{
	int ret_val = 0;
	mhi_device_ctxt *mhi_dev_ctxt =
		*(mhi_device_ctxt **)((pcie_dev->dev).platform_data);
	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER, "Entered, state %d\n",
						state.event);
	if (NULL == mhi_dev_ctxt)
		return 0;

	if (mhi_dev_ctxt->mhi_state == MHI_STATE_M0 ||
	    mhi_dev_ctxt->mhi_state == MHI_STATE_M1 ||
	    mhi_dev_ctxt->mhi_state == MHI_STATE_M2) {
	if (0 != mhi_initiate_M3(mhi_dev_ctxt))
		return -EIO;
	} else {
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"State is already %d did we get M3 completion?\n",
				mhi_dev_ctxt->mhi_state);
	}
	ret_val =
		msm_bus_scale_client_update_request(mhi_dev_ctxt->bus_client, 0);
	if (ret_val)
		mhi_log(MHI_MSG_INFO, "%s", "Failed to update bus request\n");

	if (!mhi_dev_ctxt->link_up) {
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER, "%s",
				"Link is not up, nothing to do.\n");
		return 0;
	}
	if (MHI_STATUS_SUCCESS != mhi_turn_off_pcie_link(mhi_dev_ctxt)) {
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
				"Failed to turn off PCIe link.\n");
		return 0;
	}

	mhi_deassert_device_wake(mhi_dev_ctxt);
	return 0;
}

int mhi_resume(struct pci_dev *pcie_dev)
{
	int r = 0;
	int max_retries = 10;
	mhi_device_ctxt *mhi_dev_ctxt =
		*(mhi_device_ctxt **)((pcie_dev->dev).platform_data);
	if (NULL == mhi_dev_ctxt)
		return 0;

	if (r)
		mhi_log(MHI_MSG_ERROR | MHI_DBG_POWER, "%s",
				"Could not set DEVICE WAKE GPIO HIGH\n");

	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to set power state 0x%x\n", r);
		return -EIO;
	}

	while (--max_retries) {
		if (!atomic_cmpxchg(&mhi_dev_ctxt->link_ops_flag, 0, 1)) {
			if (MHI_STATUS_SUCCESS != mhi_turn_on_pcie_link(mhi_dev_ctxt)) {
				mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
						"Failed to turn on PCIe link.\n");
				atomic_set(&mhi_dev_ctxt->link_ops_flag, 0);
				return -EIO;
			}
			if (!mhi_dev_ctxt->link_up) {
				mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
						"Link is not up, nothing to do.\n");
				atomic_set(&mhi_dev_ctxt->link_ops_flag, 0);
				return 0;
			}

			if (mhi_dev_ctxt->pending_M3) {
				mhi_log(MHI_MSG_CRITICAL,
						"Device did not ACK previous suspend request MHI STATE is 0x%x\n",
						mhi_dev_ctxt->mhi_state);
				atomic_set(&mhi_dev_ctxt->link_ops_flag, 0);
				return -ENETRESET;
			}

			mhi_initiate_M0(mhi_devices.device_list[0].mhi_ctxt);

			r = wait_event_interruptible_timeout(*mhi_devices.device_list[0].mhi_ctxt->M0_event,
					mhi_devices.device_list[0].mhi_ctxt->mhi_state != MHI_STATE_M3,
					msecs_to_jiffies(MHI_MAX_RESUME_TIMEOUT));
			switch(r) {
				case 0:
					mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
							"MDM failed to resume after %d ms\n",
							MHI_MAX_RESUME_TIMEOUT);
					r = -ETIMEDOUT;
					break;
				case -ERESTARTSYS:
					mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
							"Going Down...\n");
					r = -ENETRESET;
					break;
				default:
					if (mhi_devices.device_list[0].mhi_ctxt->mhi_state == MHI_STATE_RESET){
						mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
							"MHI state is RESET, SSR is occured\n");
						r = - ENETRESET;
					} else {
						mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
								"M0 event received\n");
						r = 0;
					}
					break;
			}
			atomic_set(&mhi_dev_ctxt->link_ops_flag, 0);
			return r;
		} else {
			mhi_log(MHI_MSG_INFO| MHI_DBG_POWER,
					"Could not acquire critical waiting retrt %d\n", max_retries);
			msleep(10);
		}
	}
	return r;
}

enum hrtimer_restart mhi_initiate_M1(struct hrtimer *timer)
{
	int ret_val = 0;
	unsigned long flags;
	ktime_t curr_time, timer_inc;
	mhi_device_ctxt *mhi_dev_ctxt = container_of(timer,
						mhi_device_ctxt,
						inactivity_tmr);
	write_lock_irqsave(&mhi_dev_ctxt->xfer_lock, flags);

	/* We will allow M1 if no data is pending, the current
	 * state is M0 and no M3 transition is pending */
	if ((0 == atomic_read(&mhi_dev_ctxt->data_pending)) &&
			(MHI_STATE_M1 == mhi_dev_ctxt->mhi_state ||
			 MHI_STATE_M0 == mhi_dev_ctxt->mhi_state) &&
			(0 == mhi_dev_ctxt->pending_M3) &&
			mhi_dev_ctxt->mhi_initialized &&
			(0 == atomic_read(&mhi_dev_ctxt->outbound_acks))) {
		mhi_dev_ctxt->mhi_state = MHI_STATE_M1;
		ret_val = mhi_deassert_device_wake(mhi_dev_ctxt);
		mhi_dev_ctxt->m0_m1++;
		if (ret_val)
			mhi_log(MHI_MSG_ERROR | MHI_DBG_POWER, "%s",
				"Could not set DEVICE WAKE GPIO LOW\n");
	}
	write_unlock_irqrestore(&mhi_dev_ctxt->xfer_lock, flags);
	if (mhi_dev_ctxt->mhi_state == MHI_STATE_M0 ||
	    mhi_dev_ctxt->mhi_state == MHI_STATE_M1 ||
	    mhi_dev_ctxt->mhi_state == MHI_STATE_READY) {
		curr_time = ktime_get();
		timer_inc = ktime_set(0, MHI_M1_ENTRY_DELAY_MS * 1E6L);
		hrtimer_forward(timer, curr_time, timer_inc);
		return HRTIMER_RESTART;
	}
	return HRTIMER_NORESTART;
}

int mhi_initiate_M0(mhi_device_ctxt *mhi_dev_ctxt)
{
	int ret_val = 0;
	int r = 0;
	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER, "%s",
			"Initializing state transiton to M0\n");

	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER, "%s",
			"Setting WAKE GPIO HIGH.\n");
	ret_val = mhi_assert_device_wake(mhi_dev_ctxt);
	if (ret_val)
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Failed to set DEVICE WAKE GPIO ret 0x%d.\n", ret_val);
	if (!mhi_dev_ctxt->link_up) {
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER, "%s",
			"Link is not up, nothing to do, quitting.\n");
		return 0;
	}
	if (mhi_dev_ctxt->mhi_state == MHI_STATE_M2) {
		r = wait_event_interruptible_timeout(*mhi_dev_ctxt->M0_event,
		mhi_dev_ctxt->mhi_state != MHI_STATE_M2,
		msecs_to_jiffies(MHI_MAX_RESUME_TIMEOUT));
		if (r) {
			mhi_log(MHI_MSG_INFO | MHI_DBG_POWER, "%s",
				"MDM failed to come out of M2.\n");
			return -ENETRESET;
		}
	} else {
	MHI_REG_WRITE_FIELD(mhi_dev_ctxt->mmio_addr, MHICTRL,
			MHICTRL_MHISTATE_MASK,
			MHICTRL_MHISTATE_SHIFT,
			MHI_STATE_M0);
	}
	r = wait_event_interruptible_timeout(*mhi_devices.device_list[0].mhi_ctxt->M0_event,
		mhi_devices.device_list[0].mhi_ctxt->mhi_state != MHI_STATE_M3,
		msecs_to_jiffies(MHI_MAX_RESUME_TIMEOUT));
	switch(r) {
	case 0:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"MDM failed to resume after 0x%x ms\n",
			MHI_MAX_RESUME_TIMEOUT);
		mhi_dev_ctxt->m0_event_timeouts++;
		break;
	case -ERESTARTSYS:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER, "%s",
			"Going Down...\n");
		break;
	default:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER, "%s",
				"M0 event received\n");
		break;
	}
	return 0;
}

int mhi_initiate_M3(mhi_device_ctxt *mhi_dev_ctxt)
{
	unsigned long flags = 0;
	u32 i = 0;
	u32 failed_stop = 1;
	u32 ret_val = 0;
	int r = 1;
	u32 pcie_word_val = 0;

	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER, "%s", "Entering...\n");

	if (ret_val)
		mhi_log(MHI_MSG_CRITICAL,
			"Could not set bus frequency ret: %d\n",
			ret_val);
	write_lock_irqsave(&mhi_dev_ctxt->xfer_lock, flags);
	mhi_dev_ctxt->pending_M3 = 1;
	mhi_assert_device_wake(mhi_dev_ctxt);
	write_unlock_irqrestore(&mhi_dev_ctxt->xfer_lock, flags);

	if (mhi_dev_ctxt->mhi_state == MHI_STATE_M2) {
		r = wait_event_interruptible_timeout(*mhi_dev_ctxt->M0_event,
			MHI_STATE_M0 == mhi_dev_ctxt->mhi_state,
			msecs_to_jiffies(MHI_M0_EVENT_TIMEOUT_MS));

	}
	if (r == 0)	{
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
				"Timed out while waiting for M2 completion\n");
		return -ETIMEDOUT;
	} else if (-ERESTARTSYS == r) {
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
					"Caught signal, quitting\n");
		return r;
	}

	while (i < MHI_MAX_SUSPEND_RETRIES) {
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Waiting for clients to stop, clients still active %d\n",
			atomic_read(&mhi_dev_ctxt->data_pending));
		if (atomic_read(&mhi_dev_ctxt->data_pending) > 0) {
			++i;
			usleep(20000);
		} else {
			failed_stop = 0;
			break;
		}
	}
	if (failed_stop)
		return -EPERM;

	/* Since we are going down, inform all clients
	 * that no further reads are possible */
	MHI_REG_WRITE_FIELD(mhi_dev_ctxt->mmio_addr, MHICTRL,
			MHICTRL_MHISTATE_MASK,
			MHICTRL_MHISTATE_SHIFT,
			MHI_STATE_M3);
	MHI_REG_READ_FIELD(mhi_dev_ctxt->mmio_addr, MHICTRL,
			MHICTRL_MHISTATE_MASK,
			MHICTRL_MHISTATE_SHIFT, pcie_word_val);
	if (pcie_word_val != MHI_STATE_M3) {
		mhi_log(MHI_MSG_CRITICAL,
			"Failed to read back M3 state. Set %d, Read %d\n",
			MHI_STATE_M3,
			pcie_word_val);
		mhi_dev_ctxt->failed_m3_write++;
	}
	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER, "%s",
			"Waiting for M3 completion.\n");
	ret_val = wait_event_interruptible_timeout(*mhi_dev_ctxt->M3_event,
			mhi_dev_ctxt->mhi_state == MHI_STATE_M3,
		msecs_to_jiffies(MHI_MAX_SUSPEND_TIMEOUT));
	switch(ret_val) {
	case 0:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"MDM failed to suspend after %d ms\n",
			MHI_MAX_SUSPEND_TIMEOUT);
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"STT RP %p WP %p BASE %p Len %ld\n",
			mhi_dev_ctxt->state_change_work_item_list.q_info.rp,
			mhi_dev_ctxt->state_change_work_item_list.q_info.wp,
			mhi_dev_ctxt->state_change_work_item_list.q_info.base,
		mhi_dev_ctxt->state_change_work_item_list.q_info.len);
		mhi_dev_ctxt->m3_event_timeouts++;
		ret_val = -ETIMEDOUT;
		break;
	case -ERESTARTSYS:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER, "%s",
			"Going Down...\n");
		ret_val = -ENETRESET;
		break;
	default:
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER, "%s",
			"M3 completion received\n");
		ret_val = 0;
		break;
	}
	mhi_deassert_device_wake(mhi_dev_ctxt);
	return ret_val;
}

int mhi_init_pm_sysfs(struct device *dev)
{
	return sysfs_create_group(&dev->kobj, &mhi_attribute_group);
}

ssize_t sysfs_init_M3(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int r = 0;
	mhi_device_ctxt *mhi_dev_ctxt = mhi_devices.device_list[0].mhi_ctxt;
	r = mhi_initiate_M3(mhi_dev_ctxt);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to suspend %d\n", r);
		return r;
	}
	if (MHI_STATUS_SUCCESS != mhi_turn_off_pcie_link(mhi_dev_ctxt))
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to turn off link\n");

	return count;
}

ssize_t sysfs_get_mhi_state(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	return 0;
}

ssize_t sysfs_init_M0(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{

	mhi_device_ctxt *mhi_dev_ctxt = mhi_devices.device_list[0].mhi_ctxt;
	if (MHI_STATUS_SUCCESS != mhi_turn_on_pcie_link(mhi_dev_ctxt)) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to resume link\n");
		return count;
	}
	mhi_initiate_M0(mhi_dev_ctxt);
	mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"Current mhi_state = 0x%x\n",
			mhi_dev_ctxt->mhi_state);
	return count;
}
ssize_t sysfs_init_M1(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	return count;
}

MHI_STATUS mhi_turn_off_pcie_link(mhi_device_ctxt *mhi_dev_ctxt)
{
	int r;
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_log(MHI_MSG_INFO, "Entered...\n");
	mutex_lock(&mhi_dev_ctxt->mhi_link_state);
	r = pci_save_state(mhi_dev_ctxt->dev_info->pcie_device);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to save pci device state ret %d\n", r);
		ret_val = MHI_STATUS_ERROR;
		goto exit;
	}
	r = msm_pcie_shadow_control(mhi_dev_ctxt->dev_info->pcie_device, 0);
	if (r)
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"Failed to stop shadow config space: %d\n", r);

	r = pci_set_power_state(mhi_dev_ctxt->dev_info->pcie_device, PCI_D3hot);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to set pcie power state to D3 hotret: %x\n", r);
		ret_val = MHI_STATUS_ERROR;
		goto exit;
	}
	r = msm_pcie_pm_control(MSM_PCIE_SUSPEND,
			mhi_dev_ctxt->dev_info->pcie_device->bus->number,
			mhi_dev_ctxt->dev_info->pcie_device,
			NULL,
			0);
	if (r)
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to suspend pcie bus ret 0x%x\n", r);
	mhi_dev_ctxt->link_up = 0;
exit:
	mutex_unlock(&mhi_dev_ctxt->mhi_link_state);
	mhi_log(MHI_MSG_INFO, "Exited...\n");
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS mhi_turn_on_pcie_link(mhi_device_ctxt *mhi_dev_ctxt)
{
	int r = 0;
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_log(MHI_MSG_INFO, "Entered...\n");
	mutex_lock(&mhi_dev_ctxt->mhi_link_state);
	if (mhi_dev_ctxt->link_up)
		goto exit;
	r = msm_pcie_pm_control(MSM_PCIE_RESUME,
			mhi_dev_ctxt->dev_info->pcie_device->bus->number,
			mhi_dev_ctxt->dev_info->pcie_device,
			NULL, 0);

	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to resume pcie bus ret 0x%x\n", r);
		ret_val = MHI_STATUS_ERROR;
		goto exit;
	}

	r = pci_set_power_state(mhi_dev_ctxt->dev_info->pcie_device,
			PCI_D0);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to set power state 0x%x\n", r);
		ret_val = MHI_STATUS_ERROR;
		goto exit;
	}
	r = msm_pcie_recover_config(mhi_dev_ctxt->dev_info->pcie_device);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to recover configuration space ret 0x%x\n", r);
		goto exit;
	}
	mhi_dev_ctxt->link_up = 1;
exit:
	mutex_unlock(&mhi_dev_ctxt->mhi_link_state);
	mhi_log(MHI_MSG_INFO, "Exited...\n");
	return ret_val;

}

