/*
 * An exposure adjustment driver based on Qcom DSPP for OLED devices
 *
 * Copyright (C) 2012-2014, The Linux Foundation. All rights reserved.
 * Copyright (C) Sony Mobile Communications Inc. All rights reserved.
 * Copyright (C) 2014-2018, AngeloGioacchino Del Regno <kholk11@gmail.com>
 * Copyright (C) 2018, Devries <therkduan@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

#include "dsi_display.h"
#include "dsi_panel.h"
#include "../sde/sde_crtc.h"
#include "../sde/sde_plane.h"
#include "exposure_adjustment.h"

static bool pcc_backlight_enable = false;
static u32 last_level = ELVSS_OFF_THRESHOLD;

/*
 * Restoring the backlight and dropping the PCC cannot happen atomically: the
 * backlight level is sent over DSI and only reaches the panel on the next
 * frame, while the PCC is latched on the next atomic commit.  Clearing the PCC
 * first would leave a frame at the threshold backlight with no dimming, which
 * is the bright flash seen when the feature was switched off, so it is cleared
 * once the restored backlight level is on the panel.
 */
#define EA_PCC_OFF_DELAY_MS	40

static void ea_clamp_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(ea_clamp_work, ea_clamp_work_fn);

static void ea_pcc_off_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(ea_pcc_off_work, ea_pcc_off_work_fn);

static int ea_panel_get_pcc(struct drm_crtc **crtc, struct drm_property **prop)
{
	struct dsi_display *display;
	struct msm_drm_private *priv;

	display = get_main_display();
	if (!display || !display->drm_conn || !display->drm_conn->state) {
		pr_err("ERROR: Display is not ready\n");
		return -ENODEV;
	}

	*crtc = display->drm_conn->state->crtc;
	if (!*crtc) {
		pr_err("ERROR: Cannot find display panel with CRTC\n");
		return -ENODEV;
	}

	priv = (*crtc)->dev->dev_private;
	*prop = priv->cp_property[1]; /* SDE_CP_CRTC_DSPP_PCC */
	if (!*prop) {
		pr_err("FAIL! PCC is not supported!!?!?!\n");
		return -EINVAL;
	}

	return 0;
}

static int ea_panel_send_pcc(u32 bl_lvl)
{
	int rc;
	struct drm_crtc *crtc;
	struct drm_msm_pcc pcc_blk = {0};
	struct drm_property *prop;
	struct drm_property_blob *blob;
	u32 ea_coeff;
	uint64_t val;

	rc = ea_panel_get_pcc(&crtc, &prop);
	if (rc)
		return rc;

	rc = sde_cp_crtc_get_property(crtc, prop, &val);
	if (rc) {
		pr_err("Cannot get CRTC property. Things may go wrong.\n");
	}

	pr_debug("%s: Backlight = %d\n", __func__, bl_lvl);

	if (bl_lvl < ELVSS_OFF_THRESHOLD) {
		ea_coeff = bl_lvl * PCC_BACKLIGHT_SCALE + EXPOSURE_ADJUSTMENT_MIN;
	} else {
		ea_coeff = EXPOSURE_ADJUSTMENT_MAX;
	}

	pr_debug("%s: coeff = %d\n", __func__, ea_coeff);

	pcc_blk.r.r = ea_coeff;
	pcc_blk.g.g = ea_coeff;
	pcc_blk.b.b = ea_coeff;

	blob = drm_property_create_blob(crtc->dev, sizeof(pcc_blk), &pcc_blk);
	if (IS_ERR_OR_NULL(blob)) {
		pr_err("Failed to create blob. Bailing out.\n");
		return -EINVAL;
	}
	pr_debug("DSPP Blob ID %d has length %zu\n",
			prop->base.id, blob->length);

	rc = sde_cp_crtc_set_property(crtc, prop, blob->base.id);
	if (rc) {
		pr_err("DSPP: Cannot set PCC: %d.\n", rc);
		return rc;
	}

	return rc;
}

bool ea_panel_is_enabled(void)
{
	return pcc_backlight_enable;
}

void ea_panel_mode_ctrl(struct dsi_panel *panel, bool enable)
{
	bool changed;
	u32 level;

	mutex_lock(&ea_lock);
	changed = (pcc_backlight_enable != enable);
	if (changed) {
		pcc_backlight_enable = enable;
		/*
		 * Do not pin the backlight yet: the PCC is only latched on the
		 * next atomic commit, so the panel would show a bright frame
		 * without any dimming.
		 */
		pcc_clamped = false;
	}
	level = last_level;
	mutex_unlock(&ea_lock);

	if (changed) {
		cancel_delayed_work_sync(&ea_clamp_work);
		cancel_delayed_work_sync(&ea_pcc_off_work);
		pr_debug("Recover backlight level = %d\n", last_level);
		dsi_panel_set_backlight(panel, level);
		if (!enable)
			schedule_delayed_work(&ea_pcc_off_work,
					msecs_to_jiffies(EA_PCC_OFF_DELAY_MS));
	} else if (level == 0 && !pcc_backlight_enable) {
		ea_panel_disable_pcc();
	}
}

static void ea_pcc_off_work_fn(struct work_struct *work)
{
	bool disable;

	mutex_lock(&ea_lock);
	disable = !pcc_backlight_enable && pcc_applied;
	mutex_unlock(&ea_lock);

	if (disable)
		ea_panel_disable_pcc();
}

u32 ea_panel_calc_backlight(u32 bl_lvl)
{
	bool clamped;

	mutex_lock(&ea_lock);
	last_level = bl_lvl;

	if (!pcc_backlight_enable || bl_lvl == 0 ||
			bl_lvl >= ELVSS_OFF_THRESHOLD) {
		bool applied;

		/*
		 * The pin is not needed at this level.  When the brightness
		 * went above the threshold while the feature was on, the
		 * dimming coefficient has to go away as well, otherwise the
		 * panel stays dimmed without anything to compensate for it.
		 * Do not do it when the screen is being turned off: the
		 * threshold level would be visible until the backlight is
		 * actually down.
		 */
		applied = pcc_backlight_enable && pcc_applied &&
				bl_lvl >= ELVSS_OFF_THRESHOLD;
		pcc_clamped = false;
		mutex_unlock(&ea_lock);
		cancel_delayed_work(&ea_clamp_work);

		if (applied)
			ea_panel_disable_pcc();

		return bl_lvl;
	}
	mutex_unlock(&ea_lock);

	if (ea_panel_send_pcc(bl_lvl)) {
		pr_err("ERROR: Failed to send PCC\n");
		/*
		 * Never pin the backlight when the dimming could not be
		 * programmed, otherwise the panel stays at full brightness.
		 */
		mutex_lock(&ea_lock);
		pcc_clamped = false;
		mutex_unlock(&ea_lock);
		return bl_lvl;
	}

	mutex_lock(&ea_lock);
	clamped = pcc_clamped;
	mutex_unlock(&ea_lock);

	if (clamped)
		return ELVSS_OFF_THRESHOLD;

	/*
	 * Keep the real backlight level for a couple of frames, until the PCC
	 * blob queued above has been latched by the display controller.
	 */
	schedule_delayed_work(&ea_clamp_work,
			      msecs_to_jiffies(EA_CLAMP_DELAY_MS));

	return bl_lvl;
}
