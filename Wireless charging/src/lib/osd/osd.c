/*
 * 	miaofng@2010 initial version
 *		- stores all osd_xx_t object in flash
 *		- setup a fast link in memory for all dialog, all group and runtime items inside that group
 *		- do not construct several dialog at the same time unless they takes up different osd region
 */

#include "config.h"
#include "osd/osd_event.h"
#include "osd/osd_cmd.h"
#include "osd/osd_dialog.h"
#include "osd/osd_eng.h"
#include "ulp_time.h"
#include "sys/task.h"

static time_t osd_update_always_timer;
static time_t osd_update_focus_timer;

#define OSD_UPDATE_ALWAYS_MS	200
#define OSD_UPDATE_FOCUS_MS	50

void osd_Init(void)
{
	osd_eng_init();
	osd_event_init();
	osd_SetActiveDialog(NULL);
}

void osd_Update(void)
{
	osd_dialog_k *kdlg;
	osd_group_k *kgrp;
	int event, ret = -1;
	
	kdlg = (osd_dialog_k *) osd_GetActiveDialog();
	if(kdlg == NULL)
		return;
	
	event = osd_event_get();
	kgrp = kdlg->active_kgrp;
	
	if(event != KEY_NONE) {
		if(kgrp != NULL)
			ret = osd_HandleCommand(event, kgrp->grp->cmds);
		if(ret)
			osd_HandleCommand(event, kdlg->dlg->cmds);
		if(!ret)
			osd_ShowDialog(kdlg, ITEM_UPDATE_AFTERCOMMAND);
	}
	
	//update always
	if(time_left(osd_update_always_timer) < 0) {
		osd_update_always_timer = time_get(OSD_UPDATE_ALWAYS_MS);
		osd_ShowDialog(kdlg, ITEM_UPDATE_ALWAYS);
	}

	//update focus
	if(time_left(osd_update_focus_timer) < 0) {
		osd_update_focus_timer = time_get(OSD_UPDATE_FOCUS_MS);
			if(!osd_eng_is_visible(&kgrp->margin)) { //need scroll now
				osd_HideDialog(kdlg);
				osd_eng_scroll(kgrp->margin.x1, kgrp->margin.y1);
				osd_ShowDialog(kdlg, ITEM_UPDATE_INIT);
			}
			else
				osd_ShowDialog(kdlg, ITEM_UPDATE_AFTERFOCUSCHANGE);
	}
}

DECLARE_LIB(osd_Init, osd_Update)
