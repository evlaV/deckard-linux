/* SPDX-License-Identifier: GPL-2.0 */
#ifndef QC_MSM_CAMSS_VFE_880_PP_H
#define QC_MSM_CAMSS_VFE_880_PP_H

#include <linux/types.h>

struct vfe_device;

void vfe_880_setup_pixel_pipe(struct vfe_device *vfe, u32 width, u32 height);
void vfe_880_set_aec_bhist_region(struct vfe_device *vfe, u32 h_offset,
				  u32 v_offset, u32 h_num, u32 v_num);

#endif
