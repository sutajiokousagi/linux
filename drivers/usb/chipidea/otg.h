#ifndef __DRIVERS_USB_CHIPIDEA_OTG_H
#define __DRIVERS_USB_CHIPIDEA_OTG_H

int ci_hdrc_otg_init(struct ci_hdrc *ci);
void ci_clear_otg_interrupt(struct ci_hdrc *ci, u32 bits);
void ci_enable_otg_interrupt(struct ci_hdrc *ci, u32 bits);
void ci_disable_otg_interrupt(struct ci_hdrc *ci, u32 bits);

#endif /* __DRIVERS_USB_CHIPIDEA_OTG_H */
