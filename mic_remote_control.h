#ifndef _MOD_AUDIO_PROCESSING_H_
#define _MOD_AUDIO_PROCESSING_H_

void create_mic_selector_thd(void);

void stop_mic_selector_thd(void);
void pause_mic_selector_thd(void);
void resume_mic_selector_thd(void);

#endif /* _MOD_AUDIO_PROCESSING_H_ */
