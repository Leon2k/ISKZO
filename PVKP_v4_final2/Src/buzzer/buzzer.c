#include "buzzer.h"
#include "tim.h"

void BuzzerSetFreq(uint16_t freq) {
	htim2.Instance->PSC = (SystemCoreClock / (2 * BUZZER_VOLUME_MAX * freq)) - 1;
}

void BuzzerSetVolume(uint16_t volume) {
	if (volume > BUZZER_VOLUME_MAX) volume = BUZZER_VOLUME_MAX;
	htim2.Instance->CCR4 = volume;
}

void PlayMusic (void) {
	uint32_t HappyBirthday[] = {
		262, 262, 294, 262, 349, 330, 262,
		262, 294, 262, 392, 349, 262, 262,
		523, 440, 349, 330, 294, 466, 466,
		440, 349, 392, 349};

	for(int i = 0; i < sizeof(HappyBirthday) / sizeof(uint32_t); i++) {
		BuzzerSetFreq(HappyBirthday[i]);
		BuzzerSetVolume(BUZZER_VOLUME_MAX);
		HAL_Delay(400);
		//DelayTime(400);
		BuzzerSetVolume(BUZZER_VOLUME_MUTE);
	}
}
