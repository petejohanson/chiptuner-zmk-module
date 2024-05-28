if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
	target_sources_ifdef(CONFIG_CHIPTUNER_BEHAVIOR_TONE app PRIVATE behaviors/behavior_tone.c)
endif()
