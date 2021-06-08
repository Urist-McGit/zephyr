#if defined(CONFIG_CAN_SAMV71)
    SECTION_DATA_PROLOGUE(can_msg_sram,(NOLOAD),) {
        *(.can_msg_sram)
        *(".can_msg_sram.*")
        __can_msg_sram_end = .;
    } GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
    ASSERT((__can_msg_sram_end - _image_ram_start) <= 0x10000, "MSG SRAM must be within first 64KB of SRAM!")
#endif
