module;

#include <cstdint>

import drv8711;

export module drv8711_config;
export namespace drv8711 {

// DRV8711 Register objects with default settings
// you can change these before sending them to
// the driver over SPI

drv8711::CTRL reg_ctrl {
    0x0003, // dtime
    0x0003, // isgain
    0x0000, // exstall
    0x0000, // mode 1 step
    0x0000, // rstep
    0x0000, // rdir
    0x0001  // enbl
};

drv8711::TORQUE reg_torque {
    0x0000, // simplth
    0x00ba  // torque
};

drv8711::OFF reg_off{
    0x0000, // pwmmode
    0x0030  // toff
};

drv8711::BLANK reg_blank {
    0x0001, // abt
    0x0008  // tblank
};

drv8711::DECAY reg_decay {
    0x0003, // decmod
    0x0010  // tdecay
};

drv8711::STALL reg_stall {
    0x0003, // vdiv
    0x0003, // sdcnt
    0x0040  // sdthr
};

drv8711::DRIVE reg_drive {
    0x0000, // idrivep
    0x0000, // idriven
    0x0001, // tdrivep
    0x0001, // tdriven
    0x0001, // ocpdeg
    0x0001  // ocpth
};

drv8711::STATUS reg_status {
    0x0000, // stdlat
    0x0000, // std
    0x0000, // uvlo
    0x0000, // bpdf
    0x0000, // apdf
    0x0000, // bocp
    0x0000, // aocp
    0x0000  // ots
};

} // namespace drv8711