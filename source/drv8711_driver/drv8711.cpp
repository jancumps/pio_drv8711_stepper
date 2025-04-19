module;

#include <cstdint>

export module drv8711;
export namespace drv8711 {

struct CTRL {
                            // address 14-12
    unsigned int dtime;     // 11-10
    unsigned int isgain;    // 9-8
    unsigned int exstall;   // 7
    unsigned int mode;      // 6-3
    unsigned int rstep;     // 2
    unsigned int rdir;      // 1
    unsigned int enbl;      // 0
    inline operator uint16_t() const {
        return (0x0000 << 12) | (dtime << 10) | (isgain << 8) |(exstall << 7) | (mode << 3) | (rstep << 2) | (rdir << 1) | (enbl);
    }
};
 
struct TORQUE {
                            // address 14-12
                            // 11
    unsigned int simplth;   // 10-8
    unsigned int torque;    // 7-0
    inline operator uint16_t() const {
        return (0x0001 << 12) | (simplth << 8) | torque;
    }
};
 
struct OFF {
                            // address 14-12
                            // 11-9
    unsigned int pwmmode;   // 8
    unsigned int toff;      // 7-0
    inline operator uint16_t() const {
        return (0x0002 << 12) | (pwmmode << 8) | toff;
    }
};
 
struct BLANK {
                            // address 14-12
                            // 11-9
    unsigned int abt;       // 8
    unsigned int tblank;    // 7-0
    inline operator uint16_t() const {
        return (0x0003 << 12) | (abt << 8) | tblank;
    }
};
 
struct DECAY {
                            // address 14-12
                            // 11
    unsigned int decmod;    // 10-8
    unsigned int tdecay;    // 7-0
    inline operator uint16_t() const {
        return (0x0004 << 12) | (decmod << 8) | tdecay;
    }
};
 
struct STALL {
                            // address 14-12
    unsigned int vdiv;      // 11-10
    unsigned int sdcnt;     // 9-8
    unsigned int sdthr;     // 7-0
    inline operator uint16_t() const {
        return (0x0005 << 12) | (vdiv << 10) | (sdcnt << 8) | sdthr;
    }
};
 
struct DRIVE {
                            // address 14-12
    unsigned int idrivep;   // 11-10
    unsigned int idriven;   // 9-8
    unsigned int tdrivep;   // 7-6
    unsigned int tdriven;   // 5-4
    unsigned int ocpdeg;    // 3-2
    unsigned int ocpth;     // 1-0
    inline operator uint16_t() const {
        return (0x0006 << 12) | (idrivep << 10) | (idriven << 8) | (tdrivep << 6) | (tdriven << 4) | (ocpdeg << 2) | (ocpth);
    }
};
 
struct STATUS {
                            // address 14-12
                            // 11-8
    unsigned int stdlat;    // 7
    unsigned int std;       // 6
    unsigned int uvlo;      // 5
    unsigned int bpdf;      // 4
    unsigned int apdf;      // 3
    unsigned int bocp;      // 2
    unsigned int aocp;      // 1
    unsigned int ots;       // 0
    inline operator uint16_t() const {
        return (0x0007 << 12) | (stdlat << 7) | (std << 6) | (uvlo << 5) | (bpdf << 4) | (apdf << 3) | (bocp << 2) | (aocp << 1) | (ots);
    }
};

class driver {
public:
    driver() {}
    virtual void write(uint16_t data) = 0;
    virtual void enable(bool enable) = 0;
};

class wakeup { // driver out of sleep as long as object in scope
public:    
    wakeup(driver& driver) : driver_(driver) { driver.enable(true); }
    ~wakeup() { driver_.enable(false); }
private:
    driver& driver_;
};


} // namespace drv8711