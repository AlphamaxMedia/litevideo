from litex.gen import *

from litevideo.output.hdmi.encoder import Encoder


# This assumes a 100MHz base clock
class S7HDMIOutClocking(Module):
    def __init__(self, clk100):
        self.clock_domains.cd_pix = ClockDomain("pix")
        self.clock_domains.cd_pix5x = ClockDomain("pix5x", reset_less=True)

        mmcm_locked = Signal()
        mmcm_fb = Signal()

        self.specials += Instance("MMCME2_BASE",
                    p_BANDWIDTH="OPTIMIZED", i_RST=0, o_LOCKED=mmcm_locked,

                    # VCO
                    p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=10.0,
                    p_CLKFBOUT_MULT_F=30.0, p_CLKFBOUT_PHASE=0.000, p_DIVCLK_DIVIDE=4,
                    i_CLKIN1=clk100, i_CLKFBIN=mmcm_fb, o_CLKFBOUT=mmcm_fb,

                    # CLK0
                    p_CLKOUT0_DIVIDE_F=5.0, p_CLKOUT0_PHASE=0.000, o_CLKOUT0=self.cd_pix.clk,
                    # CLK1
                    p_CLKOUT1_DIVIDE=1, p_CLKOUT1_PHASE=0.000, o_CLKOUT1=self.cd_pix5x.clk
        )
        self.comb += self.cd_pix.rst.eq(~mmcm_locked)


class S7HDMIOutEncoderSerializer(Module):
    def __init__(self, pad_p, pad_n, bypass_encoder=False):
        if not bypass_encoder:
            self.submodules.encoder = ClockDomainsRenamer("pix")(Encoder())
            self.d, self.c, self.de = self.encoder.d, self.encoder.c, self.encoder.de
            self.data = self.encoder.out
        else:
            self.data = Signal(10)

        # # #

        rst = ResetSignal("pix")
        ce = Signal()
        self.sync.pix += ce.eq(~rst)

        shift = Signal(2)
        pad_se = Signal()

        # OSERDESE2 master
        self.specials += Instance("OSERDESE2",
            p_DATA_RATE_OQ="DDR",
            p_DATA_RATE_TQ="DDR",
            p_DATA_WIDTH=10,
            p_INIT_OQ=1,
            p_INIT_TQ=1,
            p_SERDES_MODE="MASTER",
            p_SRVAL_OQ=0,
            p_SRVAL_TQ=0,
            p_TBYTE_CTL="FALSE",
            p_TBYTE_SRC="FALSE",
            p_TRISTATE_WIDTH=1,

            o_OQ=pad_se,
            i_CLK=ClockSignal("pix5x"),
            i_CLKDIV=ClockSignal("pix"),
            i_D1=self.data[0],
            i_D2=self.data[1],
            i_D3=self.data[2],
            i_D4=self.data[3],
            i_D5=self.data[4],
            i_D6=self.data[5],
            i_D7=self.data[6],
            i_D8=self.data[7],
            i_OCE=ce,
            i_RST=rst,

            i_SHIFTIN1=shift[0],
            i_SHIFTIN2=shift[1],
            i_T1=0,
            i_T2=0,
            i_T3=0,
            i_T4=0,
            i_TBYTEIN=0,
            i_TCE=0
        )

        # OSERDESE2 slave
        self.specials += Instance("OSERDESE2",
            p_DATA_RATE_OQ="DDR",
            p_DATA_RATE_TQ="DDR",
            p_DATA_WIDTH=10,
            p_INIT_OQ=1,
            p_INIT_TQ=1,
            p_SERDES_MODE="SLAVE",
            p_SRVAL_OQ=0,
            p_SRVAL_TQ=0,
            p_TBYTE_CTL="FALSE",
            p_TBYTE_SRC="FALSE",
            p_TRISTATE_WIDTH=1,

            i_CLK=ClockSignal("pix5x"),
            i_CLKDIV=ClockSignal("pix"),

            o_SHIFTOUT1=shift[0],
            o_SHIFTOUT2=shift[1],

            i_D1=0,
            i_D2=0,
            i_D3=self.data[8],
            i_D4=self.data[9],
            i_D5=0,
            i_D6=0,
            i_D7=0,
            i_D8=0,
            i_OCE=ce,
            i_RST=rst,

            i_SHIFTIN1=0,
            i_SHIFTIN2=0,
            i_T1=0,
            i_T2=0,
            i_T3=0,
            i_T4=0,
            i_TBYTEIN=0,
            i_TCE=0
        )

        self.specials += Instance("OBUFDS",
                p_IOSTANDARD="TDMS_33", p_SLEW="FAST",
                i_I=pad_se, o_O=pad_p, o_OB=pad_n
        )


class S7HDMIOutPHY(Module):
    def __init__(self, pads):
        self.hsync = Signal()
        self.vsync = Signal()
        self.de = Signal()
        self.r = Signal(8)
        self.g = Signal(8)
        self.b = Signal(8)

        # # #

        self.submodules.es0 = S7HDMIOutEncoderSerializer(pads.data0_p, pads.data0_n)
        self.submodules.es1 = S7HDMIOutEncoderSerializer(pads.data1_p, pads.data1_n)
        self.submodules.es2 = S7HDMIOutEncoderSerializer(pads.data2_p, pads.data2_n)
        self.comb += [
            self.es0.d.eq(self.b),
            self.es1.d.eq(self.g),
            self.es2.d.eq(self.r),
            self.es0.c.eq(Cat(self.hsync, self.vsync)),
            self.es1.c.eq(0),
            self.es2.c.eq(0),
            self.es0.de.eq(self.de),
            self.es1.de.eq(self.de),
            self.es2.de.eq(self.de)
        ]