from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer, BusSynchronizer
from migen.genlib.misc import WaitTimer
from migen.genlib.cdc import MultiReg, Gearbox

from litex.soc.interconnect.csr import *


class S6DataCapture(Module, AutoCSR):
    def __init__(self, pad_p, pad_n, ntbits=8):
        self.serdesstrobe = Signal()
        self.d = Signal(10)

        self._dly_ctl = CSR(6)
        self._dly_busy = CSRStatus(2)
        self._phase = CSRStatus(2)
        self._phase_reset = CSR()

        # # #

        # IO
        pad_se = Signal()
        self.specials += Instance("IBUFDS",
                                  i_I=pad_p, i_IB=pad_n,
                                  o_O=pad_se)

        pad_delayed_master = Signal()
        pad_delayed_slave = Signal()
        delay_inc = Signal()
        delay_ce = Signal()
        delay_master_cal = Signal()
        delay_master_rst = Signal()
        delay_master_busy = Signal()
        delay_slave_cal = Signal()
        delay_slave_rst = Signal()
        delay_slave_busy = Signal()
        self.specials += [
            Instance("IODELAY2",
                p_SERDES_MODE="MASTER",
                p_DELAY_SRC="IDATAIN", p_IDELAY_TYPE="DIFF_PHASE_DETECTOR",
                p_COUNTER_WRAPAROUND="STAY_AT_LIMIT", p_DATA_RATE="SDR",

                i_IDATAIN=pad_se, o_DATAOUT=pad_delayed_master,
                i_CLK=ClockSignal("pix2x"), i_IOCLK0=ClockSignal("pix10x"),

                i_INC=delay_inc, i_CE=delay_ce,
                i_CAL=delay_master_cal, i_RST=delay_master_rst, o_BUSY=delay_master_busy,
                i_T=1),
            Instance("IODELAY2",
                p_SERDES_MODE="SLAVE",
                p_DELAY_SRC="IDATAIN", p_IDELAY_TYPE="DIFF_PHASE_DETECTOR",
                p_COUNTER_WRAPAROUND="WRAPAROUND", p_DATA_RATE="SDR",

                i_IDATAIN=pad_se, o_DATAOUT=pad_delayed_slave,
                i_CLK=ClockSignal("pix2x"), i_IOCLK0=ClockSignal("pix10x"),

                i_INC=delay_inc, i_CE=delay_ce,
                i_CAL=delay_slave_cal, i_RST=delay_slave_rst, o_BUSY=delay_slave_busy,
                i_T=1)
        ]

        dsr2 = Signal(5)
        pd_valid = Signal()
        pd_incdec = Signal()
        pd_edge = Signal()
        pd_cascade = Signal()
        self.specials += [
            Instance("ISERDES2",
                p_SERDES_MODE="MASTER",
                p_BITSLIP_ENABLE="FALSE", p_DATA_RATE="SDR", p_DATA_WIDTH=5,
                p_INTERFACE_TYPE="RETIMED",

                i_D=pad_delayed_master,
                o_Q4=dsr2[4], o_Q3=dsr2[3], o_Q2=dsr2[2], o_Q1=dsr2[1],

                i_BITSLIP=0, i_CE0=1, i_RST=0,
                i_CLK0=ClockSignal("pix10x"), i_CLKDIV=ClockSignal("pix2x"),
                i_IOCE=self.serdesstrobe,

                o_VALID=pd_valid, o_INCDEC=pd_incdec,
                i_SHIFTIN=pd_edge, o_SHIFTOUT=pd_cascade),
            Instance("ISERDES2",
                p_SERDES_MODE="SLAVE",
                p_BITSLIP_ENABLE="FALSE", p_DATA_RATE="SDR", p_DATA_WIDTH=5,
                p_INTERFACE_TYPE="RETIMED",

                i_D=pad_delayed_slave,
                o_Q4=dsr2[0],

                i_BITSLIP=0, i_CE0=1, i_RST=0,
                i_CLK0=ClockSignal("pix10x"), i_CLKDIV=ClockSignal("pix2x"),
                i_IOCE=self.serdesstrobe,

                i_SHIFTIN=pd_cascade, o_SHIFTOUT=pd_edge)
        ]

        # Phase error accumulator
        lateness = Signal(ntbits, reset=2**(ntbits - 1))
        too_late = Signal()
        too_early = Signal()
        reset_lateness = Signal()
        self.comb += [
            too_late.eq(lateness == (2**ntbits - 1)),
            too_early.eq(lateness == 0)
        ]
        self.sync.pix2x += [
            If(reset_lateness,
                lateness.eq(2**(ntbits - 1))
            ).Elif(~delay_master_busy & ~delay_slave_busy & ~too_late & ~too_early,
                If(pd_valid & pd_incdec, lateness.eq(lateness - 1)),
                If(pd_valid & ~pd_incdec, lateness.eq(lateness + 1))
            )
        ]

        # Delay control
        self.submodules.delay_master_done = PulseSynchronizer("pix2x", "sys")
        delay_master_pending = Signal()
        self.sync.pix2x += [
            self.delay_master_done.i.eq(0),
            If(~delay_master_pending,
                If(delay_master_cal | delay_ce, delay_master_pending.eq(1))
            ).Else(
                If(~delay_master_busy,
                    self.delay_master_done.i.eq(1),
                    delay_master_pending.eq(0)
                )
            )
        ]
        self.submodules.delay_slave_done = PulseSynchronizer("pix2x", "sys")
        delay_slave_pending = Signal()
        self.sync.pix2x += [
            self.delay_slave_done.i.eq(0),
            If(~delay_slave_pending,
                If(delay_slave_cal | delay_ce, delay_slave_pending.eq(1))
            ).Else(
                If(~delay_slave_busy,
                    self.delay_slave_done.i.eq(1),
                    delay_slave_pending.eq(0)
                )
            )
        ]

        self.submodules.do_delay_master_cal = PulseSynchronizer("sys", "pix2x")
        self.submodules.do_delay_master_rst = PulseSynchronizer("sys", "pix2x")
        self.submodules.do_delay_slave_cal = PulseSynchronizer("sys", "pix2x")
        self.submodules.do_delay_slave_rst = PulseSynchronizer("sys", "pix2x")
        self.submodules.do_delay_inc = PulseSynchronizer("sys", "pix2x")
        self.submodules.do_delay_dec = PulseSynchronizer("sys", "pix2x")
        self.comb += [
            delay_master_cal.eq(self.do_delay_master_cal.o),
            delay_master_rst.eq(self.do_delay_master_rst.o),
            delay_slave_cal.eq(self.do_delay_slave_cal.o),
            delay_slave_rst.eq(self.do_delay_slave_rst.o),
            delay_inc.eq(self.do_delay_inc.o),
            delay_ce.eq(self.do_delay_inc.o | self.do_delay_dec.o),
        ]

        sys_delay_master_pending = Signal()
        self.sync += [
            If(self.do_delay_master_cal.i |
               self.do_delay_inc.i |
               self.do_delay_dec.i,
                sys_delay_master_pending.eq(1)
            ).Elif(self.delay_master_done.o,
                sys_delay_master_pending.eq(0)
            )
        ]
        sys_delay_slave_pending = Signal()
        self.sync += [
            If(self.do_delay_slave_cal.i |
               self.do_delay_inc.i |
               self.do_delay_dec.i,
                sys_delay_slave_pending.eq(1)
            ).Elif(self.delay_slave_done.o,
                sys_delay_slave_pending.eq(0)
            )
        ]

        self.comb += [
            self.do_delay_master_cal.i.eq(self._dly_ctl.re & self._dly_ctl.r[0]),
            self.do_delay_master_rst.i.eq(self._dly_ctl.re & self._dly_ctl.r[1]),
            self.do_delay_slave_cal.i.eq(self._dly_ctl.re & self._dly_ctl.r[2]),
            self.do_delay_slave_rst.i.eq(self._dly_ctl.re & self._dly_ctl.r[3]),
            self.do_delay_inc.i.eq(self._dly_ctl.re & self._dly_ctl.r[4]),
            self.do_delay_dec.i.eq(self._dly_ctl.re & self._dly_ctl.r[5]),
            self._dly_busy.status.eq(Cat(sys_delay_master_pending,
                                         sys_delay_slave_pending))
        ]

        # Phase detector control
        self.specials += MultiReg(Cat(too_late, too_early), self._phase.status)
        self.submodules.do_reset_lateness = PulseSynchronizer("sys", "pix2x")
        self.comb += [
            reset_lateness.eq(self.do_reset_lateness.o),
            self.do_reset_lateness.i.eq(self._phase_reset.re)
        ]

        # 5:10 deserialization
        dsr = Signal(10)
        self.sync.pix2x += dsr.eq(Cat(dsr[5:], dsr2))
        if hasattr(pad_p, "inverted"):
            self.sync.pix += self.d.eq(~dsr)
        else:
            self.sync.pix += self.d.eq(dsr)


class S7PhaseDetector(Module, AutoCSR):
    def __init__(self):
        self.mdata = Signal(8)
        self.sdata = Signal(8)

        self.inc = Signal()
        self.dec = Signal()
        self.unsure = Signal()

        # # #

        # ideal sampling (middle of the eye):
        #  _____       _____       _____
        # |     |_____|     |_____|     |_____|   data
        #    +     +     +     +     +     +      master sampling
        #       -     -     -     -     -     -   slave sampling (90°/bit period)
        # Since taps are fixed length delays, this ideal case is not possible
        # and we will fall in the 2 following possible cases:
        #
        # 1) too late sampling (idelay needs to be decremented):
        #  _____       _____       _____
        # |     |_____|     |_____|     |_____|   data
        #     +     +     +     +     +     +     master sampling
        #        -     -     -     -     -     -  slave sampling (90°/bit period)
        # on mdata transition, mdata != sdata
        #
        #
        # 2) too early sampling (idelay needs to be incremented):
        #  _____       _____       _____
        # |     |_____|     |_____|     |_____|   data
        #   +     +     +     +     +     +       master sampling
        #      -     -     -     -     -     -    slave sampling (90°/bit period)
        # on mdata transition, mdata == sdata

        transition = Signal()

        # find transition
        mdata_d = Signal(8)
        self.sync.pix1p25x_r += mdata_d.eq(self.mdata)
        self.comb += transition.eq(mdata_d != self.mdata)


        # find what to do
        self.comb += [
            self.inc.eq(transition & (self.mdata == self.sdata)),
            self.dec.eq(transition & (mdata_d == self.sdata)),
            self.unsure.eq(transition & (self.mdata != self.sdata)),
        ]


class S7DataCapture(Module, AutoCSR):
    def __init__(self, pad_p, pad_n, ntbits=8, iodelay_clk_freq=200e6, alt_delay=False):
        self.d = Signal(10)

        self._dly_ctl = CSR(5)
        self._phase = CSRStatus(2)
        self._phase_reset = CSR()
        self._cntvalueout_m = CSRStatus(5)
        self._cntvalueout_s = CSRStatus(5)
        self._lateness = CSRStatus(ntbits)
        self._algorithm = CSRStorage(2)
        self._eye = CSRStatus(32)
        self._monitor = CSRStatus(32)
        self._auto_ctl = CSRStorage(5)

        # # #

        # use 2 serdes for phase detection: master & slave
        serdes_m_i_nodelay = Signal()
        serdes_s_i_nodelay = Signal()
        self.specials += [
            Instance("IBUFDS_DIFF_OUT",
                i_I=pad_p,
                i_IB=pad_n,
                o_O=serdes_m_i_nodelay,
                o_OB=serdes_s_i_nodelay,
            )
        ]

        # auto_ctl:
        #  0 = enable_phase_detector
        #  1 = enable_monitor
        #  2 = delay_mech select
        #  3 = enable bitslip controller
        #  4 = search again
        auto_ctl = Signal(self._auto_ctl.size)
        self.submodules.sync_auto_ctl = BusSynchronizer(self._auto_ctl.size, "sys", "pix1p25x_r")
        self.comb += [
            self.sync_auto_ctl.i.eq(self._auto_ctl.storage),
            auto_ctl.eq(self.sync_auto_ctl.o)
        ]

        # algorithm:
        # 0 = original
        # 1 = delay criteria mod on original
        # 2 = fully autonomous
        algo = Signal(2)
        self.specials += MultiReg(self._algorithm.storage, algo, odomain="pix1p25x_r")

        bitslip = Signal()
        alg_bitslip=Signal()

        delay_rst = Signal()
        delay_master_inc = Signal()
        delay_master_ce = Signal()
        delay_slave_inc = Signal()
        delay_slave_ce = Signal()

        # master serdes
        serdes_m_i_delayed = Signal()
        serdes_m_q = Signal(8)
        serdes_m_d = Signal(8)
        serdes_m_cntvalue = Signal(5)
        serdes_m_cntvalue_in = Signal(5)
        alg_serdes_m_cntvalue_in = Signal(5)

        alg_delay_rst = Signal()
        alg_delay_master_ce = Signal()
        alg_delay_master_inc = Signal()
        self.comb += [
            If(algo[1],
               alg_delay_rst.eq(1),
               alg_delay_master_ce.eq(0),
               alg_delay_master_inc.eq(0),
               alg_serdes_m_cntvalue_in.eq(serdes_m_cntvalue_in),
               If(auto_ctl[3],
                  alg_bitslip.eq(bitslip),
                ).Else(
                  alg_bitslip.eq(0)
               )
            ).Else(
                alg_delay_rst.eq(delay_rst),
                alg_delay_master_ce.eq(delay_master_ce),
                alg_delay_master_inc.eq(delay_master_inc),
                alg_serdes_m_cntvalue_in.eq(0),
                alg_bitslip.eq(0)
            )
        ]

        self.specials += [
            Instance("IDELAYE2",
                     p_DELAY_SRC="IDATAIN", p_SIGNAL_PATTERN="DATA",
                     p_CINVCTRL_SEL="FALSE", p_HIGH_PERFORMANCE_MODE="TRUE",
                     p_REFCLK_FREQUENCY=iodelay_clk_freq / 1e6, p_PIPE_SEL="FALSE",
                     p_IDELAY_TYPE="VAR_LOAD", p_IDELAY_VALUE=0,

                     i_C=ClockSignal("pix1p25x_r"),
                     i_LD=alg_delay_rst,
                     i_CE=alg_delay_master_ce,
                     i_LDPIPEEN=0,
                     i_INC=alg_delay_master_inc,

                     i_CNTVALUEIN=alg_serdes_m_cntvalue_in,
                     i_IDATAIN=serdes_m_i_nodelay, o_DATAOUT=serdes_m_i_delayed,
                     o_CNTVALUEOUT=serdes_m_cntvalue
                     ),
            Instance("ISERDESE2",
                     p_DATA_WIDTH=8, p_DATA_RATE="DDR",
                     p_SERDES_MODE="MASTER", p_INTERFACE_TYPE="NETWORKING",
                     p_NUM_CE=1, p_IOBDELAY="IFD",

                     i_DDLY=serdes_m_i_delayed,
                     i_CE1=1,
                     i_RST=ResetSignal("pix1p25x_r"),
                     i_CLK=ClockSignal("pix5x"), i_CLKB=~ClockSignal("pix5x"),
                     i_CLKDIV=ClockSignal("pix1p25x_r"),
                     i_BITSLIP= alg_bitslip,
                     o_Q8=serdes_m_q[0], o_Q7=serdes_m_q[1],
                     o_Q6=serdes_m_q[2], o_Q5=serdes_m_q[3],
                     o_Q4=serdes_m_q[4], o_Q3=serdes_m_q[5],
                     o_Q2=serdes_m_q[6], o_Q1=serdes_m_q[7]
                     ),
        ]

        # slave serdes
        # idelay_value must be preloaded with a 90° phase shift but we
        # do it dynamically by software to support all resolutions
        serdes_s_i_delayed = Signal()
        serdes_s_q = Signal(8)
        serdes_s_d = Signal(8)
        serdes_s_cntvalue = Signal(5)
        serdes_s_cntvalue_in = Signal(5)
        alg_serdes_s_cntvalue_in = Signal(5)

        alg_delay_slave_ce = Signal()
        alg_delay_slave_inc = Signal()
        self.comb += [
            If(algo[1],
               alg_delay_slave_ce.eq(0),
               alg_delay_slave_inc.eq(0),
               alg_serdes_s_cntvalue_in.eq(serdes_s_cntvalue_in)
            ).Else(
                alg_delay_slave_ce.eq(delay_slave_ce),
                alg_delay_slave_inc.eq(delay_slave_inc),
                alg_serdes_s_cntvalue_in.eq(0)
            )
        ]
        self.specials += [
            Instance("IDELAYE2",
                     p_DELAY_SRC="IDATAIN", p_SIGNAL_PATTERN="DATA",
                     p_CINVCTRL_SEL="FALSE", p_HIGH_PERFORMANCE_MODE="TRUE",
                     p_REFCLK_FREQUENCY=iodelay_clk_freq / 1e6, p_PIPE_SEL="FALSE",
                     p_IDELAY_TYPE="VAR_LOAD", p_IDELAY_VALUE=0,

                     i_C=ClockSignal("pix1p25x_r"),
                     i_LD=alg_delay_rst,
                     i_CE=alg_delay_slave_ce,
                     i_LDPIPEEN=0, i_INC=alg_delay_slave_inc,

                     i_CNTVALUEIN=alg_serdes_s_cntvalue_in,
                     i_IDATAIN=~serdes_s_i_nodelay, o_DATAOUT=serdes_s_i_delayed,
                     o_CNTVALUEOUT=serdes_s_cntvalue
                     ),
            Instance("ISERDESE2",
                     p_DATA_WIDTH=8, p_DATA_RATE="DDR",
                     p_SERDES_MODE="MASTER", p_INTERFACE_TYPE="NETWORKING",
                     p_NUM_CE=1, p_IOBDELAY="IFD",

                     i_DDLY=serdes_s_i_delayed,
                     i_CE1=1,
                     i_RST=ResetSignal("pix1p25x_r"),
                     i_CLK=ClockSignal("pix5x"), i_CLKB=~ClockSignal("pix5x"),
                     i_CLKDIV=ClockSignal("pix1p25x_r"),
                     i_BITSLIP=alg_bitslip,
                     o_Q8=serdes_s_q[0], o_Q7=serdes_s_q[1],
                     o_Q6=serdes_s_q[2], o_Q5=serdes_s_q[3],
                     o_Q4=serdes_s_q[4], o_Q3=serdes_s_q[5],
                     o_Q2=serdes_s_q[6], o_Q1=serdes_s_q[7]
                     ),
        ]


        alt_delay_data_out=Signal(8)
        alt_delay_data_out_inv = Signal(8)
        # polarity
        if hasattr(pad_p, "inverted"):
            self.comb += [
                serdes_m_d.eq(~serdes_m_q),
                serdes_s_d.eq(~serdes_s_q),
                alt_delay_data_out_inv.eq(~alt_delay_data_out)
            ]
        else:
            self.comb += [
                serdes_m_d.eq(serdes_m_q),
                serdes_s_d.eq(serdes_s_q),
                alt_delay_data_out_inv.eq(alt_delay_data_out)
            ]

        # datapath
        self.submodules.gearbox = Gearbox(8, "pix1p25x", 10, "pix")
        self.comb += [
            If(algo[1],
               self.gearbox.i.eq(alt_delay_data_out_inv),
            ).Else(
               self.gearbox.i.eq(serdes_m_d),
            ),
            self.d.eq(self.gearbox.o)
        ]


        self._eye_bit_time=CSRStorage(5)
        self.submodules.sync_eye_bit_time = BusSynchronizer(5, "sys", "pix1p25x_r")
        eye_bit_time_sync=Signal(5)
        self.comb += [
            self.sync_eye_bit_time.i.eq(self._eye_bit_time.storage),
            eye_bit_time_sync.eq(self.sync_eye_bit_time.o)
        ]
        if alt_delay:
            eye_result=Signal(32)
            delay_result=Signal(32)
            self.specials += [
                Instance("delay_controller",
                         # p_S = 8,
                         i_m_datain=serdes_m_q,
                         i_s_datain=serdes_s_q,
                         i_enable_phase_detector=auto_ctl[0],
                         i_enable_monitor=auto_ctl[1],
                         i_del_mech=auto_ctl[2],
                         i_reset=ResetSignal("pix1p25x_r"),
                         i_clk=ClockSignal("pix1p25x_r"),
                         o_m_delay_out=serdes_m_cntvalue_in,
                         o_s_delay_out=serdes_s_cntvalue_in,
                         o_data_out=alt_delay_data_out,
                         i_bt_val=eye_bit_time_sync,
                         o_results=eye_result,
                         o_m_delay_1hot=delay_result,
                         )
            ]
            self.submodules.sync_eye = BusSynchronizer(32, "pix1p25x_r", "sys")
            self.submodules.sync_result = BusSynchronizer(32, "pix1p25x_r", "sys")
            self.comb += [
                self.sync_eye.i.eq(eye_result),
                self._eye.status.eq(self.sync_eye.o),
                self.sync_result.i.eq(delay_result),
                self._monitor.status.eq(self.sync_result.o),
            ]

            self.submodules.do_search_again = PulseSynchronizer("sys", "pix")
            self.comb += [
                self.do_search_again.i.eq(self._auto_ctl.re & self._auto_ctl.storage[4])
            ]
            raw_bitslip = Signal()
            self.specials += [
                Instance("phsaligner",
                         i_clk=ClockSignal("pix"),
                         i_rst=ResetSignal("pix"),
                         i_sdata=self.gearbox.o,
                         o_bitslip=raw_bitslip,
                         i_search_again=self.do_search_again.o
                         ),
            ]
            self.submodules.bitslip_sync = PulseSynchronizer("pix", "pix1p25x_r") #### this could be a problem, need to test. Inserted to fix a timing closure issue but the extra latency could be a problem.
            self.comb += [
                self.bitslip_sync.i.eq(raw_bitslip),
                bitslip.eq(self.bitslip_sync.o),
            ]


        # cntvalue sync
        self.submodules.sync_mcntvalue = BusSynchronizer(5, "pix1p25x_r", "sys")
        self.submodules.sync_scntvalue = BusSynchronizer(5, "pix1p25x_r", "sys")
        self.comb += [
            self.sync_mcntvalue.i.eq(serdes_m_cntvalue),
            self._cntvalueout_m.status.eq(self.sync_mcntvalue.o),
            self.sync_scntvalue.i.eq(serdes_s_cntvalue),
            self._cntvalueout_s.status.eq(self.sync_scntvalue.o),
        ]

        # phase detector
        self.submodules.phase_detector = ClockDomainsRenamer("pix1p25x_r")(
            S7PhaseDetector())
        self.comb += [
            self.phase_detector.mdata.eq(serdes_m_d),
            self.phase_detector.sdata.eq(serdes_s_d)
        ]

        # phase error accumulator
        lateness = Signal(ntbits, reset=2**(ntbits - 1))
        too_late = Signal()
        too_early = Signal()
        reset_lateness = Signal()
        self.comb += [
            too_late.eq(lateness == (2**ntbits - 1)),
            too_early.eq(lateness == 0)
        ]
        self.sync.pix1p25x_r += [
            If(reset_lateness,
                lateness.eq(2**(ntbits - 1))
            ).Elif(~too_late & ~too_early,
                If(algo[0],
                   If(self.phase_detector.dec, lateness.eq(lateness + 1)),
                   If(self.phase_detector.inc, lateness.eq(lateness - 1))
                ).Else(
                   If(self.phase_detector.unsure, lateness.eq(lateness + 1)),
                   If(self.phase_detector.inc, lateness.eq(lateness - 1))
                )
            )
        ]

        self.submodules.sync_lateness = BusSynchronizer(ntbits, "pix1p25x_r", "sys")
        self.comb += [
            self.sync_lateness.i.eq(lateness),
            self._lateness.status.eq(self.sync_lateness.o),
        ]

        # delay control
        self.submodules.do_delay_rst = PulseSynchronizer("sys", "pix1p25x_r")
        self.submodules.do_delay_master_inc = PulseSynchronizer("sys", "pix1p25x_r")
        self.submodules.do_delay_master_dec = PulseSynchronizer("sys", "pix1p25x_r")
        self.submodules.do_delay_slave_inc = PulseSynchronizer("sys", "pix1p25x_r")
        self.submodules.do_delay_slave_dec = PulseSynchronizer("sys", "pix1p25x_r")
        self.comb += [
            delay_rst.eq(self.do_delay_rst.o),
            delay_master_inc.eq(self.do_delay_master_inc.o),
            delay_master_ce.eq(self.do_delay_master_inc.o | self.do_delay_master_dec.o),
            delay_slave_inc.eq(self.do_delay_slave_inc.o),
            delay_slave_ce.eq(self.do_delay_slave_inc.o | self.do_delay_slave_dec.o)
        ]

        self.comb += [
            self.do_delay_rst.i.eq(self._dly_ctl.re & self._dly_ctl.r[0]),
            self.do_delay_master_inc.i.eq(self._dly_ctl.re & self._dly_ctl.r[1]),
            self.do_delay_master_dec.i.eq(self._dly_ctl.re & self._dly_ctl.r[2]),
            self.do_delay_slave_inc.i.eq(self._dly_ctl.re & self._dly_ctl.r[3]),
            self.do_delay_slave_dec.i.eq(self._dly_ctl.re & self._dly_ctl.r[4])
        ]

        # phase detector control
        self.specials += MultiReg(Cat(too_late, too_early, self.phase_detector.unsure), self._phase.status)
        self.submodules.do_reset_lateness = PulseSynchronizer("sys", "pix1p25x_r")
        self.comb += [
            reset_lateness.eq(self.do_reset_lateness.o),
            self.do_reset_lateness.i.eq(self._phase_reset.re)
        ]
