from migen import *

from litex.soc.interconnect.csr import AutoCSR

from litevideo.input.edid import EDID, _default_edid
from litevideo.input.clocking import S6Clocking, S7Clocking
from litevideo.input.datacapture import S6DataCapture, S7DataCapture
from litevideo.input.charsync import CharSync
from litevideo.input.wer import WER
from litevideo.input.decoding import Decoding, DecodeTERC4
from litevideo.input.chansync import ChanSync
from litevideo.input.analysis import SyncPolarity, ResolutionDetection
from litevideo.input.analysis import FrameExtraction
from litevideo.input.dma import DMA

from litex.soc.interconnect import stream
from litevideo.input.common import channel_layout
from litevideo.output.common import list_signals

clocking_cls = {
    "xc6" : S6Clocking,
    "xc7" : S7Clocking,
}

datacapture_cls = {
    "xc6" : S6DataCapture,
    "xc7" : S7DataCapture
}

class TimingDelayChannel(Module):
    def __init__(self, latency):
        self.sink = Record(channel_layout) # inputs
        self.source = Record(channel_layout) # outputs

        # # #

        for name in list_signals(channel_layout):
            s = getattr(self.sink, name)
            for i in range(latency):
                next_s = Signal(len(s))  # without len(s), this makes only one-bit wide delay lines
                self.sync.pix += next_s.eq(s)
                s = next_s
            self.comb += getattr(self.source, name).eq(s)


class HDMIIn(Module, AutoCSR):
    def __init__(self, pads, dram_port=None, n_dma_slots=2, fifo_depth=512, device="xc6",
                 default_edid=_default_edid, clkin_freq=148.5e6, split_mmcm=False, mode="ycbcr422",
                 hdmi=False, iodelay_clk_freq=200e6, alt_delay=False):
        if hasattr(pads, "scl"):
            self.submodules.edid = EDID(pads, default_edid)
        self.submodules.clocking = clocking_cls[device](pads, clkin_freq, split_mmcm)

        for datan in range(3):
            name = "data" + str(datan)

            cap = datacapture_cls[device](getattr(pads, name + "_p"),
                                          getattr(pads, name + "_n"), iodelay_clk_freq=iodelay_clk_freq, alt_delay=alt_delay)
            setattr(self.submodules, name + "_cap", cap)
            if hasattr(cap, "serdesstrobe"):
                self.comb += cap.serdesstrobe.eq(self.clocking.serdesstrobe)

            charsync = CharSync()
            setattr(self.submodules, name + "_charsync", charsync)
            self.comb += charsync.raw_data.eq(cap.d)

            auto_mode = Signal()
            self.sync.pix += auto_mode.eq(cap.auto_ctl[5])

            wer = WER()
            setattr(self.submodules, name + "_wer", wer)
            self.comb += [
               If(auto_mode,
                  wer.data.eq(cap.d)
               ).Else(
                   wer.data.eq(charsync.data)
               )
            ]

            decoding = Decoding()
            setattr(self.submodules, name + "_decod", decoding)
            self.comb += [
                If(auto_mode,
                   decoding.valid_i.eq(cap.phsaligned),
                   decoding.input.eq(cap.d)
                ).Else(
                  decoding.valid_i.eq(charsync.synced),
                  decoding.input.eq(charsync.data)
                )
            ]


        data0_bonded = Signal(10)
        data1_bonded = Signal(10)
        data2_bonded = Signal(10)
        data0_iamrdy = Signal()
        data1_iamrdy = Signal()
        data2_iamrdy = Signal()
        all_rdy = Signal()
        self.use_alt_bond = use_alt_bond = Signal()
        self.comb += use_alt_bond.eq(self.data0_cap.auto_ctl[6])
        if alt_delay:
            self.specials += [
                Instance("chnlbond",
                     i_clk=ClockSignal("pix"),
                     i_rawdata=self.data0_cap.d,
                     i_iamvld=self.data0_cap.phsaligned,
                     i_other_ch0_vld=self.data1_cap.phsaligned,
                     i_other_ch0_rdy=data1_iamrdy,
                     i_other_ch1_vld=self.data2_cap.phsaligned,
                     i_other_ch1_rdy=data2_iamrdy,
                     o_iamrdy=data0_iamrdy,
                     o_sdata=data0_bonded,
                     ),
                Instance("chnlbond",
                     i_clk=ClockSignal("pix"),
                     i_rawdata=self.data1_cap.d,
                     i_iamvld=self.data1_cap.phsaligned,
                     i_other_ch0_vld=self.data0_cap.phsaligned,
                     i_other_ch0_rdy=data0_iamrdy,
                     i_other_ch1_vld=self.data2_cap.phsaligned,
                     i_other_ch1_rdy=data2_iamrdy,
                     o_iamrdy=data1_iamrdy,
                     o_sdata=data1_bonded,
                     ),
                Instance("chnlbond",
                     i_clk=ClockSignal("pix"),
                     i_rawdata=self.data2_cap.d,
                     i_iamvld=self.data2_cap.phsaligned,
                     i_other_ch0_vld=self.data1_cap.phsaligned,
                     i_other_ch0_rdy=data1_iamrdy,
                     i_other_ch1_vld=self.data0_cap.phsaligned,
                     i_other_ch1_rdy=data0_iamrdy,
                     o_iamrdy=data2_iamrdy,
                     o_sdata=data2_bonded,
                     ),
            ]
        self.comb += all_rdy.eq(data0_iamrdy & data1_iamrdy & data2_iamrdy)
        self.submodules.auto0_decoding = Decoding()
        self.submodules.auto1_decoding = Decoding()
        self.submodules.auto2_decoding = Decoding()
        self.comb += [
            self.auto0_decoding.valid_i.eq(data0_iamrdy),
            self.auto1_decoding.valid_i.eq(data1_iamrdy),
            self.auto2_decoding.valid_i.eq(data2_iamrdy),
            self.auto0_decoding.input.eq(data0_bonded),
            self.auto1_decoding.input.eq(data1_bonded),
            self.auto2_decoding.input.eq(data2_bonded)
        ]

        self.submodules.chansync = ChanSync()
        self.comb += [
            self.chansync.valid_i.eq(self.data0_decod.valid_o &
                                     self.data1_decod.valid_o &
                                     self.data2_decod.valid_o),
            self.chansync.data_in0.eq(self.data0_decod.output),
            self.chansync.data_in1.eq(self.data1_decod.output),
            self.chansync.data_in2.eq(self.data2_decod.output)
        ]

        if hdmi:
            decode_terc4 = DecodeTERC4()
            self.submodules.decode_terc4 = ClockDomainsRenamer("pix")(decode_terc4) # rename so state machine is in pix domain, not default sys domain
            self.comb += [
                If(use_alt_bond,
                   self.decode_terc4.valid_i.eq(all_rdy),
                   self.decode_terc4.data_in0.eq(self.auto0_decoding.output),
                   self.decode_terc4.data_in1.eq(self.auto1_decoding.output),
                   self.decode_terc4.data_in2.eq(self.auto2_decoding.output),
                ).Else(
                    self.decode_terc4.valid_i.eq(self.chansync.chan_synced),
                    self.decode_terc4.data_in0.eq(self.chansync.data_out0),
                    self.decode_terc4.data_in1.eq(self.chansync.data_out1),
                    self.decode_terc4.data_in2.eq(self.chansync.data_out2),
                )
            ]

            self.submodules.syncpol = SyncPolarity(hdmi, split_mmcm)
            self.comb += self.syncpol.de_int.eq(self.decode_terc4.de_o) # manually wire up the fancy de signal in case hdmi is True

            # delay the rest of the signals to match the time it took to derive the fancy de signal
            #for datan in range(3):
            #    name = "data" + str(datan)

             #   timingdelay = TimingDelayChannel(1)
             #   timingdelay = ClockDomainsRenamer("pix")(timingdelay)
             #   setattr(self.submodules, name + "_timingdelay", timingdelay)
                # self.comb += timingdelay.sink.eq(getattr(self, "self.chansync.data_out" + str(datan)))  # this code doesn't work. why???
            self.submodules.data0_timingdelay = TimingDelayChannel(1)
            self.submodules.data1_timingdelay = TimingDelayChannel(1)
            self.submodules.data2_timingdelay = TimingDelayChannel(1)

            self.comb += [
                If(use_alt_bond,
                   self.data0_timingdelay.sink.eq(self.auto0_decoding.output),
                   self.data1_timingdelay.sink.eq(self.auto1_decoding.output),
                   self.data2_timingdelay.sink.eq(self.auto2_decoding.output),
                ).Else(
                    self.data0_timingdelay.sink.eq(self.chansync.data_out0),
                    self.data1_timingdelay.sink.eq(self.chansync.data_out1),
                    self.data2_timingdelay.sink.eq(self.chansync.data_out2),
                )
            ]
            self.comb += [
                self.syncpol.data_in0.eq(self.data0_timingdelay.source),
                self.syncpol.data_in1.eq(self.data1_timingdelay.source),
                self.syncpol.data_in2.eq(self.data2_timingdelay.source),
                If(use_alt_bond,
                   self.syncpol.valid_i.eq(all_rdy)
                ).Else(
                   self.syncpol.valid_i.eq(self.chansync.chan_synced)  # OK not to delay because it's a "once in a blue moon" transition that sorts itself out on the next VSYNC
                )
            ]

        else:
            self.submodules.syncpol = SyncPolarity(hdmi, split_mmcm)
            self.comb += [
                self.syncpol.valid_i.eq(self.chansync.chan_synced),
                self.syncpol.data_in0.eq(self.chansync.data_out0),
                self.syncpol.data_in1.eq(self.chansync.data_out1),
                self.syncpol.data_in2.eq(self.chansync.data_out2)
            ]

        self.submodules.resdetection = ResolutionDetection()
        self.comb += [
            self.resdetection.valid_i.eq(self.syncpol.valid_o),
            self.resdetection.de.eq(self.syncpol.de),
            self.resdetection.vsync.eq(self.syncpol.vsync)
        ]

        if dram_port is not None:
            self.submodules.frame = FrameExtraction(dram_port.dw, fifo_depth, mode)
            self.comb += [
                self.frame.valid_i.eq(self.syncpol.valid_o),
                self.frame.de.eq(self.syncpol.de),
                self.frame.vsync.eq(self.syncpol.vsync),
                self.frame.r.eq(self.syncpol.r),
                self.frame.g.eq(self.syncpol.g),
                self.frame.b.eq(self.syncpol.b)
            ]


            self.submodules.dma = DMA(dram_port, n_dma_slots)
            self.comb += self.frame.frame.connect(self.dma.frame)
            self.ev = self.dma.ev

    autocsr_exclude = {"ev"}
