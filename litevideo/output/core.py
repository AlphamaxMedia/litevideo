from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer, BusSynchronizer

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

from litedram.frontend.dma import LiteDRAMDMAReader

from litevideo.output.common import *
from litevideo.output.hdmi.s6 import S6HDMIOutClocking, S6HDMIOutPHY
from litevideo.output.hdmi.s7 import S7HDMIOutClocking, S7HDMIOutPHY


class Initiator(Module, AutoCSR):
    """Initiator

    Generates the H/V and DMA parameters of a frame.

    CSR -> local clock domain via 2-deep FIFO. The FIFO is only read when the timing generator "pulls" it
    which I think allows for "intelligent" queueing of new values coming in (e.g. no mid-frame timing changes)
    """
    def __init__(self, cd): # CD is the clock domain of the dram port
        self.source = stream.Endpoint(frame_parameter_layout +
                                      frame_dma_layout)  # outputs are a cd-synchronized set of parameter from CSRs

        # # #

        cdc = stream.AsyncFIFO(self.source.description, 2)
        cdc = ClockDomainsRenamer({"write": "sys",
                                   "read": cd})(cdc)
        self.submodules += cdc

        self.enable = CSRStorage()
        for name, width in frame_parameter_layout + frame_dma_layout:
            setattr(self, name, CSRStorage(width, name=name, atomic_write=True))  # builds the CSR list
            self.comb += getattr(cdc.sink, name).eq(getattr(self, name).storage)  # assigns them to the sink
        self.comb += cdc.sink.valid.eq(self.enable.storage)  # I don't quite get this line, seems source.valid should be assigned here??
        self.comb += cdc.source.connect(self.source)   # FIFO's output ("source") is now our output


class DMAReader(Module, AutoCSR):
    """DMA reader

    Generates the data stream of a frame.
    """
    def __init__(self, dram_port, fifo_depth=512, genlock_stream=None):
        self.sink = sink = stream.Endpoint(frame_dma_layout)  # "inputs" are the DMA frame parameters
        self.source = source = stream.Endpoint([("data", dram_port.dw)])  # "output" is the data stream

        # # #

        self.submodules.dma = LiteDRAMDMAReader(dram_port, fifo_depth, True)
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")

        shift = log2_int(dram_port.dw//8)
        base = Signal(dram_port.aw)
        length = Signal(dram_port.aw)
        offset = Signal(dram_port.aw)
        self.delay_base = CSRStorage(32)
        delay_base = Signal(32)
        self.line_align = CSRStorage(16)
        line_align = Signal(16)
        self.line_skip = CSRStorage(32)
        line_skip = Signal(32)
        self.hres_out = CSRStorage(16)
        self.vres_out = CSRStorage(16)
        self.comb += [
            base.eq(sink.base[shift:]),   # ignore the lower bits of the base + length to match the DMA's expectations
            length.eq(sink.length[shift:]), # need to noodle on what that expectation is, exactly...
        ]
        hres = Signal(16)
        vres = Signal(16)
        self.comb += [
            hres.eq(self.hres_out.storage),
            vres.eq(self.vres_out.storage),
        ]
        hcount = Signal(16)
        vcount = Signal(16)
        linecount = Signal(16)
        self.field = Signal()
        vsyncpos = Signal(16)
        even_loc = Signal(16)
        odd_loc = Signal(16)
        self.even_pos = CSRStatus(16)
        self.odd_pos = CSRStatus(16)
        self.comb += [
            self.even_pos.status.eq(even_loc),
            self.odd_pos.status.eq(odd_loc),
        ]
        self.field_pos = CSRStorage(16)
        self.interlace = CSRStorage(2)  # bit 0 is enable, bit 1 is even/odd priority
        field_pos = Signal(16)
        interlace = Signal(2)
        self.submodules.sync_field_pos = BusSynchronizer(self.field_pos.size, "sys", "pix_o")
        self.submodules.sync_interlace = BusSynchronizer(self.interlace.size, "sys", "pix_o")
        self.submodules.sync_delay_base = BusSynchronizer(self.delay_base.size, "sys", "pix_o")
        self.submodules.sync_line_skip = BusSynchronizer(self.line_skip.size, "sys", "pix_o")
        self.submodules.sync_line_align = BusSynchronizer(self.line_align.size, "sys", "pix_o")
        self.comb += [
            self.sync_field_pos.i.eq(self.field_pos.storage),
            self.sync_interlace.i.eq(self.interlace.storage),
            self.sync_delay_base.i.eq(self.delay_base.storage),
            self.sync_line_skip.i.eq(self.line_skip.storage),
            self.sync_line_align.i.eq(self.line_align.storage),
            field_pos.eq(self.sync_field_pos.o),
            interlace.eq(self.sync_interlace.o),
            delay_base.eq(self.sync_delay_base.o),
            line_skip.eq(self.sync_line_skip.o),
            line_align.eq(self.sync_line_align.o),
        ]
        squash = Signal()

        if genlock_stream != None:
            self.v = Signal()
            self.v_r = Signal()
            self.sof = Signal()
            self.sync += [
                self.v.eq(genlock_stream.vsync),
                self.v_r.eq(self.v),
                self.sof.eq(self.v & ~self.v_r),
            ]
            self.h = Signal()
            self.h_r = Signal()
            self.sol = Signal()
            self.sync += [
                self.h.eq(genlock_stream.hsync),
                self.h_r.eq(self.h),
                self.sol.eq(self.h & ~self.h_r),
            ]
            self.sync += [
                If(self.sol,
                   vsyncpos.eq(0)
                ).Else(
                   vsyncpos.eq(vsyncpos + 1)
                ),
                If(self.sof,
                   self.field.eq(~self.field),
                   If(self.field,
                         odd_loc.eq(vsyncpos),
                    ).Else(
                         even_loc.eq(vsyncpos),
                    ),
                ),
            ]

        if genlock_stream == None:
            fsm.act("IDLE",
                NextValue(offset, 0),
                If(sink.valid,  # if our parameters are valid, start reading
                       NextState("READ")
                    ).Else(
                        dram_port.flush.eq(1),
                    )
                )
            fsm.act("READ",
                self.dma.sink.valid.eq(1),  # tell the DMA reader that we've got a valid address for it
                If(self.dma.sink.ready, # if the LiteDRAMDMAReader shows it's ready for an address (e.g. taken the current address)
                    NextValue(offset, offset + 1), # increment the offset
                    If(offset == (length - 1),  # at the end...
                        self.sink.ready.eq(1),  # indicate we're ready for more parameters
                        NextState("IDLE")
                    )
                )
            )
        else:
            fsm.act("IDLE",
                NextValue(linecount, 0),
                If( self.interlace.storage[0],
                    # interlace
                    If(self.field ^ interlace[1] ^ (odd_loc < vsyncpos), # xor against actual odd_loc vs vsyncpos in case we caught the wrong field polarity
                       NextValue(offset, delay_base),
                        NextValue(hcount, 0),
                        NextValue(vcount, 0),
                    ).Else(
                        NextValue(offset, delay_base + hres + 1),
                        NextValue(hcount, 0),
                        NextValue(vcount, 1),
                    )
                ).Else(
                    # non-interlace
                    NextValue(offset, delay_base),
                    NextValue(hcount, 0),
                    NextValue(vcount, 0),
                ),
                If(sink.valid,  # if our parameters are valid, start reading
                   If(line_align == 0,
                      NextState("READ"),
                    ).Else (
                      NextState("WAIT_LINE"),
                   )
                ).Else(
                        dram_port.flush.eq(1),
                )
            )
            fsm.act("WAIT_LINE", # insert dummy waits until wait_line is done
                squash.eq(1),
                self.dma.sink.valid.eq(1),  # tell the DMA reader that we've got a valid address for it
                If(self.dma.sink.ready, # if the LiteDRAMDMAReader shows it's ready for an address (e.g. taken the current address)
                   If(linecount < line_align,
                      NextValue(linecount, linecount + 1),
                   ).Else(
                     NextState("READ"),
                   )
                )
            )
            fsm.act("READ",
                self.dma.sink.valid.eq(1),  # tell the DMA reader that we've got a valid address for it
                If(self.dma.sink.ready, # if the LiteDRAMDMAReader shows it's ready for an address (e.g. taken the current address)
                    NextValue(hcount, hcount + 1),
                    If(hcount >= hres,
                      If( interlace[0],
                          # interlaced
                          NextValue(offset, offset + line_skip + 1 + hres + 1),
                          NextValue(hcount, 0),
                          NextValue(vcount, vcount + 2),
                      ).Else(
                          # not interlaced
                          NextValue(offset, offset + line_skip + 1),
                          NextValue(hcount, 0),
                          NextValue(vcount, vcount + 1),
                      )
                    ).Else(
                      NextValue(offset, offset + 1), # increment the offset
                    ),
                    If( (offset >= (length - 1)) | vcount >= vres,  # at the end...
                        self.sink.ready.eq(1),  # indicate we're ready for more parameters
                        NextState("WAIT_SOF")
                    )
                )
            )
            fsm.act("WAIT_SOF",  # wait till vsync/start of frame
                If(self.sof,
                   NextState("IDLE")
                )
            )

        self.comb += [
            self.dma.sink.address.eq(base + offset),  # input to the DMA is an address of base + offset
            self.dma.source.connect(self.source)      # connect the DMA's output to the output of this module
        ]


class TimingGenerator(Module):
    """Timing Generator

    Generates the H/V timings of a frame.
    """
    def __init__(self, genlock_stream=None):
        self.sink = sink = stream.Endpoint(frame_parameter_layout)   # "inputs" are the parameter layout (via CSR via initiator)
        self.source = source = stream.Endpoint(frame_timing_layout)  # "outputs" are a frame timing layout

        # # #
        if genlock_stream == None:
            hactive = Signal()
            vactive = Signal()
            active = Signal()

            hcounter = Signal(hbits)
            vcounter = Signal(vbits)

            self.comb += [
                If(sink.valid,  # if the frame parameters are valid...
                    active.eq(hactive & vactive),  # go ahead and let the logic update for active, valid
                    source.valid.eq(1),
                    If(active,
                        source.de.eq(1),
                    )
                ), ### but else...what? they don't revert to 0, so they stay "stuck" on when sink is invalid???
                sink.ready.eq(source.ready & source.last)
            ]

            self.sync += \
                If(~sink.valid,  # if our parameters aren't valid, reset everything
                    hactive.eq(0),
                    vactive.eq(0),
                    hcounter.eq(0),
                    vcounter.eq(0)
                ).Elif(source.ready,  # otherwise, if the thing downstream from us is ready...
                    source.last.eq(0),  # self.sync is blocking, so this will get over-ridden later as needed
                    hcounter.eq(hcounter + 1),

                    If(hcounter == 0, hactive.eq(1)),
                    If(hcounter == sink.hres, hactive.eq(0)),  # sink is our "input" of parameters
                    If(hcounter == sink.hsync_start, source.hsync.eq(1)),
                    If(hcounter == sink.hsync_end, source.hsync.eq(0)),
                    If(hcounter == sink.hscan,  # if we hit the end of the line
                        hcounter.eq(0),  # reset the counter, overriding the +1 earlier coz this is a "blocking" syntax
                        If(vcounter == sink.vscan,
                            vcounter.eq(0),
                            source.last.eq(1)
                        ).Else(
                            vcounter.eq(vcounter + 1)
                        )
                    ),

                    If(vcounter == 0, vactive.eq(1)),
                    If(vcounter == sink.vres, vactive.eq(0)),
                    If(vcounter == sink.vsync_start, source.vsync.eq(1)),
                    If(vcounter == sink.vsync_end, source.vsync.eq(0))
                )
        else:
            self.comb += genlock_stream.connect(self.source)



modes_dw = {
    "raw":      32,
    "rgb":      24,
    "ycbcr422": 16
}


class VideoOutCore(Module, AutoCSR):
    """Video out core

    Generates a video stream from memory.
    """
    def __init__(self, dram_port, mode="rgb", fifo_depth=512, genlock_stream=None):
        try:
            dw = modes_dw[mode]
        except:
            raise ValueError("Unsupported {} video mode".format(mode))
        assert dram_port.dw >= dw
        assert dram_port.dw == 2**log2_int(dw, need_pow2=False)
        self.source = source = stream.Endpoint(video_out_layout(dw))  # "output" is a video layout that's dw wide

        self.underflow_enable = CSRStorage()
        self.underflow_update = CSR()
        self.underflow_counter = CSRStatus(32)

        # # #

        cd = dram_port.cd

        self.submodules.initiator = initiator = Initiator(cd)
        if genlock_stream == None:
            self.submodules.timing = timing = ClockDomainsRenamer(cd)(TimingGenerator())
        else:
            self.submodules.timing = timing = ClockDomainsRenamer(cd)(TimingGenerator(genlock_stream))
        self.submodules.dma = dma = ClockDomainsRenamer(cd)(DMAReader(dram_port, fifo_depth, genlock_stream))

        # ctrl path
        self.comb += timing.sink.valid.eq(initiator.source.valid) # if the CSR FIFO data is valid, timing may proceed

        self.comb += [
            # dispatch initiator parameters to timing & dma
            dma.sink.valid.eq(initiator.source.valid),   # the DMA's parameter input "pushed" from the initiator, so connect the valids
            initiator.source.ready.eq(timing.sink.ready), # timing's parameters come from initiator, but this is "pulled" by timing so connect readys

            # combine timing and dma
            source.valid.eq(timing.source.valid & (~timing.source.de | dma.source.valid)), # our output is valid only when timing's outputs are valid and (when the dma's output is valid or de is low)
              # the "or de is low" thing seems like a hack to fix some edge case??
            # flush dma/timing when disabled
            If(~initiator.source.valid,  # if the initiator's (e.g. CSR) outputs aren't valid
                timing.source.ready.eq(1), # force the outputs to 1 to keep the DMA running
                dma.source.ready.eq(1)
            ).Elif(source.valid & source.ready, # else if our DMA output stream has valid data, and is ready to accept addresses
                timing.source.ready.eq(1),  # output stream of timing is ready to go, which kicks off the timing generator...
                dma.source.ready.eq(timing.source.de | (mode == "raw"))  # and the DMA's DMAReader source ready is tied to the timing's DE signal
            )
        ]

        # data path
        self.comb += [
            # dispatch initiator parameters to timing & dma
            initiator.source.connect(timing.sink, keep=list_signals(frame_parameter_layout)), # initiator is a compound source, so use "keep" to demux. initiator sources config data to the timer
            initiator.source.connect(dma.sink, keep=list_signals(frame_dma_layout)),  # the initiator sources parameters to the DMA, which are DMA layout config data

            # combine timing and dma
            source.de.eq(timing.source.de),  # manually assign this block's video de, hsync, vsync outputs,, to the respective timing or DMA outputs
            source.hsync.eq(timing.source.hsync),
            source.vsync.eq(timing.source.vsync),
            source.data.eq(dma.source.data)
        ]

        # underflow detection
        underflow_enable = Signal()
        underflow_update = Signal()
        underflow_counter = Signal(32)
        self.specials += MultiReg(self.underflow_enable.storage, underflow_enable)
        underflow_update_synchronizer = PulseSynchronizer("sys", cd)
        self.submodules += underflow_update_synchronizer
        self.comb += [
            underflow_update_synchronizer.i.eq(self.underflow_update.re),
            underflow_update.eq(underflow_update_synchronizer.o)
        ]
        sync = getattr(self.sync, cd)
        sync += [
            If(underflow_enable,
                If(~source.valid,  # count whenever the source isn't valid...
                    underflow_counter.eq(underflow_counter + 1)
                )
            ).Else(
                underflow_counter.eq(0)
            ),
            If(underflow_update,
                self.underflow_counter.status.eq(underflow_counter)
            )
        ]
