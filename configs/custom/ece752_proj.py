from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import PrivateL1PrivateL2CacheHierarchy
from gem5.components.memory.single_channel import SingleChannelDDR3_1600
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.resources.resource import *
from gem5.simulate.simulator import Simulator
from gem5.resources.workload import CustomWorkload

import m5
from m5.objects import *

cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(l1d_size="32KiB", l1i_size="32KiB", l2_size="256KiB")
memory = SingleChannelDDR3_1600("1GiB")

# AtomicSimpleCPU
#processor = SimpleProcessor(isa=ISA.X86,cpu_type=CPUTypes.ATOMIC, num_cores=1)
# TimingSimpleCPU
#processor = SimpleProcessor(isa=ISA.X86,cpu_type=CPUTypes.TIMING, num_cores=1)
# O3CPU
processor = SimpleProcessor(isa=ISA.X86,cpu_type=CPUTypes.O3, num_cores=1)

processor.cores[0].core.max_insts_any_thread=250000000

# Add them to the board
board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
    )

# binary = CustomResource("../spectre/spectre.gcc");
# board.set_se_binary_workload(binary)

board.set_se_binary_workload(
    binary = CustomResource("../spec2006/gcc/gcc_base.x86_64_sse"),
    arguments = ["../spec2006/gcc/input/scilab.i", "-o scilab.o"],
    )

simulator = Simulator(board=board)

print ("Beginning simulation!")
simulator.run()
