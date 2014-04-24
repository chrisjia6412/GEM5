import ConfigParser, optparse, sys, operator, re
from itertools import *
import os

import m5
from m5.objects import *
from Nehalem import *

# Parse arguments
parser = optparse.OptionParser()
parser.add_option("--cfg", action="store", type="string", dest="cfg",
                  help="The system config file to use.")


# Benchmark specific options
parser.add_option("--cmd", action="store", type="string", dest="cmd",
                  help="The binary to run in syscall emulation mode.")
parser.add_option("-o", "--options", action="store", type="string",
                  help="Program arguments.")
parser.add_option("--stdin", action="store", type="string", dest="stdin",
                  help="Read stdin from a file.")
parser.add_option("--stdout", action="store", type="string", dest="stdout",
                  help="Redirect stdout to a file")
parser.add_option("--stderr", action="store", type="string", dest="stderr",
                  help="Redirect stderr to a file")

# Simpoint options
parser.add_option("--simpoint-points", action="store", type="string",
                  default=None, dest="simpoint_points",
                  help="File containing simpoint instruction points")

parser.add_option("--simpoint-interval", type="int", default=100000000);

parser.add_option("--simpoint-mode", type="choice", default="none",
                  choices=["none", "generate", "fastfwd", "checkpoint_gen",
                           "checkpoint"],
                  help="Simpoint simulation methods")

parser.add_option("--checkpoint-dir", action="store", type="string",
                  default=None, dest="checkpoint_dir",
                  help="Directory containing simpoint checkpoints")

parser.add_option("--checkpoint-num", type="int", default=0,
                  help="Checkpoint number to restore")

(options, args) = parser.parse_args()

# Sanity check arguments parsed
if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

if options.cfg == None:
    print "Error: no system config."
    sys.exit(1)

if options.cmd == None:
    print "Error: no command."
    sys.exit(1)

# Create process
process = LiveProcess()
process.executable = options.cmd

if options.options != None:
    process.cmd = [options.cmd] + options.options.split()
else:
    process.cmd = [options.cmd]

if options.stdin != None:
    process.input = options.stdin

if options.stdout != None:
    process.output = options.stdout

if options.stderr != None:
    process.errout = options.stderr


# Parse system config file
config = ConfigParser.RawConfigParser()
config.optionxform = str
config.read(options.cfg)

detailed_switched_out = options.simpoint_mode != "none" and options.simpoint_mode != "checkpoint";

# Configure CPU
if config.has_option('cpu', 'type') == False:
    print "Error: missing cpu type."
    sys.exit(1)
cpu_type = config.get('cpu', 'type')
cpu_options = dict(config.items('cpu'))
del cpu_options['type']
cpu = None
if cpu_type == 'nehalem':
    detailed_cpu = Nehalem(cpu_id = 0, switched_out=detailed_switched_out, **cpu_options)
else:
    print "Error: unknown cpu type."
    sys.exit(1)


simple_cpu = AtomicSimpleCPU(cpu_id = 0, clock=detailed_cpu.clock, switched_out=(not detailed_switched_out))

if detailed_switched_out == True:
    cpu = simple_cpu
    starting_mem_mode = "atomic"
else:
    cpu = detailed_cpu
    starting_mem_mode = "timing"

cpu.workload = process
cpu.createThreads()

# Configure L1 caches
icache = None
if config.has_section('icache') == False:
    print "Error: no icache defined."
    sys.exit(1)
icache_options = dict(config.items('icache'))
if icache_options.has_key('prefetcher'):
    icache_options['prefetcher'] = eval(icache_options['prefetcher'])
icache = BaseCache(**icache_options)

dcache = None
if config.has_section('dcache') == False:
    print "Error: no dcache defined."
    sys.exit(1)
dcache_options = dict(config.items('dcache'))
if dcache_options.has_key('prefetcher'):
    dcache_options['prefetcher'] = eval(dcache_options['prefetcher'])
dcache = BaseCache(**dcache_options)

cpu.addPrivateSplitL1Caches(icache, dcache)
cpu.createInterruptController()

# Configure DRAM and System
physmem = SimpleDDR3(range = AddrRange('1GB'), zero = True)
system = System(cpu = cpu,
                physmem = physmem,
                membus = CoherentBus(),
                mem_mode = starting_mem_mode)

system.system_port = system.membus.slave
system.physmem.port = system.membus.master

# Configure L2 cache
if config.has_section('l2'):
    l2_options = dict(config.items('l2'))
    if l2_options.has_key('prefetcher'):
        l2_options['prefetcher'] = eval(l2_options['prefetcher'])
    system.l2 = BaseCache(clock = cpu.clock,
                          **l2_options)
    system.tol2bus = CoherentBus(clock = cpu.clock, width = 32)
    system.l2.cpu_side = system.tol2bus.master
    if config.has_section('l3') == False:
        system.l2.mem_side = system.membus.slave
    cpu.connectAllPorts(system.tol2bus)
else:
     cpu.connectAllPorts(system.membus)

# Configure L3 cache
if config.has_section('l3'):
    l3_options = dict(config.items('l3'))
    if l3_options.has_key('prefetcher'):
        l3_options['prefetcher'] = eval(l3_options['prefetcher'])
    system.l3 = BaseCache(clock = cpu.clock,
                          **l3_options)
    system.tol3bus = CoherentBus(clock = cpu.clock, width = 16)
    system.l2.mem_side = system.tol3bus.slave
    system.l3.cpu_side = system.tol3bus.master
    system.l3.mem_side = system.membus.slave

# Parse simpoint info if file exists
if options.simpoint_mode == "generate":
    cpu.simpoint_profile = True
    cpu.simpoint_interval = options.simpoint_interval
elif options.simpoint_mode == "fastfwd":
    simpoints = sorted([max(int(line.split()[0]) * options.simpoint_interval, 1) for line in open(options.simpoint_points)])
    detailed_end_points = []
    simple_end_points = []
    simple_sim_inst = 0
    detailed_sim_inst = 0
    total_sim_inst = 0
    for point in simpoints:
        simple_sim_inst = max(point-detailed_sim_inst, simple_sim_inst + 1)
        simple_end_points.append(simple_sim_inst)

        detailed_inst = detailed_sim_inst + options.simpoint_interval
        detailed_end_points.append(detailed_inst)
        detailed_sim_inst += options.simpoint_interval

    simple_cpu.simpoint_start_insts = simple_end_points
    detailed_cpu.simpoint_start_insts = detailed_end_points

    switch_cpu_list = [(simple_cpu, detailed_cpu)]
    system.switch_cpu = detailed_cpu
    detailed_cpu.workload = cpu.workload
elif options.simpoint_mode == "checkpoint_gen":
    simpoints = sorted([max(int(line.split()[0]) * options.simpoint_interval, 1) for line in open(options.simpoint_points)])
    simple_end_points = simpoints
    simple_cpu.simpoint_start_insts = simple_end_points

    if os.path.exists(options.checkpoint_dir):
       fatal("Checkpoint directory '%s' already exists.", options.checkpoint_dir)
    else:
        os.mkdir(options.checkpoint_dir)

root = Root(full_system = False, system = system)

ckpt_dir = None
if options.simpoint_mode == "checkpoint":
    simpoints = sorted([max(int(line.split()[0]) * options.simpoint_interval, 1) for line in open(options.simpoint_points)])
    if options.checkpoint_num >= len(simpoints):
        fatal("Invalid checkpoint number %d.  Max possible %d", options.checkpoint_num,  len(simple_end_points))
    point_inst = simpoints[options.checkpoint_num];
    cpu.simpoint_start_insts = [ options.simpoint_interval ]
    ckpt_dir = os.path.join(options.checkpoint_dir, "simpoint.ckpt.inst.%d" % point_inst)

m5.instantiate(ckpt_dir)

if options.simpoint_mode == "none" or options.simpoint_mode == "checkpoint" or options.simpoint_mode == "generate":
    exit_event = m5.simulate()
    exit_cause = exit_event.getCause()
elif options.simpoint_mode == "fastfwd":
    exit_event = m5.simulate()
    exit_cause = exit_event.getCause()
    while exit_cause == "simpoint starting point found":
        m5.switchCpus(system, switch_cpu_list)
        tmp_cpu_list = []
        for old_cpu, new_cpu in switch_cpu_list:
            tmp_cpu_list.append((new_cpu, old_cpu))
        switch_cpu_list = tmp_cpu_list
        m5.stats.reset()

        exit_event = m5.simulate()
        exit_cause = exit_event.getCause()
        if switch_cpu_list[0][0].type == 'DerivO3CPU':
            m5.stats.dump()
elif options.simpoint_mode == "checkpoint_gen":
    exit_event = m5.simulate()
    exit_cause = exit_event.getCause()

    checkpoint_points = simple_end_points[:]
    checkpoint_points.reverse()
    while exit_cause == "simpoint starting point found":
        point_inst = checkpoint_points.pop()
        m5.checkpoint(os.path.join(options.checkpoint_dir, "simpoint.ckpt.inst.%d" % point_inst))

        if not checkpoint_points:
            exit_cause = "final simpoint checkpoint created"
            break;

        exit_event = m5.simulate()
        exit_cause = exit_event.getCause()


print "Exiting @ tick %i because %s" % (m5.curTick(), exit_cause)

