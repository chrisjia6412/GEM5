# Copyright (c) 2012-2013 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2006-2008 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Steve Reinhardt

# Simple test script
#
# "m5 test.py"

import optparse
import sys
import os

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../common')
addToPath('../ruby')
addToPath('../topologies')
import re
import Options
import Ruby
import Simulation
import CacheConfig
import MemConfig
from Caches import *
from cpu2000 import *

icache_readreq_misses = 0;
dcache_readreq_misses = 0;
icache_readreq_accesses = 0;
dcache_readreq_hits = 0;
dcache_writereq_hits = 0;
icache_tags = 0;
dcache_tags = 0;
dcache_writereq_misses = 0;

def get_processes(options):
    """Interprets provided options and returns a list of processes"""

    multiprocesses = []
    inputs = []
    outputs = []
    errouts = []
    pargs = []

    workloads = options.cmd.split(';')
    if options.input != "":
        inputs = options.input.split(';')
    if options.output != "":
        outputs = options.output.split(';')
    if options.errout != "":
        errouts = options.errout.split(';')
    if options.options != "":
        pargs = options.options.split(';')

    idx = 0
    for wrkld in workloads:
        process = LiveProcess()
        process.executable = wrkld
        process.cwd = os.getcwd()

        if len(pargs) > idx:
            process.cmd = [wrkld] + pargs[idx].split()
        else:
            process.cmd = [wrkld]

        if len(inputs) > idx:
            process.input = inputs[idx]
        if len(outputs) > idx:
            process.output = outputs[idx]
        if len(errouts) > idx:
            process.errout = errouts[idx]

        multiprocesses.append(process)
        idx += 1

    if options.smt:
        assert(options.cpu_type == "detailed" or options.cpu_type == "inorder")
        return multiprocesses, idx
    else:
        return multiprocesses, 1


parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addSEOptions(parser)

if '--ruby' in sys.argv:
    Ruby.define_options(parser)

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

multiprocesses = []
numThreads = 1

if options.bench:
    apps = options.bench.split("-")
    if len(apps) != options.num_cpus:
        print "number of benchmarks not equal to set num_cpus!"
        sys.exit(1)

    for app in apps:
        try:
            if buildEnv['TARGET_ISA'] == 'alpha':
                exec("workload = %s('alpha', 'tru64', 'ref')" % app)
            else:
                exec("workload = %s(buildEnv['TARGET_ISA'], 'linux', 'ref')" % app)
            multiprocesses.append(workload.makeLiveProcess())
        except:
            print >>sys.stderr, "Unable to find workload for %s: %s" % (buildEnv['TARGET_ISA'], app)
            sys.exit(1)
elif options.cmd:
    multiprocesses, numThreads = get_processes(options)
else:
    print >> sys.stderr, "No workload specified. Exiting!\n"
    sys.exit(1)


#(CPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)
#CPUClass.numThreads = numThreads

MemClass = Simulation.setMemClass(options)

# Check -- do not allow SMT with multiple CPUs
if options.smt and options.num_cpus > 1:
    fatal("You cannot use SMT with multiple CPUs!")

detailed_switched_out = options.simpoint_mode != "none" and options.simpoint_mode != "checkpoint";
if options.cpu_type == 'detailed':
    detailed_cpu = DerivO3CPU(cpu_id = 0, switched_out=detailed_switched_out)
else:
    print "Error: unknown cpu type."
    sys.exit(1)
simple_cpu = AtomicSimpleCPU(cpu_id = 0, switched_out=(not detailed_switched_out))

if detailed_switched_out == True:
    cpu = simple_cpu
    starting_mem_mode = "atomic"
else:
    cpu = detailed_cpu
    starting_mem_mode = "timing"



np = options.num_cpus
system = System(cpu = cpu,
                mem_mode = starting_mem_mode,
                mem_ranges = [AddrRange(options.mem_size)],
                cache_line_size = options.cacheline_size)

# Create a top-level voltage domain
system.voltage_domain = VoltageDomain(voltage = options.sys_voltage)

# Create a source clock for the system and set the clock period
system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                                   voltage_domain = system.voltage_domain)

# Create a CPU voltage domain
system.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs
system.cpu_clk_domain = SrcClockDomain(clock = options.cpu_clock,
                                       voltage_domain =
                                       system.cpu_voltage_domain)

# All cpus belong to a common cpu_clk_domain, therefore running at a common
# frequency.
#for cpu in system.cpu:
cpu.clk_domain = system.cpu_clk_domain

# Sanity check
if options.fastmem:
    if CPUClass != AtomicSimpleCPU:
        fatal("Fastmem can only be used with atomic CPU!")
    if (options.caches or options.l2cache):
        fatal("You cannot use fastmem in combination with caches!")

if options.simpoint_profile:
    if not options.fastmem:
        # Atomic CPU checked with fastmem option already
        fatal("SimPoint generation should be done with atomic cpu and fastmem")
    if np > 1:
        fatal("SimPoint generation not supported with more than one CPUs")

#Qi: deactivate at this time, maybe used future
#if options.activate_mcpat:
    #system.mcpat_activated= "TRUE"

#if options.mcpat_arch_file_name is not None:
    #system.mcpat_arch_file_name = options.mcpat_arch_file_name


for i in xrange(np):
    if options.smt:
        system.cpu[i].workload = multiprocesses
    elif len(multiprocesses) == 1:
        system.cpu[i].workload = multiprocesses[0]
    else:
        system.cpu[i].workload = multiprocesses[i]

    if options.fastmem:
        system.cpu[i].fastmem = True

    if options.simpoint_profile:
        system.cpu[i].simpoint_profile = True
        system.cpu[i].simpoint_interval = options.simpoint_interval

    if options.checker:
        system.cpu[i].addCheckerCpu()

    system.cpu[i].createThreads()

if options.ruby:
    if not (options.cpu_type == "detailed" or options.cpu_type == "timing"):
        print >> sys.stderr, "Ruby requires TimingSimpleCPU or O3CPU!!"
        sys.exit(1)

    # Set the option for physmem so that it is not allocated any space
    system.physmem = MemClass(range=AddrRange(options.mem_size),
                              null = True)
#    print "I AM PRINTING",system.physmem
    options.use_map = True
    Ruby.create_system(options, system)
    assert(options.num_cpus == len(system.ruby._cpu_ruby_ports))

    for i in xrange(np):
        ruby_port = system.ruby._cpu_ruby_ports[i]

        # Create the interrupt controller and connect its ports to Ruby
        # Note that the interrupt controller is always present but only
        # in x86 does it have message ports that need to be connected
        system.cpu[i].createInterruptController()

        # Connect the cpu's cache ports to Ruby
        system.cpu[i].icache_port = ruby_port.slave
        system.cpu[i].dcache_port = ruby_port.slave
        if buildEnv['TARGET_ISA'] == 'x86':
            system.cpu[i].interrupts.pio = ruby_port.master
            system.cpu[i].interrupts.int_master = ruby_port.slave
            system.cpu[i].interrupts.int_slave = ruby_port.master
            system.cpu[i].itb.walker.port = ruby_port.slave
            system.cpu[i].dtb.walker.port = ruby_port.slave
else:
    system.membus = CoherentBus()
    system.system_port = system.membus.slave
    CacheConfig.config_cache(options, system)
    MemConfig.config_mem(options, system)

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



#Simulation.run(options, root, system, FutureClass)
