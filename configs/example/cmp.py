#cmp.py  
# Simple configuration script  
  
import os, optparse, sys  
  
import m5  
from m5.defines import buildEnv  
from m5.objects import *  
from m5.util import addToPath  
  
addToPath('../common')  
addToPath('../ruby')  
addToPath('../topologies')  
  
  
import Options  
import Ruby  
import Simulation  
from Caches import *  
import CacheConfig  
import cpu2006  
  
# Get paths we might need.  It's expected this file is in m5/configs/example.  
config_path = os.path.dirname(os.path.abspath(__file__))  
print config_path   # 'configs/cpu2006'  
config_root = os.path.dirname(config_path)  
print config_root  
m5_root = os.path.dirname(config_root)  
print m5_root  
  
parser = optparse.OptionParser()  
Options.addCommonOptions(parser)  
Options.addSEOptions(parser)  
  
# Benchmark options  
  
parser.add_option("-b", "--benchmark", default="",  
                 help="The benchmark to be loaded.")  
  
parser.add_option("--chkpt", default="",  
                 help="The checkpoint to load.")  
  
execfile(os.path.join(config_root, "common", "Options.py"))  
  
if '--ruby' in sys.argv:  
    Ruby.define_options(parser)  
  
(options, args) = parser.parse_args()  
  
if args:  
    print "Error: script doesn't take any positional arguments"  
    sys.exit(1)  
  
if options.benchmark == 'perlbench':  
   process = cpu2006.perlbench  
elif options.benchmark == 'bzip2':  
   process = cpu2006.bzip2  
elif options.benchmark == 'gcc':  
   process = cpu2006.gcc  
elif options.benchmark == 'bwaves':  
   process = cpu2006.bwaves  
elif options.benchmark == 'gamess':  
   process = cpu2006.gamess  
elif options.benchmark == 'mcf':  
   process = cpu2006.mcf  
elif options.benchmark == 'milc':  
   process = cpu2006.milc  
elif options.benchmark == 'zeusmp':  
   process = cpu2006.zeusmp  
elif options.benchmark == 'gromacs':  
   process = cpu2006.gromacs  
elif options.benchmark == 'cactusADM':  
   process = cpu2006.cactusADM  
elif options.benchmark == 'leslie3d':  
   process = cpu2006.leslie3d  
elif options.benchmark == 'namd':  
   process = cpu2006.namd  
elif options.benchmark == 'gobmk':  
   process = cpu2006.gobmk;  
elif options.benchmark == 'dealII':  
   process = cpu2006.dealII  
elif options.benchmark == 'soplex':  
   process = cpu2006.soplex  
elif options.benchmark == 'povray':  
   process = cpu2006.povray  
elif options.benchmark == 'calculix':  
   process = cpu2006.calculix  
elif options.benchmark == 'hmmer':  
   process = cpu2006.hmmer  
elif options.benchmark == 'sjeng':  
   process = cpu2006.sjeng  
elif options.benchmark == 'GemsFDTD':  
   process = cpu2006.GemsFDTD  
elif options.benchmark == 'libquantum':  
   process = cpu2006.libquantum  
elif options.benchmark == 'h264ref':  
   process = cpu2006.h264ref  
elif options.benchmark == 'tonto':  
   process = cpu2006.tonto  
elif options.benchmark == 'lbm':  
   process = cpu2006.lbm  
elif options.benchmark == 'omnetpp':  
   process = cpu2006.omnetpp  
elif options.benchmark == 'astar':  
   process = cpu2006.astar  
elif options.benchmark == 'wrf':  
   process = cpu2006.wrf  
elif options.benchmark == 'sphinx3':  
   process = cpu2006.sphinx3  
elif options.benchmark == 'xalancbmk':  
   process = cpu2006.xalancbmk  
elif options.benchmark == 'specrand_i':  
   process = cpu2006.specrand_i  
elif options.benchmark == 'specrand_f':  
   process = cpu2006.specrand_f  
  
if options.chkpt != "":  
   process.chkpt = options.chkpt  
  
(CPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)  
  
CPUClass.clock = '1.0GHz'  
  
#np = options.num_cpus   
np = 1  
  
system = System(cpu = [CPUClass(cpu_id=i) for i in xrange(np)],  
                physmem = SimpleMemory(range=AddrRange("1024MB")),  
                membus = CoherentBus(), mem_mode = 'timing')  
  
if options.ruby:  
    if not (options.cpu_type == "detailed" or options.cpu_type == "timing"):  
        print >> sys.stderr, "Ruby requires TimingSimpleCPU or O3CPU!!"  
        sys.exit(1)  
  
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
    system.physmem.port = system.membus.master  
    system.system_port = system.membus.slave  
    CacheConfig.config_cache(options,system)  
  
for i in xrange(np):     
#    if options.caches:  
#        system.cpu[i].addPrivateSplitL1Caches(L1Cache(size = '64kB'),  
#                                              L1Cache(size = '64kB'))  
#    if options.l2cache:  
#        system.l2 = L2Cache(size='2MB')  
#        system.tol2bus = Bus()  
#        system.l2.cpu_side = system.tol2bus.port  
#        system.l2.mem_side = system.membus.port  
#        system.cpu[i].connectMemPorts(system.tol2bus)  
#    else:  
#        system.cpu[i].connectMemPorts(system.membus)  
    system.cpu[i].workload = process[i]  
root = Root(full_system = False,system = system)  
Simulation.run(options, root, system, FutureClass) 
