from m5.objects import *

# My attempt to model Nehalem, need to add/extend instruction classes to
# really model. Latencies are also probably off in some places, but set
# to what is used by crib

# Execution Unit Resources
class Port0_FU(FUDesc):
    opList = [ OpDesc(opClass="IntAlu", opLat=1),
               OpDesc(opClass="IntDiv", opLat=20, issueLat=20),
               OpDesc(opClass="FloatMult", opLat=5),
               OpDesc(opClass="FloatCvt", opLat=3),
               OpDesc(opClass="FloatDiv", opLat=10),
               OpDesc(opClass="FloatSqrt", opLat=10),
               OpDesc(opClass="SimdFloatMult", opLat=5),
               OpDesc(opClass="SimdFloatMultAcc", opLat=6),
               OpDesc(opClass="SimdFloatCvt", opLat=3),
               OpDesc(opClass="SimdFloatDiv", opLat=10),
               OpDesc(opClass="SimdFloatSqrt", opLat=10),
               OpDesc(opClass="SimdAddAcc", opLat=1),
               OpDesc(opClass="SimdAdd", opLat=1),
               OpDesc(opClass="SimdAlu", opLat=1),
               OpDesc(opClass="SimdShiftAcc", opLat=1),
               OpDesc(opClass="SimdShift", opLat=1) ]
    count = 1

class Port1_FU(FUDesc):
    opList = [ OpDesc(opClass="IntAlu", opLat=1),
               OpDesc(opClass="IntMult", opLat=3),
               OpDesc(opClass="IprAccess", opLat=3),
               OpDesc(opClass="FloatAdd", opLat=3),
               OpDesc(opClass="SimdFloatAlu", opLat=3),
               OpDesc(opClass="SimdFloatAdd", opLat=3),
               OpDesc(opClass="SimdMult", opLat=3),
               OpDesc(opClass="SimdMultAcc", opLat=4),
               OpDesc(opClass="SimdSqrt", opLat=4),
               OpDesc(opClass="SimdCvt", opLat=3) ]
    count = 1

class Port2_FU(FUDesc):
    opList = [ OpDesc(opClass="MemRead", opLat=1) ]
    count = 4

class Port34_FU(FUDesc):
    opList = [ OpDesc(opClass="MemWrite", opLat=1) ]
    count = 1

class Port5_FU(FUDesc):
    opList = [ OpDesc(opClass="IntAlu", opLat=1),
               OpDesc(opClass="FloatCmp", opLat=1),
               OpDesc(opClass="SimdFloatCmp", opLat=3),
               OpDesc(opClass="SimdFloatMisc", opLat=3),
               OpDesc(opClass="SimdCmp", opLat=1),
               OpDesc(opClass="SimdMisc", opLat=3),
               OpDesc(opClass="SimdAdd", opLat=1),
               OpDesc(opClass="SimdAddAcc", opLat=1),
               OpDesc(opClass="SimdShiftAcc", opLat=1),
               OpDesc(opClass="SimdShift", opLat=1),
               OpDesc(opClass="SimdAlu", opLat=1) ]
    count = 1

class Nehalem_FUPool(FUPool):
    FUList = [ Port0_FU(), Port1_FU(), Port2_FU(), Port34_FU(), Port5_FU() ]

class Nehalem_BP(BranchPredictor):
    predType = "tournament"
    localCtrBits = 2
    localHistoryTableSize = 64
    #localHistoryBits = 6
    globalPredictorSize = 8192
    globalCtrBits = 2
    #globalHistoryBits = 13
    choicePredictorSize = 8192
    choiceCtrBits = 2
    BTBEntries = 2048
    BTBTagSize = 18
    RASSize = 16
    instShiftAmt = 2

class Nehalem(DerivO3CPU):
    LQEntries = 48
    SQEntries = 32
    LSQDepCheckShift = 0
    LFSTSize = 1024
    SSITSize = 1024
    decodeToFetchDelay = 1
    renameToFetchDelay = 1
    iewToFetchDelay = 1
    commitToFetchDelay = 1
    renameToDecodeDelay = 1
    iewToDecodeDelay = 1
    commitToDecodeDelay = 1
    iewToRenameDelay = 1
    commitToRenameDelay = 1
    commitToIEWDelay = 1
    fetchWidth = 4
    fetchToDecodeDelay = 2
    decodeWidth = 4
    decodeToRenameDelay = 2
    renameWidth = 4
    renameToIEWDelay = 2
    issueToExecuteDelay = 1
    dispatchWidth = 4
    issueWidth = 5
    wbWidth = 5
    wbDepth = 1
    fuPool = Nehalem_FUPool()
    iewToCommitDelay = 1
    renameToROBDelay = 1
    commitWidth = 4
    squashWidth = 16
    trapLatency = 13
    backComSize = 10
    forwardComSize = 5
    numPhysIntRegs = 128
    numPhysFloatRegs = 128
    numIQEntries = 36
    numROBEntries = 128

    switched_out = False
    branchPred = Nehalem_BP()
