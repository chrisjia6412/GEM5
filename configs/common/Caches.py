# Copyright (c) 2012 ARM Limited
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
# Copyright (c) 2006-2007 The Regents of The University of Michigan
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
# Authors: Lisa Hsu

from m5.objects import *

# Base implementations of L1, L2, IO and TLB-walker caches. There are
# used in the regressions and also as base components in the
# system-configuration scripts. The values are meant to serve as a
# starting point, and specific parameters can be overridden in the
# specific instantiations.

class L1Cache(BaseCache):
    assoc = 4
    size = '32kB'
    hit_latency = 2
    write_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20
    is_top_level = True
    prefetcher = NULL

class L2Cache(BaseCache):
    assoc = 8
    #stt_assoc = 16
    size = '256kB'
    #stt_size = '2MB'
    hit_latency = 6
    write_latency = 6
    response_latency = 6
    mshrs = 20
    #mshrs = 80
    #is_bottom_level = False
    #eDRAM_cache_line_size = 512
    #expired_period = 20000000
    tgts_per_mshr = 20
    write_buffers = 8
    #stt_read_latency = 8
    #stt_write_latency = 40

class L3Cache(BaseCache):
    assoc = 16
    stt_assoc = 16
    size = '4MB'
    stt_size = '16MB'
    hit_latency = 10
    write_latency = 10
    response_latency = 10
    #mshrs = 20
    mshrs = 2048
    is_bottom_level = False
    is_LLC = False
    alternative_mech = 4
    performance_threshold = 0.05
    K_value = 8;
    sram_assoc = 4
    sram_per_set = 2
    locality_Mode = False
    testTimeStamp_Mode = False
    large_block_enabled = False
    eDRAM_cache_line_size = 64
    expired_period = 20000000
    refresh_period = 20000000
    tgts_per_mshr = 12
    write_buffers = 1024
    stt_read_latency = 10
    stt_write_latency = 40
    sram_read_latency = 10
    sram_write_latency = 10

class IOCache(BaseCache):
    assoc = 8
    hit_latency = 50
    write_latency = 50
    response_latency = 50
    mshrs = 20
    size = '1kB'
    tgts_per_mshr = 12
    forward_snoops = False
    is_top_level = True
    prefetcher = NULL

class PageTableWalkerCache(BaseCache):
    assoc = 2
    hit_latency = 2
    write_latency = 2
    response_latency = 2
    mshrs = 10
    size = '1kB'
    tgts_per_mshr = 12
    is_top_level = True
    prefetcher = NULL
