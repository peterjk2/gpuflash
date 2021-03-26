# Automatically generated by SST
import sst

# Define SST Program Options:
sst.setProgramOption("timebase", "1ps")
sst.setProgramOption("stopAtCycle", "0 ns")

# Define the SST Components:
comp_cpu0 = sst.Component("cpu0", "macsimComponent.macsimComponent")
comp_cpu0.addParams({
     "trace_file" : "trace_file_list",
     "mem_size" : "4*1024*1024*1024",
     "command_line" : "--use_memhierarchy=1 --max_insts=100000 --use_wb=1 --acq_rel=1",
     "debug_level" : "8",
     "num_link" : "4",
     "frequency" : "4 Ghz",
     "output_dir" : "results",
     "debug" : "0",
     # "param_file" : "../sst-unit-test/a64_tests/params.in"
     "param_file" : "params.in"
})
comp_core0l1icache = sst.Component("core0l1icache", "memHierarchy.Cache")
comp_core0l1icache.addParams({
     "debug_level" : "8",
     "debug" : "0",
     "access_latency_cycles" : "1",
     "cache_frequency" : "4 Ghz",
     "replacement_policy" : "lru",
     "coherence_protocol" : "MESI",
     "associativity" : "8",
     "cache_line_size" : "128",
     "directory_at_next_level" : "0",
     "statistics" : "0",
     "L1" : "1",
     "cache_size" : "64 KB",
     "mshr_latency_cycles" : "0"
})
comp_core0l1dcache = sst.Component("core0l1dcache", "memHierarchy.Cache")
comp_core0l1dcache.addParams({
     "debug_level" : "8",
     "debug" : "0",
     "access_latency_cycles" : "3",
     "cache_frequency" : "4 Ghz",
     "replacement_policy" : "lru",
     "coherence_protocol" : "MESI",
     "associativity" : "8",
     "cache_line_size" : "128",
     "directory_at_next_level" : "0",
     "statistics" : "1",
     "L1" : "1",
     "cache_size" : "64 KB",
     "mshr_latency_cycles" : "0"
})
comp_core1l1icache = sst.Component("core1l1icache", "memHierarchy.Cache")
comp_core1l1icache.addParams({
     "debug_level" : "8",
     "debug" : "0",
     "access_latency_cycles" : "1",
     "cache_frequency" : "4 Ghz",
     "replacement_policy" : "lru",
     "coherence_protocol" : "MESI",
     "associativity" : "8",
     "cache_line_size" : "128",
     "directory_at_next_level" : "0",
     "statistics" : "0",
     "L1" : "1",
     "cache_size" : "64 KB",
     "mshr_latency_cycles" : "0"
})
comp_core1l1dcache = sst.Component("core1l1dcache", "memHierarchy.Cache")
comp_core1l1dcache.addParams({
     "debug_level" : "8",
     "debug" : "0",
     "access_latency_cycles" : "3",
     "cache_frequency" : "4 Ghz",
     "replacement_policy" : "lru",
     "coherence_protocol" : "MESI",
     "associativity" : "8",
     "cache_line_size" : "128",
     "directory_at_next_level" : "0",
     "statistics" : "1",
     "L1" : "1",
     "cache_size" : "64 KB",
     "mshr_latency_cycles" : "0"
})
comp_core2l1icache = sst.Component("core2l1icache", "memHierarchy.Cache")
comp_core2l1icache.addParams({
     "debug_level" : "8",
     "debug" : "0",
     "access_latency_cycles" : "1",
     "cache_frequency" : "4 Ghz",
     "replacement_policy" : "lru",
     "coherence_protocol" : "MESI",
     "associativity" : "8",
     "cache_line_size" : "128",
     "directory_at_next_level" : "0",
     "statistics" : "0",
     "L1" : "1",
     "cache_size" : "64 KB",
     "mshr_latency_cycles" : "0"
})
comp_core2l1dcache = sst.Component("core2l1dcache", "memHierarchy.Cache")
comp_core2l1dcache.addParams({
     "debug_level" : "8",
     "debug" : "0",
     "access_latency_cycles" : "3",
     "cache_frequency" : "4 Ghz",
     "replacement_policy" : "lru",
     "coherence_protocol" : "MESI",
     "associativity" : "8",
     "cache_line_size" : "128",
     "directory_at_next_level" : "0",
     "statistics" : "1",
     "L1" : "1",
     "cache_size" : "64 KB",
     "mshr_latency_cycles" : "0"
})
comp_core3l1icache = sst.Component("core3l1icache", "memHierarchy.Cache")
comp_core3l1icache.addParams({
     "debug_level" : "8",
     "debug" : "0",
     "access_latency_cycles" : "1",
     "cache_frequency" : "4 Ghz",
     "replacement_policy" : "lru",
     "coherence_protocol" : "MESI",
     "associativity" : "8",
     "cache_line_size" : "128",
     "directory_at_next_level" : "0",
     "statistics" : "0",
     "L1" : "1",
     "cache_size" : "64 KB",
     "mshr_latency_cycles" : "0"
})
comp_core3l1dcache = sst.Component("core3l1dcache", "memHierarchy.Cache")
comp_core3l1dcache.addParams({
     "debug_level" : "8",
     "debug" : "0",
     "access_latency_cycles" : "3",
     "cache_frequency" : "4 Ghz",
     "replacement_policy" : "lru",
     "coherence_protocol" : "MESI",
     "associativity" : "8",
     "cache_line_size" : "128",
     "directory_at_next_level" : "0",
     "statistics" : "1",
     "L1" : "1",
     "cache_size" : "64 KB",
     "mshr_latency_cycles" : "0"
})

comp_cpu0l1l2bus = sst.Component("cpu0l1l2bus", "memHierarchy.Bus")
comp_cpu0l1l2bus.addParams({
     "debug" : "0",
     "bus_frequency" : "4 Ghz"
})
comp_cpu0l2cache = sst.Component("cpu0l2cache", "memHierarchy.Cache")
comp_cpu0l2cache.addParams({
     "debug_level" : "8",
     "debug" : "0",
     "access_latency_cycles" : "8",
     "cache_frequency" : "4 Ghz",
     "replacement_policy" : "lru",
     "coherence_protocol" : "MESI",
     "associativity" : "8",
     "cache_line_size" : "128",
     "directory_at_next_level" : "0",
     "statistics" : "1",
     "L1" : "0",
     "cache_size" : "512 KB",
     "mshr_latency_cycles" : "0"
})
comp_memory0 = sst.Component("memory0", "memHierarchy.MemController")
comp_memory0.addParams({
     "debug" : "0",
     "coherence_protocol" : "MESI",
     "statistics" : "1",
     "backend.mem_size" : "4096",
     "clock" : "1 GHz",
     "access_time" : "97 ns",
     "rangeStart" : "0"
})


# Define the SST Component Statistics Information
# Define SST Statistics Options:

# Define Component Statistics Information:


# Define SST Simulation Link Information
link_c0_icache = sst.Link("link_c0_icachec0_icache")
link_c0_icache.connect( (comp_cpu0, "core0-icache", "1000ps"), (comp_core0l1icache, "high_network_0", "1000ps") )
link_c0_dcache = sst.Link("link_c0_dcachec0_dcache")
link_c0_dcache.connect( (comp_cpu0, "core0-dcache", "1000ps"), (comp_core0l1dcache, "high_network_0", "1000ps") )
link_c1_icache = sst.Link("link_c1_icachec1_icache")
link_c1_icache.connect( (comp_cpu0, "core1-icache", "1000ps"), (comp_core1l1icache, "high_network_0", "1000ps") )
link_c1_dcache = sst.Link("link_c1_dcachec1_dcache")
link_c1_dcache.connect( (comp_cpu0, "core1-dcache", "1000ps"), (comp_core1l1dcache, "high_network_0", "1000ps") )
link_c2_icache = sst.Link("link_c2_icachec2_icache")
link_c2_icache.connect( (comp_cpu0, "core2-icache", "1000ps"), (comp_core2l1icache, "high_network_0", "1000ps") )
link_c2_dcache = sst.Link("link_c2_dcachec2_dcache")
link_c2_dcache.connect( (comp_cpu0, "core2-dcache", "1000ps"), (comp_core2l1dcache, "high_network_0", "1000ps") )
link_c3_icache = sst.Link("link_c3_icachec3_icache")
link_c3_icache.connect( (comp_cpu0, "core3-icache", "1000ps"), (comp_core3l1icache, "high_network_0", "1000ps") )
link_c3_dcache = sst.Link("link_c3_dcachec3_dcache")
link_c3_dcache.connect( (comp_cpu0, "core3-dcache", "1000ps"), (comp_core3l1dcache, "high_network_0", "1000ps") )
link_c0_icache_bus = sst.Link("link_c0_icache_busc0_icache_bus")
link_c0_icache_bus.connect( (comp_core0l1icache, "low_network_0", "10000ps"), (comp_cpu0l1l2bus, "high_network_0", "10000ps") )
link_c0_dcache_bus = sst.Link("link_c0_dcache_busc0_dcache_bus")
link_c0_dcache_bus.connect( (comp_core0l1dcache, "low_network_0", "10000ps"), (comp_cpu0l1l2bus, "high_network_1", "10000ps") )
link_c1_icache_bus = sst.Link("link_c1_icache_busc1_icache_bus")
link_c1_icache_bus.connect( (comp_core1l1icache, "low_network_0", "10000ps"), (comp_cpu0l1l2bus, "high_network_2", "10000ps") )
link_c1_dcache_bus = sst.Link("link_c1_dcache_busc1_dcache_bus")
link_c1_dcache_bus.connect( (comp_core1l1dcache, "low_network_0", "10000ps"), (comp_cpu0l1l2bus, "high_network_3", "10000ps") )
link_c2_icache_bus = sst.Link("link_c2_icache_busc2_icache_bus")
link_c2_icache_bus.connect( (comp_core2l1icache, "low_network_0", "10000ps"), (comp_cpu0l1l2bus, "high_network_4", "10000ps") )
link_c2_dcache_bus = sst.Link("link_c2_dcache_busc2_dcache_bus")
link_c2_dcache_bus.connect( (comp_core2l1dcache, "low_network_0", "10000ps"), (comp_cpu0l1l2bus, "high_network_5", "10000ps") )
link_c3_icache_bus = sst.Link("link_c3_icache_busc3_icache_bus")
link_c3_icache_bus.connect( (comp_core3l1icache, "low_network_0", "10000ps"), (comp_cpu0l1l2bus, "high_network_6", "10000ps") )
link_c3_dcache_bus = sst.Link("link_c3_dcache_busc3_dcache_bus")
link_c3_dcache_bus.connect( (comp_core3l1dcache, "low_network_0", "10000ps"), (comp_cpu0l1l2bus, "high_network_7", "10000ps") )
link_bus_cpu0l2cache = sst.Link("link_bus_cpu0l2cachebus_cpu0l2cache")
link_bus_cpu0l2cache.connect( (comp_cpu0l1l2bus, "low_network_0", "10000ps"), (comp_cpu0l2cache, "high_network_0", "10000ps") )
link_cpu0l2_mem0 = sst.Link("link_cpu0l2_mem0cpu0l2_mem0")
link_cpu0l2_mem0.connect( (comp_cpu0l2cache, "low_network_0", "10000ps"), (comp_memory0, "direct_link", "10000ps") )
# End of generated output.

