verilator -DBENCH -DPASSTHROUGH_PLL -Wno-fatal --top-module soc -cc -exe sim_main.cpp soc.sv
