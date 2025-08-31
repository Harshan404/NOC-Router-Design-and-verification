// noc_router.sv
// 5-port Wormhole NoC Router (N,S,E,W,Local) - SystemVerilog
// Features:
//  - XY deterministic routing
//  - Wormhole flits: HEAD/BODY/TAIL
//  - Per-input VC FIFOs
//  - Per-output round-robin arbiter (per output port)
//  - Per-VC credit tracking (optional): credit_mode=1 => credits required to send flit into output VC
//  - credit pulses are single-cycle pulses (credit_in from downstream; credit_out to upstream)
//
// Notes:
//  - Designed for functional simulation; latency/buffer sizing parameterizable.
//  - Flit format is user-configurable via parameters, below defaults assume:
//       [FLIT_WIDTH-1:0] where header flit encodes dest_x, dest_y and type in low bits.
//  - Routing uses tile coordinates; this module assumes destination X/Y are embedded in head flit.
//
// Parameters to adjust:
//  - FLIT_WIDTH, V (VCs per input), FIFO_DEPTH, COORD_WIDTH (dest coords bitwidth), NUM_PORTS (5).
//
// Ports order mapping: 0=N,1=E,2=S,3=W,4=L (local)

`timescale 1ps/1ps

module noc_router #(
    parameter integer FLIT_WIDTH = 64,
    parameter integer V = 2,                 // VCs per input
    parameter integer FIFO_DEPTH = 4,        // depth per VC FIFO (flits)
    parameter integer COORD_WIDTH = 4,       // bits for X and Y destination coordinates
    parameter integer NUM_PORTS = 5
)(
    input  logic clk,
    input  logic rst_n,
    // bring-up mode: 0 => ready/valid only; 1 => enable credit-based per-VC flow control
    input  logic credit_mode,

    // per-port input side (from neighbor -> this router)
    input  logic [NUM_PORTS-1:0][FLIT_WIDTH-1:0] in_flit,
    input  logic [NUM_PORTS-1:0]                 in_valid,
    output logic [NUM_PORTS-1:0]                 in_ready,   // ready towards upstream

    // credit pulses to upstream: single-cycle one-hot per VC when router frees a slot
    output logic [NUM_PORTS-1:0][V-1:0]          in_credit_out,

    // per-port output side (to neighbor <- this router)
    output logic [NUM_PORTS-1:0][FLIT_WIDTH-1:0] out_flit,
    output logic [NUM_PORTS-1:0]                 out_valid,
    input  logic [NUM_PORTS-1:0]                 out_ready,  // backpressure from downstream

    // credit pulses from downstream: single-cycle pulses per VC when downstream frees a slot
    input  logic [NUM_PORTS-1:0][V-1:0]          out_credit_in
);

    // Port index mapping constants
    localparam int P_N = 0;
    localparam int P_E = 1;
    localparam int P_S = 2;
    localparam int P_W = 3;
    localparam int P_L = 4;

    // Flit type encoding (2 LSBs used for type)
    localparam logic [1:0] FLIT_TYPE_BODY = 2'b00;
    localparam logic [1:0] FLIT_TYPE_TAIL = 2'b01;
    localparam logic [1:0] FLIT_TYPE_HEAD = 2'b10;

    // Define how header encodes destination:
    // [FLIT_WIDTH-1 : (COORD_WIDTH*2 + 2)]  payload...
    // [COORD_WIDTH+1 +: COORD_WIDTH] = dest_x
    // [1 +: COORD_WIDTH] = dest_y
    // [1:0] = flit_type
    function automatic logic [COORD_WIDTH-1:0] flit_dest_x(input logic [FLIT_WIDTH-1:0] fl);
        flit_dest_x = fl[COORD_WIDTH+1 +: COORD_WIDTH];
    endfunction
    function automatic logic [COORD_WIDTH-1:0] flit_dest_y(input logic [FLIT_WIDTH-1:0] fl);
        flit_dest_y = fl[1 +: COORD_WIDTH];
    endfunction
    function automatic logic [1:0] flit_type(input logic [FLIT_WIDTH-1:0] fl);
        flit_type = fl[1:0];
    endfunction

    // For routing we need current tile coords. For a tileable router you will usually set
    // these by parameter or ports. For simulation, we'll use generic (0,0) local coords
    // unless user wires them externally. To keep module self-contained we declare regs:
    // For real tiled system, prefer external wiring.
    logic [COORD_WIDTH-1:0] my_x, my_y;
    // default to 0,0 (tileable)
    always_comb begin
        my_x = '0;
        my_y = '0;
    end

    // Basic structs: per-input per-VC FIFO storage (simplified circular fifo)
    typedef struct {
        logic [FLIT_WIDTH-1:0] mem [0:FIFO_DEPTH-1];
        logic [$clog2(FIFO_DEPTH+1)-1:0] rd_ptr, wr_ptr;
        logic [$clog2(FIFO_DEPTH+1)-1:0] count;
    } fifo_t;

    // allocate FIFOs: input_ports x VCs
    fifo_t in_fifo [NUM_PORTS-1:0][V-1:0];

    // Track which FIFO is currently holding an active packet head (for partial allocation state)
    // track current_out_vc allocation mapping: which input (port,vc) is granted to which output port
    typedef struct packed { logic valid; logic [2:0] in_port; logic [$clog2(V)-1:0] in_vc; } alloc_entry_t;
    alloc_entry_t switch_alloc [NUM_PORTS-1:0]; // indexed by output port -> which input is granted

    // credits available per output VC (when credit_mode==1). Start with FIFO_DEPTH credits initially
    logic [$clog2(FIFO_DEPTH+1)-1:0] out_vc_credits [NUM_PORTS-1:0][V-1:0];

    // helper: compute desired output port given destination (XY routing)
    function automatic int route_xy(input logic [COORD_WIDTH-1:0] dst_x, input logic [COORD_WIDTH-1:0] dst_y);
        // returns one of 0..4 (N,E,S,W,L)
        if (dst_x > my_x) begin
            route_xy = P_E;
        end else if (dst_x < my_x) begin
            route_xy = P_W;
        end else begin
            if (dst_y > my_y) route_xy = P_S;
            else if (dst_y < my_y) route_xy = P_N;
            else route_xy = P_L;
        end
    endfunction

    // Ready/Valid handshake logic (upstream sees in_ready true when there is space in chosen VC FIFO)
    // We accept a head flit plus subsequent body/tail for same VC. For simplicity we select VC allocation
    // as the first free VC in input port (round-robin not needed here).
    genvar gp, gv;
    generate
        for (gp=0; gp<NUM_PORTS; gp++) begin : GEN_PORTS
            for (gv=0; gv<V; gv++) begin : GEN_VCS
                // initial conditions
                initial begin
                    in_fifo[gp][gv].rd_ptr = 0;
                    in_fifo[gp][gv].wr_ptr = 0;
                    in_fifo[gp][gv].count  = 0;
                end
            end

            // Default in_credit_out pulses low
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    for (int vci=0; vci<V; vci++) in_credit_out[gp][vci] <= 0;
                end else begin
                    for (int vci=0; vci<V; vci++) in_credit_out[gp][vci] <= 0; // pulsed when fifo pops
                end
            end
        end
    endgenerate

    // A simple input-side logic: pick a VC to push incoming flits into. We use the lowest-numbered VC with space.
    // Note: This is simplified; in full designs VC allocation is part of head processing. For functional sim it's ok.
    logic [NUM_PORTS-1:0] space_avail;
    always_comb begin
        for (int p=0; p<NUM_PORTS; p++) begin
            in_ready[p] = 1'b0; // default not ready
            // find any VC that has space
            space_avail[p] = 1'b0;
            for (int vci=0; vci<V; vci++) begin
                if (in_fifo[p][vci].count < FIFO_DEPTH)
                    space_avail = 1;
            end
            // simple: advertise ready whenever any VC has space
            in_ready[p] = space_avail[p];
        end
    end

    // Push incoming flit into first available VC FIFO on valid && ready
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // reset FIFOs
            for (int p=0; p<NUM_PORTS; p++) begin
                for (int vci=0; vci<V; vci++) begin
                    in_fifo[p][vci].rd_ptr <= 0;
                    in_fifo[p][vci].wr_ptr <= 0;
                    in_fifo[p][vci].count  <= 0;
                end
            end
        end else begin
            for (int p=0; p<NUM_PORTS; p++) begin
                if (in_valid[p] && in_ready[p]) begin
                    // find first VC with space
                    int sel_v = -1;
                    for (int vci=0; vci<V; vci++) begin
                        if (in_fifo[p][vci].count < FIFO_DEPTH) begin sel_v = vci; break; end
                    end
                    if (sel_v != -1) begin
                        // write flit
                        int wp = in_fifo[p][sel_v].wr_ptr;
                        in_fifo[p][sel_v].mem[wp] <= in_flit[p];
                        in_fifo[p][sel_v].wr_ptr <= (wp + 1) % FIFO_DEPTH;
                        in_fifo[p][sel_v].count  <= in_fifo[p][sel_v].count + 1;
                    end
                    // else drop (shouldn't happen because in_ready false)
                end
            end
        end
    end

    // Routing + switch allocation + crossbar transfer loop (cycle-by-cycle)
    // For each input VC that has a head flit available at FIFO rd_ptr, compute desired output.
    // The switch allocator grants one input VC to one output port (per output only one winner).
    // We use a round-robin pointer per output to pick among contending inputs.

    // pointers for round-robin arbitration per output
    integer rr_ptr [NUM_PORTS-1:0];
    initial begin
        for (int i=0; i<NUM_PORTS; i++) rr_ptr[i] = 0;
    end

    // Temporary arrays for contenders: for each output port, list of (port,vc) with head ready
    typedef struct packed { logic has; int p; int v; } contender_t;
    contender_t contenders [NUM_PORTS-1:0][(NUM_PORTS*V)-1:0]; // over-provision
    int cont_cnt[NUM_PORTS-1:0];

    // Extract head flit presence
    function automatic logic has_head(input int p, input int v);
        logic ret;
        ret = 0;
        if (in_fifo[p][v].count > 0) begin
            // check that flit at rd_ptr is HEAD or BODY/TAIL continuation may also be present
            // We conservatively allow BODY/TAIL too to flow if already owning the path; for simplicity we match header type only
            ret = (flit_type(in_fifo[p][v].mem[in_fifo[p][v].rd_ptr]) == FLIT_TYPE_HEAD);
        end
        has_head = ret;
    endfunction

    // zero allocs initially
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int o=0; o<NUM_PORTS; o++) begin
                switch_alloc[o].valid <= 0;
                switch_alloc[o].in_port <= 0;
                switch_alloc[o].in_vc <= 0;
            end
            // init credits
            for (int o=0; o<NUM_PORTS; o++) begin
                for (int vv=0; vv<V; vv++) begin
                    out_vc_credits[o][vv] <= FIFO_DEPTH; // assume downstream buffers initially empty so we can send FIFO_DEPTH flits
                end
            end
        end else begin
            // clear contender lists
            for (int o=0; o<NUM_PORTS; o++) cont_cnt[o] = 0;

            // build contender list from input FIFOs that have HEAD flits
            for (int p=0; p<NUM_PORTS; p++) begin
                for (int vci=0; vci<V; vci++) begin
                    if (in_fifo[p][vci].count > 0) begin
                        // peek current flit
                        logic [FLIT_WIDTH-1:0] peek_fl = in_fifo[p][vci].mem[in_fifo[p][vci].rd_ptr];
                        logic [1:0]   t = flit_type(peek_fl);
                        int desired_out = route_xy(flit_dest_x(peek_fl), flit_dest_y(peek_fl));
                        // if it's a head flit we place it as contender; also allow body/tail if continuing (not fully tracked here)
                        if (t == FLIT_TYPE_HEAD) begin
                            int idx = cont_cnt[desired_out];
                            contenders[desired_out][idx].has = 1;
                            contenders[desired_out][idx].p = p;
                            contenders[desired_out][idx].v = vci;
                            cont_cnt[desired_out] = cont_cnt[desired_out] + 1;
                        end
                    end
                end
            end

            // arbitration per output (round-robin)
            for (int o=0; o<NUM_PORTS; o++) begin
                if (cont_cnt[o] == 0) begin
                    // if no contenders, free the alloc if current packet finished (we assume switch_alloc holds until tail forwarded)
                    // For simplicity if current alloc is invalid, keep invalid
                end else begin
                    // perform RR starting at rr_ptr[o] modulo cont_cnt
                    int winner_index = -1;
                    for (int k=0; k<cont_cnt[o]; k++) begin
                        int idx = (rr_ptr[o] + k) % cont_cnt[o];
                        if (contenders[o][idx].has) begin
                            winner_index = idx;
                            break;
                        end
                    end
                    if (winner_index != -1) begin
                        // grant
                        switch_alloc[o].valid <= 1;
                        switch_alloc[o].in_port <= contenders[o][winner_index].p;
                        switch_alloc[o].in_vc <= contenders[o][winner_index].v;
                        // advance rr pointer for next arbitration
                        rr_ptr[o] = (winner_index + 1) % cont_cnt[o];
                    end
                end
            end

            // Transfer flits from granted input VC to output as long as out_ready or credits permit.
            for (int o=0; o<NUM_PORTS; o++) begin
                if (switch_alloc[o].valid) begin
                    int ip = switch_alloc[o].in_port;
                    int iv = switch_alloc[o].in_vc;
                    // if input FIFO has flits
                    if (in_fifo[ip][iv].count > 0) begin
                        logic [FLIT_WIDTH-1:0] sending = in_fifo[ip][iv].mem[in_fifo[ip][iv].rd_ptr];
                        // check flow-control: if credit_mode enabled, require an output VC credit available
                        // For simplicity we map input VC -> same-numbered output VC (mod V). More complex mapping possible.
                        int out_vc = iv % V;
                        logic can_send;
                        if (credit_mode) begin
                            can_send = (out_vc_credits[o][out_vc] > 0) && out_ready[o];
                        end else begin
                            can_send = out_ready[o];
                        end
                        if (can_send) begin
                            // send flit
                            out_flit[o]  <= sending;
                            out_valid[o] <= 1;
                            // pop input fifo
                            in_fifo[ip][iv].rd_ptr <= (in_fifo[ip][iv].rd_ptr + 1) % FIFO_DEPTH;
                            in_fifo[ip][iv].count  <= in_fifo[ip][iv].count - 1;
                            // produce credit pulse to upstream for that input VC (single-cycle)
                            in_credit_out[ip][iv] <= 1'b1;
                            // consume credit if credit_mode
                            if (credit_mode) begin
                                out_vc_credits[o][out_vc] <= out_vc_credits[o][out_vc] - 1;
                            end
                            // if this flit is TAIL, then release the allocation after transfer
                            if (flit_type(sending) == FLIT_TYPE_TAIL) begin
                                switch_alloc[o].valid <= 0;
                            end
                        end else begin
                            // cannot send; keep out_valid low
                            out_valid[o] <= 1'b0;
                            in_credit_out[ip][iv] <= 1'b0;
                        end
                    end else begin
                        // no flit to send; clear
                        out_valid[o] <= 1'b0;
                        in_credit_out[switch_alloc[o].in_port][switch_alloc[o].in_vc] <= 0;
                        switch_alloc[o].valid <= 0;
                    end
                end else begin
                    out_valid[o] <= 1'b0;
                end
            end

            // process incoming credit pulses from downstream: increase corresponding out_vc_credits
            for (int o=0; o<NUM_PORTS; o++) begin
                for (int vv=0; vv<V; vv++) begin
                    if (out_credit_in[o][vv]) begin
                        // increment credit count but saturate at FIFO_DEPTH
                        if (out_vc_credits[o][vv] < FIFO_DEPTH) out_vc_credits[o][vv] <= out_vc_credits[o][vv] + 1;
                    end
                end
            end
        end
    end

endmodule