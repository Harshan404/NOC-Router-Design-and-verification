module tb_noc_router;
    parameter FLIT_WIDTH = 64;
    parameter V = 2;
    parameter NUM_PORTS = 5;
    logic clk;
    logic rst_n;
    logic credit_mode;

    logic [NUM_PORTS-1:0][FLIT_WIDTH-1:0] in_flit;
    logic [NUM_PORTS-1:0] in_valid;
    logic [NUM_PORTS-1:0] in_ready;
    logic [NUM_PORTS-1:0][V-1:0] in_credit_out;

    logic [NUM_PORTS-1:0][FLIT_WIDTH-1:0] out_flit;
    logic [NUM_PORTS-1:0] out_valid;
    logic [NUM_PORTS-1:0] out_ready;
    logic [NUM_PORTS-1:0][V-1:0] out_credit_in;

    noc_router #(.FLIT_WIDTH(FLIT_WIDTH), .V(V), .FIFO_DEPTH(4), .COORD_WIDTH(4))
        DUT(.clk(clk), .rst_n(rst_n), .credit_mode(credit_mode),
            .in_flit(in_flit), .in_valid(in_valid), .in_ready(in_ready), .in_credit_out(in_credit_out),
            .out_flit(out_flit), .out_valid(out_valid), .out_ready(out_ready), .out_credit_in(out_credit_in));

    // clock
    initial clk = 0;
    always #5 clk = ~clk;

    // simple driver: send a HEAD->TAIL packet from Local port to East (dest_x=1,my_x=0)
    // build header flit: [dest_x (4 bits) at bits COORD_WIDTH+1 +: COORD_WIDTH], dest_y bits at [1 +: COORD_WIDTH], type in [1:0]
    function logic [FLIT_WIDTH-1:0] make_head(input int dx, input int dy);
        logic [FLIT_WIDTH-1:0] f;
        f = '0;
        f[1:0] = 2'b10; // HEAD
        f[1 +: 4] = dy;
        f[4 +: 4] = dx;
        return f;
    endfunction
    function logic [FLIT_WIDTH-1:0] make_body(int payload);
        logic [FLIT_WIDTH-1:0] f;
        f = '0;
        f[1:0] = 2'b00; // BODY
        f[FLIT_WIDTH-1 -: 32] = payload; // some payload
        return f;
    endfunction
    function logic [FLIT_WIDTH-1:0] make_tail(int payload);
        logic [FLIT_WIDTH-1:0] f;
        f = '0;
        f[1:0] = 2'b01; // TAIL
        f[FLIT_WIDTH-1 -: 32] = payload;
        return f;
    endfunction

    initial begin
        rst_n = 0;
        credit_mode = 0; // start with ready/valid mode
        in_valid = '0;
        out_ready = '1; // downstream always ready
        out_credit_in = '0;
        #20;
        rst_n = 1;
        #20;

        // send one 3-flit packet on Local port (index 4) destined to X=1,Y=0 -> East
        in_flit[4] = make_head(1,0);
        in_valid[4] = 1;
        @(posedge clk); // push head
        in_valid[4] = 0;
        // body
        in_flit[4] = make_body(32'hDEADBEEF);
        in_valid[4] = 1;
        @(posedge clk);
        in_valid[4] = 0;
        // tail
        in_flit[4] = make_tail(32'hCAFEBABE);
        in_valid[4] = 1;
        @(posedge clk);
        in_valid[4] = 0;

        // wait some cycles and enable credit mode to test both modes
        #200;
        credit_mode = 1;
        $display("Enabled credit_mode at time %0t", $time);

        // finish
        #200;
        $finish;
    end

    // monitor some signals
    always @(posedge clk) begin
        if (out_valid[1]) begin // observe East output (port 1)
            $display("[%0t] OUT(E): flit=%h valid=%0d", $time, out_flit[1], out_valid[1]);
        end
    end

endmodule