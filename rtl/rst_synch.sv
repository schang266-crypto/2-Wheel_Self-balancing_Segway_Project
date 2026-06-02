module rst_synch(RST_n, clk, rst_n);

input RST_n;
input clk;
output logic rst_n;
logic q1;

always@ (negedge clk or negedge RST_n) begin
    if (!RST_n) begin
        rst_n <= 0;
        q1    <= 0;
    end

    else begin
        q1    <= 1'b1;
        rst_n <= q1;
    end
end

endmodule